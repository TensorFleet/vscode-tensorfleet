// drone-controller.ts
/**
 * High-level drone controller:
 *  - Uses DroneStateModel (subscriptions handled there) to read vehicle state
 *  - Uses ros2Bridge to publish MAVROS setpoints and call MAVROS services
 *  - No ACK system
 *  - Requested/target state with optional automatic enforcement (tick every second)
 */

import * as RosTypes from "../../ros/ros-types"
import { DroneStateModel, LANDED } from "../drone-state-model";
import type { ROS2BridgeApi } from "../../ros/ros-bridge-api";

export enum LandedState {
  UNDEFINED = 0,
  ON_GROUND = 1,
  IN_AIR = 2,
  TAKEOFF = 3,
  LANDING = 4,
}

export type RequestedState =
  | { kind: "none" }
  | { kind: "landed"; armed: boolean | null }
  | { kind: "airborne"; altMeters: number; yawRad?: number }

export type OffboardTarget =
  | { kind: "position_local"; x: number; y: number; z: number; yawRad?: number }
  | { kind: "velocity_local"; vx: number; vy: number; vz: number; yawRate?: number }
  | {
      kind: "raw_local";
      coordinate_frame: number;
      type_mask: number;
      position?: { x: number; y: number; z: number };
      velocity?: { x: number; y: number; z: number };
      acceleration_or_force?: { x: number; y: number; z: number };
      yaw?: number;
      yaw_rate?: number;
    }
  | {
      kind: "raw_attitude";
      type_mask: number;
      orientation?: { x: number; y: number; z: number; w: number };
      body_rate?: { x: number; y: number; z: number };
      thrust?: number;
    }

export interface DroneControllerOptions {
  localFrameId?: string;                // default "map"
  offboardAutoTakeoff?: boolean;        // default false
  minBatteryForFlight?: number;         // default 0.15
  autoStateManagement?: boolean;        // default false
  stateManagementIntervalMs?: number;   // default 1000
}

export class DroneController {
  private model: DroneStateModel;
  private ros2Bridge: ROS2BridgeApi;
  private opts: Required<DroneControllerOptions>;

  private _requestedState: RequestedState = { kind: "none" };

  public get requestedState(): RequestedState {
    return this._requestedState;
  }

  private autoStateEnabled = false;
  private stateManagerInterval: any = null;
  private stateManagerTickRunning = false;

  private latestState: any = {};

  private static readonly MAV_CMD_NAV_TAKEOFF = 22;

  private static readonly T_SETPOINT_POS = "/mavros/setpoint_position/local";
  private static readonly TYPE_POSE_STAMPED = "geometry_msgs/msg/PoseStamped";

  private static readonly T_SETPOINT_VEL = "/mavros/setpoint_velocity/cmd_vel";
  private static readonly TYPE_TWIST_STAMPED = "geometry_msgs/msg/TwistStamped";

  private static readonly T_SETPOINT_RAW_LOCAL = "/mavros/setpoint_raw/local";
  private static readonly TYPE_POSITION_TARGET = "mavros_msgs/msg/PositionTarget";

  private static readonly T_SETPOINT_RAW_ATT = "/mavros/setpoint_raw/attitude";
  private static readonly TYPE_ATTITUDE_TARGET = "mavros_msgs/msg/AttitudeTarget";

  private offboardTarget: OffboardTarget | null = null;
  private offboardInterval: any = null;
  private offboardTickRunning = false;
  private lastOffboardModeAttemptMs = 0;
  private lastOffboardTakeoffAttemptMs = 0;

  constructor(model: DroneStateModel, ros2Bridge: ROS2BridgeApi, opts: DroneControllerOptions = {}) {
    this.model = model;
    this.ros2Bridge = ros2Bridge;
    this.opts = {
      localFrameId: opts.localFrameId ?? "map",
      offboardAutoTakeoff: opts.offboardAutoTakeoff ?? false,
      minBatteryForFlight: opts.minBatteryForFlight ?? 0.15,
      autoStateManagement: opts.autoStateManagement ?? false,
      stateManagementIntervalMs: opts.stateManagementIntervalMs ?? 1000,
    };

    this.model.onUpdate((s) => { this.latestState = s; });
  }

  async initialize(): Promise<void> {
    if (this.opts.autoStateManagement) {
      await this.enableAutoStateManagement(true);
    }

    this.startOffboardLoop();
  }

  // -------- Basic services --------

  async arm(): Promise<void> {
    await this._requireConnected();

    if (await this.model.isArmed()) {
      return;
    }

    console.log("[DRONE_CONTROLLER] Sending arm command...");

    // Workaround. arm might fail due to unsupported state for arm.
    if (await this.model.isLanded()) {
      console.log("[DRONE_CONTROLLER] Is in landed state while trying to arm");
      await this.setMode("AUTO.LOITER");  
    }

    const result = await this.mavrosArmDisarm(true);
    console.log("[DRONE_CONTROLLER] Arm command result:", result);
  }

  async disarm(): Promise<void> {
    await this._requireConnected();
    console.log("[DRONE_CONTROLLER] Sending disarm command...");
    const result = await this.mavrosArmDisarm(false);
    console.log("[DRONE_CONTROLLER] Disarm command result:", result);
  }

  async setMode(mode: string, base = 0): Promise<void> {
    await this._requireConnected();
    console.log(`[DRONE_CONTROLLER] Setting mode to ${mode} (base=${base})...`);
    const result = await this.mavrosSetMode(mode, base);
    console.log("[DRONE_CONTROLLER] Set mode result:", result);
  }

  async takeoff(altMeters: number = 3, yawRad = 0): Promise<void> {
    await this.arm();

    const gp = (await this.model.getState()).global_position_int;
    if (!gp) throw new Error("No GPS fix");

    const lat_deg = gp.lat;
    const lon_deg = gp.lon;
    const yaw_deg = yawRad * 180 / Math.PI;

    console.log(`[DRONE_CONTROLLER] Sending takeoff command: alt=${altMeters}m, yaw=${yaw_deg}° at lat=${lat_deg}, lon=${lon_deg}...`);
    const result = await this.mavrosCommandLong({
      command: DroneController.MAV_CMD_NAV_TAKEOFF,
      param1: 0,
      param2: 0,
      param3: 0,
      param4: yaw_deg,
      param5: lat_deg,
      param6: lon_deg,
      param7: altMeters,
      confirmation: 0,
      broadcast: false,
    });
    console.log("[DRONE_CONTROLLER] Takeoff command result:", result);
  }

  async land(): Promise<void> {
    await this._requireConnected();
    console.log("[DRONE_CONTROLLER] Sending land command...");
    const result = await this.mavrosLand();
    console.log("[DRONE_CONTROLLER] Land command result:", result);
  }

  // -------- Requested state / auto state management --------

  public setRequestedState(state: RequestedState): void {
    this._requestedState = state;

    void this._tickRequestedState();
  }

  public clearRequestedState(): void {
    this.setRequestedState({ kind: "none" });
  }

  public isInRequestedState(debug: boolean = false): boolean {
    let currentState = this.model.getCurrentState();

      const landed = DroneStateModel.isStateLanded(currentState);
      const landing = DroneStateModel.isStateLanding(currentState);
      const takingOff = DroneStateModel.isStateTakingOff(currentState);
      const onGround = currentState.extended?.landed_state === LANDED.ON_GROUND;

    if(debug) {
      console.log("[AUTO_STATE] Requested :", this.requestedState, "\nCurrent state :", { extended: currentState.extended, vehicle: currentState.vehicle});
    }

    switch (this.requestedState.kind) {
      case "none":
        return true;
      case "landed": {
        return landed;
      }
      case "airborne": {
        return (currentState.vehicle?.armed && !( landed || landing || takingOff || onGround)) ?? false;
      }
    }

    console.log("Vehicle not in target state");
    return false;
  }

  public async waitForRequestedState(timeoutMs: number = 60000): Promise<void> {
    return new Promise((resolve, reject) => {
      const startTime = Date.now();
      const check = async () => {
        if (!this.autoStateEnabled) {
          resolve();
          return;
        }

        const achieved = await this.isInRequestedState();

        if (achieved) {
          resolve();
        } else if (Date.now() - startTime > timeoutMs) {
          reject(new Error(`Timeout waiting for requested state: ${JSON.stringify(this.requestedState)}`));
        } else {
          setTimeout(check, 500);
        }
      };
      check();
    });
  }



  public async enableAutoStateManagement(enabled: boolean): Promise<void> {
    const wasEnabled = this.autoStateEnabled;
    this.autoStateEnabled = enabled;

    if (!enabled) {
      if (this.stateManagerInterval !== null) {
        clearInterval(this.stateManagerInterval);
        this.stateManagerInterval = null;
      }
      return;
    }

    // Only derive initial target state if auto management was previously disabled
    // and no explicit state has been set yet
    if (!wasEnabled && this.requestedState.kind === "none") {
      console.log("[AUTO_STATE] Deriving initial target state from current drone state");
      // Derive target state from the drone state RIGHT NOW
      const st = await this.model.getState();
      if (!st?.vehicle?.connected) return;

      const landed = st.extended?.landed_state;
      if (landed === LandedState.ON_GROUND) {
        this._requestedState = { kind: "landed", armed: !!st.vehicle.armed };
        console.log(`[AUTO_STATE] Derived initial state: landed, armed=${!!st.vehicle.armed}`);
      } else if (landed === LandedState.IN_AIR || landed === LandedState.TAKEOFF || landed === LandedState.LANDING) {
        const rel = st.global_position_int?.relative_alt;
        const alt = (typeof rel === "number" && Number.isFinite(rel) && rel > 0) ? rel : 2;
        this._requestedState = { kind: "airborne", altMeters: alt };
        console.log(`[AUTO_STATE] Derived initial state: airborne at ${alt}m`);
      } else {
        // Unknown: keep current arming preference but don't force motion
        this._requestedState = { kind: "none" };
        console.log("[AUTO_STATE] Derived initial state: none (unknown state)");
      }
    } else {
      console.log(`[AUTO_STATE] Keeping existing requested state: ${JSON.stringify(this.requestedState)}`);
    }

    if (this.stateManagerInterval !== null) return;

    this.stateManagerInterval = setInterval(() => {
      void this._tickRequestedState();
    }, this.opts.stateManagementIntervalMs);
  }

  private async _tickRequestedState(): Promise<void> {
    if (!this.autoStateEnabled) return;
    if (this.stateManagerTickRunning) return;
    this.stateManagerTickRunning = true;

    console.log(`[AUTO_STATE] Tick: requestedState=${JSON.stringify(this.requestedState)}`);

    try {
      let currentState = this.model.getCurrentState();
      if (!DroneStateModel.isStateConnected(currentState)) {
        console.log("[AUTO_STATE] Drone not connected, skipping tick");
        return;
      }

      const landed = DroneStateModel.isStateLanded(currentState);
      const landing = DroneStateModel.isStateLanding(currentState);
      const takingOff = DroneStateModel.isStateTakingOff(currentState);
      const onGround = currentState.extended?.landed_state === LANDED.ON_GROUND;

      console.log(`[AUTO_STATE] Current state: armed=${currentState.vehicle?.armed}, mode=${currentState.vehicle?.mode}, landed=${currentState.extended?.landed_state}`);

      switch (this.requestedState.kind) {
        case "none":
          console.log("[AUTO_STATE] Requested state is 'none', no action needed");
          return;

        case "landed": {
          // We need the drone landed.
          if (landing) {
            return;
          }

          if(landed) {
            // Do we want to disarm?
            if(this.requestedState.armed != currentState.vehicle?.armed) {
              // We need to change the arm state.
              if(this.requestedState.armed) {
                console.log('[AUTO_STATE] Requesting drone arm');
                await this.arm();
              } else {
                console.log('[AUTO_STATE] Requesting drone disarm');
                await this.disarm();
              }
            }

            return;
          }


          console.log(`[AUTO_STATE] Landed state check: landed=${landed}, onGround=${onGround}, landing=${landing}`);

          if (!landing) {
            console.log("[AUTO_STATE] vehicle not landing. Requesting land");
            await this.land();
            return
          }
          return;
        }

        case "airborne": {
          let requestedAltitude = this.requestedState.altMeters;

          if(landed) {
            console.log("[AUTO_STATE] Processing airborne state [landed = true]. Requesting takeoff");
            await this.takeoff(requestedAltitude);
            return;
          }

          if(landing) {
            console.log("[AUTO_STATE] Processing airborne state [landing = true]. Requesting takeoff");
            await this.takeoff(requestedAltitude);
            return;
          }
          
          return;
        }
      }
    } catch (e) {
      console.log(`[AUTO_STATE] Error in tick: ${e instanceof Error ? e.message : String(e)}`);
    } finally {
      this.stateManagerTickRunning = false;
    }
  }

  // -------- OFFBOARD --------

  public setOffboardTarget(target: OffboardTarget | null): void {
    this.offboardTarget = target;

    if(target) {
      void this._tickOffboard();
    }
  }

  public clearOffboardTarget(): void {
    this.setOffboardTarget(null);
  }

  private startOffboardLoop(): void {
    if (this.offboardInterval !== null) return;

    this.offboardInterval = setInterval(() => {
      void this._tickOffboard();
    }, 250);
  }

  private async _tickOffboard(): Promise<void> {
    if (this.offboardTarget === null) return;
    if (this.offboardTickRunning) return;
    this.offboardTickRunning = true;

    try {
      await this._requireConnected();

      const currentState = this.model.getCurrentState();
      if (!DroneStateModel.isStateConnected(currentState)) {
        return;
      }

      const takingOff = DroneStateModel.isStateTakingOff(currentState);
      const landing = DroneStateModel.isStateLanding(currentState);
      const landed = DroneStateModel.isStateLanded(currentState);
      const isOffboard = DroneStateModel.isStateOffboard(currentState);

      // Must only broadcast OFFBOARD setpoints when we're not taking off, landing, or landed.
      if (takingOff || landing || landed) {
        // Optional auto takeoff (ignored when auto state management is enabled)
        if (!this.autoStateEnabled && this.opts.offboardAutoTakeoff) {
          if (!takingOff) {
            await this.takeoff();
            return;
          }
        }
        return;
      }

      if (!isOffboard) {
        await this.setMode("OFFBOARD");
      }

      // Broadcast setpoint at 250ms cadence while airborne and not busy with takeoff/landing/landed.
      this.publishOffboardTarget(this.offboardTarget);
    } finally {
      this.offboardTickRunning = false;
    }
  }

  private publishOffboardTarget(target: OffboardTarget): void {
    switch (target.kind) {
      case "position_local": {
        const yaw = (typeof target.yawRad === "number" && Number.isFinite(target.yawRad))
          ? target.yawRad
          : (typeof (this.latestState as any)?.yaw === "number" ? (this.latestState as any).yaw : 0);

        const msg = {
          header: this._header(this.opts.localFrameId),
          pose: {
            position: { x: target.x, y: target.y, z: target.z },
            orientation: this._yawToQuat(yaw),
          },
        };

        this._publish(DroneController.T_SETPOINT_POS, DroneController.TYPE_POSE_STAMPED, msg);
        return;
      }

      case "velocity_local": {
        const yawRate = (typeof target.yawRate === "number" && Number.isFinite(target.yawRate)) ? target.yawRate : 0;

        const msg = {
          header: this._header(this.opts.localFrameId),
          twist: {
            linear: { x: target.vx, y: target.vy, z: target.vz },
            angular: { x: 0, y: 0, z: yawRate },
          },
        };

        this._publish(DroneController.T_SETPOINT_VEL, DroneController.TYPE_TWIST_STAMPED, msg);
        return;
      }

      case "raw_local": {
        const pos = target.position ?? { x: 0, y: 0, z: 0 };
        const vel = target.velocity ?? { x: 0, y: 0, z: 0 };
        const acc = target.acceleration_or_force ?? { x: 0, y: 0, z: 0 };

        const msg = {
          header: this._header(this.opts.localFrameId),
          coordinate_frame: target.coordinate_frame,
          type_mask: target.type_mask,
          position: { x: pos.x, y: pos.y, z: pos.z },
          velocity: { x: vel.x, y: vel.y, z: vel.z },
          acceleration_or_force: { x: acc.x, y: acc.y, z: acc.z },
          yaw: (typeof target.yaw === "number" && Number.isFinite(target.yaw)) ? target.yaw : 0,
          yaw_rate: (typeof target.yaw_rate === "number" && Number.isFinite(target.yaw_rate)) ? target.yaw_rate : 0,
        };

        this._publish(DroneController.T_SETPOINT_RAW_LOCAL, DroneController.TYPE_POSITION_TARGET, msg);
        return;
      }

      case "raw_attitude": {
        const msg = {
          header: this._header(this.opts.localFrameId),
          type_mask: target.type_mask,
          orientation: target.orientation ?? { x: 0, y: 0, z: 0, w: 1 },
          body_rate: target.body_rate ?? { x: 0, y: 0, z: 0 },
          thrust: (typeof target.thrust === "number" && Number.isFinite(target.thrust)) ? target.thrust : 0,
        };

        this._publish(DroneController.T_SETPOINT_RAW_ATT, DroneController.TYPE_ATTITUDE_TARGET, msg);
        return;
      }
    }
  }

  private _publish(topic: string, type: string, msg: any): void {
    const b: any = this.ros2Bridge as any;

    if (typeof b.publish === "function") {
      try { b.publish({ topic, type }, msg); return; } catch {}
      try { b.publish(topic, type, msg); return; } catch {}
      try { b.publish({ op: "publish", topic, msg }); return; } catch {}
    }

    if (typeof b.send === "function") {
      try { b.send({ op: "publish", topic, msg }); return; } catch {}
    }
  }



  // -------- MAVROS service helpers --------

  async mavrosCommandLong(req: RosTypes.CommandLong_Request): Promise<RosTypes.CommandLong_Response> {
    return await this.ros2Bridge.callService<RosTypes.CommandLong_Response>("/mavros/cmd/command", req);
  }

  async mavrosArmDisarm(value: boolean): Promise<RosTypes.CommandBool_Response> {
    const req: RosTypes.CommandBool_Request = { value };
    return await this.ros2Bridge.callService<RosTypes.CommandBool_Response>("/mavros/cmd/arming", req);
  }

  async mavrosSetMode(custom_mode: string, base_mode = 0): Promise<RosTypes.SetMode_Response> {
    const req: RosTypes.SetMode_Request = { base_mode, custom_mode };
    return await this.ros2Bridge.callService<RosTypes.SetMode_Response>("/mavros/set_mode", req);
  }

  async mavrosLand(args: {
    altitude?: number;
    yaw?: number;
    latitude?: number;
    longitude?: number;
  } = {}): Promise<RosTypes.CommandTOL_Response> {
    const req: RosTypes.CommandTOL_Request = {
      altitude: args.altitude ?? 0.0,
      min_pitch: 0.0,
      yaw: args.yaw ?? 0.0,
      latitude: args.latitude ?? 0.0,
      longitude: args.longitude ?? 0.0,
    };
    return await this.ros2Bridge.callService<RosTypes.CommandTOL_Response>("/mavros/cmd/land", req);
  }

  // -------- Helpers --------

  private async _requireConnected() {
    const s = await this.model.getState();
    if (!s?.vehicle?.connected) {
      throw new Error("FCU not connected");
    }
  }

  private async _requireBattery(min: number, action: string) {
    const pct = (await this.model.getState()).battery?.percentage;
    if (typeof pct === "number" && pct < min) {
      throw new Error(`Battery ${(pct * 100).toFixed(0)}% < ${(min * 100).toFixed(0)}% required to ${action}`);
    }
  }

  private _header(frame_id: string) {
    return { stamp: this._now(), frame_id };
  }

  private _now() {
    const now = Date.now();
    const sec = Math.floor(now / 1000);
    const nanosec = (now - sec * 1000) * 1_000_000;
    return { sec, nanosec };
  }

  private _yawToQuat(yaw: number): RosTypes.GeometryQuaternion {
    const half = yaw / 2;
    return { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) };
  }
}
