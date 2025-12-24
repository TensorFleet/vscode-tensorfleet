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
import deepEqual from "fast-deep-equal";

export enum LandedState {
  UNDEFINED = 0,
  ON_GROUND = 1,
  IN_AIR = 2,
  TAKEOFF = 3,
  LANDING = 4,
}

export type TargetAutoState =
  | null
  | { kind: "landed"; armed: boolean | null }
  | { kind: "airborne"; altMeters: number; yawRad?: number }
  | { kind: "offboard"; target: OffboardTarget}

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
  minBatteryForFlight?: number;         // default 0.15
  autoStateManagement?: boolean;        // default false
  stateManagementIntervalMs?: number;   // default 1000
}

export class DroneController {
  private model: DroneStateModel;
  private ros2Bridge: ROS2BridgeApi;
  private opts: Required<DroneControllerOptions>;

  private offboard_state_distance: number = 0.5;
  private offboard_angle_distance: number = 5 * Math.PI / 180;
  private max_offboard_velocity_diff: number = 0.1;

  private _targetAutoState: TargetAutoState = null;

  public get targetAutoState(): TargetAutoState {
    return this._targetAutoState;
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

  private offboardInterval: any = null;
  private offboardTickRunning = false;
  private lastOffboardModeAttemptMs = 0;
  private lastOffboardTakeoffAttemptMs = 0;

  constructor(model: DroneStateModel, ros2Bridge: ROS2BridgeApi, opts: DroneControllerOptions = {}) {
    this.model = model;
    this.ros2Bridge = ros2Bridge;
    this.opts = {
      localFrameId: opts.localFrameId ?? "map",
      minBatteryForFlight: opts.minBatteryForFlight ?? 0.15,
      autoStateManagement: opts.autoStateManagement ?? false,
      stateManagementIntervalMs: opts.stateManagementIntervalMs ?? 1000,
    };

    this.model.onUpdate((s) => { this.latestState = s; });
  }

  async initialize(): Promise<void> {
    this.startOffboardLoop();
    this.startAutoStateLoop();
  }

  // -------- Basic services --------

  async arm(): Promise<void> {
    await this._requireConnected();

    if (await this.model.isArmed()) {
      console.log("[DRONE_CONTROLLER] Drone already armed. Skipping arm command");
      return;
    }

    console.log("[DRONE_CONTROLLER] Sending arm command...");

    // Workaround. arm might fail due to unsupported state for arm.
    if (await this.model.isLanded()) {
      console.log("[DRONE_CONTROLLER] Is in landed state while trying to arm. Switching vehicle mode to AUTO.LOITER");
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

  async setMode(mode: string, base = 0, debug = true): Promise<void> {
    await this._requireConnected();
    if(debug) {
      console.log(`[DRONE_CONTROLLER] Setting mode to ${mode} (base=${base})...`);
    }
    const result = await this.mavrosSetMode(mode, base);
    if(debug) {
      console.log("[DRONE_CONTROLLER] Set mode result:", result);
    }
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

  async rtl(): Promise<void> {
    await this._requireConnected();
    console.log("[DRONE_CONTROLLER] Sending return-to-launch (RTL) command...");
    const result = await this.mavrosSetMode("AUTO.RTL", 0);
    console.log("[DRONE_CONTROLLER] RTL command result:", result);
  }

  // -------- Requested state / auto state management --------

  public async requestAutoState(state: TargetAutoState): Promise<void> {
    this._targetAutoState = structuredClone(state);

    if(state) {
      await this._tickAutoState();
      while(deepEqual(this._targetAutoState,state) && !this.isInRequestedAutoState()) {
        await this.sleep(100);
      }

      console.log("[DRONE_CONTROLLER] Target auto state reached:\n", state);
    } else {
      console.log("[DRONE_CONTROLLER] Target auto state cleared");
    }
  }

  public clearAutoState(): void {
    this.requestAutoState(null);
  }

  public isInRequestedAutoState(debug: boolean = false): boolean {
    let currentState = this.model.getCurrentState();

      const landed = DroneStateModel.isStateLanded(currentState);
      const landing = DroneStateModel.isStateLanding(currentState);
      const takingOff = DroneStateModel.isStateTakingOff(currentState);
      const offboard = DroneStateModel.isStateOffboard(currentState);
      const onGround = currentState.extended?.landed_state === LANDED.ON_GROUND;

    if(debug) {
      console.log("[AUTO_STATE] Requested :", this.targetAutoState, "\nCurrent state :", { extended: currentState.extended, vehicle: currentState.vehicle});
    }

    switch (this.targetAutoState?.kind) {
      case undefined:
        return true;
      case "landed": {
        return landed && (this.targetAutoState.armed === null || this.targetAutoState.armed === currentState.vehicle?.armed);
      }
      case "airborne": {
        return (currentState.vehicle?.armed && !( landed || landing || takingOff || onGround)) ?? false;
      }
      case "offboard": {
        // TODO : add offboard target checks
        if (!(currentState.vehicle?.armed && offboard)) {
          return false;
        }

        const offboardTarget = this.targetAutoState.target;
        switch(offboardTarget.kind) {
          case "position_local": {
            const currPos = currentState.local?.position;
            if (!currPos) {
              console.warn("[AUTO_STATE] No current position available for position_local check");
              return false;
            }

            const dx = currPos.x - offboardTarget.x;
            const dy = currPos.y - offboardTarget.y;
            const dz = currPos.z - offboardTarget.z;
            const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

            if (dist > this.offboard_state_distance) {
              return false;
            }

            if (offboardTarget.yawRad !== undefined) {
              const currOrient = currentState.local?.orientation;
              if (!currOrient) {
                console.warn("[AUTO_STATE] No current orientation available for yaw check");
                return false;
              }

              const currYaw = this._quatToYaw(currOrient);
              let yawDiff = currYaw - offboardTarget.yawRad;

              // Normalize yaw difference to [-pi, pi]
              yawDiff = ((yawDiff + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;

              if (Math.abs(yawDiff) > this.offboard_angle_distance) {
                return false;
              }
            }

            return true;
          }
          case "velocity_local": {
            const currVel = currentState.local?.linear;
            if (!currVel) {
              console.warn("[AUTO_STATE] No current velocity available for velocity_local check");
              return false;
            }

            const dvx = currVel.x - offboardTarget.vx;
            const dvy = currVel.y - offboardTarget.vy;
            const dvz = currVel.z - offboardTarget.vz;
            const velDiff = Math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz);

            return velDiff <= this.max_offboard_velocity_diff;
          }
          default:
            console.warn("[INFO] requested state check of type 'offboardTarget.kind' not supported yet");
            return true;
        }
      }
    }
  }

  private startAutoStateLoop(): void {
    this.stateManagerInterval = setInterval(() => {
      void this._tickAutoState();
    }, this.opts.stateManagementIntervalMs);
  }

  private async _tickAutoState(): Promise<void> {
    if (!this.targetAutoState) return;
    if (this.targetAutoState.kind === "offboard") {
      // Offboard has it's own ticker.
      return;
    }
    if (this.stateManagerTickRunning) return;
    this.stateManagerTickRunning = true;

    console.log(`[AUTO_STATE] Tick: targetAutoState=${JSON.stringify(this.targetAutoState)}`);

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

      switch (this.targetAutoState.kind) {
        case "landed": {
          // We need the drone landed.
          if (landing) {
            return;
          }

          if(landed) {
            // FIXME: TargetAutoState type allows armed: boolean | null for "landed" state, where null likely means "don't change arm state".
            // However, when armed is null, the comparison this.targetAutoState.armed != currentState.vehicle?.armed is always true (since null != true and null != false),
            // and the subsequent if(this.targetAutoState.armed) check treats null as falsy, causing an unintended disarm command regardless of current state.
            // Do we want to disarm?
            if(this.targetAutoState.armed != currentState.vehicle?.armed) {
              // We need to change the arm state.
              if(this.targetAutoState.armed) {
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
          let requestedAltitude = this.targetAutoState.altMeters;

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

          if(takingOff) {
            console.log("[AUTO_STATE] Processing airborne state [takingoff = true]. Doing nothing");
            return;
          }

          if(currentState.vehicle?.mode != "AUTO.LOITER") {
            // TODO : add more checks.
            console.log("[AUTO_STATE] airborne requested. vehicle mode not in AUTO.LOTIER mode. Setting it to AUTO.LOTIER");
            await this.setMode("AUTO.LOITER");
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
  private startOffboardLoop(): void {
    if (this.offboardInterval !== null) return;

    this.offboardInterval = setInterval(() => {
      void this._tickOffboard();
    }, 50);
  }

  private async _tickOffboard(): Promise<void> {
    const offboardTarget = this.targetAutoState?.kind === "offboard" ? this.targetAutoState.target : undefined;

    if (!offboardTarget) return;
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
      const armed = DroneStateModel.isStateArmed(currentState);

      if (!armed) {
        await this.arm();
      }


      if (!isOffboard) {
        await this.setMode("OFFBOARD", 0, false);
      }

      this.publishOffboardTarget(offboardTarget);
    } finally {
      this.offboardTickRunning = false;
    }
  }

  public publishOffboardTarget(target: OffboardTarget): void {
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

    if (!b || typeof b.publish !== "function") {
      throw new Error("ros2Bridge.publish is missing");
    }

    // Call the wrapper exactly as implemented:
    b.publish(topic, type, msg);
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

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  private _yawToQuat(yaw: number): RosTypes.GeometryQuaternion {
    const half = yaw / 2;
    return { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) };
  }

  private _quatToYaw(quat: RosTypes.GeometryQuaternion): number {
    const { x, y, z, w } = quat;
    return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  }
}
