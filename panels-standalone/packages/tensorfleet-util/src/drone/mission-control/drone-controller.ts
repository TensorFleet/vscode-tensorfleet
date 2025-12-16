// drone-controller.ts
/**
 * High-level drone controller:
 *  - Uses DroneStateModel (subscriptions handled there) to read vehicle state
 *  - Uses ros2Bridge to publish MAVROS setpoints and call MAVROS services
 *  - No ACK system
 *  - Requested/target state with optional automatic enforcement (tick every second)
 */

import * as RosTypes from "../../ros/ros-types"
import type { DroneStateModel } from "../drone-state-model";
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
  | { kind: "hold_altitude"; altMeters: number; yawRad?: number };

export interface DroneControllerOptions {
  localFrameId?: string;                // default "map"
  offboardWarmup?: { count: number; hz: number }; // default {20,20}
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

  constructor(model: DroneStateModel, ros2Bridge: ROS2BridgeApi, opts: DroneControllerOptions = {}) {
    this.model = model;
    this.ros2Bridge = ros2Bridge;
    this.opts = {
      localFrameId: opts.localFrameId ?? "map",
      offboardWarmup: opts.offboardWarmup ?? { count: 20, hz: 20 },
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
  }

  // -------- Basic services --------

  async arm(): Promise<void> {
    await this._requireConnected();
    console.log("[DRONE_CONTROLLER] Sending arm command...");
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

  async takeoff(altMeters: number, yawRad = 0): Promise<void> {
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

  public async isInRequestedState(debug: boolean = false): Promise<boolean> {
    const st = await this.model.getState();
    if (!st?.vehicle?.connected) return false;

    if(debug) {
      console.log("Requested :", this.requestedState, "\nCurrent state :", { extended: st.extended, vehicle: st.vehicle});
    }

    switch (this.requestedState.kind) {
      case "none":
        return true;
      case "landed": {
        const landed = st.extended?.landed_state;
        const onGround = landed === LandedState.ON_GROUND;
        if (this.requestedState.armed === null) {
          return onGround;
        } else {
          return onGround && (st.vehicle?.armed === this.requestedState.armed);
        }
      }
      case "airborne": {
        const landed = st.extended?.landed_state;
        const inAir = landed === LandedState.IN_AIR || landed === LandedState.TAKEOFF;
        const relAlt = st.global_position_int?.relative_alt;
        const close = relAlt !== undefined && Math.abs(relAlt - this.requestedState.altMeters) < 0.5;
        return inAir && close;
      }
      case "hold_altitude": {
        const landed = st.extended?.landed_state;
        const inAir = landed === LandedState.IN_AIR || landed === LandedState.TAKEOFF;
        const relAlt = st.global_position_int?.relative_alt;
        const close = relAlt !== undefined && Math.abs(relAlt - this.requestedState.altMeters) < 0.5;
        return inAir && close;
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
      const st = await this.model.getState();
      if (!st?.vehicle?.connected) {
        console.log("[AUTO_STATE] Drone not connected, skipping tick");
        return;
      }

      console.log(`[AUTO_STATE] Current state: armed=${st.vehicle?.armed}, mode=${st.vehicle?.mode}, landed=${st.extended?.landed_state}`);

      switch (this.requestedState.kind) {
        case "none":
          console.log("[AUTO_STATE] Requested state is 'none', no action needed");
          return;

        case "landed": {
          const landed = st.extended?.landed_state;
          const onGround = landed === LandedState.ON_GROUND;
          const landing = landed === LandedState.LANDING;

          console.log(`[AUTO_STATE] Landed state check: landed=${landed}, onGround=${onGround}, landing=${landing}`);

          if (!onGround && !landing) {
            console.log("[AUTO_STATE] Not on ground and not landing, checking if armed for landing");
            // PX4 generally needs to be armed to execute land; if disarmed mid-air, fail-safe is on the FCU.
            if (st.vehicle?.armed) {
              console.log("[AUTO_STATE] Sending land command");
              await this.mavrosLand();
            } else {
              console.log("[AUTO_STATE] Drone disarmed, cannot send land command");
            }
            return;
          }

          // on ground: enforce arming preference
          if (this.requestedState.armed === true && !st.vehicle?.armed) {
            console.log("[AUTO_STATE] Arming drone as requested");
            await this.mavrosArmDisarm(true);
            return;
          }
          if (this.requestedState.armed === false && st.vehicle?.armed) {
            console.log("[AUTO_STATE] Disarming drone as requested");
            await this.mavrosArmDisarm(false);
            return;
          }
          console.log("[AUTO_STATE] Landed state achieved, no action needed");
          return;
        }

        case "airborne": {
          console.log("[AUTO_STATE] Processing airborne state");

          try {
            await this._requireBattery(this.opts.minBatteryForFlight, "takeoff");
            console.log("[AUTO_STATE] Battery check passed");
          } catch (e) {
            console.log(`[AUTO_STATE] Battery check failed: ${e instanceof Error ? e.message : String(e)}`);
            return;
          }

          if (!st.vehicle?.armed) {
            console.log("[AUTO_STATE] Drone not armed, sending arm command");
            await this.mavrosArmDisarm(true);
            return;
          }

          const landed = st.extended?.landed_state;
          console.log(`[AUTO_STATE] Checking landed state for takeoff: landed=${landed}`);

          if (landed === LandedState.IN_AIR || landed === LandedState.TAKEOFF) {
            console.log("[AUTO_STATE] Already in air or taking off, no action needed");
            return;
          }

          if (landed === LandedState.ON_GROUND || landed === LandedState.UNDEFINED || landed === LandedState.LANDING) {
            console.log(`[AUTO_STATE] Initiating takeoff to ${this.requestedState.altMeters}m`);
            await this.takeoff(this.requestedState.altMeters, this.requestedState.yawRad ?? 0);
          } else {
            console.log(`[AUTO_STATE] Unknown landed state ${landed}, skipping takeoff`);
          }
          return;
        }

        case "hold_altitude": {
          console.log("[AUTO_STATE] Processing hold_altitude state");

          try {
            await this._requireBattery(this.opts.minBatteryForFlight, "hold altitude");
            console.log("[AUTO_STATE] Battery check passed");
          } catch (e) {
            console.log(`[AUTO_STATE] Battery check failed: ${e instanceof Error ? e.message : String(e)}`);
            return;
          }

          if (!st.vehicle?.armed) {
            console.log("[AUTO_STATE] Drone not armed, sending arm command");
            await this.mavrosArmDisarm(true);
            return;
          }

          const landed = st.extended?.landed_state;
          console.log(`[AUTO_STATE] Checking landed state for hold_altitude: landed=${landed}`);

          if (landed === LandedState.ON_GROUND || landed === LandedState.UNDEFINED) {
            console.log(`[AUTO_STATE] On ground, initiating takeoff to ${this.requestedState.altMeters}m`);
            await this.takeoff(this.requestedState.altMeters, this.requestedState.yawRad ?? 0);
            return;
          }

          if (st.vehicle?.mode !== "OFFBOARD") {
            console.log("[AUTO_STATE] Switching to OFFBOARD mode");
            await this.mavrosSetMode("OFFBOARD", 0);
          }

          const pos = this.latestState?.local?.position;
          if (!pos) return;

          const z = this.requestedState.altMeters;
          const yawRad = this.requestedState.yawRad ?? null;
          const orientation = yawRad !== null ? this._yawToQuat(yawRad) : (this.latestState?.local?.orientation ?? this._yawToQuat(0));

          const msg: any = {
            header: this._header(this.opts.localFrameId),
            pose: {
              position: { x: pos.x, y: pos.y, z },
              orientation,
            },
          };

          this.ros2Bridge.publish(
            DroneController.T_SETPOINT_POS,
            DroneController.TYPE_POSE_STAMPED,
            msg
          );

          console.log("[AUTO_STATE] Published setpoint for hold_altitude");

          return;
        }
      }
    } catch (e) {
      console.log(`[AUTO_STATE] Error in tick: ${e instanceof Error ? e.message : String(e)}`);
    } finally {
      this.stateManagerTickRunning = false;
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
