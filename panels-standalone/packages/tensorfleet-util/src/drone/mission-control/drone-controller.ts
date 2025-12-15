// drone-controller.ts
/**
 * High-level drone controller:
 *  - Uses DroneStateModel (subscriptions handled there) to read vehicle state
 *  - Uses ros2Bridge to publish MAVROS setpoints and call MAVROS services
 *  - No direct subscriptions here (per constraint). No rosbridge.
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

export interface DroneControllerOptions {
  localFrameId?: string;                // default "map"
  offboardWarmup?: { count: number; hz: number }; // default {20,20}
  minBatteryForFlight?: number;         // default 0.15
}

export class DroneController {
  private model: DroneStateModel;
  private ros2Bridge: ROS2BridgeApi;
  private opts: Required<DroneControllerOptions>;

  constructor(model: DroneStateModel, ros2Bridge: ROS2BridgeApi, opts: DroneControllerOptions = {}) {
    this.model = model;
    this.ros2Bridge = ros2Bridge;
    this.opts = {
      localFrameId: opts.localFrameId ?? "map",
      offboardWarmup: opts.offboardWarmup ?? { count: 20, hz: 20 },
      minBatteryForFlight: opts.minBatteryForFlight ?? 0.15,
    };
  }

  // -------- Basic services --------

  async arm(): Promise<void> {
    this._requireConnected();
    await this.mavrosArmDisarm(true);
  }

  async disarm(): Promise<void> {
    this._requireConnected();
    await this.mavrosArmDisarm(false);
  }

  async setMode(mode: string, base = 0): Promise<void> {
    this._requireConnected();
    await this.mavrosSetMode(mode, base);
  }

  async takeoff(altMeters: number, yawRad = 0): Promise<void> {
  const gp = this.model.getState().global_position_int;
  if (!gp) throw new Error("No GPS fix");

  // Your DroneStateModel populates degrees from NavSatFix, so use them directly.
  const lat_deg = gp.lat;
  const lon_deg = gp.lon;
  const yaw_deg = yawRad * 180 / Math.PI;

  await this.mavrosCommandLong({
      command: 22,        // MAV_CMD_NAV_TAKEOFF
      param1: 0,          // min_pitch (fixed-wing); 0 for multirotor
      param2: 0,
      param3: 0,
      param4: yaw_deg,    // yaw (deg)
      param5: lat_deg,    // latitude (deg)
      param6: lon_deg,    // longitude (deg)
      param7: altMeters,  // relative altitude (m)
      confirmation: 0,
      broadcast: false,
    });
  }

  private async _waitUntilInAir(): Promise<void> {
    const timeoutMs = 15000;
    const start = Date.now();

    while (Date.now() - start < timeoutMs) {
      const st = this.model.getState();
      const landed = st.extended?.landed_state;

      if (landed === LandedState.IN_AIR) {
        return;
      }

      // If PX4 reports ON_GROUND again after some time,
      // takeoff clearly failed:
      if (landed === LandedState.ON_GROUND && Date.now() - start > 5000) {
        throw new Error("Takeoff failed: still ON_GROUND after AUTO.TAKEOFF");
      }

      await this._sleep(100);
    }

    throw new Error("Timeout waiting for IN_AIR state after takeoff");
  }


  async land(): Promise<void> {
    this._requireConnected();
    await this.mavrosLand();
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

  async mavrosTakeoff(args: {
    altitude: number;
    min_pitch?: number;
    yaw?: number;
    latitude?: number;
    longitude?: number;
  }): Promise<RosTypes.CommandTOL_Response> {
    const req: RosTypes.CommandTOL_Request = {
      altitude: args.altitude,
      min_pitch: args.min_pitch ?? 0.0,
      yaw: args.yaw ?? 0.0,
      latitude: args.latitude ?? 0.0,
      longitude: args.longitude ?? 0.0,
    };
    return await this.ros2Bridge.callService<RosTypes.CommandTOL_Response>("/mavros/cmd/takeoff", req);
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

  async mavrosParamSet(param_id: string, value: RosTypes.ParamValue): Promise<RosTypes.ParamSet_Response> {
    const req: RosTypes.ParamSet_Request = { param_id, value };
    return await this.ros2Bridge.callService<RosTypes.ParamSet_Response>("/mavros/param/set", req);
  }

  // -------- Helpers --------

  private async _waitForArmed(timeoutMs = 3000): Promise<void> {
    const t0 = Date.now();
    while (Date.now() - t0 < timeoutMs) {
      if (this.model.getState().vehicle?.armed) return;
      await this._sleep(10);
    }
    throw new Error("Timed out waiting for ARMED");
  }

  private _requireConnected() {
    const s = this.model.getState();
    if (!s?.vehicle?.connected) {
      throw new Error("FCU not connected");
    }
  }

  private _requireBattery(min: number, action: string) {
    const pct = this.model.getState().battery?.percentage;
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

  private _sleep(ms: number) { return new Promise<void>((r) => setTimeout(r, ms)); }
}
