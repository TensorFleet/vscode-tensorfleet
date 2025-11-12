// drone-controller.ts
/**
 * High-level drone controller:
 *  - Uses DroneStateModel (subscriptions handled there) to read vehicle state
 *  - Uses ros2Bridge to publish MAVROS setpoints and call MAVROS services
 *  - No direct subscriptions here (per constraint). No rosbridge.
 */

import { ros2Bridge } from "@/ros2-bridge";
import type { GeometryQuaternion } from "@/ros2-bridge";
import type { DroneStateModel } from "./drone-state-model";

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
  private opts: Required<DroneControllerOptions>;

  private static readonly T_POSE = "/mavros/setpoint_position/local";
  private static readonly T_TWIST = "/mavros/setpoint_velocity/cmd_vel";
  private static readonly MSG_POSE = "geometry_msgs/msg/PoseStamped";
  private static readonly MSG_TWIST = "geometry_msgs/msg/TwistStamped";

  constructor(model: DroneStateModel, opts: DroneControllerOptions = {}) {
    this.model = model;
    this.opts = {
      localFrameId: opts.localFrameId ?? "map",
      offboardWarmup: opts.offboardWarmup ?? { count: 20, hz: 20 },
      minBatteryForFlight: opts.minBatteryForFlight ?? 0.15,
    };
  }

  // -------- Basic services --------

  async arm(): Promise<void> {
    this._requireConnected();
    await ros2Bridge.mavrosArmDisarm(true);
  }

  async disarm(): Promise<void> {
    this._requireConnected();
    await ros2Bridge.mavrosArmDisarm(false);
  }

  async setMode(mode: string, base = 0): Promise<void> {
    this._requireConnected();
    await ros2Bridge.mavrosSetMode(mode, base);
  }

  async takeoff(altMeters: number, yawRad = 0): Promise<void> {
    this._requireConnected();
    this._requireBattery(this.opts.minBatteryForFlight, "takeoff");
    await this._warmupOffboard({ x: 0, y: 0, z: Math.max(altMeters, 0.5), yaw: yawRad });
    await ros2Bridge.mavrosTakeoff({ altitude: altMeters, yaw: yawRad });
  }

  async land(): Promise<void> {
    this._requireConnected();
    await ros2Bridge.mavrosLand();
  }

  // -------- Setpoints --------

  async setPositionLocal(x: number, y: number, z: number, yawRad: number) {
    const header = this._header(this.opts.localFrameId);
    const q = this._yawToQuat(yawRad);
    ros2Bridge.publish(DroneController.T_POSE, DroneController.MSG_POSE, {
      header,
      pose: { position: { x, y, z }, orientation: q },
    });
  }

  async setVelocityENU(vx: number, vy: number, vz: number, yawRate = 0) {
    const header = this._header(this.opts.localFrameId);
    ros2Bridge.publish(DroneController.T_TWIST, DroneController.MSG_TWIST, {
      header,
      twist: { linear: { x: vx, y: vy, z: vz }, angular: { x: 0, y: 0, z: yawRate } },
    });
  }

  async stop() {
    await this.setVelocityENU(0, 0, 0, 0);
  }

  // -------- Helpers --------

  private async _warmupOffboard(pose: { x: number; y: number; z: number; yaw: number }) {
    const armed = !!this.model.getState().vehicle?.armed;
    if (!armed) await this.arm();

    await this.setMode("OFFBOARD");

    const { count, hz } = this.opts.offboardWarmup;
    const header = this._header(this.opts.localFrameId);
    const q = this._yawToQuat(pose.yaw);

    for (let i = 0; i < count; i++) {
      ros2Bridge.publish(DroneController.T_POSE, DroneController.MSG_POSE, {
        header: { ...header, stamp: this._now() },
        pose: { position: { x: pose.x, y: pose.y, z: pose.z }, orientation: q },
      });
      await this._sleep(1000 / hz);
    }
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

  private _yawToQuat(yaw: number): GeometryQuaternion {
    const half = yaw / 2;
    return { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) };
  }

  private _sleep(ms: number) { return new Promise<void>((r) => setTimeout(r, ms)); }
}
