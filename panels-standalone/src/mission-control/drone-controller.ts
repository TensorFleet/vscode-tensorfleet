// drone-controller.ts
import { ros2Bridge } from '../ros2-bridge';
import { DroneStateModel, DroneState } from './drone-state-model';

/**
 * Optional service adapter to support MAVROS command services.
 * Implement these calls in your transport (e.g., a small ROS2 WebSocket
 * service proxy) and pass the adapter to the controller.
 */
export interface ServiceCaller {
  /** /mavros/cmd/arming (mavros_msgs/srv/CommandBool) */
  commandArm(value: boolean): Promise<void>;
  /** /mavros/set_mode (mavros_msgs/srv/SetMode) */
  setMode(customMode: string, baseMode?: number): Promise<void>;
  /** /mavros/cmd/takeoff (mavros_msgs/srv/CommandTOL) */
  commandTakeoff(altitude: number, latitude?: number, longitude?: number, yaw?: number): Promise<void>;
  /** /mavros/cmd/land (mavros_msgs/srv/CommandTOL) */
  commandLand(altitude?: number, latitude?: number, longitude?: number, yaw?: number): Promise<void>;
  /** Optional: /mavros/cmd/command (mavros_msgs/srv/CommandLong) */
  commandLong?(command: number, params: number[]): Promise<void>;
}

/** MAVROS ExtendedState landed_state values (uint8). */
export enum LandedState {
  UNDEFINED = 0,
  ON_GROUND = 1,
  IN_AIR = 2,
  TAKEOFF = 3,
  LANDING = 4,
}

/** Controller configuration options. */
export interface DroneControllerOptions {
  /** Minimum battery percentage [0..1] required to arm or takeoff. Default: 0.15 (15%). */
  minBatteryForFlight?: number;
  /** Frame id for local setpoints. Default: "map". */
  localFrameId?: string;
  /** Optional service adapter to enable arming/mode/takeoff/land/RTL. */
  services?: ServiceCaller;
}

/**
 * Provides basic, state-checked vehicle controls.
 * Uses Pose/Twist setpoints via topics and (optionally) services for arm/mode.
 */
export class DroneControllerBasic {
  protected readonly state: DroneStateModel;
  protected readonly opts: Required<DroneControllerOptions>;

  // MAVROS topics and types used for setpoints.
  private static readonly T_POSE = '/mavros/setpoint_position/local';
  private static readonly T_TWIST = '/mavros/setpoint_velocity/cmd_vel';
  private static readonly MSG_POSE = 'geometry_msgs/msg/PoseStamped';
  private static readonly MSG_TWIST = 'geometry_msgs/msg/TwistStamped';

  constructor(stateModel: DroneStateModel, opts: DroneControllerOptions = {}) {
    this.state = stateModel;
    this.opts = {
      minBatteryForFlight: opts.minBatteryForFlight ?? 0.15,
      localFrameId: opts.localFrameId ?? 'map',
      services: opts.services ?? (undefined as any),
    };
  }

  // ----------- Public API: Safety-checked primitives -----------

  /**
   * Arms the vehicle using the provided service adapter.
   * Throws if not connected, already armed, or battery is below threshold.
   */
  async arm(): Promise<void> {
    this.requireConnected();
    this.requireNotArmed();
    this.requireBattery(this.opts.minBatteryForFlight, 'arm');

    const svc = this.requireServices('arm');
    await svc.commandArm(true);
  }

  /**
   * Disarms the vehicle using the provided service adapter.
   * Throws if not connected or already disarmed.
   */
  async disarm(): Promise<void> {
    this.requireConnected();
    this.requireArmed();
    const svc = this.requireServices('disarm');
    await svc.commandArm(false);
  }

  /**
   * Sets the autopilot mode using the provided service adapter.
   * Example modes: "OFFBOARD", "POSCTL", "AUTO.LOITER", "AUTO.RTL".
   */
  async setMode(customMode: string, baseMode = 0): Promise<void> {
    this.requireConnected();
    const svc = this.requireServices('setMode');
    await svc.setMode(customMode, baseMode);
  }

  /**
   * Performs a takeoff. If services are provided, calls CommandTOL.
   * Otherwise throws. For offboard takeoff, use setPositionLocal() or setVelocity() to climb.
   */
  async takeoff(targetAltMeters: number, yawRad?: number): Promise<void> {
    this.requireConnected();
    this.requireArmed();
    this.requireBattery(this.opts.minBatteryForFlight, 'takeoff');

    const svc = this.requireServices('takeoff');
    // Use global home if available; otherwise rely on autopilot internal logic.
    const s = this.snapshot();
    await svc.commandTakeoff(targetAltMeters, s.home?.lat, s.home?.lon, yawRad);
  }

  /**
   * Commands land using CommandTOL if services are provided.
   * Otherwise sets a downward velocity for a gentle descent.
   */
  async land(targetAltMeters = 0): Promise<void> {
    this.requireConnected();
    const svc = this.optionalServices();

    if (svc) {
      const s = this.snapshot();
      await svc.commandLand(targetAltMeters, s.global_position_int?.lat, s.global_position_int?.lon, s.yaw);
      return;
    }

    // Fallback: gentle descent via velocity setpoint.
    await this.setVelocityNED(0, 0, 0.5); // 0.5 m/s down (NED z positive = down)
  }

  /**
   * Requests Return-to-Launch by mode change if services are available.
   * Throws if not supported by the current bridge.
   */
  async rtl(): Promise<void> {
    this.requireConnected();
    const svc = this.requireServices('RTL');
    await svc.setMode('AUTO.RTL');
  }

  /**
   * Holds position via mode change if services are available; otherwise no-op.
   */
  async hold(): Promise<void> {
    this.requireConnected();
    const svc = this.requireServices('hold');
    await svc.setMode('AUTO.LOITER');
  }

  /**
   * Publishes a local position setpoint in ENU frame with optional yaw.
   * Requires the vehicle to be in OFFBOARD/POSCTL-like mode depending on stack configuration.
   */
  async setPositionLocal(x: number, y: number, z: number, yawRad?: number): Promise<void> {
    this.requireConnected();
    const header = this.makeHeader(this.opts.localFrameId);
    const q = this.yawToQuat(yawRad ?? this.snapshot().yaw ?? 0);

    const msg = {
      header,
      pose: {
        position: { x, y, z },
        orientation: q,
      },
    };

    ros2Bridge.publish(DroneControllerBasic.T_POSE, DroneControllerBasic.MSG_POSE, msg);
  }

  /**
   * Publishes a velocity setpoint (NED) converted to ENU for MAVROS.
   * Optionally includes a yaw rate [rad/s] about +Z (up).
   */
  async setVelocityNED(vn: number, ve: number, vd: number, yawRateRad?: number): Promise<void> {
    this.requireConnected();

    // Convert world velocity NED → ENU
    const { x: vx, y: vy, z: vz } = this.nedVecToEnu({ x: vn, y: ve, z: vd });

    const header = this.makeHeader(this.opts.localFrameId);
    const msg = {
      header,
      twist: {
        linear: { x: vx, y: vy, z: vz },
        angular: { x: 0, y: 0, z: yawRateRad ?? 0 },
      },
    };

    ros2Bridge.publish(DroneControllerBasic.T_TWIST, DroneControllerBasic.MSG_TWIST, msg);
  }

  /** Zeroes velocity in local frame. */
  async stop(): Promise<void> {
    await this.setVelocityNED(0, 0, 0, 0);
  }

  // ----------- Protected utilities & guards -----------

  /** Returns a shallow copy of the latest state. */
  protected snapshot(): Partial<DroneState> {
    return this.state.getState();
  }

  /** Throws if the vehicle is not connected. */
  protected requireConnected(): void {
    const s = this.snapshot();
    if (!s.vehicle?.connected) {
      throw new Error('Operation not allowed: autopilot not connected.');
    }
  }

  /** Throws if the vehicle is already armed. */
  protected requireNotArmed(): void {
    const s = this.snapshot();
    if (s.vehicle?.armed) {
      throw new Error('Operation not allowed: vehicle is already armed.');
    }
  }

  /** Throws if the vehicle is not armed. */
  protected requireArmed(): void {
    const s = this.snapshot();
    if (!s.vehicle?.armed) {
      throw new Error('Operation not allowed: vehicle is not armed.');
    }
  }

  /** Verifies battery threshold for a given action. */
  protected requireBattery(min: number, action: string): void {
    const pct = this.snapshot().battery?.percentage;
    if (typeof pct === 'number' && pct < min) {
      throw new Error(
        `Operation not allowed: battery at ${(pct * 100).toFixed(0)}% < ${(min * 100).toFixed(0)}% required to ${action}.`
      );
    }
  }

  /** Returns the service adapter or throws with a precise message. */
  protected requireServices(action: string): ServiceCaller {
    const svc = this.opts.services;
    if (!svc) {
      throw new Error(
        `Operation not available: "${action}" requires MAVROS service support (e.g., /mavros/set_mode, /mavros/cmd/arming). ` +
        `Provide a ServiceCaller to enable this action.`
      );
    }
    return svc;
  }

  /** Returns the service adapter if present, otherwise undefined. */
  protected optionalServices(): ServiceCaller | undefined {
    return this.opts.services ?? undefined;
  }

  /** Builds a ROS std_msgs/Header with current wall-clock time. */
  protected makeHeader(frame_id: string) {
    const now = Date.now();
    const sec = Math.floor(now / 1000);
    const nanosec = (now - sec * 1000) * 1_000_000;
    return { stamp: { sec, nanosec }, frame_id };
  }

  /** Converts yaw [rad] to a Z-yaw quaternion in ENU. */
  protected yawToQuat(yawRad: number) {
    const half = yawRad / 2;
    return { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) };
  }

  /**
   * Converts a vector expressed in the NED world frame to ENU.
   * Mapping: (E, N, U) = (y_NED, x_NED, -z_NED).
   */
  protected nedVecToEnu(v: { x: number; y: number; z: number }) {
    return { x: v.y, y: v.x, z: -v.z };
  }
}

/**
 * Advanced controller with helpers suitable for joystick integration and
 * higher-level behaviors. Extends state-checked primitives from the basic controller.
 */
export class DroneController extends DroneControllerBasic {
  // MAVROS RC override (optional, stack-specific).
  private static readonly T_RC_OVERRIDE = '/mavros/rc/override';
  private static readonly MSG_RC_OVERRIDE = 'mavros_msgs/msg/OverrideRCIn';

  /**
   * Applies a velocity "nudge" from joystick axes.
   * Axes are normalized in the range [-1..1] and scaled to m/s and rad/s.
   */
  async applyJoystickAxes(
    axes: { forward: number; right: number; up: number; yaw: number },
    scales: { mps: number; yawRate: number } = { mps: 2.0, yawRate: 1.0 }
  ): Promise<void> {
    this.requireConnected();
    // Joystick axes assumed: forward(+N), right(+E), up(+U). Convert to NED.
    const vn = axes.forward * scales.mps;
    const ve = axes.right * scales.mps;
    const vd = -(axes.up * scales.mps); // up(+U) → NED down(+D)
    const yawRate = axes.yaw * scales.yawRate;

    await this.setVelocityNED(vn, ve, vd, yawRate);
  }

  /**
   * Enables OFFBOARD by sending initial setpoints and switching mode via services (if available).
   * The warmup is required by PX4 to accept OFFBOARD mode.
   */
  async enableOffboard(
    warmupCount = 20,
    warmupHz = 20,
    initialLocalPose?: { x: number; y: number; z: number; yawRad?: number }
  ): Promise<void> {
    this.requireConnected();

    // Send a short burst of setpoints.
    const pose = initialLocalPose ?? { x: 0, y: 0, z: 2, yawRad: 0 };
    for (let i = 0; i < warmupCount; i++) {
      await this.setPositionLocal(pose.x, pose.y, pose.z, pose.yawRad);
      await this.sleep(1000 / warmupHz);
    }

    // Switch to OFFBOARD if services are available.
    const svc = this.optionalServices();
    if (svc) {
      await svc.setMode('OFFBOARD');
    }
  }

  /**
   * Sends RC channel overrides (if supported by the autopilot and MAVROS build).
   * Channels should be 8 or 16 entries with PWM values (e.g., 1000..2000).
   */
  async setRCOverride(channels: number[]): Promise<void> {
    this.requireConnected();
    if (!channels.length) throw new Error('RC override rejected: no channels provided.');

    const header = this.makeHeader('');
    // MAVROS message shape: { channels: uint16[18] } typically; keep flexible here.
    const msg = {
      header,
      channels,
    };

    ros2Bridge.publish(DroneController.T_RC_OVERRIDE, DroneController.MSG_RC_OVERRIDE, msg);
  }

  /** Convenience: simple position step (meters) relative to current local position if available. */
  async stepPositionLocal(dx: number, dy: number, dz: number, yawRad?: number): Promise<void> {
    this.requireConnected();
    const s = this.snapshot();
    const base = s.local?.position ?? { x: 0, y: 0, z: 0 };
    await this.setPositionLocal(base.x + dx, base.y + dy, base.z + dz, yawRad ?? s.yaw);
  }

  /** Convenience: simple velocity brake (stop) and optional hold mode request. */
  async brake(requestHoldMode = false): Promise<void> {
    await this.stop();
    if (requestHoldMode) {
      const svc = this.optionalServices();
      if (svc) await svc.setMode('AUTO.LOITER');
    }
  }

  /** Utility sleep for warmup sequencing. */
  private sleep(ms: number) {
    return new Promise<void>(resolve => setTimeout(resolve, ms));
  }
}
