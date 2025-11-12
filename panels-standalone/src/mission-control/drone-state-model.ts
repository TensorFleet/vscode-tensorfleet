import { ros2Bridge } from '../ros2-bridge';
import type {
  ROS2Bridge,
  SensorMsgsNavSatFix,
  StdMsgsFloat64,
  MavrosMsgsState,
  MavrosMsgsExtendedState,
  SensorMsgsBatteryState,
  MavrosMsgsVFRHUD,
  GeometryMsgsPoseStamped,
  GeometryMsgsTwistStamped,
  SensorMsgsImu,
  MavrosMsgsAltitude,
  MavrosMsgsHomePosition,
} from '../ros2-bridge';

/**
 * Unified drone state assembled from MAVROS topics.
 * Includes vehicle state, telemetry, and basic health assessment.
 */
export type DroneState = {
  local_position_ned?: {
    time_boot_ms: number;
    x: number; y: number; z: number;
    vx: number; vy: number; vz: number;
  };

  attitude?: {
    time_boot_ms: number;
    roll: number; pitch: number; yaw: number;
    rollspeed: number; pitchspeed: number; yawspeed: number;
  };

  rotation?: { x: number; y: number; z: number; w: number };

  /** Yaw (rad), NED (0 = North, +CW toward East). */
  yaw?: number;

  global_position_int?: {
    time_boot_ms: number;
    lat: number; lon: number; alt: number;
    relative_alt: number;
    vx: number; vy: number; vz: number;
    hdg: number; // deg
  };

  /** Connection/mode/arming state. */
  vehicle?: {
    time_boot_ms: number;
    connected: boolean;
    armed: boolean;
    guided: boolean;
    manual_input: boolean;
    mode: string;
    system_status?: number;
  };

  /** Landed/VTOL state. */
  extended?: {
    time_boot_ms: number;
    landed_state?: number;
    vtol_state?: number;
  };

  /** Battery telemetry. */
  battery?: {
    time_boot_ms: number;
    percentage?: number;
    voltage?: number;
    current?: number;
    temperature?: number | null;
  };

  /** Airspeed/groundspeed/throttle/climb. */
  vfr_hud?: {
    time_boot_ms: number;
    airspeed?: number;
    groundspeed?: number;
    heading?: number;
    throttle?: number;
    climb?: number;
  };

  /** Local pose/velocity in ENU. */
  local?: {
    time_boot_ms: number;
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
    linear: { x: number; y: number; z: number };
    angular: { x: number; y: number; z: number };
  };

  /** IMU data. */
  imu?: {
    time_boot_ms: number;
    orientation: { x: number; y: number; z: number; w: number };
    angular_velocity: { x: number; y: number; z: number };
    linear_acceleration: { x: number; y: number; z: number };
  };

  /** Altitude breakdown. */
  altitude?: {
    time_boot_ms: number;
    amsl?: number;
    agl?: number;
    local?: number;
    relative?: number;
    terrain?: number;
    bottom_clearance?: number;
  };

  /** Home position. */
  home?: {
    time_boot_ms: number;
    lat: number; lon: number; alt: number;
    orientation?: { x: number; y: number; z: number; w: number };
  };

  /**
   * Model health and link status.
   * `connected` and `gcs_link` reflect the FCU/MAVROS link (from /mavros/state).
   * `faults` provides a minimal set of health flags based on data recency and basic thresholds.
   */
  status?: {
    time_boot_ms: number;
    connected: boolean;
    gcs_link: boolean;
    faults: string[];
  };
};

export type DroneStateUpdateListener = (state: Partial<DroneState>) => void;

type RosPublish<T> = { op: 'publish'; topic: string; msg: T };
type RosFrame =
  | RosPublish<
      | SensorMsgsNavSatFix
      | StdMsgsFloat64
      | MavrosMsgsState
      | MavrosMsgsExtendedState
      | SensorMsgsBatteryState
      | MavrosMsgsVFRHUD
      | GeometryMsgsPoseStamped
      | GeometryMsgsTwistStamped
      | SensorMsgsImu
      | MavrosMsgsAltitude
      | MavrosMsgsHomePosition
    >
  | Record<string, unknown>;

type EventMap = { update: (state: Partial<DroneState>) => void };

/** Minimal event emitter for model updates. */
class Emitter {
  private listeners = new Map<keyof EventMap, Set<Function>>();
  on<K extends keyof EventMap>(event: K, cb: EventMap[K]) {
    if (!this.listeners.has(event)) this.listeners.set(event, new Set());
    this.listeners.get(event)!.add(cb as any);
    return () => this.off(event, cb);
  }
  off<K extends keyof EventMap>(event: K, cb: EventMap[K]) {
    this.listeners.get(event)?.delete(cb as any);
  }
  emit<K extends keyof EventMap>(event: K, ...args: Parameters<EventMap[K]>) {
    this.listeners.get(event)?.forEach(fn => { try { (fn as any)(...args); } catch {} });
  }
}

/** Type guards. */
function isNavSatFix(x: any): x is SensorMsgsNavSatFix {
  return x && typeof x.latitude === 'number' && typeof x.longitude === 'number' && typeof x.altitude === 'number';
}
function isFloat64(x: any): x is StdMsgsFloat64 { return x && typeof x.data === 'number'; }
function isState(x: any): x is MavrosMsgsState { return x && typeof x.mode === 'string' && typeof x.armed === 'boolean'; }
function isExtendedState(x: any): x is MavrosMsgsExtendedState { return x && ('landed_state' in x || 'vtol_state' in x); }
function isBattery(x: any): x is SensorMsgsBatteryState { return x && ('voltage' in x || 'percentage' in x || 'current' in x); }
function isVfrHud(x: any): x is MavrosMsgsVFRHUD { return x && ('groundspeed' in x || 'airspeed' in x || 'throttle' in x); }
function isPoseStamped(x: any): x is GeometryMsgsPoseStamped { return x && x.pose && x.pose.position && x.pose.orientation; }
function isTwistStamped(x: any): x is GeometryMsgsTwistStamped { return x && x.twist && x.twist.linear && x.twist.angular; }
function isImu(x: any): x is SensorMsgsImu { return x && x.orientation && x.angular_velocity && x.linear_acceleration; }
function isAltitude(x: any): x is MavrosMsgsAltitude { return x && ('amsl' in x || 'relative' in x || 'agl' in x || 'local' in x); }
function isHomePosition(x: any): x is MavrosMsgsHomePosition {
  return x && x.geo && typeof x.geo.latitude === 'number' && typeof x.geo.longitude === 'number';
}

/**
 * Maintains a unified drone state from MAVROS topics.
 * No heartbeat or param setting is performed here.
 */
export class DroneStateModel extends Emitter {
  public id: string;

  private state: Partial<DroneState> = {};
  private updateListeners = new Set<DroneStateUpdateListener>();

  private updateInterval: number | null = null;
  private updated = false;
  private updateFps: number;
  private bridge: ROS2Bridge | null = null;

  private lastSeen: Record<string, number> = {};

  // Topics
  private static readonly T_FIX = '/mavros/global_position/raw/fix';
  private static readonly T_HDG = '/mavros/global_position/compass_hdg';
  private static readonly T_STATE = '/mavros/state';
  private static readonly T_EXT_STATE = '/mavros/extended_state';
  private static readonly T_BATT = '/mavros/battery';
  private static readonly T_VFR = '/mavros/vfr_hud';
  private static readonly T_POSE = '/mavros/local_position/pose';
  private static readonly T_VEL = '/mavros/local_position/velocity_local';
  private static readonly T_IMU = '/mavros/imu/data';
  private static readonly T_ALT = '/mavros/altitude';
  private static readonly T_HOME = '/mavros/home_position/home';

  private handlers: Record<string, (msg: unknown, now: number) => void>;

  constructor(updateFps = 10) {
    super();
    this.id = Math.random().toString(36).slice(2);
    this.updateFps = updateFps;

    this.handlers = {
      [DroneStateModel.T_FIX]: this.handleGlobalFix,
      [DroneStateModel.T_HDG]: this.handleCompassHdg,
      [DroneStateModel.T_STATE]: this.handleVehicleState,
      [DroneStateModel.T_EXT_STATE]: this.handleExtendedState,
      [DroneStateModel.T_BATT]: this.handleBattery,
      [DroneStateModel.T_VFR]: this.handleVfrHud,
      [DroneStateModel.T_POSE]: this.handleLocalPose,
      [DroneStateModel.T_VEL]: this.handleLocalVelocity,
      [DroneStateModel.T_IMU]: this.handleImu,
      [DroneStateModel.T_ALT]: this.handleAltitude,
      [DroneStateModel.T_HOME]: this.handleHomePosition,
    };

    this.startUpdateLoop();
  }

  /** Subscribes to required MAVROS topics via the bridge. */
  public connect(bridge: ROS2Bridge = ros2Bridge): void {
    this.disconnect();
    this.bridge = bridge;

    const subs: Array<{ topic: string; type: string }> = [
      { topic: DroneStateModel.T_FIX, type: 'sensor_msgs/msg/NavSatFix' },
      { topic: DroneStateModel.T_HDG, type: 'std_msgs/msg/Float64' },
      { topic: DroneStateModel.T_STATE, type: 'mavros_msgs/msg/State' },
      { topic: DroneStateModel.T_EXT_STATE, type: 'mavros_msgs/msg/ExtendedState' },
      { topic: DroneStateModel.T_BATT, type: 'sensor_msgs/msg/BatteryState' },
      { topic: DroneStateModel.T_VFR, type: 'mavros_msgs/msg/VFR_HUD' },
      { topic: DroneStateModel.T_POSE, type: 'geometry_msgs/msg/PoseStamped' },
      { topic: DroneStateModel.T_VEL, type: 'geometry_msgs/msg/TwistStamped' },
      { topic: DroneStateModel.T_IMU, type: 'sensor_msgs/msg/Imu' },
      { topic: DroneStateModel.T_ALT, type: 'mavros_msgs/msg/Altitude' },
      { topic: DroneStateModel.T_HOME, type: 'mavros_msgs/msg/HomePosition' },
    ];

    subs.forEach(s => this.bridge!.subscribe(s, this.ingest));
  }

  /** Unsubscribes from all topics. */
  public disconnect(): void {
    if (this.bridge?.unsubscribe) {
      Object.keys(this.handlers).forEach(topic => this.bridge!.unsubscribe(topic, this.ingest));
    }
    this.bridge = null;
  }

  /** Registers a state update listener. */
  public onUpdate(listener: DroneStateUpdateListener) {
    this.updateListeners.add(listener);
    return () => this.updateListeners.delete(listener);
  }

  /** Returns a shallow copy of the current state. */
  public getState(): Partial<DroneState> {
    return { ...this.state };
  }

  /** Subscribed callback: dispatches to per-topic handlers. */
  public ingest = (frame: RosFrame) => {
    const topic = (frame as any).topic as string;
    const msg = (frame as any).msg;
    const now = Date.now();

    const handler = this.handlers[topic];
    if (handler) {
      this.lastSeen[topic] = now;
      handler.call(this, msg, now);
    }
  };

  // -------- Topic handlers --------

  private handleGlobalFix(msg: unknown, now: number) {
    if (!isNavSatFix(msg)) return;
    this.ensureGlobal();
    Object.assign(this.state.global_position_int!, {
      time_boot_ms: now,
      lat: msg.latitude,
      lon: msg.longitude,
      alt: msg.altitude,
    });
    this.updated = true;
  }

  private handleCompassHdg(msg: unknown, now: number) {
    if (!isFloat64(msg)) return;
    this.ensureGlobal();
    this.state.global_position_int!.hdg = msg.data;
    this.state.global_position_int!.time_boot_ms = now;
    this.updateRotationFromHeading();
    this.updated = true;
  }

  private handleVehicleState(msg: unknown, now: number) {
    if (!isState(msg)) return;
    this.ensureVehicle();
    Object.assign(this.state.vehicle!, {
      time_boot_ms: now,
      connected: !!msg.connected,
      armed: !!msg.armed,
      guided: !!msg.guided,
      manual_input: !!msg.manual_input,
      mode: msg.mode,
      system_status: msg.system_status,
    });
    this.updated = true;
  }

  private handleExtendedState(msg: unknown, now: number) {
    if (!isExtendedState(msg)) return;
    this.ensureExtended();
    Object.assign(this.state.extended!, {
      time_boot_ms: now,
      landed_state: msg.landed_state,
      vtol_state: msg.vtol_state,
    });
    this.updated = true;
  }

  private handleBattery(msg: unknown, now: number) {
    if (!isBattery(msg)) return;
    this.ensureBattery();
    Object.assign(this.state.battery!, {
      time_boot_ms: now,
      percentage: msg.percentage,
      voltage: msg.voltage,
      current: msg.current,
      temperature: msg.temperature ?? null,
    });
    this.updated = true;
  }

  private handleVfrHud(msg: unknown, now: number) {
    if (!isVfrHud(msg)) return;
    this.ensureVfrHud();
    Object.assign(this.state.vfr_hud!, {
      time_boot_ms: now,
      airspeed: msg.airspeed,
      groundspeed: msg.groundspeed,
      heading: msg.heading,
      throttle: msg.throttle,
      climb: msg.climb,
    });
    this.updated = true;
  }

  private handleLocalPose(msg: unknown, now: number) {
    if (!isPoseStamped(msg)) return;
    this.ensureLocal();
    const p = msg.pose.position;
    const q = msg.pose.orientation;
    Object.assign(this.state.local!, {
      time_boot_ms: now,
      position: { x: p.x, y: p.y, z: p.z },
      orientation: { x: q.x, y: q.y, z: q.z, w: q.w },
    });
    this.updated = true;
  }

  private handleLocalVelocity(msg: unknown, now: number) {
    if (!isTwistStamped(msg)) return;
    this.ensureLocal();
    const lin = msg.twist.linear;
    const ang = msg.twist.angular;
    Object.assign(this.state.local!, {
      time_boot_ms: now,
      linear: { x: lin.x, y: lin.y, z: lin.z },
      angular: { x: ang.x, y: ang.y, z: ang.z },
    });
    this.updated = true;
  }

  private handleImu(msg: unknown, now: number) {
    if (!isImu(msg)) return;
    this.ensureImu();
    Object.assign(this.state.imu!, {
      time_boot_ms: now,
      orientation: msg.orientation,
      angular_velocity: msg.angular_velocity,
      linear_acceleration: msg.linear_acceleration,
    });
    this.updated = true;
  }

  private handleAltitude(msg: unknown, now: number) {
    if (!isAltitude(msg)) return;
    this.ensureAltitude();
    Object.assign(this.state.altitude!, {
      time_boot_ms: now,
      amsl: msg.amsl,
      agl: msg.agl,
      local: msg.local,
      relative: msg.relative,
      terrain: msg.terrain,
      bottom_clearance: msg.bottom_clearance,
    });
    this.ensureGlobal();
    if (typeof msg.relative === 'number') {
      this.state.global_position_int!.relative_alt = msg.relative;
    }
    this.updated = true;
  }

  private handleHomePosition(msg: unknown, now: number) {
    if (!isHomePosition(msg)) return;
    this.ensureHome();
    Object.assign(this.state.home!, {
      time_boot_ms: now,
      lat: msg.geo.latitude,
      lon: msg.geo.longitude,
      alt: msg.geo.altitude,
      orientation: msg.orientation,
    });
    this.updated = true;
  }

  // -------- Section initializers --------

  private ensureGlobal() {
    if (!this.state.global_position_int) {
      this.state.global_position_int = {
        time_boot_ms: Date.now(),
        lat: 0, lon: 0, alt: 0,
        relative_alt: 0, vx: 0, vy: 0, vz: 0, hdg: 0,
      };
    }
  }
  private ensureVehicle() {
    if (!this.state.vehicle) {
      this.state.vehicle = {
        time_boot_ms: Date.now(),
        connected: false,
        armed: false,
        guided: false,
        manual_input: false,
        mode: '',
      };
    }
  }
  private ensureExtended() { if (!this.state.extended) this.state.extended = { time_boot_ms: Date.now() }; }
  private ensureBattery() { if (!this.state.battery) this.state.battery = { time_boot_ms: Date.now(), temperature: null }; }
  private ensureVfrHud() { if (!this.state.vfr_hud) this.state.vfr_hud = { time_boot_ms: Date.now() }; }
  private ensureLocal() {
    if (!this.state.local) {
      this.state.local = {
        time_boot_ms: Date.now(),
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      };
    }
  }
  private ensureImu() {
    if (!this.state.imu) {
      this.state.imu = {
        time_boot_ms: Date.now(),
        orientation: { x: 0, y: 0, z: 0, w: 1 },
        angular_velocity: { x: 0, y: 0, z: 0 },
        linear_acceleration: { x: 0, y: 0, z: 0 },
      };
    }
  }
  private ensureAltitude() { if (!this.state.altitude) this.state.altitude = { time_boot_ms: Date.now() }; }
  private ensureHome() { if (!this.state.home) this.state.home = { time_boot_ms: Date.now(), lat: 0, lon: 0, alt: 0 }; }
  private ensureStatus() {
    if (!this.state.status) {
      this.state.status = {
        time_boot_ms: Date.now(),
        connected: false,
        gcs_link: false,
        faults: [],
      };
    }
  }

  // -------- Computed helpers --------

  private updateRotationFromHeading() {
    const raw = this.state.global_position_int?.hdg ?? 0.0;
    const hdgDeg = this.hdgDegrees(raw);
    const hdgRad = (hdgDeg * Math.PI) / 180.0;
    const half = hdgRad / 2;
    this.state.rotation = { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) };
    (this.state as any).yaw = hdgRad;
    if (this.state.attitude) this.state.attitude.yaw = hdgRad;
  }

  private hdgDegrees(raw: number): number {
    return Math.abs(raw) > 360 ? raw / 100 : raw;
  }

  // -------- Update loop / health --------

  /** Periodically recomputes health and emits updates when state changed. */
  private startUpdateLoop() {
    if (this.updateInterval !== null) return;
    const intervalMs = 1000 / this.updateFps;
    this.updateInterval = window.setInterval(() => {
      this.refreshStatus();
      if (this.updated) {
        this.updateListeners.forEach(l => l(this.state));
        this.emit('update', this.state);
        this.updated = false;
      }
    }, intervalMs);
  }

  /** Updates link flags and minimal fault set based on data recency and thresholds. */
  private refreshStatus() {
    const now = Date.now();
    this.ensureStatus();

    const gcs = !!this.state.vehicle?.connected;
    const maxStaleMs = 1500;
    const seen = (t: string) => (now - (this.lastSeen[t] || 0)) <= maxStaleMs;

    const faults: string[] = [];
    if (!gcs) faults.push('vehicle.link.down');
    if (!seen(DroneStateModel.T_IMU)) faults.push('imu.stale');
    if (!seen(DroneStateModel.T_FIX)) faults.push('gps.stale');

    const pct = this.state.battery?.percentage;
    const v = this.state.battery?.voltage;
    if (typeof pct === 'number' && pct <= 0.15) faults.push('battery.low');
    if (typeof v === 'number' && v > 0 && v < 10.5) faults.push('battery.voltage.low');

    const armed = !!this.state.vehicle?.armed;
    const landed = this.state.extended?.landed_state;
    if (!armed && landed === 1 /* IN_AIR */) faults.push('state.inconsistent');

    Object.assign(this.state.status!, {
      time_boot_ms: now,
      connected: gcs,
      gcs_link: gcs,
      faults,
    });
  }
}
