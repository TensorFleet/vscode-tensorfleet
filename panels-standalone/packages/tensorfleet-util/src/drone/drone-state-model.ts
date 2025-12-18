// drone-state-model.ts
import { ROS2BridgeApi, UnsubscribeFn } from '../ros/ros-bridge-api';
import type {
  SensorMsgsNavSatFix,
  StdMsgsFloat64,
  MavrosMsgsState,
  MavrosMsgsExtendedState,
  SensorMsgsBatteryState,
  GeometryMsgsPoseStamped,
  GeometryMsgsTwistStamped,
  SensorMsgsImu,
  MavrosMsgsAltitude,
  MavrosMsgsHomePosition,
} from '../ros/ros-types';

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
    armable?: boolean;
    arm_reasons?: string[];
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
      | GeometryMsgsPoseStamped
      | GeometryMsgsTwistStamped
      | SensorMsgsImu
      | MavrosMsgsAltitude
      | MavrosMsgsHomePosition
    >
  | Record<string, unknown>;

type EventMap = { update: (state: Partial<DroneState>) => void; statusUpdate: (state: Partial<DroneState>) => void };

type SectionChangeListener<T = any> = (oldVal: T, newVal: T) => void;

/** MAV_LANDED_STATE mapping used for status logic. */
export const LANDED = {
  UNDEFINED: 0,
  ON_GROUND: 1,
  IN_AIR: 2,
  TAKEOFF: 3,
  LANDING: 4,
} as const;

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
function isPoseStamped(x: any): x is GeometryMsgsPoseStamped { return x && x.pose && x.pose.position && x.pose.orientation; }
function isTwistStamped(x: any): x is GeometryMsgsTwistStamped { return x && x.twist && x.twist.linear && x.twist.angular; }
function isImu(x: any): x is SensorMsgsImu { return x && x.orientation && x.angular_velocity && x.linear_acceleration; }
function isAltitude(x: any): x is MavrosMsgsAltitude { return x && ('amsl' in x || 'relative' in x || 'agl' in x || 'local' in x); }
function isHomePosition(x: any): x is MavrosMsgsHomePosition {
  return x && x.geo && typeof x.geo.latitude === 'number' && typeof x.geo.longitude === 'number';
}

type RosStamp = { sec: number; nanosec: number };
type HasHeaderStamp = { header?: { stamp?: RosStamp } };

/**
 * Maintains a unified drone state from MAVROS topics.
 * No heartbeat or parameter setting is performed here.
 */
export class DroneStateModel extends Emitter {
  public id: string;

  private state: Partial<DroneState> = {};
  private updateListeners = new Set<DroneStateUpdateListener>();
  private statusUpdateListeners = new Set<DroneStateUpdateListener>();
  private sectionChangeListeners = new Map<keyof DroneState, Set<SectionChangeListener>>();
  private prevSectionJsons = new Map<keyof DroneState, string>();
  private prevNonNumericalJson: string | null = null;

  private updateInterval: any = null;
  private updated = false;
  private updateFps: number;
  private bridge: ROS2BridgeApi | null = null;
  private unsubscribers: Map<string, UnsubscribeFn> = new Map();

  private lastSeen: Record<string, number> = {};

  private allTopics: Set<string>;
  private seenTopics: Set<string> = new Set();
  private allSeenPromise: Promise<void> | null = null;
  private resolveAllSeen: (() => void) | null = null;
  private debugInterval: any = null;

  private buggyTopics: string[] = [];
  private connectTime: number = 0;

  // Topics
  private static readonly T_FIX = '/mavros/global_position/raw/fix';
  private static readonly T_HDG = '/mavros/global_position/compass_hdg';
  private static readonly T_STATE = '/mavros/state';
  private static readonly T_EXT_STATE = '/mavros/extended_state';
  private static readonly T_BATT = '/mavros/battery';
  private static readonly T_POSE = '/mavros/local_position/pose';
  private static readonly T_VEL = '/mavros/local_position/velocity_local';
  private static readonly T_IMU = '/mavros/imu/data';
  private static readonly T_ALT = '/mavros/altitude';
  private static readonly T_HOME = '/mavros/home_position/home';

  private handlers: Record<string, (msg: any) => void>;

  private rosStampToMs(stamp: RosStamp): number {
    // Deterministic conversion: ROS time -> ms
    return stamp.sec * 1000 + Math.floor(stamp.nanosec / 1e6);
  }

  private msgTimeMs(msg: any): number | null {
    const stamp = (msg as HasHeaderStamp)?.header?.stamp;
    if (stamp && typeof stamp.sec === 'number' && typeof stamp.nanosec === 'number') {
      return this.rosStampToMs(stamp);
    }
    // No fallback
    return null;
  }

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
      [DroneStateModel.T_POSE]: this.handleLocalPose,
      [DroneStateModel.T_VEL]: this.handleLocalVelocity,
      [DroneStateModel.T_IMU]: this.handleImu,
      [DroneStateModel.T_ALT]: this.handleAltitude,
      [DroneStateModel.T_HOME]: this.handleHomePosition,
    };

    this.allTopics = new Set(Object.keys(this.handlers));
    this.buggyTopics = [DroneStateModel.T_BATT];
  }

  /** Subscribes to required MAVROS topics via the bridge. */
  public connect(bridge: ROS2BridgeApi): void {
    console.log('[DEBUG] DroneStateModel.connect() called');
    this.disconnect();
    this.connectTime = Date.now();
    this.bridge = bridge;

    this.seenTopics = new Set();
    this.allSeenPromise = new Promise(resolve => this.resolveAllSeen = resolve);

    const subs: Array<{ topic: string; type: string }> = [
      { topic: DroneStateModel.T_FIX, type: 'sensor_msgs/msg/NavSatFix' },
      { topic: DroneStateModel.T_HDG, type: 'std_msgs/msg/Float64' },
      { topic: DroneStateModel.T_STATE, type: 'mavros_msgs/msg/State' },
      { topic: DroneStateModel.T_EXT_STATE, type: 'mavros_msgs/msg/ExtendedState' },
      { topic: DroneStateModel.T_BATT, type: 'sensor_msgs/msg/BatteryState' },
      { topic: DroneStateModel.T_POSE, type: 'geometry_msgs/msg/PoseStamped' },
      { topic: DroneStateModel.T_VEL, type: 'geometry_msgs/msg/TwistStamped' },
      { topic: DroneStateModel.T_IMU, type: 'sensor_msgs/msg/Imu' },
      { topic: DroneStateModel.T_ALT, type: 'mavros_msgs/msg/Altitude' },
      { topic: DroneStateModel.T_HOME, type: 'mavros_msgs/msg/HomePosition' },
    ];

    console.log('[DEBUG] Subscribing to topics:', subs);
    subs.forEach(s => {
      const unsubscribe = this.bridge!.subscribe(s, (msg) => this.ingest({ topic: s.topic, msg }));
      this.unsubscribers.set(s.topic, unsubscribe);
    });
    console.log('[DEBUG] DroneStateModel.connect() completed');
    this.startUpdateLoop();
    this.debugInterval = setInterval(() => {
      if (this.seenTopics.size < this.allTopics.size) {
        const missing = Array.from(this.allTopics).filter(t => !this.seenTopics.has(t));
        console.log(`[DEBUG] Waiting for topics: ${missing.join(', ')}`);
      }
      const now = Date.now();
      if (now - this.connectTime > 10000) {
        this.buggyTopics.forEach(topic => {
          if (!this.seenTopics.has(topic)) {
            console.error(`simulation restart might be needed, topic ${topic} is not broadcasted`);
          }
        });
      }
    }, 6000);
  }

  /** Unsubscribes from all topics. */
  public disconnect(): void {
    if (this.updateInterval !== null) {
      clearInterval(this.updateInterval);
      this.updateInterval = null;
    }
    if (this.debugInterval !== null) {
      clearInterval(this.debugInterval);
      this.debugInterval = null;
    }
    // Call all stored unsubscribe functions
    this.unsubscribers.forEach(unsubscribe => unsubscribe());
    this.unsubscribers.clear();
    this.bridge = null;
    // Clear all data
    this.state = {};
    this.lastSeen = {};
    this.seenTopics.clear();
    this.allSeenPromise = null;
    this.resolveAllSeen = null;
    this.prevSectionJsons.clear();
    this.prevNonNumericalJson = null;
  }

  /** Registers a state update listener. */
  public onUpdate(listener: DroneStateUpdateListener) {
    this.updateListeners.add(listener);
    return () => this.updateListeners.delete(listener);
  }

  /** Registers a status update listener that only runs when non-numerical state parts change. */
  public onStatusUpdate(listener: DroneStateUpdateListener) {
    this.statusUpdateListeners.add(listener);
    return () => this.statusUpdateListeners.delete(listener);
  }

  /** Registers a listener for changes in a specific state section (excluding time_boot_ms). */
  public onSectionChange<K extends keyof DroneState>(section: K, listener: SectionChangeListener<Partial<DroneState>[K]>) {
    if (!this.sectionChangeListeners.has(section)) this.sectionChangeListeners.set(section, new Set());
    this.sectionChangeListeners.get(section)!.add(listener as SectionChangeListener);
    return () => this.sectionChangeListeners.get(section)!.delete(listener as SectionChangeListener);
  }

  /** Returns a shallow copy of the current state. */
  public async getState(): Promise<Partial<DroneState>> {
    if (this.allSeenPromise) {
      await this.allSeenPromise;
    }
    return { ...this.state };
  }

  /** Non async state  */
  public getCurrentState(): DroneState {
    return {...this.state};
  }

  private async waitForTopics(topics: string[]): Promise<void> {
    const unseen = topics.filter(t => !this.seenTopics.has(t));
    if (unseen.length === 0) return;
    await new Promise<void>((resolve) => {
      const check = () => {
        const stillUnseen = topics.filter(t => !this.seenTopics.has(t));
        if (stillUnseen.length === 0) {
          resolve();
        } else {
          setTimeout(check, 100);
        }
      };
      check();
    });
  }

  public isDroneConnected(): boolean {
    return DroneStateModel.isStateConnected(this.state)
  }


  public async isArmed(): Promise<boolean> {
    await this.waitForTopics([DroneStateModel.T_STATE]);
    return DroneStateModel.isStateArmed(this.state);
  }

  public async isTakingOff(): Promise<boolean> {
    await this.waitForTopics([DroneStateModel.T_STATE]);

    return DroneStateModel.isStateTakingOff(this.state);
  }

  public async isLanding(): Promise<boolean> {
    await this.waitForTopics([DroneStateModel.T_EXT_STATE]);

    return DroneStateModel.isStateLanding(this.state);
  }

  /**
   * Checks whether the drone is disarmed or is on ground.
   */
  public async isLanded(): Promise<boolean> {
    await this.waitForTopics([DroneStateModel.T_STATE, DroneStateModel.T_EXT_STATE]);

    return DroneStateModel.isStateLanded(this.state);
  }

  public async isOffboard(): Promise<boolean> {
    await this.waitForTopics([DroneStateModel.T_STATE]);
    return DroneStateModel.isStateOffboard(this.state);
  }

  public async isAirborne(): Promise<boolean> {
    await this.waitForTopics([DroneStateModel.T_STATE, DroneStateModel.T_EXT_STATE]);
    return DroneStateModel.isStateAirborne(this.state);
  }

  // Static utility functions for synchronous state checks

  /**
   * Checks if state is connected
   */
  public static isStateConnected(state: DroneState | Partial<DroneState>): boolean {
    return state.vehicle?.connected ?? false;
  }

  /**
   * Checks if the drone is armed in the given state.
   */
  public static isStateArmed(state: DroneState | Partial<DroneState>): boolean {
    return state.vehicle?.armed ?? false;
  }

  /**
   * Checks if the drone is taking off in the given state.
   */
  public static isStateTakingOff(state: DroneState | Partial<DroneState>): boolean {
    return (state.vehicle?.armed && state.vehicle?.mode === "AUTO.TAKEOFF") ?? false;
  }

  /**
   * Checks if the drone is landing in the given state.
   */
  public static isStateLanding(state: DroneState | Partial<DroneState>): boolean {
    return (state.vehicle?.armed && state.vehicle?.mode === "AUTO.LAND") ?? false;
  }

  /**
   * Checks if the drone is landed (disarmed or on ground or in AUTO.LAND mode) in the given state.
   */
  public static isStateLanded(state: DroneState | Partial<DroneState>): boolean {
    return (!state.vehicle?.armed || (state.extended?.landed_state === LANDED.ON_GROUND && state.vehicle?.mode != "AUTO.TAKEOFF")) ?? false;
  }

  public static isStateOffboard(state: DroneState | Partial<DroneState>): boolean {
    return state.vehicle?.mode === "OFFBOARD";
  }

  public static isStateAirborne(state: DroneState | Partial<DroneState>): boolean {
    
    const armed = state.vehicle?.armed ?? false;
    const landed = DroneStateModel.isStateLanded(state);
    const landing = DroneStateModel.isStateLanding(state);
    const takingOff = DroneStateModel.isStateTakingOff(state);

    return (armed && !(landed || landing || takingOff)) ?? false;
  }


  /** Subscribed callback: dispatches to per-topic handlers. */
  public ingest = (frame: RosFrame) => {
    const topic = (frame as any).topic as string;
    const innerMsg = (frame as any).msg;
    const wallNow = Date.now();

    const handler = this.handlers[topic];
    if (handler) {
      this.lastSeen[topic] = wallNow;
      // Handle both frame formats: {topic, type, msg} or direct message
      const rosMsg = innerMsg && typeof innerMsg === 'object' && 'msg' in innerMsg ? innerMsg.msg : innerMsg;
      handler.call(this, rosMsg);

      this.seenTopics.add(topic);
      if (this.seenTopics.size === this.allTopics.size) {
        this.resolveAllSeen?.();
        this.resolveAllSeen = null;
        this.allSeenPromise = Promise.resolve();
      }
    } else {
      console.log('[DEBUG] No handler found for topic:', topic);
    }
  };

  // -------- Topic handlers --------

  private handleGlobalFix(msg: any) {
    if (!isNavSatFix(msg)) return;
    const t = this.msgTimeMs(msg);
    this.ensureGlobal();
    const oldGlobal = { ...this.state.global_position_int! };
    Object.assign(this.state.global_position_int!, {
      lat: msg.latitude,
      lon: msg.longitude,
      alt: msg.altitude,
    });
    if (t !== null) this.state.global_position_int!.time_boot_ms = t;
    this.checkAndEmitChange('global_position_int', oldGlobal, this.state.global_position_int);
    this.updated = true;
  }

  private handleCompassHdg(msg: unknown) {
    if (!isFloat64(msg)) return;
    const t = this.msgTimeMs(msg);
    this.ensureGlobal();
    const oldGlobal = { ...this.state.global_position_int! };
    this.state.global_position_int!.hdg = msg.data;
    if (t !== null) this.state.global_position_int!.time_boot_ms = t;
    this.checkAndEmitChange('global_position_int', oldGlobal, this.state.global_position_int);
    this.updateRotationFromHeading();
    this.updated = true;
  }

  private handleVehicleState(msg: unknown) {
    if (!isState(msg)) {
      console.log('[DEBUG] handleVehicleState: ', msg ,' msg failed isState check');
      return;
    }
    const t = this.msgTimeMs(msg);
    const tEff = (t !== null) ? t : Date.now();
    this.ensureVehicle();
    const oldVehicle = { ...this.state.vehicle! };
    Object.assign(this.state.vehicle!, {
      connected: !!msg.connected,
      armed: !!msg.armed,
      guided: !!msg.guided,
      manual_input: !!msg.manual_input,
      mode: msg.mode,
      system_status: msg.system_status,
      time_boot_ms: tEff,
    });
    this.checkAndEmitChange('vehicle', oldVehicle, this.state.vehicle);
    this.updated = true;
  }

  private handleExtendedState(msg: unknown) {
    if (!isExtendedState(msg)) return;
    const t = this.msgTimeMs(msg);
    const tEff = (t !== null) ? t : Date.now();
    this.ensureExtended();
    const oldExtended = { ...this.state.extended! };
    Object.assign(this.state.extended!, {
      landed_state: msg.landed_state,
      vtol_state: msg.vtol_state,
      time_boot_ms: tEff,
    });
    this.checkAndEmitChange('extended', oldExtended, this.state.extended);
    this.updated = true;
  }

  private handleBattery(msg: unknown) {
    if (!isBattery(msg)) return;
    const t = this.msgTimeMs(msg);
    const tEff = (t !== null) ? t : Date.now();
    this.ensureBattery();
    const oldBattery = { ...this.state.battery! };
    Object.assign(this.state.battery!, {
      percentage: msg.percentage,
      voltage: msg.voltage,
      current: msg.current,
      temperature: msg.temperature ?? null,
      time_boot_ms: tEff,
    });
    this.checkAndEmitChange('battery', oldBattery, this.state.battery);
    this.updated = true;
  }

  private handleLocalPose(msg: unknown) {
    if (!isPoseStamped(msg)) return;
    const t = this.msgTimeMs(msg);
    const tEff = (t !== null) ? t : Date.now();
    this.ensureLocal();
    const oldLocal = { ...this.state.local! };
    const p = msg.pose.position;
    const q = msg.pose.orientation;
    Object.assign(this.state.local!, {
      position: { x: p.x, y: p.y, z: p.z },
      orientation: { x: q.x, y: q.y, z: q.z, w: q.w },
      time_boot_ms: tEff,
    });
    this.checkAndEmitChange('local', oldLocal, this.state.local);
    this.updated = true;
  }

  private handleLocalVelocity(msg: unknown) {
    if (!isTwistStamped(msg)) return;
    const t = this.msgTimeMs(msg);
    const tEff = (t !== null) ? t : Date.now();
    this.ensureLocal();
    const oldLocal = { ...this.state.local! };
    const lin = msg.twist.linear;
    const ang = msg.twist.angular;
    Object.assign(this.state.local!, {
      linear: { x: lin.x, y: lin.y, z: lin.z },
      angular: { x: ang.x, y: ang.y, z: ang.z },
      time_boot_ms: tEff,
    });
    this.checkAndEmitChange('local', oldLocal, this.state.local);
    this.updated = true;
  }

  private handleImu(msg: unknown) {
    if (!isImu(msg)) return;
    const t = this.msgTimeMs(msg);
    const tEff = (t !== null) ? t : Date.now();
    this.ensureImu();
    const oldImu = { ...this.state.imu! };
    Object.assign(this.state.imu!, {
      orientation: msg.orientation,
      angular_velocity: msg.angular_velocity,
      linear_acceleration: msg.linear_acceleration,
      time_boot_ms: tEff,
    });
    this.checkAndEmitChange('imu', oldImu, this.state.imu);
    this.updated = true;
  }

  private handleAltitude(msg: unknown) {
    if (!isAltitude(msg)) return;
    const t = this.msgTimeMs(msg);
    const tEff = (t !== null) ? t : Date.now();
    this.ensureAltitude();
    const oldAltitude = { ...this.state.altitude! };
    Object.assign(this.state.altitude!, {
      amsl: msg.amsl,
      agl: msg.agl,
      local: msg.local,
      relative: msg.relative,
      terrain: msg.terrain,
      bottom_clearance: msg.bottom_clearance,
      time_boot_ms: tEff,
    });
    this.checkAndEmitChange('altitude', oldAltitude, this.state.altitude);
    this.ensureGlobal();
    if (typeof msg.relative === 'number') {
      this.state.global_position_int!.relative_alt = msg.relative;
    }
    this.updated = true;
  }

  private handleHomePosition(msg: unknown) {
    if (!isHomePosition(msg)) return;
    const t = this.msgTimeMs(msg);
    const tEff = (t !== null) ? t : Date.now();
    this.ensureHome();
    const oldHome = { ...this.state.home! };
    Object.assign(this.state.home!, {
      lat: msg.geo.latitude,
      lon: msg.geo.longitude,
      alt: msg.geo.altitude,
      orientation: msg.orientation,
      time_boot_ms: tEff,
    });
    this.checkAndEmitChange('home', oldHome, this.state.home);
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

  // -------- Helpers --------

  private omitTime(obj: any): any {
    if (obj == null || typeof obj !== 'object') return obj;
    if (Array.isArray(obj)) return obj.map(item => this.omitTime(item));
    const result: any = {};
    for (const key in obj) {
      if (key !== 'time_boot_ms') {
        result[key] = this.omitTime(obj[key]);
      }
    }
    return result;
  }

  private checkAndEmitChange<K extends keyof DroneState>(section: K, oldVal: Partial<DroneState>[K], newVal: Partial<DroneState>[K]) {
    const jsonNew = JSON.stringify(this.omitTime(newVal));
    const prevJson = this.prevSectionJsons.get(section);
    if (prevJson && prevJson !== jsonNew) {
      this.prevSectionJsons.set(section, jsonNew);
      const listeners = this.sectionChangeListeners.get(section);
      if (listeners) {
        listeners.forEach(listener => {
          try {
            listener(oldVal, newVal);
          } catch (e) {
            // ignore
          }
        });
      }
    } else if (!prevJson) {
      this.prevSectionJsons.set(section, jsonNew);
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
    const intervalMs = 100 / this.updateFps;
    this.updateInterval = setInterval(() => {
      this.refreshStatus();
      if (this.updated) {
        this.updateListeners.forEach(l => l(this.state));
        this.emit('update', this.state);
        this.updated = false;
      }
      // Check if non-numerical parts of the state have changed
      const currentNonNumericalJson = JSON.stringify({
        vehicle: this.state.vehicle ? {
          connected: this.state.vehicle.connected,
          armed: this.state.vehicle.armed,
          guided: this.state.vehicle.guided,
          manual_input: this.state.vehicle.manual_input,
          mode: this.state.vehicle.mode,
          system_status: this.state.vehicle.system_status,
        } : undefined,
        extended: this.state.extended ? {
          landed_state: this.state.extended.landed_state,
          vtol_state: this.state.extended.vtol_state,
        } : undefined,
        battery: this.state.battery ? {
          temperature: this.state.battery.temperature,
        } : undefined,
        status: this.state.status ? {
          connected: this.state.status.connected,
          gcs_link: this.state.status.gcs_link,
          faults: this.state.status.faults,
          armable: this.state.status.armable,
          arm_reasons: this.state.status.arm_reasons,
        } : undefined,
      });
      if (this.prevNonNumericalJson !== currentNonNumericalJson) {
        this.statusUpdateListeners.forEach(l => l(this.state));
        this.emit('statusUpdate', this.state);
        this.prevNonNumericalJson = currentNonNumericalJson;
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

    // Determine a reliable landed-state indication for status (do not overwrite extended.landed_state).
    const extSeenMs = now - (this.lastSeen[DroneStateModel.T_EXT_STATE] || 0);
    const extFresh = extSeenMs <= 1000;
    let landedEff = extFresh ? this.state.extended?.landed_state : undefined;

    if (landedEff === undefined || landedEff === LANDED.UNDEFINED) {
      // Heuristic for startup: treat as ON_GROUND if unarmed and close to ground with low vertical speed.
      const armed = !!this.state.vehicle?.armed;
      const rel = this.state.global_position_int?.relative_alt ?? Number.POSITIVE_INFINITY;
      const vz = this.state.local?.linear?.z ?? 0;
      const almostOnGround = Number.isFinite(rel) && Math.abs(rel) < 0.25 && Math.abs(vz) < 0.3;
      if (!armed && almostOnGround) landedEff = LANDED.ON_GROUND;
    }

    const faults: string[] = [];
    if (!gcs) faults.push('vehicle.link.down');
    if (!seen(DroneStateModel.T_IMU)) faults.push('imu.stale');
    if (!seen(DroneStateModel.T_FIX)) faults.push('gps.stale');

    const pct = this.state.battery?.percentage;
    const v = this.state.battery?.voltage;
    if (typeof pct === 'number' && pct <= 0.15) faults.push('battery.low');
    if (typeof v === 'number' && v > 0 && v < 10.5) faults.push('battery.voltage.low');

    const armedNow = !!this.state.vehicle?.armed;
    if (!armedNow && landedEff === LANDED.IN_AIR) faults.push('state.inconsistent');

    let armable = true;
    const arm_reasons: string[] = [];

    if (!gcs) {
      armable = false;
      arm_reasons.push('link.down');
    }
    if (!seen(DroneStateModel.T_IMU)) {
      armable = false;
      arm_reasons.push('imu.stale');
    }
    if (!seen(DroneStateModel.T_FIX)) {
      armable = false;
      arm_reasons.push('gps.stale');
    }
    if (typeof pct === 'number' && pct <= 0.15) {
      armable = false;
      arm_reasons.push('battery.low');
    }
    if (typeof v === 'number' && v > 0 && v < 10.5) {
      armable = false;
      arm_reasons.push('battery.voltage.low');
    }
    if (landedEff === LANDED.IN_AIR || landedEff === LANDED.TAKEOFF || landedEff === LANDED.LANDING) {
      armable = false;
      arm_reasons.push('not.on.ground');
    }

    Object.assign(this.state.status!, {
      time_boot_ms: now,
      connected: gcs,
      gcs_link: gcs,
      faults,
      armable,
      arm_reasons,
    });
  }
}
