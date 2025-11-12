import log from 'loglevel';
import { ros2Bridge } from '../../ros2-bridge';
import type {
  ROS2Bridge,
  Subscription,
  SensorMsgsNavSatFix,
  StdMsgsFloat64,
} from '../../ros2-bridge';

export type DroneState = {
  local_position_ned: {
    time_boot_ms: number;
    x: number; y: number; z: number;
    vx: number; vy: number; vz: number;
  };
  attitude: {
    time_boot_ms: number;
    roll: number; pitch: number; yaw: number;
    rollspeed: number; pitchspeed: number; yawspeed: number;
  };
  rotation: { x: number; y: number; z: number; w: number };
  /** Yaw in radians, NED convention, 0 = North, +CW toward East */
  yaw: number;
  global_position_int: {
    time_boot_ms: number;
    lat: number; lon: number; alt: number;
    relative_alt: number;
    vx: number; vy: number; vz: number;
    hdg: number;
  };
};

export type DroneStateUpdateListener = (state: Partial<DroneState>) => void;

type RosPublish<T> = { op: 'publish'; topic: string; msg: T };
type RosFrame = RosPublish<SensorMsgsNavSatFix | StdMsgsFloat64> | Record<string, unknown>;

type EventMap = { update: (state: Partial<DroneState>) => void };

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

function isNavSatFix(x: any): x is SensorMsgsNavSatFix {
  return x && typeof x.latitude === 'number' && typeof x.longitude === 'number' && typeof x.altitude === 'number';
}
function isFloat64(x: any): x is StdMsgsFloat64 {
  return x && typeof x.data === 'number';
}

export class DroneStateModel extends Emitter {
  public id: string;
  private state: Partial<DroneState> = {};
  private updateListeners = new Set<DroneStateUpdateListener>();
  private updateInterval: number | null = null;
  private updated = false;
  private updateFps: number;
  private bridge: ROS2Bridge | null = null;

  constructor(updateFps = 10) {
    const id = Math.random().toString(36).slice(2)
    console.log(`Constructing new DroneStateModel ${id}`);
    super();
    this.id = id;
    this.updateFps = updateFps;
    this.startUpdateLoop();
  }

  public connect(bridge: ROS2Bridge = ros2Bridge): void {
    this.disconnect();
    this.bridge = bridge;
    this.bridge.subscribe(
      {
        topic: '/mavros/global_position/raw/fix',
        type: 'sensor_msgs/msg/NavSatFix'
      }, this.ingest);
    this.bridge.subscribe(
      {
        topic: '/mavros/global_position/compass_hdg',
        type: 'std_msgs/msg/Float64'
      }, this.ingest);
  }

  public disconnect(): void {
    if (this.bridge?.unsubscribe) {
      this.bridge.unsubscribe('/mavros/global_position/raw/fix', this.ingest);
      this.bridge.unsubscribe('/mavros/global_position/compass_hdg', this.ingest);
    }
    this.bridge = null;
  }

  public onUpdate(listener: DroneStateUpdateListener) {
    console.log(`Attaching onUpdate hook to drone state ${this.id}`);
    this.updateListeners.add(listener);
    return () => { 
      console.log(`Dettaching onUpdate hook from drone state ${this.id}`);
      this.updateListeners.delete(listener)
    };
  }

  public getState(): Partial<DroneState> {
    return { ...this.state };
  }

  public ingest = (frame: RosFrame) => {
    if (!frame || (frame as any).op !== 'publish') return;
    const topic = (frame as any).topic as string;
    const msg = (frame as any).msg;
    const now = Date.now();

    switch (topic) {
      case '/mavros/global_position/raw/fix':
        if (isNavSatFix(msg)) {
          this.ensureGlobal();
          this.state.global_position_int!.time_boot_ms = now;
          this.state.global_position_int!.lat = msg.latitude;
          this.state.global_position_int!.lon = msg.longitude;
          this.state.global_position_int!.alt = msg.altitude;
          this.updated = true;
        }
        break;
      case '/mavros/global_position/compass_hdg':
        if (isFloat64(msg)) {
          this.ensureGlobal();
          this.state.global_position_int!.time_boot_ms = now;
          this.state.global_position_int!.hdg = msg.data;
          this.updateRotationFromHeading();
          this.updated = true;
        }
        break;
    }
  };

  private ensureGlobal() {
    if (!this.state.global_position_int) {
      this.state.global_position_int = {
        time_boot_ms: Date.now(),
        lat: 0, lon: 0, alt: 0,
        relative_alt: 0, vx: 0, vy: 0, vz: 0, hdg: 0,
      };
    }
  }

  private updateRotationFromHeading() {
    const raw = this.state.global_position_int?.hdg ?? 0.0;

    const hdgDeg = this.hdgDegrees(raw);
    const hdgRad = (hdgDeg * Math.PI) / 180.0; // radians, 0 = North, +CW toward East (NED)

    const half = hdgRad / 2;
    const qx = 0;
    const qy = 0;
    const qz = Math.sin(half);
    const qw = Math.cos(half);

    if (!this.state.rotation) this.state.rotation = { x: 0, y: 0, z: 0, w: 1 };
    this.state.rotation.x = qx;
    this.state.rotation.y = qy;
    this.state.rotation.z = qz;
    this.state.rotation.w = qw;

    (this.state as any).yaw = hdgRad;
  }

  private hdgDegrees(raw: number): number {
    return Math.abs(raw) > 360 ? raw / 100 : raw;
  }

  private startUpdateLoop() {
    if (this.updateInterval !== null) return;
    const intervalMs = 1000 / this.updateFps;
    this.updateInterval = window.setInterval(() => {
      if (this.updated) {
        this.updateListeners.forEach(l => l(this.state));
        this.emit('update', this.state);
        this.updated = false;
      }
    }, intervalMs);
  }

  private stopUpdateLoop() {
    if (this.updateInterval !== null) {
      clearInterval(this.updateInterval);
      this.updateInterval = null;
    }
  }
}
