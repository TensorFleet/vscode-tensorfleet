/**
 * ROS2 Bridge for Standalone Mode
 * Connects directly to rosbridge or Foxglove Bridge WebSocket
 */

import { decode as cborDecode } from "cbor-x";
import { MessageReader } from "@foxglove/rosmsg2-serialization";
import * as rosMsgs from "@foxglove/rosmsg-msgs-common";

export type ConnectionMode = 'rosbridge' | 'foxglove';

export interface Subscription {
  topic: string,
  type: string
}

export interface ImageMessage {
  topic: string;
  timestamp: string;
  encoding: string;
  width: number;
  height: number;
  data: string; // base64 or data URI
}

export interface TwistMessage {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}

export interface BuiltinTime {
  sec: number;
  nanosec: number;
}

export interface StdHeader {
  stamp: BuiltinTime;
  frame_id: string;
}

export interface GeometryVector3 {
  x: number;
  y: number;
  z: number;
}

export interface GeometryPoint {
  x: number;
  y: number;
  z: number;
}

export interface GeometryQuaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface GeometryPose {
  position: GeometryPoint;
  orientation: GeometryQuaternion;
}

export interface GeometryTwist {
  linear: GeometryVector3;
  angular: GeometryVector3;
}

export interface GeometryPoseWithCovariance {
  pose: GeometryPose;
  covariance: number[]; // length 36
}

export interface GeometryTwistWithCovariance {
  twist: GeometryTwist;
  covariance: number[]; // length 36
}

export interface GeometryPoseStamped {
  header: StdHeader;
  pose: GeometryPose;
}

export interface GeometryTwistStamped {
  header: StdHeader;
  twist: GeometryTwist;
}

export interface NavMsgsOdometry {
  header: StdHeader;
  child_frame_id: string;
  pose: GeometryPoseWithCovariance;
  twist: GeometryTwistWithCovariance;
}

export interface SensorMsgsNavSatStatus {
  status: number;  // e.g., STATUS_FIX, STATUS_NO_FIX
  service: number; // SERVICE_GPS, SERVICE_GLONASS, etc.
}

export interface SensorMsgsNavSatFix {
  header: StdHeader;
  status: SensorMsgsNavSatStatus;
  latitude: number;
  longitude: number;
  altitude: number;
  position_covariance: number[]; // length 9
  position_covariance_type: number;
}

export interface StdMsgsFloat64 {
  data: number;
}

export interface GeographicMsgsGeoPoint {
  latitude: number;
  longitude: number;
  altitude: number;
}

export interface MavrosMsgsAltitude {
  header: StdHeader;
  monotonic: number;        // meters
  amsl: number;             // meters
  local: number;            // meters
  relative: number;         // meters
  terrain: number;          // meters
  bottom_clearance: number; // meters
}

export interface MavrosMsgsHomePosition {
  header: StdHeader;
  geo: GeographicMsgsGeoPoint;     // geographic (lat/lon/alt)
  position: GeometryPoint;         // local position (m)
  orientation: GeometryQuaternion; // local orientation
  approach: GeometryVector3;       // approach vector
}

export class ROS2Bridge {
  private ws: WebSocket | null = null;
  private messageHandlers: Map<string, Set<(message: any) => void>> = new Map();
  private currentMode: ConnectionMode = 'rosbridge';

  // store full Subscription objects keyed by topic
  private subscriptions: Map<string, Subscription> = new Map();

  private reconnectTimeout: number | null = null;


  connect(mode: ConnectionMode = 'rosbridge') {
    this.currentMode = mode;
    const url = mode === 'rosbridge' 
      ? 'ws://172.16.0.10:9091'
      : 'ws://172.16.0.10:8765';

    console.log(`Connecting to ${mode} at ${url}...`);

    if (this.ws) {
      this.ws.close();
    }

    this.ws = mode === 'foxglove'
      ? new WebSocket(url, 'foxglove.websocket.v1')
      : new WebSocket(url);

    if (this.currentMode === 'rosbridge') {
      this.ws.binaryType = 'arraybuffer';
    }

    this.ws.onopen = () => {
      console.log(`Connected to ${mode}`);
      this.subscriptions.forEach((sub) => {
        this._forwardSubscribtion(sub);
      });
    };

    this.ws.onmessage = async (event) => {
  try {
    if (this.currentMode !== "rosbridge") return;

    const payload = event.data;
    let data: any;

    if (typeof payload === "string") {
      data = JSON.parse(payload);
    } else if (payload instanceof ArrayBuffer) {
      data = cborDecode(new Uint8Array(payload));
    } else if (payload instanceof Blob) {
      const buf = await payload.arrayBuffer();
      data = cborDecode(new Uint8Array(buf));
    } else {
      return;
    }

    const topic = data?.topic as string | undefined;
    const rosType = (topic && this.subscriptions.get(topic)?.type) || "(unknown)";
    console.log("[RB] topic:", topic, "| ROS TYPE:", rosType);

    if (
      data &&
      data.op === "publish" &&
      data.msg &&
      typeof data.msg === "object" &&
      "bytes" in data.msg &&
      rosType !== "(unknown)"
    ) {
      const raw = (data.msg as any).bytes;
      const u8 =
        raw instanceof Uint8Array ? raw :
        raw instanceof ArrayBuffer ? new Uint8Array(raw) :
        Array.isArray(raw) ? Uint8Array.from(raw) :
        new Uint8Array(Object.values(raw) as number[]);

      // --- build a flat definitions array from @foxglove/rosmsg-msgs-common ---
      // (no name manipulation; search across all exported ROS2 groups)
      const defsFlat: any[] = [];
      for (const key of Object.keys(rosMsgs)) {
        const v: any = (rosMsgs as any)[key];
        if (Array.isArray(v)) {
          defsFlat.push(...v);
        } else if (v && Array.isArray(v.definitions)) {
          defsFlat.push(...v.definitions);
        }
      }
      console.log("[RB] defsFlat count:", defsFlat.length);

      // decode using MessageReader with the exact ROS TYPE string
      try {
        const reader = new MessageReader(defsFlat);
        const decoded = reader.readMessage(u8, rosType);
        data.msg = decoded;
        console.log("[RB] decoded keys:", Object.keys(decoded || {}));
      } catch (e) {
        console.warn("[RB] decode failed for", rosType, e);
      }
    } else {
      console.log(
        "[RB] msg typeof:", typeof data?.msg,
        "| has bytes:", !!(data?.msg && typeof data.msg === "object" && "bytes" in data.msg)
      );
    }

    this.handleRosbridgeMessage(data);
  } catch (err) {
    console.error("[RB] onmessage error:", err);
  }
};
    this.ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    this.ws.onclose = () => {
      // no auto-reconnect
    };
  }

  disconnect() {
    if (this.reconnectTimeout) {
      clearTimeout(this.reconnectTimeout);
      this.reconnectTimeout = null;
    }
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  // Ensure we always capture both topic and type in our map
  subscribe(subscription: Subscription, handler: (message: any) => void) {
    const { topic, type } = subscription;
    this.subscriptions.set(topic, { topic, type });

    let currentSet = this.messageHandlers.get(topic);
    if (!currentSet) {
      currentSet = new Set<(message: any) => void>();
      this.messageHandlers.set(topic, currentSet);
    }

    currentSet.add(handler);

    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      console.warn('Not connected, queueing subscription:', { topic, type });
      return;
    }
  
    this._forwardSubscribtion(subscription);
  }

  _forwardSubscribtion(sub: Subscription) {
    if (!this.ws) {
      return;
    }

    const { topic, type } = sub;

    if (this.currentMode === 'rosbridge') {
      this.ws.send(JSON.stringify({
        op: 'subscribe',
        topic,
        type,
        compression: 'cbor',
        throttle_rate: 0,
        queue_length: 0
      }));
    } else {
      this.ws.send(JSON.stringify({
        op: 'subscribe',
        subscriptions: [{
          id: Date.now(),
          topic
        }]
      }));
    }

    console.log(`Subscribed to [${type}] : ${topic}`);
  }

  unsubscribe(topic: string, handler: (message: any) => void) {
    const handlers = this.messageHandlers.get(topic);
    if (handlers) {
      handlers.delete(handler);
      if (handlers.size) {
        return;
      }
    }
    this.subscriptions.delete(topic);

    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return;
    }

    if (this.currentMode === 'rosbridge') {
      this.ws.send(JSON.stringify({
        op: 'unsubscribe',
        topic
      }));
    } else {
      console.warn('Foxglove unsubscribe not implemented (no id tracking).');
    }
  }

  publish(topic: string, messageType: string, message: any) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      console.warn('Cannot publish: not connected');
      return;
    }

    if (this.currentMode === 'rosbridge') {
      this.ws.send(JSON.stringify({
        op: 'publish',
        topic,
        type: messageType,
        msg: message
      }));
    } else {
      console.warn('Publishing not yet supported for Foxglove mode');
    }
  }

  isConnected(): boolean {
    return this.ws !== null && this.ws.readyState === WebSocket.OPEN;
  }

  private handleRosbridgeMessage(data: any) {
    if (data.op === 'publish' && data.msg) {
      const msg = data.msg;
      const topic = data.topic;
      const type = this.subscriptions.get(topic)?.type ?? "";

      if (type === "sensor_msgs/Image") {
        if (msg.width && msg.height && msg.encoding && msg.data) {
          try {
            const dataURI = this.convertRawImageToDataURI(msg);
            const imageMsg: ImageMessage = {
              topic: data.topic,
              timestamp: new Date().toISOString(),
              encoding: msg.encoding,
              width: msg.width,
              height: msg.height,
              data: dataURI
            };
            this.messageHandlers.get(topic)?.forEach(handler => handler(imageMsg));
          } catch (error) {
            console.error('[ROS2Bridge] Failed to convert image:', error);
          }
        }
      } else {
        this.messageHandlers.get(topic)?.forEach(handler => handler(data));
      }
    }
  }

  private convertRawImageToDataURI(msg: any): string {
    const { width, height, encoding, data } = msg;

    let imageData: Uint8Array;
    if (typeof data === 'string') {
      const binaryString = atob(data);
      imageData = new Uint8Array(binaryString.length);
      for (let i = 0; i < binaryString.length; i++) {
        imageData[i] = binaryString.charCodeAt(i);
      }
    } else if (Array.isArray(data)) {
      imageData = new Uint8Array(data);
    } else if (data instanceof Uint8Array) {
      imageData = data;
    } else {
      throw new Error('Unknown data type');
    }

    const rgba = this.convertToRGBA(imageData, encoding, width, height);

    const canvas = document.createElement('canvas');
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext('2d');
    if (!ctx) {
      throw new Error('Failed to get canvas context');
    }

    const imageDataObj = new ImageData(rgba, width, height);
    ctx.putImageData(imageDataObj, 0, 0);

    return canvas.toDataURL('image/jpeg', 0.92);
  }

  private convertToRGBA(data: Uint8Array, encoding: string, width: number, height: number): Uint8ClampedArray {
    const pixelCount = width * height;
    const rgba = new Uint8ClampedArray(pixelCount * 4);

    switch ((encoding || '').toLowerCase()) {
      case 'rgb8':
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4]     = data[i * 3];
          rgba[i * 4 + 1] = data[i * 3 + 1];
          rgba[i * 4 + 2] = data[i * 3 + 2];
          rgba[i * 4 + 3] = 255;
        }
        break;
      case 'rgba8':
        rgba.set(data.slice(0, pixelCount * 4));
        break;
      case 'bgr8':
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4]     = data[i * 3 + 2];
          rgba[i * 4 + 1] = data[i * 3 + 1];
          rgba[i * 4 + 2] = data[i * 3];
          rgba[i * 4 + 3] = 255;
        }
        break;
      case 'bgra8':
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4]     = data[i * 4 + 2];
          rgba[i * 4 + 1] = data[i * 4 + 1];
          rgba[i * 4 + 2] = data[i * 4];
          rgba[i * 4 + 3] = data[i * 4 + 3];
        }
        break;
      case 'mono8':
        for (let i = 0; i < pixelCount; i++) {
          const gray = data[i];
          rgba[i * 4] = gray; rgba[i * 4 + 1] = gray; rgba[i * 4 + 2] = gray; rgba[i * 4 + 3] = 255;
        }
        break;
      case 'mono16':
        for (let i = 0; i < pixelCount; i++) {
          const gray = data[i * 2 + 1];
          rgba[i * 4] = gray; rgba[i * 4 + 1] = gray; rgba[i * 4 + 2] = gray; rgba[i * 4 + 3] = 255;
        }
        break;
      default:
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4] = 128; rgba[i * 4 + 1] = 128; rgba[i * 4 + 2] = 128; rgba[i * 4 + 3] = 255;
        }
    }

    return rgba;
  }

  private handleFoxgloveMessage(data: any) {
    console.log('Foxglove message:', data);
  }
}

export const ros2Bridge = new ROS2Bridge();
ros2Bridge.connect('rosbridge');
