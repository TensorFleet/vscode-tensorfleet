/**
 * ROS2 Bridge for Standalone Mode
 * Connects directly to rosbridge or Foxglove Bridge WebSocket
 */
import { FoxgloveWsClient } from "./FoxgloveNetworking";

export type ConnectionMode = 'rosbridge' | 'foxglove';

export interface Subscription {
  topic: string,
  type: string
}

export interface ImageMessage {
  topic: string;
  timestamp: string; // ISO string
  timestampNanos?: number; // nanoseconds since epoch
  frameId: string;
  encoding: string;
  width: number;
  height: number;
  data: string; // base64 or data URI
  messageType: 'raw' | 'compressed';
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
  private client: FoxgloveWsClient | null;
  private messageHandlers: Map<string, Set<(message: any) => void>> = new Map();
  private currentMode: ConnectionMode = 'foxglove';

  // store full Subscription objects keyed by topic
  private subscriptions: Map<string, Subscription> = new Map();

  private reconnectTimeout: number | null = null;


  connect() {
    // TODO : this is just hardcoded ip
    const url ='ws://172.16.0.10:8765';

    console.log(`Connecting to foxglove at ${url}...`);

    if (this.client) {
      this.client.close();
    }

    this.client = new FoxgloveWsClient({ url });

    // set hooks directly
    this.client.onOpen = () => {
      console.log("connected");
      
      this.subscriptions.forEach((sub) => {
        this._forwardSubscribtion(sub);
      });
    };

    this.client.onClose = () => {
      console.log("Foxglove connection closed");
      this.reconnectTimeout = window.setTimeout(() => {
        console.log('Attempting to reconnect...');
        this.connect();
      }, 3000);
    };

    this.client.onError = (err) => {
      console.error("Foxglove client error", err);
    };

    this.client.onNewTopic = (topic, type) => {
      console.log("new Foxglove topic:", topic, "type:", type);
    };

    this.client.onMessage = (msg) => {
      console.log(
        "foxglove msg on",
        msg.topic,
        "encoding",
        msg.encoding,        // <── this is what you’re missing
        "type",
        msg.schemaName,
        "payload",
        msg.payload
      );
      try {
        this.handleFoxgloveMessage(msg)
      } catch (err) {
        console.error("[Foxglove] onmessage error:", err);
      }
    };
  }

  disconnect() {
    if (this.reconnectTimeout) {
      clearTimeout(this.reconnectTimeout);
      this.reconnectTimeout = null;
    }
    if (this.client) {
      this.client.onClose = undefined;
      this.client.close();
    }
  }

  // Ensure we always capture both topic and type in our map
  subscribe(subscription: Subscription, handler: (message: any) => void): () => void  {
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
      return () => {};
    }
  
    this._forwardSubscribtion(subscription);

    return () => { 
      this.unsubscribe(topic, handler);
    };
  }

  _forwardSubscribtion(sub: Subscription) {
    if (!this.client) {
      return;
    }

    const { topic, type } = sub;

    // Foxglove: subscribe by topic name
    this.client.subscribe(topic);

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

    if (!this.client) {
      return;
    }

    this.client.unsubscribe(topic);
  }

  publish(topic: string, messageType: string, message: any) {
    if (!this.client) {
      console.warn("Cannot publish: Foxglove client not created");
      return;
    }

    // messageType here is the Foxglove schemaName (e.g. "geometry_msgs/msg/Twist")
    this.client.publish(topic, messageType, message);
  }

  isConnected(): boolean {
    return !!this.client;
  }

  getAvailableImageTopics(): Subscription[] {
  return [
    { topic: '/camera/image_raw', type: 'sensor_msgs/msg/Image' },
    { topic: '/camera/image_compressed', type: 'sensor_msgs/msg/CompressedImage' },
    { topic: '/camera/color/image_raw', type: 'sensor_msgs/msg/Image' },
    { topic: '/camera/color/image_compressed', type: 'sensor_msgs/msg/CompressedImage' },
    { topic: '/camera/depth/image_raw', type: 'sensor_msgs/msg/Image' },
    { topic: '/camera/rgb/image_raw', type: 'sensor_msgs/msg/Image' },
    { topic: '/camera/rgb/image_compressed', type: 'sensor_msgs/msg/CompressedImage' },
    { topic: '/usb_cam/image_raw', type: 'sensor_msgs/msg/Image' },
    { topic: '/usb_cam/image_compressed', type: 'sensor_msgs/msg/CompressedImage' },
    { topic: '/image', type: 'sensor_msgs/msg/Image' },
    { topic: '/image_raw', type: 'sensor_msgs/msg/Image' },
    { topic: '/image_compressed', type: 'sensor_msgs/msg/CompressedImage' },
  ];
}

  private handleFoxgloveMessage(data: any) {
    // Expecting something like: { topic, schemaName, payload, ... }
    const topic: string | undefined = data?.topic;
    if (!topic) {
      console.warn("[FoxgloveBridge] handleFoxgloveMessage called without topic");
      return;
    }

    const type = this.subscriptions.get(topic)?.type ?? "";

    // FoxgloveWsClient gives `payload`; fall back to `msg` or the object itself if needed.
    const msg: any = data?.payload ?? data?.msg ?? data;
    if (!msg || typeof msg !== "object") {
      this.messageHandlers.get(topic)?.forEach((handler) => handler(data));
      return;
    }

    const header = msg.header || {};
    const frameId = header.frame_id || "";
    let timestamp = new Date().toISOString();
    let timestampNanos: number | undefined;

    if (header.stamp) {
      const sec = header.stamp.sec || 0;
      const nanosec = header.stamp.nanosec || header.stamp.nsec || 0;
      timestampNanos = sec * 1_000_000_000 + nanosec;
      timestamp = new Date(sec * 1000 + nanosec / 1_000_000).toISOString();
    }

    if (type === "sensor_msgs/msg/Image") {
      try {
        const dataURI = this.convertRawImageToDataURI(msg);
        const imageMsg: ImageMessage = {
          topic,
          timestamp,
          timestampNanos,
          frameId,
          encoding: msg.encoding,
          width: msg.width,
          height: msg.height,
          data: dataURI,
          messageType: "raw",
        };
        this.messageHandlers.get(topic)?.forEach((handler) => handler(imageMsg));
      } catch (error) {
        console.error("[FoxgloveBridge] Failed to convert image:", error);
      }
    } else if (type === "sensor_msgs/msg/CompressedImage") {
      try {
        this.convertCompressedImageToDataURI(
          msg,
          (dataURI, width, height) => {
            const imageMsg: ImageMessage = {
              topic,
              timestamp,
              timestampNanos,
              frameId,
              encoding: msg.format,
              width,
              height,
              data: dataURI,
              messageType: "compressed",
            };
            this.messageHandlers.get(topic)?.forEach((handler) => handler(imageMsg));
          },
        );
      } catch (error) {
        console.error("[FoxgloveBridge] Failed to convert compressed image:", error);
      }
    } else {
      this.messageHandlers.get(topic)?.forEach((handler) => handler(msg));
    }
  }

  private convertRawImageToDataURI(msg: any): string {
    const { width, height, encoding, data } = msg;

    // Decode base64 data to byte array
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

    // Convert to RGBA
    const rgba = this.convertToRGBA(imageData, encoding, width, height);

    // Create canvas and draw RGBA data
    const canvas = document.createElement('canvas');
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext('2d');
    if (!ctx) {
      throw new Error('Failed to get canvas context');
    }

    const imageDataObj = ctx.createImageData(width, height);
    imageDataObj.data.set(rgba);
    ctx.putImageData(imageDataObj, 0, 0);

    // Return as data URI (JPEG for efficiency)
    return canvas.toDataURL('image/jpeg', 0.92);
  }
  private convertToRGBA(data: Uint8Array, encoding: string, width: number, height: number): Uint8ClampedArray {
    const pixelCount = width * height;
    const rgba = new Uint8ClampedArray(pixelCount * 4);

    switch (encoding.toLowerCase()) {
      case 'rgb8':
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4] = data[i * 3];       // R
          rgba[i * 4 + 1] = data[i * 3 + 1]; // G
          rgba[i * 4 + 2] = data[i * 3 + 2]; // B
          rgba[i * 4 + 3] = 255;             // A
        }
        break;

      case 'rgba8':
        rgba.set(data.slice(0, pixelCount * 4));
        break;

      case 'bgr8':
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4] = data[i * 3 + 2];     // R (from B)
          rgba[i * 4 + 1] = data[i * 3 + 1]; // G
          rgba[i * 4 + 2] = data[i * 3];     // B (from R)
          rgba[i * 4 + 3] = 255;             // A
        }
        break;

      case 'bgra8':
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4] = data[i * 4 + 2];     // R (from B)
          rgba[i * 4 + 1] = data[i * 4 + 1]; // G
          rgba[i * 4 + 2] = data[i * 4];     // B (from R)
          rgba[i * 4 + 3] = data[i * 4 + 3]; // A
        }
        break;

      case 'mono8':
        for (let i = 0; i < pixelCount; i++) {
          const gray = data[i];
          rgba[i * 4] = gray;     // R
          rgba[i * 4 + 1] = gray; // G
          rgba[i * 4 + 2] = gray; // B
          rgba[i * 4 + 3] = 255;  // A
        }
        break;

      case 'mono16':
        for (let i = 0; i < pixelCount; i++) {
          // Convert 16-bit to 8-bit by taking high byte
          const gray = data[i * 2 + 1];
          rgba[i * 4] = gray;     // R
          rgba[i * 4 + 1] = gray; // G
          rgba[i * 4 + 2] = gray; // B
          rgba[i * 4 + 3] = 255;  // A
        }
        break;

      default:
        console.warn(`[ROS2Bridge] Unsupported encoding: ${encoding}`);
        // Fill with gray as fallback
        rgba.fill(128);
        for (let i = 3; i < rgba.length; i += 4) {
          rgba[i] = 255; // Alpha
        }
    }

    return rgba;
  }

  private convertCompressedImageToDataURI(
    msg: any,
    callback: (dataURI: string, width: number, height: number) => void
  ): void {
    const { format, data } = msg;
    
    // Determine MIME type from format
    let mimeType = 'image/jpeg'; // default
    const formatLower = format.toLowerCase();
    if (formatLower.includes('png')) {
      mimeType = 'image/png';
    } else if (formatLower.includes('webp')) {
      mimeType = 'image/webp';
    }
    
    // Create data URI from base64 data
    // rosbridge sends the data already base64 encoded
    const dataURI = `data:${mimeType};base64,${data}`;
    
    // Load image to get dimensions
    const img = new Image();
    img.onload = () => {
      callback(dataURI, img.width, img.height);
    };
    img.onerror = (error) => {
      console.error('[ROS2Bridge] Failed to load compressed image:', error);
    };
    img.src = dataURI;
  }
}

export const ros2Bridge = new ROS2Bridge();

// Auto-connect on load (using rosbridge by default)
ros2Bridge.connect('rosbridge');

