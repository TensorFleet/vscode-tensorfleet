/**
 * ROS2 Bridge for Standalone Mode
 * Connects directly to rosbridge or Foxglove Bridge WebSocket
 */


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
  private ws: WebSocket | null = null;
  private messageHandlers: Map<string, Set<(message: any) => void>> = new Map();
  private currentMode: ConnectionMode = 'rosbridge';

  // store full Subscription objects keyed by topic
  private subscriptions: Map<string, Subscription> = new Map();

  private reconnectTimeout: number | null = null;


  connect(mode: ConnectionMode = 'rosbridge') {
    this.currentMode = mode;
    // TODO : this is just hardcoded ip
    const url = mode === 'rosbridge' 
      ? 'ws://172.16.0.10:9091'
      : 'ws://172.16.0.10:8765';

    console.log(`Connecting to ${mode} at ${url}...`);

    if (this.ws) {
      this.ws.close();
    }

    // Foxglove requires the subprotocol to be specified
    this.ws = mode === 'foxglove'
      ? new WebSocket(url, 'foxglove.websocket.v1')
      : new WebSocket(url);

    this.ws.onopen = () => {
      console.log(`Connected to ${mode}`);
      this.subscriptions.forEach((sub) => {
        this._forwardSubscribtion(sub);
      });
    };

    this.ws.onmessage = async (event) => {
      try {
        if (this.currentMode === "rosbridge") {
          const payload = event.data;
          let data: any;

          if (typeof payload === "string") {
            data = JSON.parse(payload);
          } else {
            console.log("Unsupported ROS2Bridge payload :", typeof payload);
            return
          }

          this.handleRosbridgeMessage(data);

        } else {
          console.log("Foxglove protocol not implemented");
        }

        
      } catch (err) {
        console.error("[RB] onmessage error:", err);
      }
    };
    this.ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    this.ws.onclose = () => {
      console.log(`Disconnected from ${mode}`);
      // Auto-reconnect after 3 seconds (use currentMode in case mode changed)
      this.reconnectTimeout = window.setTimeout(() => {
        console.log('Attempting to reconnect...');
        this.connect(this.currentMode);
      }, 3000);
    };
  }

  disconnect() {
    if (this.reconnectTimeout) {
      clearTimeout(this.reconnectTimeout);
      this.reconnectTimeout = null;
    }
    if (this.ws) {
      this.ws.onclose = null;
      this.ws.close();
      this.ws = null;
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
    if (!this.ws) {
      return;
    }

    const { topic, type } = sub;

    if (this.currentMode === 'rosbridge') {
      this.ws.send(JSON.stringify({
        op: 'subscribe',
        topic,
        type,
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

  private handleRosbridgeMessage(data: any) {
    if (data.op === 'publish' && data.msg) {
      const msg = data.msg;
      const topic = data.topic;
      const type = this.subscriptions.get(topic)?.type ?? "";

      // Extract header information (common to both message types)

      const header = msg.header || {};
      const frameId = header.frame_id || '';
      let timestamp = new Date().toISOString();
      let timestampNanos: number | undefined;
      
      // Extract timestamp from header if available
      if (header.stamp) {
        const sec = header.stamp.sec || 0;
        const nanosec = header.stamp.nanosec || 0;
        timestampNanos = sec * 1_000_000_000 + nanosec;
        timestamp = new Date(sec * 1000 + nanosec / 1_000_000).toISOString();
      }

      // Handle raw Image messages (sensor_msgs/Image)
      if (type === "sensor_msgs/msg/Image") {
        try {
          const dataURI = this.convertRawImageToDataURI(msg);
          const imageMsg: ImageMessage = {
            topic: data.topic,
            timestamp,
            timestampNanos,
            frameId,
            encoding: msg.encoding,
            width: msg.width,
            height: msg.height,
            data: dataURI,
            messageType: 'raw'
          };
          this.messageHandlers.get(topic)?.forEach(handler => handler(imageMsg));
        } catch (error) {
          console.error('[ROS2Bridge] Failed to convert image:', error);
        }
      } else if (type === 'sensor_msgs/msg/CompressedImage') {
          try {
            this.convertCompressedImageToDataURI(msg, (dataURI, width, height) => {
              const imageMsg: ImageMessage = {
                topic: data.topic,
                timestamp,
                timestampNanos,
                frameId,
                encoding: msg.format, // e.g., "jpeg", "png"
                width,
                height,
                data: dataURI,
                messageType: 'compressed'
              };
              this.messageHandlers.get(topic)?.forEach(handler => handler(imageMsg));
            });
          } catch (error) {
            console.error('[ROS2Bridge] Failed to convert compressed image:', error);
          }
      } else {
        this.messageHandlers.get(topic)?.forEach(handler => handler(data));
      }
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

  private handleFoxgloveMessage(data: any) {
    // Foxglove Bridge sends binary data in a different format
    // TODO: Implement Foxglove message parsing
    console.log('Foxglove message:', data);
  }
}

export const ros2Bridge = new ROS2Bridge();

// Auto-connect on load (using rosbridge by default)
ros2Bridge.connect('rosbridge');

