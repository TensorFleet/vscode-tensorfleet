// ros2-bridge.ts
/**
 * ROS2 Bridge for Standalone Mode
 * Connects directly to Foxglove Bridge WebSocket (CDR + services).
 *
 * Strict constraints honored:
 *  - No rosbridge usage.
 *  - Controller only interacts with DroneStateModel & this ROS2Bridge for publish/service.
 *  - Subscriptions are forwarded to the Foxglove client; reconnect resubscribes automatically.
 *  - No "workarounds" (no std_msgs/String hacks). Use real MAVROS types & Foxglove services.
 */

import { FoxgloveWsClient } from "./foxglove-networking";

export type ConnectionMode = "foxglove";

export interface Subscription {
  topic: string;
  type: string;
}

/** ---------- Common message structs ---------- */
export interface ImageMessage {
  topic: string;
  timestamp: string; // ISO string
  timestampNanos?: number; // nanoseconds since epoch
  frameId: string;
  encoding: string;
  width: number;
  height: number;
  data: string; // base64 or data URI
  messageType: "raw" | "compressed";
}

export interface TwistMessage {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}
export interface BuiltinTime { sec: number; nanosec: number }
export interface StdHeader { stamp: BuiltinTime; frame_id: string }

export interface GeometryVector3 { x: number; y: number; z: number }
export interface GeometryPoint { x: number; y: number; z: number }
export interface GeometryQuaternion { x: number; y: number; z: number; w: number }

export interface GeometryPose { position: GeometryPoint; orientation: GeometryQuaternion }
export interface GeometryTwist { linear: GeometryVector3; angular: GeometryVector3 }

export interface GeometryPoseWithCovariance { pose: GeometryPose; covariance: number[] }
export interface GeometryTwistWithCovariance { twist: GeometryTwist; covariance: number[] }

export interface GeometryPoseStamped { header: StdHeader; pose: GeometryPose }
export interface GeometryTwistStamped { header: StdHeader; twist: GeometryTwist }

export interface NavMsgsOdometry {
  header: StdHeader;
  child_frame_id: string;
  pose: GeometryPoseWithCovariance;
  twist: GeometryTwistWithCovariance;
}

export interface SensorMsgsNavSatStatus { status: number; service: number }
export interface SensorMsgsNavSatFix {
  header: StdHeader;
  status: SensorMsgsNavSatStatus;
  latitude: number; longitude: number; altitude: number;
  position_covariance: number[]; position_covariance_type: number;
}

export interface StdMsgsFloat64 { data: number }

export interface GeographicMsgsGeoPoint { latitude: number; longitude: number; altitude: number }

export interface MavrosMsgsAltitude {
  header: StdHeader;
  monotonic: number; amsl: number; local: number; relative: number; terrain: number; bottom_clearance: number;
}

export interface MavrosMsgsHomePosition {
  header: StdHeader;
  geo: GeographicMsgsGeoPoint;
  position: GeometryPoint;
  orientation: GeometryQuaternion;
  approach: GeometryVector3;
}

/** MAVROS State & ExtendedState */
export interface MavrosMsgsState {
  header?: StdHeader;
  connected: boolean; armed: boolean; guided: boolean; manual_input: boolean;
  mode: string; system_status: number;
}
export interface MavrosMsgsExtendedState { header?: StdHeader; landed_state: number; vtol_state: number }

/** Battery & IMU */
export interface SensorMsgsBatteryState {
  header: StdHeader;
  voltage: number; temperature?: number | null; current?: number; charge?: number;
  capacity?: number; design_capacity?: number; percentage?: number;
  power_supply_status?: number; power_supply_health?: number; power_supply_technology?: number;
  present?: boolean; cell_voltage?: number[]; cell_temperature?: number[];
  location?: string; serial_number?: string;
}
export interface MavrosMsgsVFRHUD {
  airspeed?: number; groundspeed?: number; heading?: number; throttle?: number; altitude?: number; climb?: number;
}
export interface SensorMsgsImu {
  header: StdHeader;
  orientation: GeometryQuaternion; orientation_covariance?: number[];
  angular_velocity: GeometryVector3; angular_velocity_covariance?: number[];
  linear_acceleration: GeometryVector3; linear_acceleration_covariance?: number[];
}

/** Aliases for geometry_msgs names used elsewhere */
export type GeometryMsgsPoseStamped = GeometryPoseStamped;
export type GeometryMsgsTwistStamped = GeometryTwistStamped;

/** Convenience for consumers that expect decoded images */
export interface ImageMessage {
  topic: string;
  timestamp: string;
  timestampNanos?: number;
  frameId: string;
  encoding: string;
  width: number;
  height: number;
  data: string; // data URI
  messageType: "raw" | "compressed";
}

export interface TwistMessage {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}

/** ---------- MAVROS service request/response types ---------- */
/** mavros_msgs/srv/CommandBool */
export interface CommandBool_Request { value: boolean }
export interface CommandBool_Response { success: boolean; result: number }

/** mavros_msgs/srv/SetMode */
export interface SetMode_Request { base_mode: number; custom_mode: string }
export interface SetMode_Response { mode_sent: boolean }

/** mavros_msgs/srv/CommandTOL */
export interface CommandTOL_Request {
  min_pitch: number; yaw: number; latitude: number; longitude: number; altitude: number;
}
export interface CommandTOL_Response { success: boolean; result: number }

/** mavros_msgs/srv/ParamSet */
export interface ParamValue { integer: number; real: number }
export interface ParamSet_Request { param_id: string; value: ParamValue }
export interface ParamSet_Response { success: boolean; value: ParamValue }

/** ---------- Bridge Implementation ---------- */
export class ROS2Bridge {
  private client: FoxgloveWsClient | null = null;

  private messageHandlers = new Map<string, Set<(message: any) => void>>();
  private subscriptions = new Map<string, Subscription>();

  private reconnectTimeout: number | null = null;

  // Topics that should be (re)published once on connect (e.g., latched configs if you need them).
  private setupPublishes: Array<{ topic: string; type: string; message: any }> = [];

  // Services that should be (re)called on every connect before normal ops (optional).
  private setupServiceCalls: Array<{ name: string; request: any }> = [];

  private setupROSParams: Array<{ name: string; value: any }> = [];

  constructor() {
    this._configureDefault();
  }

  connect(_mode: ConnectionMode = "foxglove") {
    const url = "ws://172.16.0.10:8765";

    if (this.client) {
      try { this.client.close(); } catch {}
    }
    this.client = new FoxgloveWsClient({ url });

    // re-seed setup publishes and service calls to the new client
    for (const cmd of this.setupPublishes) {
      this.client.publishSetup(cmd.topic, cmd.type, cmd.message);
    }

    // Startup service calls will only be sent one all of them are available.
    console.log("[ROS2Bridge] Forwarding setup service calls :", this.setupServiceCalls);
    this.setupServiceCalls.forEach(({name, request}) => this.client?.registerSetupServiceCall(
      {
        serviceName: name,
        request: request
      }
    ));

    console.log("[ROS2Bridge] Forwarding setup ros params :", this.setupROSParams);
    this.setupROSParams.forEach(({name, value}) => this.client?.registerSetupParameterSet(name, value));

    this.client.onOpen = async () => {
      // forward queued subscriptions. If the topics are not available they will just go into pending till they are.
      this.subscriptions.forEach((sub) => this._forwardSubscription(sub));
    };

    this.client.onClose = () => {
      console.log("Foxglove connection closed");
      this.reconnectTimeout = window.setTimeout(() => {
        console.log("Attempting to reconnect...");
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
      const ref = { topic: msg.topic, type: msg.schemaName, msg: msg.payload };
      this.handleFoxgloveMessage(ref);
    };
  }

  disconnect() {
    if (this.reconnectTimeout) {
      clearTimeout(this.reconnectTimeout);
      this.reconnectTimeout = null;
    }
    try { this.client?.close(); } catch {}
    this.client = null;
  }

  /** Store a subscription and (re)apply it on connect. */
  subscribe(subscription: Subscription, handler: (message: any) => void): () => void {
    const { topic, type } = subscription;
    this.subscriptions.set(topic, { topic, type });

    let set = this.messageHandlers.get(topic);
    if (!set) {
      set = new Set();
      this.messageHandlers.set(topic, set);
    }

    set.add(handler);

    if (!this.client || !this.client.isConnected()) {
      console.warn("Not connected, queueing subscription:", { topic, type });
      return () => {
        this.unsubscribe(topic, handler);
      };
    }

    this._forwardSubscription(subscription);

    return () => this.unsubscribe(topic, handler);
  }

  private _forwardSubscription(sub: Subscription) {
    if (!this.client) return;
    // Foxglove subscribes by *topic name* (schemaName is resolved server-side)
    this.client.subscribe(sub.topic);
  }

  unsubscribe(topic: string, handler: (message: any) => void) {
    const set = this.messageHandlers.get(topic);
    if (set) {
      set.delete(handler);
      if (set.size === 0) {
        this.messageHandlers.delete(topic);
        this.subscriptions.delete(topic);
        this.client?.unsubscribe(topic);
      }
    }
  }

  /** Generic topic publish. Uses exact ROS 2 schemaName for serialization. */
  publish(topic: string, messageType: string, message: any) {
    if (!this.client) {
      console.warn("publish() ignored: Foxglove client not ready");
      return;
    }
    this.client.publish(topic, messageType, message);
  }

  /** Arrange for a topic publish to be sent immediately after connect and on every reconnect. */
  publishSetup(topic: string, type: string, message: any) {
    this.setupPublishes.push({ topic, type, message });
    if (this.client) {
      this.client.publishSetup(topic, type, message);
    }
  }

  /** Arrange for a service call to run once on every (re)connect before normal ops. */
  registerSetupServiceCall(name: string, request: any) {
    this.setupServiceCalls.push({ 
      name,
      request
     });
  }

  /**
   * Set default configs to this.client for the drone.
   * Will use publishSetup to set configurations.
   */
  _configureDefault() {
    // (Also registers setup service calls to run on connect.)

    // Ensure PX4 does not require RC for arming in SITL:
    // const setRcNotRequired = {
    //   "param_id": "COM_RC_IN_MODE",
    //   "value": { "integer": 1, "real": 0.0 }
    // };
    // this.registerSetupServiceCall("/mavros/param/set", setRcNotRequired);

    // const heartbeatConfigPayload = {
    //   parameters: [
    //     {
    //       name: "conn/heartbeat_rate",
    //       value: { type: 3, double_value: 2.0 } // 2.0 Hz; use 0.0 to disable
    //     }
    //   ]
    // }

    // this.registerSetupServiceCall("/mavros/set_parameters", heartbeatConfigPayload);

    // Configure heartbeat :
    this.registerSetupROSParameterSet("/mavros/sys.heartbeat_mav_type", "GCS");
    this.registerSetupROSParameterSet("/mavros/sys.heartbeat_rate", 2.0);
  }

  registerSetupROSParameterSet(name: string, value: any): void {
    this.setupROSParams.push({ name, value});
  }

  isConnected(): boolean {
    return !!this.client && this.client.isConnected();
  }

  getAvailableImageTopics(): Subscription[] {
    return [
      { topic: "/camera/image_raw", type: "sensor_msgs/msg/Image" },
      { topic: "/camera/image_compressed", type: "sensor_msgs/msg/CompressedImage" },
      { topic: "/camera/color/image_raw", type: "sensor_msgs/msg/Image" },
      { topic: "/camera/color/image_compressed", type: "sensor_msgs/msg/CompressedImage" },
      { topic: "/camera/depth/image_raw", type: "sensor_msgs/msg/Image" },
      { topic: "/camera/rgb/image_raw", type: "sensor_msgs/msg/Image" },
      { topic: "/camera/rgb/image_compressed", type: "sensor_msgs/msg/CompressedImage" },
      { topic: "/usb_cam/image_raw", type: "sensor_msgs/msg/Image" },
      { topic: "/usb_cam/image_compressed", type: "sensor_msgs/msg/CompressedImage" },
      { topic: "/image", type: "sensor_msgs/msg/Image" },
      { topic: "/image_raw", type: "sensor_msgs/msg/Image" },
      { topic: "/image_compressed", type: "sensor_msgs/msg/CompressedImage" },
    ];
  }

  /** Generic Foxglove service call (requires FoxgloveWsClient service support). */
  async callService<T = any>(name: string, request: any): Promise<T> {
    if (!this.client) throw new Error("callService() before connect");
    if (typeof (this.client as any).callService !== "function") {
      throw new Error("FoxgloveWsClient.callService() not available");
    }
    return await (this.client as any).callService<T>({
      serviceName: name,
      request: request
    });
  }

  // ---------- MAVROS service helpers (exact names and request fields) ----------

  /** /mavros/cmd/arming (mavros_msgs/srv/CommandBool) */
  async mavrosArmDisarm(value: boolean): Promise<CommandBool_Response> {
    console.log("[ROS2Bridge] calling mavrosArmDisarm with ", value);

    const req: CommandBool_Request = { value };
    return await this.callService<CommandBool_Response>("/mavros/cmd/arming", req);
  }

  /** /mavros/set_mode (mavros_msgs/srv/SetMode) */
  async mavrosSetMode(custom_mode: string, base_mode = 0): Promise<SetMode_Response> {
    const req: SetMode_Request = { base_mode, custom_mode };
    return await this.callService<SetMode_Response>("/mavros/set_mode", req);
  }

  /** /mavros/cmd/takeoff (mavros_msgs/srv/CommandTOL) */
  async mavrosTakeoff(args: { altitude: number; min_pitch?: number; yaw?: number; latitude?: number; longitude?: number }): Promise<CommandTOL_Response> {
    const req: CommandTOL_Request = {
      altitude: args.altitude,
      min_pitch: args.min_pitch ?? 0.0,
      yaw: args.yaw ?? 0.0,
      latitude: args.latitude ?? 0.0,
      longitude: args.longitude ?? 0.0,
    };
    return await this.callService<CommandTOL_Response>("/mavros/cmd/takeoff", req);
  }

  /** /mavros/cmd/land (mavros_msgs/srv/CommandTOL) */
  async mavrosLand(args: { altitude?: number; yaw?: number; latitude?: number; longitude?: number } = {}): Promise<CommandTOL_Response> {
    const req: CommandTOL_Request = {
      altitude: args.altitude ?? 0.0,
      yaw: args.yaw ?? 0.0,
      latitude: args.latitude ?? 0.0,
      longitude: args.longitude ?? 0.0,
    };
    return await this.callService<CommandTOL_Response>("/mavros/cmd/land", req);
  }

  /** /mavros/param/set (mavros_msgs/srv/ParamSet) */
  async mavrosParamSet(param_id: string, value: ParamValue): Promise<ParamSet_Response> {
    const req: ParamSet_Request = { param_id, value };
    return await this.callService<ParamSet_Response>("/mavros/param/set", req);
  }

  // ---------- Image conversions (used by consumers that want a Data URI) ----------

  private handleFoxgloveMessage(data: any) {
    // Expecting something like: { topic, type, msg }
    const topic: string = data.topic;
    const type = data.type;
    const msg: any = data.msg;

    const header = msg?.header || {};
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
        this.convertCompressedImageToDataURI(msg, (dataURI, width, height) => {
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
        });
      } catch (error) {
        console.error("[FoxgloveBridge] Failed to convert compressed image:", error);
      }
    } else {
      this.messageHandlers.get(topic)?.forEach((handler) => handler(data));
    }
  }

  private convertRawImageToDataURI(msg: any): string {
    const { width, height, encoding, data } = msg;

    // Decode base64 data to byte array
    let imageData: Uint8Array;
    if (typeof data === "string") {
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
      throw new Error("Unknown data type");
    }

    // Convert to RGBA
    const rgba = this.convertToRGBA(imageData, encoding, width, height);

    // Create canvas and draw RGBA data
    const canvas = document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext("2d");
    if (!ctx) {
      throw new Error("Failed to get canvas context");
    }

    const imageDataObj = ctx.createImageData(width, height);
    imageDataObj.data.set(rgba);
    ctx.putImageData(imageDataObj, 0, 0);

    // Return as data URI (JPEG for efficiency)
    return canvas.toDataURL("image/jpeg", 0.92);
  }

  private convertToRGBA(
    data: Uint8Array,
    encoding: string,
    width: number,
    height: number,
  ): Uint8ClampedArray {
    const pixelCount = width * height;
    const rgba = new Uint8ClampedArray(pixelCount * 4);

    switch ((encoding || "").toLowerCase()) {
      case "rgb8":
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4] = data[i * 3]; // R
          rgba[i * 4 + 1] = data[i * 3 + 1]; // G
          rgba[i * 4 + 2] = data[i * 3 + 2]; // B
          rgba[i * 4 + 3] = 255; // A
        }
        break;

      case "rgba8":
        rgba.set(data.slice(0, pixelCount * 4));
        break;

      case "bgr8":
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4] = data[i * 3 + 2]; // R (from B)
          rgba[i * 4 + 1] = data[i * 3 + 1]; // G
          rgba[i * 4 + 2] = data[i * 3]; // B (from R)
          rgba[i * 4 + 3] = 255; // A
        }
        break;

      case "bgra8":
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4] = data[i * 4 + 2]; // R (from B)
          rgba[i * 4 + 1] = data[i * 4 + 1]; // G
          rgba[i * 4 + 2] = data[i * 4]; // B (from R)
          rgba[i * 4 + 3] = data[i * 4 + 3]; // A
        }
        break;

      case "mono8":
        for (let i = 0; i < pixelCount; i++) {
          const gray = data[i];
          rgba[i * 4] = gray; // R
          rgba[i * 4 + 1] = gray; // G
          rgba[i * 4 + 2] = gray; // B
          rgba[i * 4 + 3] = 255; // A
        }
        break;

      case "mono16":
        for (let i = 0; i < pixelCount; i++) {
          // Convert 16-bit to 8-bit by taking high byte
          const gray = data[i * 2 + 1];
          rgba[i * 4] = gray; // R
          rgba[i * 4 + 1] = gray; // G
          rgba[i * 4 + 2] = gray; // B
          rgba[i * 4 + 3] = 255; // A
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
    callback: (dataURI: string, width: number, height: number) => void,
  ): void {
    const { format, data } = msg;

    // Determine MIME type from format
    let mimeType = "image/jpeg"; // default
    const formatLower = (format || "").toLowerCase();
    if (formatLower.includes("png")) {
      mimeType = "image/png";
    } else if (formatLower.includes("webp")) {
      mimeType = "image/webp";
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
      console.error("[ROS2Bridge] Failed to load compressed image:", error);
    };
    img.src = dataURI;
  }
}

export const ros2Bridge = new ROS2Bridge();

// Auto-connect Foxglove (no rosbridge)
ros2Bridge.connect("foxglove");
