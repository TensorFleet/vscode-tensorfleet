// ros2-bridge.ts
/**
 * ROS2 Bridge for Standalone Mode
 * Connects to Foxglove Bridge WebSocket (CDR + services) via the vm-manager proxy.
 *
 * Constraints:
 *  - No rosbridge usage.
 *  - Subscriptions are forwarded directly to Foxglove.
 *  - reconnect logic is handled here.
 */

import { FoxgloveWsClient } from "./foxglove-networking";
import { ROS_PORTS } from "./ws-proxy-client";
import * as RosTypes from "tensorfleet-util/ros/ros-types";
import { ROS2BridgeApi } from "tensorfleet-util/ros/ros-bridge-api";

// (optional but nice) local type for the rosapi response
type RosapiNodesResponse = { nodes: string[] };

export type ConnectionMode = "foxglove";

interface ConnectionSettings {
  useProxy: boolean
  proxyUrl: string;
  vmManagerUrl: string;
  nodeId: string;
  token: string;
  targetPort: number;
}

export interface Subscription {
  topic: string;
  type: string;
}

/** ---------- Bridge Implementation ---------- */

export class ROS2Bridge {
  private client: FoxgloveWsClient | null = null;

  private messageHandlers = new Map<string, Set<(message: any) => void>>();
  private subscriptions = new Map<string, Subscription>();

  // track discovered topics from the bridge
  private discoveredTopics: Map<string, string> = new Map(); // topic -> type

  private reconnectTimeout: number | null = null;

  // Topics that should be (re)published once on connect (e.g., latched configs).
  private setupPublishes: Array<{ topic: string; type: string; message: any }> = [];

  // Services that should be (re)called on every connect before normal ops (optional).
  private setupServiceCalls: Array<{ name: string; request: any }> = [];

  private setupROSParams: Array<{ name: string; value: any }> = [];

  // All known frame_ids (from headers, tf, odom, etc.)
  private frameIds = new Set<string>();

  // frame_id -> topics that have produced that frame
  private frameTopics = new Map<string, Set<string>>();

  // Available topics change notifications
  private availableTopicsListeners = new Set<(topics: Subscription[]) => void>();
  private topicsWatchTimer: number | null = null;
  private _lastTopicsSig: string | null = null;

  // Stored connection settings (copy, not reference)
  private connectionSettings: ConnectionSettings | null = null;

  // Timer for checking settings changes
  private settingsCheckTimer: number | null = null;

  // Server info readiness promise
  private serverInfoReady: Promise<void> | null = null;
  private serverInfoResolver: (() => void) | null = null;

  constructor() {
    this._configureDefault();
    this._startSettingsWatcher();
  }

  connect(_mode: ConnectionMode = "foxglove", targetPort?: number, settings?: ConnectionSettings) {
    console.log('ROS2Bridge: Starting connection process...');
    // Reset server info readiness for new connection
    this.serverInfoReady = new Promise<void>((resolve) => {
      this.serverInfoResolver = resolve;
    });

    // Use provided settings or fall back to window globals
    const proxyUrl = settings?.proxyUrl ?? (window as any).TENSORFLEET_PROXY_URL;
    const vmManagerUrl = settings?.vmManagerUrl ?? (window as any).TENSORFLEET_VM_MANAGER_URL;
    const nodeId = settings?.nodeId ?? (window as any).TENSORFLEET_NODE_ID;
    const token = settings?.token ?? (window as any).TENSORFLEET_JWT;
    const port = settings?.targetPort ?? targetPort ?? ROS_PORTS.FOXGLOVE_BRIDGE;
    const useProxy = settings?.useProxy ?? (window as any).TENSORFLEET_USE_PROXY ?? true;

    // Store a copy of the connection settings (not reference)
    this.connectionSettings = {
      useProxy: useProxy,
      proxyUrl: proxyUrl || '',
      vmManagerUrl: vmManagerUrl || '',
      nodeId: nodeId || '',
      token: token || '',
      targetPort: port,
    };

    if (!proxyUrl && !vmManagerUrl) {
      // eslint-disable-next-line no-console
      console.error("[ROS2Bridge] Missing proxy URL for vm-manager WebSocket proxy. Expected window.TENSORFLEET_PROXY_URL or window.TENSORFLEET_VM_MANAGER_URL to be set in the webview HTML.", {
        proxyUrl,
        vmManagerUrl,
      });
      return;
    }

    if (this.client) {
      try {
        this.client.close();
      } catch {
        // ignore
      }
    }

    const tokenPreview = typeof token === "string" ? `${token.slice(0, 8)}…` : undefined;
    // eslint-disable-next-line no-console
    console.log("[ROS2Bridge] Connecting via vm-manager proxy", {
      proxyUrl: proxyUrl ?? null,
      vmManagerUrl: vmManagerUrl ?? null,
      nodeId,
      port,
      tokenPreview,
    });

    this.client = new FoxgloveWsClient({
      useProxy,
      proxyUrl,
      vmManagerUrl,
      token,
      nodeId,
      targetPort: port,
    });

    // Needed to compute 3D transforms.
    this.client.subscribe("/tf");
    this.client.subscribe("/tf_static");

    // re-seed setup publishes and service calls to the new client
    for (const cmd of this.setupPublishes) {
      this.client.publishSetup(cmd.topic, cmd.type, cmd.message);
    }

    // Startup service calls will only be sent once all of them are available.
    // (Actual availability is handled inside FoxgloveWsClient.)
    // eslint-disable-next-line no-console
    console.log("[ROS2Bridge] Forwarding setup service calls:", this.setupServiceCalls);
    this.setupServiceCalls.forEach(({ name, request }) =>
      this.client?.registerSetupServiceCall({
        serviceName: name,
        request,
      }),
    );

    // eslint-disable-next-line no-console
    console.log("[ROS2Bridge] Forwarding setup ros params:", this.setupROSParams);
    this.setupROSParams.forEach(({ name, value }) =>
      this.client?.registerSetupParameterSet(name, value),
    );

    this.client.onOpen = async () => {
      // forward queued subscriptions. If topics are not available they go pending.
      this.subscriptions.forEach((sub) => this._forwardSubscription(sub));

      // start topics watcher
      this._startTopicsWatcher(1000);
    };

    // Print all ROS parameters after server capabilities are received
    this.client.onServerInfo = async () => {
      console.log('ROS2Bridge: onServerInfo callback fired - server capabilities received');
      // Mark server info as ready
      this.serverInfoResolver?.();
      console.log('ROS2Bridge: serverInfoReady promise resolved');

      if (!this.client) return;
      try {
        await this.client.waitForService("/rosapi/nodes");
        console.log("[Tensorfleet] All nodes :", await this.listNodes());
      } catch (error) {
        console.warn("[Tensorfleet] Failed to get nodes (service not available):", error);
      }
      try {
        console.log("[Tensorfleet] All params :", await this.getAllROSParameters());
      } catch (error) {
        console.warn("[Tensorfleet] Failed to get params:", error);
      }
    };

    this.client.onClose = () => {
      // eslint-disable-next-line no-console
      console.log("[ROS2Bridge] Foxglove connection closed");
      this._stopTopicsWatcher();
      this.reconnectTimeout = window.setTimeout(() => {
        // eslint-disable-next-line no-console
        console.log("[ROS2Bridge] Attempting to reconnect...");
        this.connect();
      }, 3000);
    };

    this.client.onError = (err) => {
      // eslint-disable-next-line no-console
      console.error("[ROS2Bridge] Foxglove client error", err);
    };

    this.client.onNewTopic = (topic, type) => {
      console.log("new Foxglove topic:", topic, "type:", type);
      this.discoveredTopics.set(topic, type);
    };

    this.client.onNewTopic = (topic, type) => {
      console.log("new Foxglove topic:", topic, "type:", type);
      this.discoveredTopics.set(topic, type);
    };

    this.client.onMessage = (msg) => {
      const ref = {
        topic: msg.topic,
        type: msg.schemaName,
        msg: msg.payload,
      };
      this.handleFoxgloveMessage(ref);
    };
  }

  disconnect() {
    if (this.reconnectTimeout != null) {
      clearTimeout(this.reconnectTimeout);
      this.reconnectTimeout = null;
    }
    this._stopTopicsWatcher();
    try {
      this.client?.close();
    } catch {
      // ignore
    }
    this.client = null;
    this.discoveredTopics.clear();
    // Clear server info on disconnect
    this.serverInfoReady = null;
    this.serverInfoResolver = null;
  }

  /** Update connection settings and reconnect if they changed. */
  updateConnectionSettings(settings: ConnectionSettings) {
    const settingsChanged =
      !this.connectionSettings ||
      this.connectionSettings.proxyUrl !== settings.proxyUrl ||
      this.connectionSettings.vmManagerUrl !== settings.vmManagerUrl ||
      this.connectionSettings.nodeId !== settings.nodeId ||
      this.connectionSettings.token !== settings.token ||
      this.connectionSettings.targetPort !== settings.targetPort;

    if (settingsChanged) {
      // eslint-disable-next-line no-console
      console.log("[ROS2Bridge] Connection settings changed, reconnecting...");
      this.disconnect();
      this.connect("foxglove", undefined, settings);
    }
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
      // eslint-disable-next-line no-console
      console.warn("[ROS2Bridge] Not connected, queueing subscription:", { topic, type });
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
      // eslint-disable-next-line no-console
      console.warn("[ROS2Bridge] publish() ignored: Foxglove client not ready");
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

  async setROSParameter(name: string, value: any): Promise<void> {
    this.client?.setParameter(name, value);
    return Promise.resolve();
  }

  /** Read one ROS 2 parameter value */
  async getROSParameter(name: string, opts?: { force?: boolean; timeoutMs?: number }): Promise<unknown> {
    if (!this.client) throw new Error("getROSParameter() before connect");
    if (!this.client.isConnected()) throw new Error("getROSParameter() while disconnected");
    return await this.client.getParameter(name, opts);
  }

  /** Read many ROS 2 parameters */
  async getROSParameters(names: string[], opts?: { timeoutMs?: number }): Promise<Record<string, unknown>> {
    if (!this.client) throw new Error("getROSParameters() before connect");
    if (!this.client.isConnected()) throw new Error("getROSParameters() while disconnected");
    const map = await this.client.getParameters(names, opts);
    const out: Record<string, unknown> = {};
    for (const [k, v] of map.entries()) out[k] = v;
    return out;
  }

  /** Fetch all params (Foxglove bridge: empty list means all) */
  async getAllROSParameters(opts?: { timeoutMs?: number }): Promise<Record<string, unknown>> {
    console.log('ROS2Bridge: getAllROSParameters called');
    if (!this.serverInfoReady) throw new Error("getAllROSParameters() before connect");

    // Add timeout to prevent infinite hanging when connection fails
    const timeoutMs = opts?.timeoutMs ?? 15000; // 15 second default timeout
    console.log('ROS2Bridge: waiting for serverInfoReady with timeout...', timeoutMs, 'ms');

    try {
      await Promise.race([
        this.serverInfoReady,
        new Promise<never>((_, reject) =>
          setTimeout(() => reject(new Error(`getAllROSParameters timeout after ${timeoutMs}ms - WebSocket connection failed`)), timeoutMs)
        )
      ]);
      console.log('ROS2Bridge: serverInfoReady resolved');
    } catch (timeoutError) {
      console.error('ROS2Bridge: serverInfoReady timeout - connection failed');
      throw timeoutError;
    }

    if (!this.client) throw new Error("getAllROSParameters() before connect");
    if (!this.client.isConnected()) throw new Error("getAllROSParameters() while disconnected");
    console.log('ROS2Bridge: calling client.getAllParameters...');
    const map = await this.client.getAllParameters(opts);
    console.log('ROS2Bridge: client.getAllParameters returned', map.size, 'parameters');
    const out: Record<string, unknown> = {};
    for (const [k, v] of map.entries()) out[k] = v;
    console.log('ROS2Bridge: getAllROSParameters completed');
    return out;
  }

  /** Local-only: list what we've seen so far */
  listCachedROSParameters(): string[] {
    return this.client?.listCachedParameters() ?? [];
  }

  /** Local-only: return cached value if present */
  getCachedROSParameter(name: string): unknown | undefined {
    return this.client?.getCachedParameter(name);
  }

  /** Optional: subscribe to param changes */
  onROSParameterChanged(cb: (changed: { name: string; value: unknown }[]) => void): () => void {
    if (!this.client) return () => {};
    return this.client.onParameterChanged(cb);
  }

  /** Arrange for a service call to run once on every (re)connect before normal ops. */
  registerSetupServiceCall(name: string, request: any) {
    this.setupServiceCalls.push({
      name,
      request,
    });
  }

  /**
   * Set default configs to this.client for the drone.
   * Will use publishSetup and setup ROS params.
   */
  private _configureDefault() {
    this.registerSetupROSParameterSet("/mavros/sys.heartbeat_mav_type", "GCS");
    this.registerSetupROSParameterSet("/mavros/sys.heartbeat_rate", 2.0);
  }

  registerSetupROSParameterSet(name: string, value: any): void {
    this.setupROSParams.push({ name, value });
  }

  isConnected(): boolean {
    return this.client?.isConnected() ?? false;
  }

  getAvailableTopics(): Subscription[] {
    return this.client?.getAvailableTopics() ?? [];
  }

  getTopicType(topic: string): string | undefined {
    return this.client?.getTopicType(topic);
  }

  getAvailableImageTopics(): Subscription[] {
    // A few common image topic guesses for convenience
    return [
      { topic: "/so_arm101/agent_camera/image_raw", type: "sensor_msgs/msg/Image" },
      { topic: "/so_arm101/side_camera/image_raw", type: "sensor_msgs/msg/Image" },
      { topic: "/so_arm101/wrist_camera/image_raw", type: "sensor_msgs/msg/Image" },
      { topic: "/drone_camera/image_raw", type: "sensor_msgs/msg/Image" },
      { topic: "/camera/image_raw", type: "sensor_msgs/msg/Image" },
      { topic: "/camera/image_annotated", type: "sensor_msgs/msg/Image" },
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
    if (!this.serverInfoReady) throw new Error("callService() before connect");
    await this.serverInfoReady;
    if (!this.client) throw new Error("callService() before connect");
    if (typeof (this.client as any).callService !== "function") {
      throw new Error("FoxgloveWsClient.callService() not available");
    }
    return await (this.client as any).callService({
      serviceName: name,
      request: request
    }) as T;
  }

  async waitForService(name: string, timeoutMs = 10_000): Promise<void> {
    if (!this.serverInfoReady) throw new Error("waitForService() before connect");
    await this.serverInfoReady;
    if (!this.client) throw new Error("waitForService() before connect");
    if (typeof this.client.waitForService !== "function") {
      throw new Error("FoxgloveWsClient.waitForService() not available");
    }
    await this.client.waitForService(name, timeoutMs);
  }

  /**
   * Return ROS2 node names via rosapi:
   *   ros2 service call /rosapi/nodes rosapi_msgs/srv/Nodes "{}"
   *
   * Requires /rosapi/nodes to be advertised by Foxglove Bridge (services enabled).
   */
  async listNodes(): Promise<string[]> {
    if (!this.serverInfoReady) throw new Error("listNodes() before connect");
    await this.serverInfoReady;
    if (!this.client) throw new Error("listNodes() before connect");
    if (!this.client.isConnected()) throw new Error("listNodes() while disconnected");

    const resp = await this.client.callService<RosapiNodesResponse>({
      serviceName: "/rosapi/nodes",
      request: {}, // rosapi_msgs/srv/Nodes_Request has no fields
    });

    // Be defensive in case bridge returns a slightly different shape
    const nodes = (resp as any)?.nodes;
    if (!Array.isArray(nodes)) {
      throw new Error(`Unexpected /rosapi/nodes response: ${JSON.stringify(resp)}`);
    }
    return nodes;
  }

  /**
   * Convenience wrapper matching your grep use-case:
   * returns only node names containing `substr`
   */
  async listNodesMatching(substr: string): Promise<string[]> {
    const nodes = await this.listNodes();
    return nodes.filter((n) => n.includes(substr));
  }


  // ---------- RawImage normalization helper ----------

  /**
   * Some producers send foxglove.RawImage with step/row_stride = 0.
   * That blows up Foxglove's decodeRGB8 ("row step (0) must be at least 3*width").
   * Fix it up here by inferring a sane step based on encoding and width.
   */
  private normalizeRawImage(raw: any): any {
    if (!raw || typeof raw !== "object") {
      return raw;
    }

    const width = raw.width;
    const height = raw.height;
    if (typeof width !== "number" || typeof height !== "number") {
      return raw;
    }

    const enc = String(raw.encoding ?? "").toLowerCase();

    let bytesPerPixel = 1;
    if (enc === "rgb8" || enc === "bgr8") {
      bytesPerPixel = 3;
    } else if (enc === "rgba8" || enc === "bgra8") {
      bytesPerPixel = 4;
    } else if (enc === "mono16") {
      bytesPerPixel = 2;
    } else {
      // mono8 or unknown – treat as 1 byte per pixel
      bytesPerPixel = 1;
    }

    const minStep = width * bytesPerPixel;

    let step = raw.step ?? raw.row_stride ?? 0;
    if (typeof step !== "number" || step < minStep) {
      const oldStep = step;
      step = minStep;
      // eslint-disable-next-line no-console
      console.warn(
        "[ROS2Bridge] Normalizing RawImage step",
        { encoding: enc, width, height, oldStep, newStep: step },
      );
    }

    return {
      ...raw,
      step,
      row_stride: raw.row_stride ?? step,
    };
  }

  // ---------- Image conversions ----------

  private handleFoxgloveMessage(data: any) {
    // Expecting something like: { topic, type, msg }
    const topic: string = data.topic;
    const type: string = data.type;
    let msg: any = data.msg;

    // --- fix RawImage-like messages regardless of schemaName ---
    if (msg && typeof msg === "object") {
      const looksLikeRawImage =
        typeof msg.width === "number" &&
        typeof msg.height === "number" &&
        "data" in msg &&
        "encoding" in msg;

      if (looksLikeRawImage) {
        const stepVal = msg.step ?? msg.row_stride ?? 0;
        const enc = String(msg.encoding ?? "").toLowerCase();
        const bytesPerPixel =
          enc === "rgb8" || enc === "bgr8"
            ? 3
            : enc === "rgba8" || enc === "bgra8"
              ? 4
              : enc === "mono16"
                ? 2
                : 1;

        const minStep = msg.width * bytesPerPixel;

        if (typeof stepVal !== "number" || stepVal < minStep) {
          msg = this.normalizeRawImage(msg);
          data = { ...data, msg };
        }
      }
    }

    const header = msg?.header || {};
    const frameId =
      header.frame_id || msg?.frame_id || msg?.child_frame_id || "";

    let timestamp = new Date().toISOString();
    let timestampNanos: number | undefined;

    if (header.stamp) {
      const sec = header.stamp.sec || 0;
      const nanosec = header.stamp.nanosec || header.stamp.nsec || 0;
      timestampNanos = sec * 1_000_000_000 + nanosec;
      timestamp = new Date(sec * 1000 + nanosec / 1_000_000).toISOString();
    }

    if (frameId) {
      this.noteFrame(frameId, topic);
    }

    if (type === "tf2_msgs/msg/TFMessage" && Array.isArray(msg.transforms)) {
      for (const tf of msg.transforms as Array<{
        header?: { frame_id?: string };
        child_frame_id?: string;
      }>) {
        const parent = tf.header?.frame_id;
        const child = tf.child_frame_id;
        if (parent) this.noteFrame(parent, topic);
        if (child) this.noteFrame(child, topic);
      }
    }

    if (type === "sensor_msgs/msg/Image") {
      try {
        const dataURI = this.convertRawImageToDataURI(msg);
        const imageMsg: RosTypes.ImageMessage = {
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
        // eslint-disable-next-line no-console
        console.error("[ROS2Bridge] Failed to convert raw image:", error);
      }
    } else if (type === "sensor_msgs/msg/CompressedImage") {
      try {
        this.convertCompressedImageToDataURI(msg, (dataURI, width, height) => {
          const imageMsg: RosTypes.ImageMessage = {
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
        // eslint-disable-next-line no-console
        console.error("[ROS2Bridge] Failed to load compressed image:", error);
      }
    } else {
      // Non-image (or RawImage) topics: forward patched { topic, type, msg } as-is
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
      throw new Error("Unknown data type for sensor_msgs/Image.data");
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
          rgba[i * 4] = data[i * 3];     // R
          rgba[i * 4 + 1] = data[i * 3 + 1]; // G
          rgba[i * 4 + 2] = data[i * 3 + 2]; // B
          rgba[i * 4 + 3] = 255;         // A
        }
        break;

      case "rgba8":
        rgba.set(data.slice(0, pixelCount * 4));
        break;

      case "bgr8":
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4] = data[i * 3 + 2];     // R (from B)
          rgba[i * 4 + 1] = data[i * 3 + 1]; // G
          rgba[i * 4 + 2] = data[i * 3];     // B (from R)
          rgba[i * 4 + 3] = 255;             // A
        }
        break;

      case "bgra8":
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4] = data[i * 4 + 2];     // R (from B)
          rgba[i * 4 + 1] = data[i * 4 + 1]; // G
          rgba[i * 4 + 2] = data[i * 4];     // B (from R)
          rgba[i * 4 + 3] = data[i * 4 + 3]; // A
        }
        break;

      case "mono8":
        for (let i = 0; i < pixelCount; i++) {
          const gray = data[i];
          rgba[i * 4] = gray;
          rgba[i * 4 + 1] = gray;
          rgba[i * 4 + 2] = gray;
          rgba[i * 4 + 3] = 255;
        }
        break;

      case "mono16":
        for (let i = 0; i < pixelCount; i++) {
          // Convert 16-bit to 8-bit by taking high byte
          const gray = data[i * 2 + 1];
          rgba[i * 4] = gray;
          rgba[i * 4 + 1] = gray;
          rgba[i * 4 + 2] = gray;
          rgba[i * 4 + 3] = 255;
        }
        break;

      default:
        // eslint-disable-next-line no-console
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

    // rosbridge/Foxglove usually send the data base64-encoded
    const dataURI = `data:${mimeType};base64,${data}`;

    // Load image to get dimensions
    const img = new Image();
    img.onload = () => {
      callback(dataURI, img.width, img.height);
    };
    img.onerror = (error) => {
      // eslint-disable-next-line no-console
      console.error("[ROS2Bridge] Failed to load compressed image:", error);
    };
    img.src = dataURI;
  }

  private noteFrame(frameId: string, topic: string) {
    if (!frameId) return;
    this.frameIds.add(frameId);
    let topics = this.frameTopics.get(frameId);
    if (!topics) {
      topics = new Set();
      this.frameTopics.set(frameId, topics);
    }
    topics.add(topic);
  }

  getKnownFrames(): string[] {
    return Array.from(this.frameIds.values()).sort();
  }

  getFrameSources(frameId: string): string[] {
    const set = this.frameTopics.get(frameId);
    return set ? Array.from(set.values()).sort() : [];
  }

  // ---------------- Available topics change API ----------------

  /**
   * Subscribe to changes in the available topics list.
   * Returns an unsubscribe function.
   */
  onAvailableTopicsChanged(cb: (topics: Subscription[]) => void): () => void {
    this.availableTopicsListeners.add(cb);
    // fire immediately with current list
    try {
      cb(this.getAvailableTopics());
    } catch (e) {
      // eslint-disable-next-line no-console
      console.error("[ROS2Bridge] topicsChanged initial callback error:", e);
    }
    return () => {
      this.availableTopicsListeners.delete(cb);
    };
  }

  private _notifyAvailableTopicsChanged(topics: Subscription[]) {
    for (const fn of this.availableTopicsListeners) {
      try {
        fn(topics);
      } catch (e) {
        // eslint-disable-next-line no-console
        console.error("[ROS2Bridge] topicsChanged listener error:", e);
      }
    }
  }

  private _startTopicsWatcher(intervalMs = 1000) {
    const tick = () => {
      const topics = this.getAvailableTopics() || [];
      const sig = JSON.stringify(
        topics
          .map((t) => `${t.topic}:${t.type}`)
          .sort(),
      );
      if (sig !== this._lastTopicsSig) {
        this._lastTopicsSig = sig || null;
        this._notifyAvailableTopicsChanged(topics);
      }
    };
    if (this.topicsWatchTimer) clearInterval(this.topicsWatchTimer);
    this.topicsWatchTimer = window.setInterval(tick, intervalMs);
    tick(); // initial fire
  }

  private _stopTopicsWatcher() {
    if (this.topicsWatchTimer) {
      clearInterval(this.topicsWatchTimer);
      this.topicsWatchTimer = null;
    }
    this._lastTopicsSig = null;
  }

  private _startSettingsWatcher() {
    if (this.settingsCheckTimer) clearInterval(this.settingsCheckTimer);
    this.settingsCheckTimer = window.setInterval(() => {
      const currentSettings: ConnectionSettings = {
        useProxy: (window as any).TENSORFLEET_USE_PROXY || '',
        proxyUrl: (window as any).TENSORFLEET_PROXY_URL || '',
        vmManagerUrl: (window as any).TENSORFLEET_VM_MANAGER_URL || '',
        nodeId: (window as any).TENSORFLEET_NODE_ID || '',
        token: (window as any).TENSORFLEET_JWT || '',
        targetPort: (window as any).TENSORFLEET_TARGET_PORT || 8765,
      };
      this.updateConnectionSettings(currentSettings);
    }, 1000); // check every second
  }

  private _stopSettingsWatcher() {
    if (this.settingsCheckTimer) {
      clearInterval(this.settingsCheckTimer);
      this.settingsCheckTimer = null;
    }
  }
}

export const ros2Bridge: ROS2BridgeApi = new ROS2Bridge();

// Auto-connect on load
(ros2Bridge as any).connect();
