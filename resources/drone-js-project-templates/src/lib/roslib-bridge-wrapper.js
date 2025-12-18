/**
 * ROS2BridgeApi wrapper that uses ROSLIB internally.
 * Implements the ROS2BridgeApi interface using the exact connection mechanism from drone_utils.js
 */

const ROSLIB = require("roslib");
const { getTensorfleetSettings } = require("./tensorfleet_config");
const { createProxyWebSocket } = require("./proxy_ws_client");
const socketAdapter = require("roslib/src/core/SocketAdapter.js");

class ROSLibBridgeWrapper {
  constructor() {
    this.ros = null;
    this.isConnectedFlag = false;

    this.subscriptions = new Map();
    this.messageHandlers = new Map();
    this.topicsChangedHandlers = new Set();

    this.setupPublishes = [];
    this.setupServiceCalls = [];
    this.setupROSParams = [];

    this._publisherCache = new Map();
    this._topicTypesCache = new Map();
    this._resubscribeTimer = null;

    this._knownFrames = new Map(); // child -> { parent, sources:Set<string> }
    this._tfSubscribed = false;

    this._mavrosParamsTimer = null;
    this._mavrosParamsSet = false;

    this.connect();
  }

  connect() {
    console.log("[ROS] Connected")
    const settings = getTensorfleetSettings();

    const useProxy = settings.useProxy && settings.proxyUrl;
    let ros;

    if (useProxy) {
      const proxyWs = createProxyWebSocket({
        proxyUrl: settings.proxyUrl,
        vmManagerUrl: settings.vmManagerUrl,
        token: settings.token,
        nodeId: settings.nodeId,
        targetPort: settings.targetPort
      });

      ros = new ROSLIB.Ros({});
      this.attachProxySocket(ros, proxyWs);
    } else {
      const directUrl = settings.rosbridgeUrl || "ws://127.0.0.1:9091";
      ros = new ROSLIB.Ros({ url: directUrl });
    }

    this.ros = ros;
    this._rebuildAllTopics();
    this._publisherCache.clear();
    this.setupConnectionHandlers();
  }

  setupConnectionHandlers() {
    if (!this.ros) return;

    this.ros.on("connection", async () => {
      console.log('[ROS] Ros "connection" event');
      this.isConnectedFlag = true;

      this._startMavrosHeartbeatParamSetter();

      await this.performSetupOperations();

      await this._refreshTopicsAndTypes();
      this._notifyTopicsChanged();

      this._ensureTFSubscriptions();
      this._rebuildAllTopics();
    });

    this.ros.on("error", () => {
      this.isConnectedFlag = false;
    });

    this.ros.on("close", () => {
      this.isConnectedFlag = false;

      if (this._resubscribeTimer) {
        clearInterval(this._resubscribeTimer);
        this._resubscribeTimer = null;
      }

      if (this._mavrosParamsTimer) {
        clearInterval(this._mavrosParamsTimer);
        this._mavrosParamsTimer = null;
      }

      this._mavrosParamsSet = false;
      this._tfSubscribed = false;
    });
  }

  async performSetupOperations() {
    for (const { name, value } of this.setupROSParams) {
      await this.setParam(name, value);
    }

    for (const { name, request } of this.setupServiceCalls) {
      try {
        await this.callService(name, request);
      } catch (error) {
        console.error(`Failed to call service ${name}:`, error);
      }
    }

    for (const { topic, type, message } of this.setupPublishes) {
      this.publish(topic, type, message);
    }
  }

  _startMavrosHeartbeatParamSetter() {
    if (this._mavrosParamsTimer) {
      clearInterval(this._mavrosParamsTimer);
      this._mavrosParamsTimer = null;
    }

    this._mavrosParamsSet = false;

    const attempt = async () => {
      if (this._mavrosParamsSet) return;
      if (!this.isConnected() || !this.ros) return;

      try {
        await this.setParam("/mavros/sys.heartbeat_mav_type", "GCS");
        await this.setParam("/mavros/sys.heartbeat_rate", 2.0);

        this._mavrosParamsSet = true;

        if (this._mavrosParamsTimer) {
          clearInterval(this._mavrosParamsTimer);
          this._mavrosParamsTimer = null;
        }
      } catch (_) {}
    };

    attempt();
    this._mavrosParamsTimer = setInterval(attempt, 500);
  }

  attachProxySocket(ros, proxyWs) {
    ros.socket = Object.assign(proxyWs, socketAdapter(ros));
  }

  isConnected() {
    return this.isConnectedFlag;
  }

  subscribe(subscription, handler) {
    console.log("[ROS] Subscribing to ", subscription);
    const { topic, type } = subscription;

    if (!this.ros) throw new Error("ROS not connected");

    let handlers = this.messageHandlers.get(topic);
    if (!handlers) {
      handlers = new Set();
      this.messageHandlers.set(topic, handlers);
    }
    handlers.add(handler);

    let rosTopic = this.subscriptions.get(topic);
    if (!rosTopic) {
      rosTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: topic,
        messageType: type,
      });

      rosTopic.subscribe((message) => {
        const hs = this.messageHandlers.get(topic);
        if (!hs) return;
        for (const h of hs) {
          try {
            h(message);
          } catch (e) {
            console.error(e);
          }
        }
      });

      this.subscriptions.set(topic, rosTopic);
    }

    return () => this.unsubscribe(topic, handler);
  }

  unsubscribe(topic, handler) {
    const handlers = this.messageHandlers.get(topic);
    if (handlers) {
      handlers.delete(handler);
      if (handlers.size === 0) {
        this.messageHandlers.delete(topic);
        const rosTopic = this.subscriptions.get(topic);
        if (rosTopic) {
          try {
            rosTopic.unsubscribe();
          } catch (_) {}
          this.subscriptions.delete(topic);
        }
      }
    }
  }

  publish(topic, messageType, message) {
    if (!this.ros) {
      return;
    }

    const key = `${topic}::${messageType}`;
    let rosTopic = this._publisherCache.get(key);
    if (!rosTopic) {
      rosTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: topic,
        messageType
      });
      this._publisherCache.set(key, rosTopic);
    }

    rosTopic.publish(new ROSLIB.Message(message));
  }

  publishSetup(topic, type, message) {
    this.setupPublishes.push({ topic, type, message });
    if (this.isConnected()) {
      this.publish(topic, type, message);
    }
  }

  registerSetupServiceCall(name, request) {
    this.setupServiceCalls.push({ name, request });
  }

  registerSetupROSParameterSet(name, value) {
    this.setupROSParams.push({ name, value });
  }

  async getAvailableTopics() {
    if (!this.ros) return [];
    const res = await this._refreshTopicsAndTypes();
    if (res && Array.isArray(res.topics) && Array.isArray(res.types)) {
      return res.topics.map((t, i) => ({ topic: t, type: res.types[i] }));
    }
    if (Array.isArray(res?.topics)) {
      return res.topics.map((t) => ({ topic: t, type: this._topicTypesCache.get(t) || "" }));
    }
    if (Array.isArray(res)) {
      return res.map((t) => ({ topic: t, type: this._topicTypesCache.get(t) || "" }));
    }
    return [];
  }

  async getTopicType(topic) {
    if (!topic) return undefined;
    if (this._topicTypesCache.has(topic)) return this._topicTypesCache.get(topic);
    const res = await this._getTopicsAndTypesSafe();
    if (res && Array.isArray(res.topics) && Array.isArray(res.types)) {
      for (let i = 0; i < res.topics.length; i++) {
        this._topicTypesCache.set(res.topics[i], res.types[i]);
      }
      return this._topicTypesCache.get(topic);
    }
    return undefined;
  }

  async getAvailableImageTopics() {
    const topics = await this.getAvailableTopics();
    return topics.filter((t) => {
      const ty = (t.type || "").toLowerCase();
      return ty === "sensor_msgs/image" ||
        ty === "sensor_msgs/msg/image" ||
        ty === "sensor_msgs/compressedimage" ||
        ty === "sensor_msgs/msg/compressedimage";
    });
  }

  async _getTopicsAndTypesSafe() {
    if (!this.ros) return null;

    return new Promise((resolve, reject) => {
      if (typeof this.ros.getTopicsAndTypes === "function") {
        this.ros.getTopicsAndTypes(
          (result) => resolve(result),
          (err) => reject(err)
        );
        return;
      }

      if (typeof this.ros.getTopics === "function") {
        this.ros.getTopics(
          (result) => resolve(result),
          (err) => reject(err)
        );
        return;
      }

      reject(new Error("Topics API not available"));
    });
  }

  async _refreshTopicsAndTypes() {
    const res = await this._getTopicsAndTypesSafe();
    if (res && Array.isArray(res.topics) && Array.isArray(res.types)) {
      this._topicTypesCache.clear();
      for (let i = 0; i < res.topics.length; i++) {
        this._topicTypesCache.set(res.topics[i], res.types[i]);
      }
    }
    return res;
  }

  async _notifyTopicsChanged() {
    let topics = [];
    try {
      topics = await this.getAvailableTopics();
    } catch (_) {}

    for (const cb of this.topicsChangedHandlers) {
      try {
        cb(topics);
      } catch (e) {
        console.error(e);
      }
    }
  }

  _rebuildAllTopics() {
    console.log("[ROS] Rebuilding all topic subscriptions");
    if (!this.ros) return;

    this._refreshTopicsAndTypes()
      .then(() => this._notifyTopicsChanged())
      .catch(() => {});

    for (const [topic, rosTopic] of this.subscriptions.entries()) {
      if (!rosTopic || rosTopic.ros !== this.ros) {
        this.subscriptions.delete(topic);
      }
    }

    for (const [topic, handlers] of this.messageHandlers.entries()) {
      if (!handlers || handlers.size === 0) continue;

      if (!this.subscriptions.has(topic)) {
        const messageType = this._topicTypesCache.get(topic);
        if (!messageType) continue;

        const rosTopic = new ROSLIB.Topic({
          ros: this.ros,
          name: topic,
          messageType
        });

        console.log("[ROS] Rebuilding subscription for ", topic);

        rosTopic.subscribe((message) => {
          const hs = this.messageHandlers.get(topic);
          if (!hs || hs.size === 0) return;
          for (const h of hs) {
            try {
              h(message);
            } catch (e) {
              console.error(e);
            }
          }
        });

        this.subscriptions.set(topic, rosTopic);
      }
    }
  }

  async callService(name, request, timeout = 5000, serviceType = "std_srvs/Trigger") {
    if (!this.ros) {
      throw new Error("ROS not connected");
    }

    const service = new ROSLIB.Service({
      ros: this.ros,
      name,
      serviceType
    });

    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error("Service call timeout")), timeout);

      service.callService(
        new ROSLIB.ServiceRequest(request),
        (result) => {
          clearTimeout(timer);
          resolve(result);
        },
        (err) => {
          clearTimeout(timer);
          reject(err);
        }
      );
    });
  }

  async setParam(name, value) {
    if (!this.ros) throw new Error("ROS not connected");
    return new Promise((resolve, reject) => {
      const param = new ROSLIB.Param({ ros: this.ros, name });
      param.set(value, resolve, reject);
    });
  }

  async setROSParameter(name, value) {
    return this.setParam(name, value);
  }

  _ensureTFSubscriptions() {
    if (!this.ros) return;
    if (this._tfSubscribed) return;

    const tfTypeCandidates = ["tf2_msgs/TFMessage", "tf2_msgs/msg/TFMessage"];
    const trySubscribe = (topicName) => {
      const cachedType = this._topicTypesCache.get(topicName);
      const messageType = cachedType || tfTypeCandidates[0];

      const t = new ROSLIB.Topic({
        ros: this.ros,
        name: topicName,
        messageType
      });

      t.subscribe((msg) => {
        const transforms = msg && Array.isArray(msg.transforms) ? msg.transforms : [];
        for (const tr of transforms) {
          const parent = (tr?.header?.frame_id || "").replace(/^\//, "");
          const child = (tr?.child_frame_id || "").replace(/^\//, "");
          if (!parent || !child) continue;

          let entry = this._knownFrames.get(child);
          if (!entry) {
            entry = { parent, sources: new Set() };
            this._knownFrames.set(child, entry);
          } else {
            entry.parent = parent;
          }
          entry.sources.add(topicName);
        }
      });

      return t;
    };

    try {
      trySubscribe("/tf");
      trySubscribe("/tf_static");
      this._tfSubscribed = true;
    } catch (e) {
      console.error(e);
    }
  }

  getKnownFrames() {
    const frames = new Set();
    for (const [child, entry] of this._knownFrames.entries()) {
      frames.add(child);
      if (entry && entry.parent) frames.add(entry.parent);
    }
    return Array.from(frames);
  }

  getFrameSources(frameId) {
    const key = (frameId || "").replace(/^\//, "");
    if (!key) return [];
    const entry = this._knownFrames.get(key);
    if (!entry) return [];
    return Array.from(entry.sources || []);
  }

  onAvailableTopicsChanged(cb) {
    this.topicsChangedHandlers.add(cb);
    return () => {
      this.topicsChangedHandlers.delete(cb);
    };
  }

  async waitForConnection(timeoutMs = 10000) {
    if (this.isConnected()) return;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error("Connection timeout")), timeoutMs);
      const check = () => {
        if (this.isConnected()) {
          clearTimeout(timer);
          resolve();
        } else {
          setTimeout(check, 100);
        }
      };
      check();
    });
  }
}

module.exports = {
  ROSLibBridgeWrapper
};
