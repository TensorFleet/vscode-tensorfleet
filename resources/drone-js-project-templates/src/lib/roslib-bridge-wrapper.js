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

    this.connect();
  }

  connect() {
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
      const directUrl = settings.rosbridgeUrl || 'ws://127.0.0.1:9091';
      ros = new ROSLIB.Ros({ url: directUrl });
    }

    this.ros = ros;
    this.setupConnectionHandlers();
  }

  setupConnectionHandlers() {
    if (!this.ros) return;

    this.ros.on('connection', () => {
      this.isConnectedFlag = true;
      this.performSetupOperations();
    });

    this.ros.on('error', (error) => {
      this.isConnectedFlag = false;
    });

    this.ros.on('close', () => {
      this.isConnectedFlag = false;
    });
  }

  async performSetupOperations() {
    // Set ROS parameters first
    for (const { name, value } of this.setupROSParams) {
      await this.setParam(name, value);
    }

    // Call services
    for (const { name, request } of this.setupServiceCalls) {
      try {
        await this.callService(name, request);
      } catch (error) {
        console.error(`Failed to call service ${name}:`, error);
      }
    }

    // Publish setup messages
    for (const { topic, type, message } of this.setupPublishes) {
      this.publish(topic, type, message);
    }
  }

  attachProxySocket(ros, proxyWs) {
    ros.socket = Object.assign(proxyWs, socketAdapter(ros));
  }

  isConnected() {
    return this.isConnectedFlag;
  }

  subscribe(subscription, handler) {
    const { topic, type } = subscription;

    if (!this.ros) {
      throw new Error('ROS not connected');
    }

    let rosTopic = this.subscriptions.get(topic);
    if (!rosTopic) {
      rosTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: topic,
        messageType: type,
      });
      this.subscriptions.set(topic, rosTopic);
    }

    let handlers = this.messageHandlers.get(topic);
    if (!handlers) {
      handlers = new Set();
      this.messageHandlers.set(topic, handlers);
    }
    handlers.add(handler);

    rosTopic.subscribe((message) => {
      handler(message);
    });

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
          rosTopic.unsubscribe();
          this.subscriptions.delete(topic);
        }
      }
    }
  }

  publish(topic, messageType, message) {
    if (!this.ros) {
      return;
    }

    const rosTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: topic,
      messageType,
    });

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

  getAvailableTopics() {
    return [];
  }

  getTopicType(topic) {
    return undefined;
  }

  async callService(name, request) {
    if (!this.ros) {
      throw new Error('ROS not connected');
    }

    const service = new ROSLIB.Service({
      ros: this.ros,
      name,
      serviceType: 'std_srvs/Trigger',
    });

    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error("Service call timeout")), 5000);

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

  getKnownFrames() {
    return [];
  }

  getFrameSources(frameId) {
    return [];
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
  ROSLibBridgeWrapper,
};
