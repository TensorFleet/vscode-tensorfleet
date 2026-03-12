/**
 * Serial <-> MAVLink WebSocket bridge.
 *
 * Bridges local serial bytes to VM Manager proxied WebSocket bytes, with
 * automatic reconnect logic for both legs.
 */

import { EventEmitter } from "node:events";
import { SerialPort } from "serialport";
import { createMavlinkWsClient } from "./mavlink_ws_client.js";

/**
 * @typedef {{
 *   serialPath: string;
 *   baudRate: number;
 *   token: string;
 *   nodeId: string;
 *   targetPort: number;
 *   vmManagerUrl: string;
 *   proxyUrl?: string;
 *   reconnectIntervalMs?: number;
 * }} SerialMavlinkBridgeOptions
 */

/**
 * @typedef {{
 *   bytes_in_serial: number;
 *   bytes_out_serial: number;
 *   bytes_in_ws: number;
 *   bytes_out_ws: number;
 * }} BridgeStats
 */

export class SerialMavlinkBridge extends EventEmitter {
  /**
   * @param {SerialMavlinkBridgeOptions} options
   */
  constructor(options) {
    super();
    if (!options?.serialPath) {
      throw new Error("serialPath is required");
    }

    this.options = {
      ...options,
      baudRate: Number(options.baudRate) || 115200,
      targetPort: Number(options.targetPort),
      reconnectIntervalMs: Number(options.reconnectIntervalMs) || 3000,
    };

    if (!Number.isFinite(this.options.targetPort) || this.options.targetPort <= 0) {
      throw new Error("targetPort must be a positive number");
    }

    /** @type {SerialPort | null} */
    this.serialPort = null;
    this.serialConnecting = false;

    /** @type {ReturnType<typeof createMavlinkWsClient> | null} */
    this.wsClient = null;
    this.wsConnecting = false;

    /** @type {NodeJS.Timeout | null} */
    this.serialReconnectTimer = null;
    /** @type {NodeJS.Timeout | null} */
    this.wsReconnectTimer = null;

    this.running = false;
    this.stats = {
      bytes_in_serial: 0,
      bytes_out_serial: 0,
      bytes_in_ws: 0,
      bytes_out_ws: 0,
    };
  }

  start() {
    if (this.running) return;
    this.running = true;
    this.emit("status", `Starting bridge on ${this.options.serialPath} @ ${this.options.baudRate}`);
    this.#ensureSerialConnected();
    this.#ensureWsConnected();
  }

  async stop() {
    if (!this.running) return;
    this.running = false;

    if (this.serialReconnectTimer) {
      clearTimeout(this.serialReconnectTimer);
      this.serialReconnectTimer = null;
    }
    if (this.wsReconnectTimer) {
      clearTimeout(this.wsReconnectTimer);
      this.wsReconnectTimer = null;
    }

    if (this.wsClient) {
      try {
        this.wsClient.close();
      } catch (_) {}
      this.wsClient = null;
    }

    if (this.serialPort) {
      const port = this.serialPort;
      this.serialPort = null;
      try {
        await new Promise((resolve) => port.close(() => resolve(null)));
      } catch (_) {}
    }

    this.emit("status", "Bridge stopped");
  }

  /** @returns {BridgeStats} */
  getStats() {
    return { ...this.stats };
  }

  #scheduleSerialReconnect() {
    if (!this.running || this.serialReconnectTimer) return;
    this.serialReconnectTimer = setTimeout(() => {
      this.serialReconnectTimer = null;
      this.#ensureSerialConnected();
    }, this.options.reconnectIntervalMs);
  }

  #scheduleWsReconnect() {
    if (!this.running || this.wsReconnectTimer) return;
    this.wsReconnectTimer = setTimeout(() => {
      this.wsReconnectTimer = null;
      this.#ensureWsConnected();
    }, this.options.reconnectIntervalMs);
  }

  #ensureSerialConnected() {
    if (!this.running || this.serialPort || this.serialConnecting) return;
    this.serialConnecting = true;

    const port = new SerialPort({
      path: this.options.serialPath,
      baudRate: this.options.baudRate,
      autoOpen: false,
    });

    this.serialPort = port;

    port.on("open", () => {
      this.serialConnecting = false;
      this.emit("status", `Serial connected: ${this.options.serialPath}`);
    });

    port.on("data", (chunk) => {
      const buffer = Buffer.from(chunk);
      this.stats.bytes_in_serial += buffer.length;

      if (!this.wsClient || !this.wsClient.isConnected()) {
        return;
      }

      try {
        this.wsClient.send(buffer);
        this.stats.bytes_out_ws += buffer.length;
      } catch (error) {
        this.emit("error", error);
      }
    });

    port.on("error", (error) => {
      this.emit("error", error);
    });

    port.on("close", () => {
      if (this.serialPort === port) {
        this.serialPort = null;
      }
      this.serialConnecting = false;
      this.emit("status", "Serial disconnected; retrying...");
      this.#scheduleSerialReconnect();
    });

    port.open((error) => {
      if (!error) return;
      if (this.serialPort === port) {
        this.serialPort = null;
      }
      this.serialConnecting = false;
      this.emit("error", error);
      this.emit("status", "Serial open failed; retrying...");
      this.#scheduleSerialReconnect();
    });
  }

  #ensureWsConnected() {
    if (!this.running || this.wsClient || this.wsConnecting) return;
    this.wsConnecting = true;

    const client = createMavlinkWsClient({
      proxyUrl: this.options.proxyUrl,
      vmManagerUrl: this.options.vmManagerUrl,
      token: this.options.token,
      nodeId: this.options.nodeId,
      targetPort: this.options.targetPort,
    });

    this.wsClient = client;

    const unsubscribeData = client.onData((buffer) => {
      this.stats.bytes_in_ws += buffer.length;
      if (!this.serialPort || !this.serialPort.isOpen) {
        return;
      }

      this.serialPort.write(buffer, (error) => {
        if (error) {
          this.emit("error", error);
          return;
        }
        this.stats.bytes_out_serial += buffer.length;
      });
    });

    const unsubscribeClose = client.onClose((info) => {
      this.wsConnecting = false;
      if (this.wsClient === client) {
        this.wsClient = null;
      }
      unsubscribeData();
      unsubscribeClose();
      this.emit(
        "status",
        `Proxy disconnected (code=${info.code}${info.reason ? `, reason=${info.reason}` : ""}); retrying...`
      );
      this.#scheduleWsReconnect();
    });

    client
      .connect()
      .then(() => {
        this.wsConnecting = false;
        this.emit("status", `Proxy connected: ${client.getProxyUrl()}`);
      })
      .catch((error) => {
        this.wsConnecting = false;
        if (this.wsClient === client) {
          this.wsClient = null;
        }
        unsubscribeData();
        unsubscribeClose();
        try {
          client.close();
        } catch (_) {}
        this.emit("error", error);
        this.emit("status", "Proxy connect failed; retrying...");
        this.#scheduleWsReconnect();
      });
  }
}

/**
 * @param {SerialMavlinkBridgeOptions} options
 */
export function createSerialMavlinkBridge(options) {
  return new SerialMavlinkBridge(options);
}
