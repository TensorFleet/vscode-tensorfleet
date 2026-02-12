/**
 * MAVLink WebSocket proxy client.
 *
 * Connects to TensorFleet VM Manager proxy, performs login handshake, and then
 * forwards raw binary MAVLink frames.
 */

import WebSocket from "ws";

function toProxyWebSocketUrl(vmManagerUrl) {
  if (!vmManagerUrl) return "";
  try {
    const url = new URL(vmManagerUrl);
    if (url.protocol === "ws:" || url.protocol === "wss:") {
      if (!url.pathname || url.pathname === "/") {
        url.pathname = "/ws";
      }
      return url.toString();
    }
    const protocol = url.protocol === "https:" ? "wss:" : "ws:";
    const basePath = url.pathname.replace(/\/$/, "");
    const pathName = basePath.endsWith("/ws") ? basePath : `${basePath}/ws`;
    return `${protocol}//${url.host}${pathName}`;
  } catch {
    return "";
  }
}

function toBuffer(data) {
  if (Buffer.isBuffer(data)) {
    return data;
  }
  if (data instanceof ArrayBuffer) {
    return Buffer.from(data);
  }
  if (ArrayBuffer.isView(data)) {
    return Buffer.from(data.buffer, data.byteOffset, data.byteLength);
  }
  if (typeof data === "string") {
    return Buffer.from(data, "utf8");
  }
  throw new Error("Unsupported WebSocket message data type");
}

/**
 * @param {{
 *   proxyUrl?: string;
 *   vmManagerUrl?: string;
 *   token: string;
 *   nodeId: string;
 *   targetPort: number;
 * }} config
 */
export function createMavlinkWsClient(config) {
  const { proxyUrl, vmManagerUrl, token, nodeId } = config;
  const targetPort = Number(config.targetPort);
  const resolvedProxyUrl = proxyUrl || toProxyWebSocketUrl(vmManagerUrl);

  if (!resolvedProxyUrl) {
    throw new Error("Missing proxyUrl/vmManagerUrl for MAVLink WebSocket client");
  }
  if (!token) {
    throw new Error("Missing token for MAVLink WebSocket client");
  }
  if (!nodeId) {
    throw new Error("Missing nodeId for MAVLink WebSocket client");
  }
  if (!Number.isFinite(targetPort) || targetPort <= 0) {
    throw new Error("Invalid targetPort for MAVLink WebSocket client");
  }

  /** @type {WebSocket | null} */
  let ws = null;
  /** @type {"idle" | "connecting" | "authenticating" | "connected" | "closed" | "error"} */
  let state = "idle";
  /** @type {Promise<void> | null} */
  let connectPromise = null;
  /** @type {Set<(data: Buffer) => void>} */
  const dataHandlers = new Set();
  /** @type {Set<(info: { code: number; reason: string }) => void>} */
  const closeHandlers = new Set();

  const emitData = (buffer) => {
    for (const handler of dataHandlers) {
      try {
        handler(buffer);
      } catch (_) {}
    }
  };

  const emitClose = (info) => {
    for (const handler of closeHandlers) {
      try {
        handler(info);
      } catch (_) {}
    }
  };

  const connect = async () => {
    if (state === "connected" && ws?.readyState === WebSocket.OPEN) {
      return;
    }
    if (connectPromise) {
      return connectPromise;
    }

    connectPromise = new Promise((resolve, reject) => {
      let settled = false;
      state = "connecting";
      ws = new WebSocket(resolvedProxyUrl);
      ws.binaryType = "arraybuffer";

      const fail = (err) => {
        if (settled) return;
        settled = true;
        state = "error";
        connectPromise = null;
        reject(err instanceof Error ? err : new Error(String(err)));
      };

      ws.on("open", () => {
        state = "authenticating";
        try {
          ws?.send(
            JSON.stringify({
              type: "login",
              token,
              nodeId,
              targetPort,
            })
          );
        } catch (err) {
          fail(err);
        }
      });

      ws.on("message", (rawData) => {
        if (state === "authenticating") {
          try {
            const msg = JSON.parse(toBuffer(rawData).toString("utf8"));
            if (msg?.type === "loginResponse" && msg?.success === true) {
              state = "connected";
              if (!settled) {
                settled = true;
                connectPromise = null;
                resolve();
              }
              return;
            }
            if (msg?.type === "loginResponse" && msg?.success === false) {
              fail(new Error(msg.message || "MAVLink proxy login failed"));
              try {
                ws?.close(4001, "Auth failed");
              } catch (_) {}
              return;
            }
            fail(new Error("Unexpected authentication response from MAVLink proxy"));
          } catch (err) {
            fail(err);
          }
          return;
        }

        if (state !== "connected") {
          return;
        }

        try {
          emitData(toBuffer(rawData));
        } catch (_) {}
      });

      ws.on("close", (code, reason) => {
        const reasonText =
          typeof reason === "string" ? reason : Buffer.from(reason || "").toString("utf8");
        const wasAuthenticated = state === "connected";
        state = "closed";
        connectPromise = null;
        if (!settled && !wasAuthenticated) {
          fail(
            new Error(
              `MAVLink proxy closed during authentication (code=${code}, reason=${reasonText})`
            )
          );
          return;
        }
        emitClose({ code, reason: reasonText });
      });

      ws.on("error", (err) => {
        if (!settled && state !== "connected") {
          fail(err);
        }
      });
    });

    return connectPromise;
  };

  const send = (buffer) => {
    if (!Buffer.isBuffer(buffer)) {
      throw new TypeError("MAVLink client send() expects Buffer");
    }
    if (state !== "connected" || !ws || ws.readyState !== WebSocket.OPEN) {
      throw new Error("MAVLink client is not connected");
    }
    ws.send(buffer);
  };

  const onData = (callback) => {
    dataHandlers.add(callback);
    return () => dataHandlers.delete(callback);
  };

  const onClose = (callback) => {
    closeHandlers.add(callback);
    return () => closeHandlers.delete(callback);
  };

  const close = () => {
    state = "closed";
    connectPromise = null;
    if (ws) {
      try {
        ws.close(1000, "Client closed");
      } catch (_) {}
      ws = null;
    }
  };

  return {
    connect,
    send,
    onData,
    onClose,
    close,
    isConnected: () => state === "connected" && ws?.readyState === WebSocket.OPEN,
    getProxyUrl: () => resolvedProxyUrl,
  };
}
