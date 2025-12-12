/**
 * Minimal WebSocket proxy client for Node.js scripts.
 *
 * Connects to the TensorFleet VM Manager WebSocket proxy and performs a
 * login handshake, then exposes a WebSocket-like object compatible with roslib.
 */

const WebSocket = require("ws");
const { toProxyWebSocketUrl } = require("./url_utils");

/**
 * Create a proxy WebSocket that routes traffic through the VM Manager.
 *
 * config: {
 *   proxyUrl?: string;    // ws(s)://.../ws (derived from vmManagerUrl if omitted)
 *   vmManagerUrl?: string; // Optional base to derive proxyUrl when proxyUrl is empty
 *   token: string;        // JWT token
 *   nodeId: string;       // Target VM/node identifier
 *   targetPort: number;   // Port inside VM (9091 for rosbridge)
 * }
 */

function createProxyWebSocket(config) {
  const { proxyUrl, token, nodeId, targetPort, vmManagerUrl } = config;
  const resolvedProxyUrl = proxyUrl || toProxyWebSocketUrl(vmManagerUrl);

  if (!resolvedProxyUrl) {
    throw new Error("Missing proxyUrl for VM Manager WebSocket proxy");
  }

  const ws = new WebSocket(resolvedProxyUrl);
  let state = "connecting"; // connecting | authenticating | connected | error | closed
  const queue = [];

  const wrapper = {
    binaryType: "arraybuffer",
    readyState: WebSocket.CONNECTING,

    // Event handlers assigned by roslib
    onopen: null,
    onmessage: null,
    onclose: null,
    onerror: null,

    // WebSocket-like send API
    send(data) {
      if (state !== "connected" || ws.readyState !== WebSocket.OPEN) {
        queue.push(data);
        return;
      }
      ws.send(data);
    },

    close(code, reason) {
      try {
        ws.close(code || 1000, reason);
      } catch {
        // ignore
      }
    },

    // Read-only properties/consts for compatibility
    CONNECTING: WebSocket.CONNECTING,
    OPEN: WebSocket.OPEN,
    CLOSING: WebSocket.CLOSING,
    CLOSED: WebSocket.CLOSED,
    bufferedAmount: 0,
    extensions: "",
    protocol: "",
    url: resolvedProxyUrl,
  };

  function flushQueue() {
    while (queue.length > 0 && ws.readyState === WebSocket.OPEN) {
      const msg = queue.shift();
      ws.send(msg);
    }
  }

  ws.on("open", () => {
    state = "authenticating";
    wrapper.readyState = WebSocket.CONNECTING;

    const loginMsg = {
      type: "login",
      token,
      nodeId,
      targetPort,
    };

    try {
      ws.send(JSON.stringify(loginMsg));
    } catch (err) {
      state = "error";
      wrapper.readyState = WebSocket.CLOSED;
      if (typeof wrapper.onerror === "function") {
        wrapper.onerror(err);
      }
      ws.close();
    }
  });

  ws.on("message", (data) => {
    if (state === "authenticating") {
      let text;
      try {
        text = typeof data === "string" ? data : data.toString("utf8");
        const msg = JSON.parse(text);
        if (msg && msg.type === "loginResponse") {
          if (msg.success) {
            state = "connected";
            wrapper.readyState = WebSocket.OPEN;
            flushQueue();
            if (typeof wrapper.onopen === "function") {
              wrapper.onopen({});
            }
          } else {
            state = "error";
            wrapper.readyState = WebSocket.CLOSED;
            const err = new Error(msg.message || "VM Manager proxy login failed");
            if (typeof wrapper.onerror === "function") {
              wrapper.onerror(err);
            }
            ws.close(4001, "Auth failed");
          }
          return;
        }
      } catch {
        // Non-JSON or unexpected data while authenticating; treat as error.
        state = "error";
        wrapper.readyState = WebSocket.CLOSED;
        if (typeof wrapper.onerror === "function") {
          wrapper.onerror(
            new Error("Unexpected message while authenticating with VM Manager proxy")
          );
        }
        ws.close(4002, "Unexpected auth response");
        return;
      }

      // Ignore non-login messages while authenticating
      return;
    }

    // Forward normal messages to consumer
    if (typeof wrapper.onmessage === "function") {
      const text =
        typeof data === "string" ? data : data.toString("utf8");
      wrapper.onmessage({ data: text });
    }
  });

  ws.on("close", (code, reason) => {
    state = "closed";
    wrapper.readyState = WebSocket.CLOSED;
    if (typeof wrapper.onclose === "function") {
      wrapper.onclose({
        code,
        reason: typeof reason === "string" ? reason : String(reason || ""),
      });
    }
  });

  ws.on("error", (err) => {
    state = "error";
    wrapper.readyState = WebSocket.CLOSED;
    if (typeof wrapper.onerror === "function") {
      wrapper.onerror(err);
    }
  });

  return wrapper;
}

module.exports = {
  createProxyWebSocket,
};
