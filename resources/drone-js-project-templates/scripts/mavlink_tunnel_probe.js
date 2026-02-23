#!/usr/bin/env node

import { execFile } from "node:child_process";
import { readdir } from "node:fs/promises";
import { promisify } from "node:util";

const execFileAsync = promisify(execFile);

function parseArgs(argv) {
  /** @type {Record<string, string>} */
  const args = {};
  for (let i = 0; i < argv.length; i += 1) {
    const token = argv[i];
    if (!token.startsWith("--")) continue;
    const key = token.slice(2);
    const next = argv[i + 1];
    if (next && !next.startsWith("--")) {
      args[key] = next;
      i += 1;
    } else {
      args[key] = "true";
    }
  }
  return args;
}

function toNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function toBoolean(value, fallback = false) {
  if (value === undefined || value === null || value === "") {
    return fallback;
  }
  const normalized = String(value).trim().toLowerCase();
  if (["1", "true", "yes", "y", "on"].includes(normalized)) {
    return true;
  }
  if (["0", "false", "no", "n", "off"].includes(normalized)) {
    return false;
  }
  return fallback;
}

function usage() {
  console.log(
    [
      "Usage:",
      "  node scripts/mavlink_tunnel_probe.js \\",
      "    [--serial-path /dev/ttyACM0] \\",
      "    [--baud-rate 57600|115200] \\",
      "    [--target-port 14600] \\",
      "    [--duration-seconds 10] \\",
      "    [--require-ws-rx false]",
      "",
      "Default baud is inferred from serial path when omitted:",
      "  - /dev/ttyUSB* (telemetry radios) -> 57600",
      "  - otherwise -> 115200",
      "",
      "Connection fields (vm-manager-url, token, node-id) are auto-loaded",
      "from src/lib/tensorfleet_config.js by default.",
    ].join("\n")
  );
}

async function loadTensorfleetSettings() {
  // tensorfleet_config.js in this template is currently CommonJS-style.
  // It runs cleanly in Bun, so we use Bun as a tiny settings resolver and
  // keep the probe itself on Node for serialport compatibility.
  try {
    const evalCode =
      "import { getTensorfleetSettings } from './src/lib/tensorfleet_config.js'; " +
      "console.log(JSON.stringify(getTensorfleetSettings()));";
    const { stdout } = await execFileAsync("bun", ["-e", evalCode], {
      cwd: process.cwd(),
      env: { ...process.env },
    });

    const lines = String(stdout || "")
      .split(/\r?\n/)
      .map((line) => line.trim())
      .filter(Boolean);
    const jsonLine = lines[lines.length - 1] || "{}";
    const parsed = JSON.parse(jsonLine);
    return parsed && typeof parsed === "object" ? parsed : {};
  } catch (error) {
    const text = error instanceof Error ? error.message : String(error);
    console.warn(`[MAVLINK][WARN] Failed to load tensorfleet_config.js via Bun (${text})`);
  }
  return {};
}

async function detectSerialPath() {
  if (process.platform !== "linux") {
    return "";
  }

  try {
    const entries = await readdir("/dev");
    const candidates = entries
      .filter((entry) => /^tty(ACM|USB)\d+$/.test(entry))
      .map((entry) => `/dev/${entry}`)
      .sort();
    return candidates[0] || "";
  } catch {
    return "";
  }
}

function isLikelyTelemetryRadioSerialPath(serialPath) {
  const normalized = String(serialPath || "").trim().toLowerCase();
  if (!normalized) {
    return false;
  }
  if (/\/dev\/ttyusb\d+/.test(normalized)) {
    return true;
  }
  if (normalized.includes("/dev/cu.usbserial") || normalized.includes("/dev/tty.usbserial")) {
    return true;
  }
  return normalized.includes("telemetry") || normalized.includes("radio") || normalized.includes("sik");
}

function suggestBaudRate(serialPath) {
  return isLikelyTelemetryRadioSerialPath(serialPath) ? 57600 : 115200;
}

async function main() {
  if (typeof Bun !== "undefined") {
    console.error(
      "[MAVLINK][FATAL] Run this probe with Node (not Bun) due serialport NAPI incompatibility in Bun."
    );
    console.error(
      "[MAVLINK][FATAL] Example: node scripts/mavlink_tunnel_probe.js --duration-seconds 0"
    );
    process.exit(2);
  }

  const args = parseArgs(process.argv.slice(2));
  const settings = await loadTensorfleetSettings();
  const autoSerialPath = await detectSerialPath();

  const serialPath = args["serial-path"] || autoSerialPath || "";
  const baudRate = toNumber(args["baud-rate"], suggestBaudRate(serialPath));
  const vmManagerUrl = args["vm-manager-url"] || settings.vmManagerUrl || settings.baseUrl || "";
  const token = args.token || settings.token || "";
  const nodeId = args["node-id"] || settings.nodeId || "";
  const targetPort = toNumber(args["target-port"], 14600);
  const durationSeconds = toNumber(args["duration-seconds"], 10);
  const requireWsRx = toBoolean(args["require-ws-rx"], false);

  if (!serialPath || !vmManagerUrl || !token || !nodeId || !Number.isFinite(targetPort)) {
    usage();
    if (!serialPath) {
      console.error("[MAVLINK][ERROR] serial path missing (provide --serial-path or attach /dev/ttyACM*/ttyUSB*)");
    }
    if (!vmManagerUrl) {
      console.error("[MAVLINK][ERROR] vmManagerUrl missing (set in .env/.tensorfleet via tensorfleet_config.js)");
    }
    if (!token) {
      console.error("[MAVLINK][ERROR] token missing (TENSORFLEET_JWT)");
    }
    if (!nodeId) {
      console.error("[MAVLINK][ERROR] nodeId missing (TENSORFLEET_NODE_ID / .tensorfleet env.nodeId)");
    }
    process.exit(2);
  }

  const { createSerialMavlinkBridge } = await import("../src/lib/serial_mavlink_bridge.js");

  const bridge = createSerialMavlinkBridge({
    serialPath,
    baudRate,
    vmManagerUrl,
    token,
    nodeId,
    targetPort,
  });

  bridge.on("status", (msg) => {
    console.log(`[MAVLINK][STATUS] ${msg}`);
  });

  bridge.on("error", (err) => {
    const text = err instanceof Error ? err.message : String(err);
    console.error(`[MAVLINK][ERROR] ${text}`);
  });

  bridge.start();
  console.log(
    `[MAVLINK][CONFIG] serial=${serialPath} baud=${baudRate} target_port=${targetPort} duration_seconds=${durationSeconds} require_ws_rx=${String(
      requireWsRx
    )} vm_manager=${vmManagerUrl} node_id=${nodeId}`
  );

  let previous = bridge.getStats();
  const ticker = setInterval(() => {
    const current = bridge.getStats();
    const delta = {
      serialIn: current.bytes_in_serial - previous.bytes_in_serial,
      serialOut: current.bytes_out_serial - previous.bytes_out_serial,
      wsIn: current.bytes_in_ws - previous.bytes_in_ws,
      wsOut: current.bytes_out_ws - previous.bytes_out_ws,
    };
    previous = current;

    console.log(
      `[MAVLINK][BPS] serial_in=${delta.serialIn} serial_out=${delta.serialOut} ws_in=${delta.wsIn} ws_out=${delta.wsOut}`
    );
    console.log(
      `[MAVLINK][TOTAL] serial_in=${current.bytes_in_serial} serial_out=${current.bytes_out_serial} ws_in=${current.bytes_in_ws} ws_out=${current.bytes_out_ws}`
    );
  }, 1000);

  const shutdown = async (exitCode) => {
    clearInterval(ticker);
    await bridge.stop();
    process.exit(exitCode);
  };

  process.on("SIGINT", () => {
    void shutdown(130);
  });
  process.on("SIGTERM", () => {
    void shutdown(143);
  });

  if (durationSeconds > 0) {
    setTimeout(async () => {
      const finalStats = bridge.getStats();
      const success = finalStats.bytes_out_ws > 0 && (!requireWsRx || finalStats.bytes_in_ws > 0);
      const hasRoundTrip = finalStats.bytes_in_ws > 0;

      if (success) {
        if (hasRoundTrip) {
          console.log("[MAVLINK][RESULT] success: ws tx/rx observed");
        } else {
          console.log("[MAVLINK][RESULT] success: ws tx observed (rx not yet observed)");
        }
      } else {
        if (finalStats.bytes_out_ws <= 0) {
          console.error("[MAVLINK][RESULT] failure: no serial->ws bytes observed");
        } else if (requireWsRx && finalStats.bytes_in_ws <= 0) {
          console.error("[MAVLINK][RESULT] failure: ws rx required but not observed");
        } else {
          console.error("[MAVLINK][RESULT] failure");
        }
      }

      await shutdown(success ? 0 : 1);
    }, durationSeconds * 1000);
  }
}

main().catch((err) => {
  console.error(`[MAVLINK][FATAL] ${err instanceof Error ? err.stack || err.message : String(err)}`);
  process.exit(1);
});
