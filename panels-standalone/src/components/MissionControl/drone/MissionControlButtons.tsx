import React, { useCallback, useMemo, useState } from "react";

/**
 * MissionControlPanel
 * React UI for emitting mission-control requests via a global CustomEvent.
 *
 * Event contract
 *   name: "app:request"
 *   detail: {
 *     category: "mission_control";
 *     action: "arm" | "disarm" | "takeoff" | "land" | "rtl" | "goto";
 *     payload?: unknown;
 *     requestId: string;
 *     timestamp: number;
 *   }
 *
 * The component only emits requests. It does not perform control actions.
 */

type MissionAction = "arm" | "disarm" | "takeoff" | "land" | "rtl" | "goto";

interface AppRequestDetail<T = any> {
  category: "mission_control";
  action: MissionAction;
  payload?: T;
  requestId: string;
  timestamp: number;
}

const EVENT_NAME = "app:request";

/** Dispatches a global mission-control request event. */
function postMissionRequest<T = any>(action: MissionAction, payload?: T) {
  const requestId =
    (globalThis as any).crypto?.randomUUID?.() ??
    Math.random().toString(36).slice(2);
  const detail: AppRequestDetail<T> = {
    category: "mission_control",
    action,
    payload,
    requestId,
    timestamp: Date.now(),
  };
  window.dispatchEvent(new CustomEvent<AppRequestDetail<T>>(EVENT_NAME, { detail }));
  return detail;
}

const styles: Record<string, React.CSSProperties> = {
  container: {
    maxWidth: 880,
    margin: "0 auto",
    padding: 16,
    fontFamily:
      "ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial",
  },
  card: {
    border: "1px solid #e5e7eb",
    borderRadius: 12,
    boxShadow: "0 1px 2px rgba(0,0,0,0.04)",
    padding: 16,
  },
  header: { fontSize: 18, fontWeight: 600, marginBottom: 12 },
  row: { display: "grid", gridTemplateColumns: "1fr 1fr", gap: 8 },
  col: { display: "flex", gap: 8 },
  label: { fontSize: 12, color: "#6b7280", marginBottom: 4 },
  input: {
    width: "100%",
    height: 40,
    border: "1px solid #e5e7eb",
    borderRadius: 8,
    padding: "0 10px",
    fontSize: 14,
  },
  inputCompact: {
    width: "100%",
    height: 36,
    border: "1px solid #e5e7eb",
    borderRadius: 8,
    padding: "0 10px",
    fontSize: 14,
  },
  button: {
    height: 44,
    padding: "0 14px",
    borderRadius: 10,
    background: "#111827",
    color: "white",
    border: "1px solid #111827",
    cursor: "pointer",
    fontSize: 14,
  },
  buttonSecondary: {
    height: 44,
    padding: "0 14px",
    borderRadius: 10,
    background: "white",
    color: "#111827",
    border: "1px solid #e5e7eb",
    cursor: "pointer",
    fontSize: 14,
  },
  buttonDanger: {
    height: 44,
    padding: "0 14px",
    borderRadius: 10,
    background: "#ef4444",
    color: "white",
    border: "1px solid #ef4444",
    cursor: "pointer",
    fontSize: 14,
  },
  sectionTitle: { fontSize: 14, fontWeight: 600, marginTop: 8 },
  divider: { height: 1, background: "#f3f4f6", margin: "8px 0 12px" },
  right: { display: "flex", justifyContent: "flex-end" },
  toggleWrap: { display: "flex", alignItems: "center", justifyContent: "space-between" },
  help: { fontSize: 12, color: "#6b7280" },
  checkbox: { width: 18, height: 18 },
  debug: {
    background: "#f9fafb",
    border: "1px solid #e5e7eb",
    borderRadius: 10,
    padding: 10,
    fontSize: 12,
    color: "#6b7280",
  },
};

export default function MissionControlPanel() {
  // Auto transition flag; forwarded in mission-style requests.
  const [auto, setAuto] = useState<boolean>(true);

  // Takeoff altitude (meters).
  const [tkAlt, setTkAlt] = useState<string>("2.0");

  // Goto target parameters (global LLA).
  const [lat, setLat] = useState<string>("");
  const [lon, setLon] = useState<string>("");
  const [alt, setAlt] = useState<string>("");
  const [yaw, setYaw] = useState<string>(""); // radians; optional

  // Last dispatched request (debug).
  const [lastRequest, setLastRequest] = useState<AppRequestDetail | null>(null);

  const onArm = useCallback(() => setLastRequest(postMissionRequest("arm", { auto })), [auto]);
  const onDisarm = useCallback(() => setLastRequest(postMissionRequest("disarm")), []);
  const onLand = useCallback(() => setLastRequest(postMissionRequest("land")), []);
  const onRTL = useCallback(() => setLastRequest(postMissionRequest("rtl")), []);

  const onTakeoff = useCallback(() => {
    const altitude = Number(tkAlt);
    setLastRequest(postMissionRequest("takeoff", { altitude, auto }));
  }, [tkAlt, auto]);

  const onGoto = useCallback(() => {
    const payload = {
      lat: lat === "" ? undefined : Number(lat),
      lon: lon === "" ? undefined : Number(lon),
      alt: alt === "" ? undefined : Number(alt),
      yaw: yaw === "" ? undefined : Number(yaw),
      auto,
    };
    setLastRequest(postMissionRequest("goto", payload));
  }, [lat, lon, alt, yaw, auto]);

  // Emits a simulation-control restart request (kept separate from lastRequest to avoid widening its type).
  const onRestartSim = useCallback(() => {
    const requestId =
      (globalThis as any).crypto?.randomUUID?.() ??
      Math.random().toString(36).slice(2);
    const detail = {
      category: "simulation_control",
      action: "restart",
      requestId,
      timestamp: Date.now(),
    };
    window.dispatchEvent(new CustomEvent(EVENT_NAME, { detail }));
  }, []);

  const disabledGoto = useMemo(() => lat === "" || lon === "" || alt === "", [lat, lon, alt]);

  return (
    <div style={styles.container}>
      <div style={styles.card}>
        <div style={styles.header}>Mission Control</div>

        {/* Arm / Disarm */}
        <div style={styles.row}>
          <button style={styles.button} onClick={onArm}>Arm</button>
          <button style={styles.buttonDanger} onClick={onDisarm}>Disarm</button>
        </div>

        <div style={styles.divider} />

        {/* Auto toggle */}
        <div style={styles.toggleWrap}>
          <div>
            <div style={styles.sectionTitle}>Auto transitions</div>
            <div style={styles.help}>Allow arming/mode/takeoff when required by mission requests.</div>
          </div>
          <input
            type="checkbox"
            checked={auto}
            onChange={(e) => setAuto(e.target.checked)}
            style={styles.checkbox}
            aria-label="Toggle auto transitions"
          />
        </div>

        <div style={styles.divider} />

        {/* Takeoff / Land / RTL */}
        <div className="takeoff-row" style={{ display: "grid", gridTemplateColumns: "1fr 2fr", gap: 8, alignItems: "end" }}>
          <div>
            <div style={styles.label}>Takeoff altitude (m)</div>
            <input
              id="tkAlt"
              inputMode="decimal"
              value={tkAlt}
              onChange={(e) => setTkAlt(e.target.value)}
              placeholder="2.0"
              style={styles.input}
            />
          </div>
          <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: 8 }}>
            <button style={styles.button} onClick={onTakeoff}>Takeoff</button>
            <button style={styles.buttonSecondary} onClick={onLand}>Land</button>
            <button style={styles.buttonSecondary} onClick={onRTL}>RTL</button>
          </div>
        </div>

        <div style={styles.divider} />

        {/* Goto (global) — compact vertical stack */}
        <div>
          <div style={styles.sectionTitle}>Goto (global)</div>
          <div style={{ display: "flex", flexDirection: "column", gap: 6 }}>
            <div>
              <div style={styles.label}>Latitude</div>
              <input
                id="lat"
                inputMode="decimal"
                placeholder="e.g. 40.1772"
                value={lat}
                onChange={(e) => setLat(e.target.value)}
                style={styles.inputCompact}
              />
            </div>
            <div>
              <div style={styles.label}>Longitude</div>
              <input
                id="lon"
                inputMode="decimal"
                placeholder="e.g. 44.5126"
                value={lon}
                onChange={(e) => setLon(e.target.value)}
                style={styles.inputCompact}
              />
            </div>
            <div>
              <div style={styles.label}>Altitude (m)</div>
              <input
                id="alt"
                inputMode="decimal"
                placeholder="e.g. 30"
                value={alt}
                onChange={(e) => setAlt(e.target.value)}
                style={styles.inputCompact}
              />
            </div>
            <div>
              <div style={styles.label}>Yaw (rad)</div>
              <input
                id="yaw"
                inputMode="decimal"
                placeholder="optional"
                value={yaw}
                onChange={(e) => setYaw(e.target.value)}
                style={styles.inputCompact}
              />
            </div>
          </div>
          <div style={{ ...styles.right, marginTop: 8 }}>
            <button
              style={styles.button}
              onClick={onGoto}
              disabled={disabledGoto}
              aria-disabled={disabledGoto}
            >
              Goto
            </button>
          </div>
        </div>

        <div style={styles.divider} />

        {/* Simulation */}
        <div>
          <div style={styles.sectionTitle}>Simulation</div>
          <div style={styles.right}>
            <button style={styles.buttonSecondary} onClick={onRestartSim}>Restart simulation</button>
          </div>
        </div>

        {lastRequest && (
          <div style={{ marginTop: 12 }}>
            <div style={styles.label}>Last request (debug)</div>
            <pre style={styles.debug}>{JSON.stringify(lastRequest, null, 2)}</pre>
          </div>
        )}
      </div>
    </div>
  );
}
