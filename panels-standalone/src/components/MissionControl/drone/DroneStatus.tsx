import { DroneStateModel, LANDED } from "@/mission-control/drone-state-model";
import React, { useEffect, useState } from "react";

// Pure React component using ONLY the model you already have (onUpdate + getState).
// No new enums/types defined here. No external UI libs.

function useDroneState(model: DroneStateModel) {
  const [state, setState] = useState(() => (model && model.getState ? model.getState() : {}));
  useEffect(() => {
    if (!model || !model.onUpdate) return;
    setState(model.getState ? model.getState() : {});
    const off = model.onUpdate((s) => setState({ ...s }));
    return () => (typeof off === "function" ? off() : undefined);
  }, [model]);
  return state || {};
}

const NDASH = "–";

const pctText = (p?: number | null) => {
  if (p === undefined || p === null || Number.isNaN(p) || p < 0) return NDASH;
  // BatteryState.percentage can be 0..1 or 0..100 depending on source
  const val = p > 1.01 ? p : p * 100;
  return `${Math.round(val)}%`;
};

const num = (n?: number | null, unit = "", d = 1) =>
  n === undefined || n === null || Number.isNaN(n) ? NDASH : `${n.toFixed(d)}${unit}`;

const deg = (v?: number | null) =>
  v === undefined || v === null || Number.isNaN(v) ? NDASH : `${v.toFixed(0)}°`;

const lat = (v?: number | null) =>
  v === undefined || v === null || Number.isNaN(v)
    ? NDASH
    : `${Math.abs(v).toFixed(6)}° ${v >= 0 ? "N" : "S"}`;

const lon = (v?: number | null) =>
  v === undefined || v === null || Number.isNaN(v)
    ? NDASH
    : `${Math.abs(v).toFixed(6)}° ${v >= 0 ? "E" : "W"}`;

function Row({ label, value }: { label: React.ReactNode; value: React.ReactNode }) {
  return (
    <div style={{ display: "grid", gridTemplateColumns: "1fr auto", gap: 12, alignItems: "baseline", padding: "4px 0" }}>
      <div style={{ color: "#6b7280", overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}>{label}</div>
      <div style={{ textAlign: "right", fontFamily: "ui-monospace, SFMono-Regular, Menlo, monospace" }}>{value}</div>
    </div>
  );
}

function Dot({ ok, title }: { ok: boolean; title: string }) {
  return <span title={title} style={{ color: ok ? "#16a34a" : "#dc2626" }}>●</span>;
}

function landedText(n?: number) {
  switch (n) {
    case LANDED.UNDEFINED: return "Undefined";
    case LANDED.ON_GROUND: return "On Ground";
    case LANDED.IN_AIR: return "In Air";
    case LANDED.TAKEOFF: return "Takeoff";
    case LANDED.LANDING: return "Landing";
    default: return NDASH;
  }
}

export function DroneStatusPanel({ model }: { model: DroneStateModel }) {
  const s: any = useDroneState(model);

  // Heading from VFR HUD first, then global HDG fallback
  const heading =
    (s?.vfr_hud?.heading ?? null) !== null
      ? s.vfr_hud.heading
      : (s?.global_position_int?.hdg ?? undefined);

  // Throttle can be 0..1 or 0..100; render robustly
  const throttleText = (() => {
    const t = s?.vfr_hud?.throttle;
    if (t === undefined || t === null || Number.isNaN(t)) return NDASH;
    const val = t > 1.5 ? t : t * 100;
    return `${Math.round(val)}%`;
  })();

  const faults: string[] = s?.status?.faults || [];
  const fcuOk = !!s?.vehicle?.connected;
  const gcsOk = !!(s?.status?.gcs_link ?? s?.vehicle?.connected);

  return (
    <div style={{ width: "100%", maxWidth: 400, margin: "0 auto", border: "1px solid #e5e7eb", borderRadius: 12, boxShadow: "0 1px 2px rgba(0,0,0,0.05)" }}>
      <div style={{ padding: 16, borderBottom: "1px solid #e5e7eb", display: "flex", gap: 8, alignItems: "center" }}>
        <div style={{ fontWeight: 600 }}>✈️ Drone Status</div>
        <div style={{ marginLeft: "auto", display: "flex", gap: 12, fontSize: 14 }}>
          <span style={{ display: "inline-flex", gap: 6, alignItems: "center" }}><Dot ok={fcuOk} title="FCU link" /> FCU</span>
          <span style={{ display: "inline-flex", gap: 6, alignItems: "center" }}><Dot ok={gcsOk} title="GCS link" /> GCS</span>
        </div>
      </div>

      <div style={{ padding: 16 }}>
        {/* Vehicle */}
        <Row label={<span>📡 Mode</span>} value={s?.vehicle?.mode ?? NDASH} />
        <Row label={<span>Armed</span>} value={s?.vehicle?.armed ? "Yes" : "No"} />
        <Row label={<span>Guided</span>} value={s?.vehicle?.guided ? "Yes" : "No"} />
        <Row label={<span>Landed State</span>} value={landedText(s?.extended?.landed_state)} />

        <hr style={{ border: 0, borderTop: "1px solid #e5e7eb", margin: "12px 0" }} />

        {/* Positioning */}
        <Row label={<span>🛰️ Latitude</span>} value={lat(s?.global_position_int?.lat)} />
        <Row label={<span>🛰️ Longitude</span>} value={lon(s?.global_position_int?.lon)} />
        <Row
          label={<span>🗻 Alt (AMSL)</span>}
          value={num(s?.altitude?.amsl ?? s?.global_position_int?.alt, " m", 1)}
        />
        <Row
          label={<span>Rel Alt</span>}
          value={num(s?.altitude?.relative ?? s?.global_position_int?.relative_alt, " m", 1)}
        />
        <Row label={<span>🧭 Heading</span>} value={deg(heading)} />

        <hr style={{ border: 0, borderTop: "1px solid #e5e7eb", margin: "12px 0" }} />

        {/* Speeds */}
        <Row label={<span>🏎️ Airspeed</span>} value={num(s?.vfr_hud?.airspeed, " m/s", 1)} />
        <Row label={<span>Groundspeed</span>} value={num(s?.vfr_hud?.groundspeed, " m/s", 1)} />
        <Row label={<span>Throttle</span>} value={throttleText} />
        <Row label={<span>Climb</span>} value={num(s?.vfr_hud?.climb, " m/s", 1)} />

        <hr style={{ border: 0, borderTop: "1px solid #e5e7eb", margin: "12px 0" }} />

        {/* Battery */}
        <Row label={<span>🔋 Battery</span>} value={pctText(s?.battery?.percentage)} />
        <Row label={<span>Voltage</span>} value={s?.battery?.voltage !== undefined ? `${s.battery.voltage.toFixed(2)} V` : NDASH} />
        <Row label={<span>Current</span>} value={s?.battery?.current !== undefined ? `${s.battery.current.toFixed(1)} A` : NDASH} />
        <Row label={<span>Temp</span>} value={s?.battery?.temperature !== null && s?.battery?.temperature !== undefined ? `${s.battery.temperature.toFixed(1)} °C` : NDASH} />

        <hr style={{ border: 0, borderTop: "1px solid #e5e7eb", margin: "12px 0" }} />

        {/* Home */}
        <Row label={<span>🏠 Home Lat</span>} value={lat(s?.home?.lat)} />
        <Row label={<span>🏠 Home Lon</span>} value={lon(s?.home?.lon)} />
        <Row label={<span>Home Alt</span>} value={num(s?.home?.alt, " m", 1)} />

        <hr style={{ border: 0, borderTop: "1px solid #e5e7eb", margin: "12px 0" }} />

        {/* Faults */}
        <Row label={<span>🛠️ Faults</span>} value={(s?.status?.faults?.length ? s.status.faults.join(", ") : "None")} />
      </div>
    </div>
  );
}
