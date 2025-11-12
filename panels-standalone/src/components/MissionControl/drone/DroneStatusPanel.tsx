import { DroneStateModel, LANDED } from "@/mission-control/drone-state-model";
import React, { useEffect, useState } from "react";
import MissionControlPanel from "./MissionControlButtons";
import "./DroneStatusPanel.css";

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
    <div className="dsp-row">
      <div className="dsp-row-label">{label}</div>
      <div className="dsp-row-value">{value}</div>
    </div>
  );
}

function Dot({ ok, title }: { ok: boolean; title: string }) {
  return <span title={title} className={`dsp-dot ${ok ? "ok" : "bad"}`}>●</span>;
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
    <div className="drone-status">
      <div className="dsp-header">
        <div className="dsp-title">✈️ Drone Status</div>
        <div className="dsp-links">
          <span className="dsp-link-item"><Dot ok={fcuOk} title="FCU link" /> FCU</span>
          <span className="dsp-link-item"><Dot ok={gcsOk} title="GCS link" /> GCS</span>
        </div>
      </div>

      <div className="dsp-body">
        {/* Vehicle */}
        <Row label={<span>📡 Mode</span>} value={s?.vehicle?.mode ?? NDASH} />
        <Row label={<span>Armed</span>} value={s?.vehicle?.armed ? "Yes" : "No"} />
        <Row label={<span>Guided</span>} value={s?.vehicle?.guided ? "Yes" : "No"} />
        <Row label={<span>Landed State</span>} value={landedText(s?.extended?.landed_state)} />

        <hr className="dsp-sep" />

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

        <hr className="dsp-sep" />

        {/* Speeds */}
        <Row label={<span>🏎️ Airspeed</span>} value={num(s?.vfr_hud?.airspeed, " m/s", 1)} />
        <Row label={<span>Groundspeed</span>} value={num(s?.vfr_hud?.groundspeed, " m/s", 1)} />
        <Row label={<span>Throttle</span>} value={throttleText} />
        <Row label={<span>Climb</span>} value={num(s?.vfr_hud?.climb, " m/s", 1)} />

        <hr className="dsp-sep" />

        {/* Battery */}
        <Row label={<span>🔋 Battery</span>} value={pctText(s?.battery?.percentage)} />
        <Row label={<span>Voltage</span>} value={s?.battery?.voltage !== undefined ? `${s.battery.voltage.toFixed(2)} V` : NDASH} />
        <Row label={<span>Current</span>} value={s?.battery?.current !== undefined ? `${s.battery.current.toFixed(1)} A` : NDASH} />
        <Row label={<span>Temp</span>} value={s?.battery?.temperature !== null && s?.battery?.temperature !== undefined ? `${s.battery.temperature.toFixed(1)} °C` : NDASH} />

        <hr className="dsp-sep" />

        {/* Home */}
        <Row label={<span>🏠 Home Lat</span>} value={lat(s?.home?.lat)} />
        <Row label={<span>🏠 Home Lon</span>} value={lon(s?.home?.lon)} />
        <Row label={<span>Home Alt</span>} value={num(s?.home?.alt, " m", 1)} />

        <hr className="dsp-sep" />

        {/* Faults */}
        <Row label={<span>🛠️ Faults</span>} value={(faults.length ? faults.join(", ") : "None")} />

        <hr className="dsp-sep" />

        <MissionControlPanel />
      </div>
    </div>
  );
}
