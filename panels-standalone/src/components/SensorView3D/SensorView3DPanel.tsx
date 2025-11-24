// Sensor3DViewPanel.tsx
import React, { useEffect, useMemo, useRef, useState } from "react";
import { ros2Bridge } from "@/ros2-bridge";
import { Renderer } from "@lichtblick/suite-base/panels/ThreeDeeRender/Renderer";
import { DEFAULT_SCENE_EXTENSION_CONFIG } from "@lichtblick/suite-base/panels/ThreeDeeRender/SceneExtensionConfig";

// Uses the real Lichtblick Renderer. No external UI libs. Owns its <canvas>.
// Feeds TF + point-like topics via renderer.addMessageEvent(...).

const POINTLIKE_TYPES = new Set<string>([
  "sensor_msgs/msg/PointCloud2",
  "sensor_msgs/msg/LaserScan",
  "sensor_msgs/msg/PointCloud",
]);

// Minimal default config; scene extensions provide the rest.
const DEFAULT_RENDERER_CONFIG: any = {
  cameraState: {},
  imageMode: { calibrationTopic: undefined },
  scene: {
    transforms: { enablePreloading: false },
    ignoreColladaUpAxis: false,
    meshUpAxis: undefined,
  },
  layers: {},
};

export default function Sensor3DViewPanel() {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const rendererRef = useRef<Renderer | null>(null);
  const unsubMapRef = useRef<Map<string, () => void>>(new Map());

  const [availableFrames, setAvailableFrames] = useState<string[]>([]);
  const [baseFrame, setBaseFrame] = useState<string>("");

  const [availablePointTopics, setAvailablePointTopics] = useState<Array<{ topic: string; type: string }>>([]);
  const [enabledPointTopics, setEnabledPointTopics] = useState<Record<string, boolean>>({});

  // Global point settings (apply via updateConfig once exact keys are provided)
  const [settings, setSettings] = useState({
    decayTimeSec: 5.0,
    pointSize: 2.0,
    gradientStart: "#3fb3ff",
    gradientEnd: "#ff3f6f",
    falloff: 0.25,
  });

  // ---------- Bootstrap renderer ----------
  useEffect(() => {
    if (!canvasRef.current) return;

    const fetchAsset = async (uri: string, _opts?: any) => {
      const res = await fetch(uri);
      const arrayBuffer = await res.arrayBuffer();
      return { uri, arrayBuffer } as any;
    };

    const r = new Renderer({
      canvas: canvasRef.current,
      config: DEFAULT_RENDERER_CONFIG,
      interfaceMode: "3d",
      sceneExtensionConfig: DEFAULT_SCENE_EXTENSION_CONFIG,
      customCameraModels: new Map(),
      fetchAsset,
      testOptions: { debugPicking: false },
      displayTemporaryError: (m: string) => console.warn("[Renderer]", m),
    });

    // Normalize ROS frame ids (strip leading "/")
    (r as any).ros = true;

    rendererRef.current = r;

    // Seed topic list once
    try {
      const topics = ros2Bridge.getAvailableTopics?.() || [];
      r.setTopics?.(topics.map((t: any) => ({ name: t.topic, datatype: t.type })));
    } catch {}

    return () => {
      try { r.dispose(); } catch {}
      rendererRef.current = null;
    };
  }, []);

  // ---------- TF wiring ----------
  useEffect(() => {
    const r = rendererRef.current; if (!r) return;
    const tfTopics = ["/tf", "/tf_static"];

    tfTopics.forEach((topic) => {
      if (unsubMapRef.current.has(topic)) return;
      const unsub = ros2Bridge.subscribe({ topic, type: "tf2_msgs/msg/TFMessage" }, (msg: any) => {
        feedMessage(r, topic, "tf2_msgs/msg/TFMessage", msg);
        try {
          const frames = ros2Bridge.getKnownFrames?.() ?? [];
          setAvailableFrames(frames);
          if (!baseFrame && frames.length > 0) setBaseFrame((p) => p || frames[0]);
        } catch {}
      });
      unsubMapRef.current.set(topic, unsub);
    });

    return () => {
      tfTopics.forEach((t) => {
        const u = unsubMapRef.current.get(t);
        if (u) { try { u(); } catch {} ; unsubMapRef.current.delete(t); }
      });
    };
  }, [baseFrame]);

  // ---------- Base frame selection ----------
  useEffect(() => {
    const r = rendererRef.current; if (!r) return;
    if (baseFrame) r.setFollowFrameId(baseFrame);
  }, [baseFrame]);

  // ---------- Topics discovery ----------
  const refreshPointTopics = () => {
    try {
      const all = ros2Bridge.getAvailableTopics();
      const clouds = (all || []).filter((t: any) => POINTLIKE_TYPES.has(t.type));
      setAvailablePointTopics(clouds);
      setEnabledPointTopics((prev) => {
        const next = { ...prev } as Record<string, boolean>;
        clouds.forEach(({ topic }: any) => { if (next[topic] === undefined) next[topic] = false; });
        return next;
      });
      const r = rendererRef.current; if (r) r.setTopics?.(all.map((t: any) => ({ name: t.topic, datatype: t.type })));
    } catch (e) { console.warn("refreshPointTopics() failed:", e); }
  };

  // Optional: auto-update when bridge announces changes (we'll add this in the bridge later)
  useEffect(() => {
    const off = (ros2Bridge as any).onAvailableTopicsChanged?.((topics: Array<{ topic: string; type: string }>) => {
      const clouds = (topics || []).filter((t) => POINTLIKE_TYPES.has(t.type));
      setAvailablePointTopics(clouds);
      setEnabledPointTopics((prev) => {
        const next: Record<string, boolean> = { ...prev };
        const names = new Set(clouds.map((c) => c.topic));
        Object.keys(next).forEach((name) => {
          if (!names.has(name)) {
            const u = unsubMapRef.current.get(name);
            if (u) { try { u(); } catch {} ; unsubMapRef.current.delete(name); }
            delete next[name];
          }
        });
        clouds.forEach(({ topic }) => { if (next[topic] === undefined) next[topic] = false; });
        return next;
      });
      rendererRef.current?.setTopics?.(topics.map((t) => ({ name: t.topic, datatype: t.type })));
    });
    return () => { try { off?.(); } catch {} };
  }, []);

  // ---------- Enable/disable point topics ----------
  const togglePointTopic = (topic: string, type: string, enabled: boolean) => {
    setEnabledPointTopics((prev) => ({ ...prev, [topic]: enabled }));
    const r = rendererRef.current; if (!r) return;

    if (enabled) {
      const unsub = ros2Bridge.subscribe({ topic, type }, (msg: any) => {
        const schema = ros2Bridge.getTopicType?.(topic) || type;
        feedMessage(r, topic, schema, msg);
      });
      unsubMapRef.current.set(topic, unsub);
    } else {
      const u = unsubMapRef.current.get(topic);
      if (u) { try { u(); } catch {} ; unsubMapRef.current.delete(topic); }
    }
  };

  // ---------- Apply global point settings (stub until exact keys are provided) ----------
  const applySettings = () => {
    const r = rendererRef.current; if (!r) return;
    // Provide exact config keys from your PointClouds extension and replace this block:
    // r.updateConfig((draft: any) => {
    //   draft.layers.pointClouds = draft.layers.pointClouds ?? {};
    //   draft.layers.pointClouds.global = {
    //     decayTimeSec: settings.decayTimeSec,
    //     pointSize: settings.pointSize,
    //     gradient: { start: settings.gradientStart, end: settings.gradientEnd },
    //     falloff: settings.falloff,
    //   };
    // });
  };

  // ---------- Reset availability ----------
  const canReset = useMemo(() => {
    const r = rendererRef.current as any;
    try { return !!r?.canResetView?.(); } catch { return false; }
  }, [baseFrame]);

  return (
    <div style={{ position: "absolute", inset: 0 }}>
      <canvas ref={canvasRef} style={{ width: "100%", height: "100%", display: "block" }} />

      {/* Overlay UI */}
      <div
        style={{
          position: "absolute",
          top: 8,
          right: 8,
          display: "flex",
          flexDirection: "column",
          gap: 8,
          pointerEvents: "none",
        }}
      >
        {/* Toolbar */}
        <div style={panel()}>
          {/* Base frame selector */}
          <label style={{ display: "flex", alignItems: "center", gap: 6 }}>
            <span style={{ opacity: 0.85 }}>Base</span>
            <select
              value={baseFrame}
              onChange={(e) => setBaseFrame(e.target.value)}
              style={selectStyle}
            >
              {availableFrames.length === 0 ? (
                <option value="">{`(no frames)`}</option>
              ) : (
                availableFrames.map((f) => (
                  <option key={f} value={f}>
                    {f}
                  </option>
                ))
              )}
            </select>
            <button
              style={btn(true)}
              title="Refresh frames"
              onClick={() => setAvailableFrames(ros2Bridge.getKnownFrames?.() ?? [])}
            >
              ↻
            </button>
          </label>

          <button
            style={btn(canReset)}
            disabled={!canReset}
            title={canReset ? "Reset view" : "Reset unavailable"}
            onClick={() => rendererRef.current?.resetView?.()}
          >
            Reset
          </button>
        </div>

        {/* Global point settings */}
        <div style={card()}>
          <div style={cardHeader()}>Point Settings</div>
          <div style={row()}>
            <label style={label()}>Decay (s)</label>
            <input
              type="number"
              step={0.1}
              min={0}
              value={settings.decayTimeSec}
              onChange={(e) =>
                setSettings((s) => ({ ...s, decayTimeSec: parseFloat(e.target.value || "0") }))
              }
              style={inputNumber}
            />
          </div>
          <div style={row()}>
            <label style={label()}>Point size</label>
            <input
              type="number"
              step={0.1}
              min={0.1}
              value={settings.pointSize}
              onChange={(e) =>
                setSettings((s) => ({ ...s, pointSize: parseFloat(e.target.value || "0.1") }))
              }
              style={inputNumber}
            />
          </div>
          <div style={row()}>
            <label style={label()}>Falloff</label>
            <input
              type="range"
              min={0}
              max={1}
              step={0.01}
              value={settings.falloff}
              onChange={(e) => setSettings((s) => ({ ...s, falloff: parseFloat(e.target.value) }))}
              style={{ width: 160 }}
            />
            <span style={{ width: 40, textAlign: "right", opacity: 0.8 }}>
              {settings.falloff.toFixed(2)}
            </span>
          </div>
          <div style={row()}>
            <label style={label()}>Gradient</label>
            <input
              type="color"
              value={settings.gradientStart}
              onChange={(e) =>
                setSettings((s) => ({ ...s, gradientStart: e.target.value }))
              }
              style={inputColor}
              aria-label="Gradient start color"
            />
            <div style={{ width: 32, textAlign: "center", opacity: 0.7 }}>→</div>
            <input
              type="color"
              value={settings.gradientEnd}
              onChange={(e) =>
                setSettings((s) => ({ ...s, gradientEnd: e.target.value }))
              }
              style={inputColor}
              aria-label="Gradient end color"
            />
            <div
              title="Preview"
              style={{
                marginLeft: 8,
                width: 80,
                height: 18,
                borderRadius: 6,
                background: `linear-gradient(90deg, ${settings.gradientStart}, ${settings.gradientEnd})`,
              }}
            />
          </div>
          <div style={{ display: "flex", justifyContent: "flex-end" }}>
            <button style={btn(true)} onClick={applySettings}>
              Apply
            </button>
          </div>
        </div>

        {/* Point sources */}
        <div style={card()}>
          <div style={{ ...cardHeader(), display: "flex", justifyContent: "space-between" }}>
            <span>Point Sources</span>
            <div style={{ display: "flex", gap: 6 }}>
              <button style={btn(true)} title="Refresh topics" onClick={refreshPointTopics}>
                Refresh
              </button>
              <button
                style={btn(true)}
                title="Disable all"
                onClick={() =>
                  availablePointTopics.forEach(({ topic }) => {
                    if (enabledPointTopics[topic]) togglePointTopic(topic, "", false);
                  })
                }
              >
                None
              </button>
            </div>
          </div>
          {availablePointTopics.length === 0 ? (
            <div style={emptyMsg}>No point-like topics discovered yet.</div>
          ) : (
            <div
              style={{
                display: "flex",
                flexDirection: "column",
                gap: 6,
                maxHeight: 220,
                overflow: "auto",
              }}
            >
              {availablePointTopics.map(({ topic, type }) => (
                <label key={topic} style={topicRow}>
                  <input
                    type="checkbox"
                    checked={!!enabledPointTopics[topic]}
                    onChange={(e) => togglePointTopic(topic, type, e.target.checked)}
                  />
                  <div style={{ display: "flex", flexDirection: "column" }}>
                    <span style={{ fontWeight: 600 }}>{topic}</span>
                    <span style={{ opacity: 0.7, fontSize: 12 }}>{type}</span>
                  </div>
                </label>
              ))}
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

// ---------- helpers ----------
function toReceiveTime(message: any): { sec: number; nsec: number } {
  const s = message?.header?.stamp || message?.header?.stamp?.stamp || {};
  const sec = typeof s.sec === "number" ? s.sec : 0;
  const nsec =
    typeof s.nanosec === "number" ? s.nanosec : (typeof s.nsec === "number" ? s.nsec : 0);
  if (sec || nsec) return { sec, nsec };
  const now = Date.now();
  return { sec: Math.floor(now / 1000), nsec: (now % 1000) * 1_000_000 };
}

function feedMessage(renderer: Renderer, topic: string, schemaName: string, message: any) {
  const receiveTime = toReceiveTime(message);
  const ev: any = { topic, schemaName, receiveTime, message };
  renderer.addMessageEvent(ev);
}

// ---------- styles ----------
const panel = (): React.CSSProperties => ({
  pointerEvents: "auto",
  background: "rgba(0,0,0,0.55)",
  color: "#fff",
  padding: 8,
  borderRadius: 10,
  display: "flex",
  gap: 8,
  alignItems: "center",
  backdropFilter: "blur(6px)",
});

const btn = (active: boolean): React.CSSProperties => ({
  appearance: "none",
  border: "1px solid rgba(255,255,255,0.25)",
  background: active ? "rgba(255,255,255,0.22)" : "rgba(255,255,255,0.08)",
  color: "white",
  padding: "6px 10px",
  borderRadius: 8,
  cursor: "pointer",
  fontSize: 12,
});

const selectStyle: React.CSSProperties = {
  background: "rgba(0,0,0,0.4)",
  color: "#fff",
  border: "1px solid rgba(255,255,255,0.25)",
  borderRadius: 6,
  padding: "4px 6px",
};

const card = (): React.CSSProperties => ({
  pointerEvents: "auto",
  background: "rgba(0,0,0,0.6)",
  color: "#fff",
  padding: 10,
  borderRadius: 12,
  minWidth: 320,
  backdropFilter: "blur(6px)",
});

const cardHeader = (): React.CSSProperties => ({
  fontWeight: 700,
  fontSize: 13,
  marginBottom: 8,
  letterSpacing: 0.2,
});

const row = (): React.CSSProperties => ({
  display: "flex",
  alignItems: "center",
  gap: 8,
  marginBottom: 6,
});

const label = (): React.CSSProperties => ({ width: 90, opacity: 0.9 });

const inputNumber: React.CSSProperties = {
  width: 120,
  background: "rgba(0,0,0,0.4)",
  color: "#fff",
  border: "1px solid rgba(255,255,255,0.25)",
  borderRadius: 6,
  padding: "4px 6px",
};

const inputColor: React.CSSProperties = {
  width: 28,
  height: 24,
  padding: 0,
  border: "1px solid rgba(255,255,255,0.25)",
  borderRadius: 4,
  background: "transparent",
};

const emptyMsg: React.CSSProperties = { opacity: 0.8, fontStyle: "italic" };

const topicRow: React.CSSProperties = {
  display: "grid",
  gridTemplateColumns: "20px 1fr",
  alignItems: "center",
  gap: 8,
  padding: 6,
  borderRadius: 8,
  background: "rgba(255,255,255,0.06)",
};
