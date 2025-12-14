// SensorView3DPanel.tsx
// Standalone 3D view that uses the global ros2Bridge and Lichtblick's 3D renderer.

/*
SPDX-FileCopyrightText: Copyright (C) 2023-2025
SPDX-License-Identifier: MPL-2.0
*/

import React, {
  useCallback,
  useEffect,
  useRef,
  useState,
} from "react";

import { fromNanoSec } from "@lichtblick/rostime";
import type { MessageEvent, Topic } from "@lichtblick/suite";

import ThemeProvider from "@lichtblick/suite-base/theme/ThemeProvider";
import { PANEL_STYLE } from "@lichtblick/suite-base/panels/ThreeDeeRender/constants";
import { Renderer } from "@lichtblick/suite-base/panels/ThreeDeeRender/Renderer";
import { RendererOverlay } from "@lichtblick/suite-base/panels/ThreeDeeRender/RendererOverlay";
import { RendererContext } from "@lichtblick/suite-base/panels/ThreeDeeRender/RendererContext";
import type {
  RendererConfig,
  ImageModeConfig,
  RendererSubscription,
} from "@lichtblick/suite-base/panels/ThreeDeeRender/IRenderer";
import { DEFAULT_CAMERA_STATE } from "@lichtblick/suite-base/panels/ThreeDeeRender/camera";
import { DEFAULT_PUBLISH_SETTINGS } from "@lichtblick/suite-base/panels/ThreeDeeRender/renderables/PublishSettings";
import { DEFAULT_SCENE_EXTENSION_CONFIG } from "@lichtblick/suite-base/panels/ThreeDeeRender/SceneExtensionConfig";
import type { InterfaceMode } from "@lichtblick/suite-base/panels/ThreeDeeRender/types";
import type { Asset } from "@lichtblick/suite-base/components/PanelExtensionAdapter";

import { ros2Bridge } from "@/ros2-bridge";
import type { Subscription as Ros2BridgeSubscription } from "@/ros2-bridge";
import { ConnectionSettingsProvider, ConnectionSettingsTrigger } from '../ConnectionSettingsProvider';

export type Sensor3DViewPanelProps = {
  className?: string;
  style?: React.CSSProperties;
};

const LS_POINT_SIZE_KEY = "sensor3d.pointSize";
const LS_DECAY_TIME_KEY = "sensor3d.decayTime";

export const Sensor3DViewPanel: React.FC<Sensor3DViewPanelProps> = (props) => {
  const interfaceMode: InterfaceMode = "3d";

  const [canvas, setCanvas] = useState<HTMLCanvasElement | null>(null);
  const [renderer, setRenderer] = useState<Renderer | undefined>(undefined);

  // Simple, fixed config – no external panel state
  const initialConfigRef = useRef<RendererConfig>({
    cameraState: { ...DEFAULT_CAMERA_STATE },
    followMode: "follow-pose",
    followTf: undefined,
    scene: {},
    transforms: {},
    topics: {},
    layers: {},
    publish: { ...DEFAULT_PUBLISH_SETTINGS },
    imageMode: {} as Partial<ImageModeConfig> as ImageModeConfig,
  });

  const [measureActive, setMeasureActive] = useState(false);
  const [perspective, setPerspective] = useState(
    initialConfigRef.current.cameraState.perspective,
  );

  // Global point size & decay (cached in localStorage)
  const [pointSize, setPointSize] = useState<number>(() => {
    if (typeof window === "undefined") return 3;
    const stored = window.localStorage.getItem(LS_POINT_SIZE_KEY);
    const n = stored != null ? Number(stored) : NaN;
    return Number.isFinite(n) && n > 0 ? n : 3;
  });

  const [decayTime, setDecayTime] = useState<number>(() => {
    if (typeof window === "undefined") return 0;
    const stored = window.localStorage.getItem(LS_DECAY_TIME_KEY);
    const n = stored != null ? Number(stored) : NaN;
    return Number.isFinite(n) && n >= 0 ? n : 0;
  });

  // Available topics (for topic visibility UI)
  const [topics, setTopics] = useState<Topic[]>([]);

  // Frames discovered from TF messages
  const framesSetRef = useRef<Set<string>>(new Set());
  const [frames, setFrames] = useState<string[]>([]);
  const [displayFrame, setDisplayFrame] = useState<string | undefined>(undefined);

  // Active ROS topic subscriptions created via ros2Bridge.subscribe
  const liveSubsRef = useRef<Map<string, () => void>>(new Map());

  // ---- Create / dispose Renderer --------------------------------------------------

  useEffect(() => {
    if (!canvas) {
      setRenderer(undefined);
      return;
    }

    const fetchAsset = async (
      uri: string,
      _options?: { signal?: AbortSignal; baseUrl?: string },
    ): Promise<Asset> => {
      const res = await fetch(uri);
      if (!res.ok) {
        throw new Error(`Failed to fetch asset ${uri}: ${res.status} ${res.statusText}`);
      }
      const buffer = await res.arrayBuffer();
      return { uri, data: new Uint8Array(buffer) } as Asset;
    };

    const r = new Renderer({
      canvas,
      config: initialConfigRef.current,
      interfaceMode,
      fetchAsset,
      sceneExtensionConfig: DEFAULT_SCENE_EXTENSION_CONFIG,
      displayTemporaryError: (msg: string) => {
        // Hook into your snackbar/toast here if you want
        // eslint-disable-next-line no-console
        console.error("[Sensor3DViewPanel] temporary error:", msg);
      },
      testOptions: {},
      customCameraModels: new Map(),
    });

    // Tell renderer we're on a ROS data source
    r.ros = true;
    // Default to light theme
    r.setColorScheme("light", undefined);

    setRenderer(r);

    return () => {
      r.dispose();
      setRenderer(undefined);
    };
  }, [canvas, interfaceMode]);

  // ---- Measurement tool wiring ----------------------------------------------------

  useEffect(() => {
    if (!renderer) return;

    const onStart = () => setMeasureActive(true);
    const onEnd = () => setMeasureActive(false);

    renderer.measurementTool.addEventListener("foxglove.measure-start", onStart);
    renderer.measurementTool.addEventListener("foxglove.measure-end", onEnd);

    return () => {
      renderer.measurementTool.removeEventListener("foxglove.measure-start", onStart);
      renderer.measurementTool.removeEventListener("foxglove.measure-end", onEnd);
    };
  }, [renderer]);

  const onClickMeasure = useCallback(() => {
    if (!renderer) return;
    if (measureActive) {
      renderer.measurementTool.stopMeasuring();
    } else {
      renderer.measurementTool.startMeasuring();
      renderer.publishClickTool.stop();
    }
  }, [measureActive, renderer]);

  // ---- Perspective toggle (2D/3D camera) -----------------------------------------

  const onTogglePerspective = useCallback(() => {
    if (!renderer) return;
    const current = renderer.getCameraState() ?? DEFAULT_CAMERA_STATE;
    const next = { ...current, perspective: !current.perspective };
    renderer.setCameraState(next);
    setPerspective(next.perspective);
    renderer.queueAnimationFrame();
  }, [renderer]);

  const onKeyDown = useCallback(
    (event: React.KeyboardEvent) => {
      if (event.key === "3" && !(event.metaKey || event.ctrlKey)) {
        onTogglePerspective();
        event.stopPropagation();
        event.preventDefault();
      }
    },
    [onTogglePerspective],
  );

  // ---- TF frame collection helper -------------------------------------------------

  const updateFramesFromTfMessage = useCallback((msg: any) => {
    if (!msg || !Array.isArray(msg.transforms)) {
      return;
    }
    const set = framesSetRef.current;
    let changed = false;

    for (const t of msg.transforms) {
      const parent = t?.header?.frame_id as string | undefined;
      const child = t?.child_frame_id as string | undefined;
      if (parent && !set.has(parent)) {
        set.add(parent);
        changed = true;
      }
      if (child && !set.has(child)) {
        set.add(child);
        changed = true;
      }
    }

    if (changed) {
      const arr = Array.from(set).sort();
      setFrames(arr);
      setDisplayFrame((prev) => prev ?? arr[0]);
    }
  }, []);

  // ---- ROS2Bridge wiring: available topics + live messages -----------------------

  useEffect(() => {
    if (!renderer) return;

    const liveSubs = liveSubsRef.current;

    const computeDesiredSubscriptions = (
      topicsList: Ros2BridgeSubscription[],
      r: Renderer,
    ): Ros2BridgeSubscription[] => {
      const desired: Ros2BridgeSubscription[] = [];

      const schemaSubs = r.schemaSubscriptions as Map<string, RendererSubscription[]>;
      const topicSubs = r.topicSubscriptions as Map<string, RendererSubscription[]>;

      for (const t of topicsList) {
        const topicName = t.topic;
        const schemaName = t.type;

        const subsForTopic = topicSubs.get(topicName) ?? [];
        const subsForSchema = schemaSubs.get(schemaName) ?? [];

        const allSubs = subsForTopic.concat(subsForSchema);
        if (allSubs.length === 0) {
          continue;
        }

        let shouldSubscribe = false;
        for (const sub of allSubs) {
          const res = sub.shouldSubscribe?.(topicName);
          if (res === undefined || res === true) {
            shouldSubscribe = true;
            break;
          }
        }
        if (shouldSubscribe) {
          desired.push({ topic: topicName, type: schemaName });
        }
      }

      // dedupe by topic
      const seen = new Set<string>();
      const unique: Ros2BridgeSubscription[] = [];
      for (const d of desired) {
        if (!seen.has(d.topic)) {
          seen.add(d.topic);
          unique.push(d);
        }
      }
      return unique;
    };

    const reconcileSubscriptions = (
      desired: Ros2BridgeSubscription[],
      r: Renderer,
    ) => {
      const current = liveSubs;

      const desiredTopics = new Set(desired.map((d) => d.topic));

      // Unsubscribe from topics no longer needed
      for (const [topic, unsubscribe] of current.entries()) {
        if (!desiredTopics.has(topic)) {
          unsubscribe();
          current.delete(topic);
        }
      }

      // Subscribe to new topics
      for (const d of desired) {
        if (current.has(d.topic)) {
          continue;
        }

        const unsubscribe = ros2Bridge.subscribe(
          { topic: d.topic, type: d.type },
          (raw: any) => {
            if (!r) return;

            // Normal (non-image) path: { topic, type, msg }
            // Image path: already-converted image object
            let msg = raw;
            if (
              raw &&
              typeof raw === "object" &&
              "msg" in raw &&
              "topic" in raw &&
              "type" in raw
            ) {
              msg = (raw as { msg: any }).msg;
            }

            // If this is a TF message, harvest frames
            if (
              d.topic === "/tf" ||
              d.topic === "/tf_static" ||
              (typeof d.type === "string" && d.type.includes("tf2_msgs/msg/TFMessage"))
            ) {
              updateFramesFromTfMessage(msg);
            }

            const header = msg?.header as
              | { stamp?: { sec?: number; nanosec?: number; nsec?: number } }
              | undefined;

            let publishNs: bigint | undefined;
            if (header?.stamp) {
              const sec = header.stamp.sec ?? 0;
              const nanosec =
                header.stamp.nanosec ??
                (header.stamp as any).nsec ??
                0;
              publishNs =
                BigInt(sec) * 1_000_000_000n + BigInt(nanosec);
            }

            const nowNs = BigInt(Date.now()) * 1_000_000n;
            const receiveNs = publishNs ?? nowNs;

            const event: MessageEvent<any> = {
              topic: d.topic,
              schemaName: d.type,
              receiveTime: fromNanoSec(receiveNs),
              publishTime: publishNs ? fromNanoSec(publishNs) : undefined,
              message: msg,
              sizeInBytes: estimateSizeInBytes(msg),
            };

            // Drive renderer time forward and feed the message
            r.setCurrentTime(receiveNs);
            r.addMessageEvent(event);
            r.queueAnimationFrame();
          },
        );

        current.set(d.topic, unsubscribe);
      }
    };

    const handleTopicsChanged = (topicsList: Ros2BridgeSubscription[]) => {
      const topicObjects: Topic[] = topicsList.map(
        (t) =>
          ({
            name: t.topic,
            schemaName: t.type,
            datatype: t.type,
          } as unknown as Topic),
      );

      setTopics(topicObjects);
      renderer.setTopics(topicObjects);

      const desired = computeDesiredSubscriptions(topicsList, renderer);
      reconcileSubscriptions(desired, renderer);
    };

    const unsubscribeTopicsChanged =
      ros2Bridge.onAvailableTopicsChanged(handleTopicsChanged);

    // Run once with current topics
    handleTopicsChanged(ros2Bridge.getAvailableTopics());

    return () => {
      unsubscribeTopicsChanged();
      for (const unsubscribe of liveSubs.values()) {
        unsubscribe();
      }
      liveSubs.clear();
    };
  }, [renderer, updateFramesFromTfMessage]);

  // ---- Global point size & decay: persist + push into renderer config -----------

  useEffect(() => {
    if (typeof window !== "undefined") {
      try {
        window.localStorage.setItem(LS_POINT_SIZE_KEY, String(pointSize));
      } catch {
        // ignore
      }
    }
    if (!renderer) return;
    const cfg = renderer.config as any;
    cfg.topics ??= {};
    for (const name of Object.keys(cfg.topics)) {
      cfg.topics[name] ??= {};
      cfg.topics[name].pointSize = pointSize;
    }
    renderer.queueAnimationFrame();
  }, [pointSize, renderer]);

  useEffect(() => {
    if (typeof window !== "undefined") {
      try {
        window.localStorage.setItem(LS_DECAY_TIME_KEY, String(decayTime));
      } catch {
        // ignore
      }
    }
    if (!renderer) return;
    const cfg = renderer.config as any;
    cfg.topics ??= {};
    for (const name of Object.keys(cfg.topics)) {
      cfg.topics[name] ??= {};
      cfg.topics[name].decayTime = decayTime;
    }
    renderer.queueAnimationFrame();
  }, [decayTime, renderer]);

  // ---- Apply selected display frame into renderer config ------------------------

  useEffect(() => {
    if (!renderer || !displayFrame) return;
    renderer.config = {
      ...renderer.config,
      followTf: displayFrame,
    };
    renderer.queueAnimationFrame();
  }, [renderer, displayFrame]);

  // ---- Topic visibility helpers ---------------------------------------------------

  const isTopicVisible = useCallback(
    (topicName: string): boolean => {
      const cfg: any = (renderer as any)?.config?.topics?.[topicName];
      if (!cfg || cfg.visible === undefined) return true;
      return !!cfg.visible;
    },
    [renderer],
  );

  const toggleTopicVisibility = useCallback(
    (topicName: string) => {
      if (!renderer) return;
      const cfg: any = (renderer as any).config;
      cfg.topics ??= {};
      cfg.topics[topicName] ??= {};
      const current = cfg.topics[topicName].visible ?? true;
      cfg.topics[topicName].visible = !current;
      renderer.queueAnimationFrame();
    },
    [renderer],
  );

  // ---- addPanel stub (RendererOverlay expects it) ---------------------------------

  const addPanel = useCallback((_params: any) => {
    // No-op in standalone mode
  }, []);

  // ---- Render ---------------------------------------------------------------------

  const { className, style } = props;

  return (
    <ConnectionSettingsProvider onSettingsChange={(settings) => {
      // Handle connection settings changes - could trigger reconnection
      console.log('Connection settings changed:', settings);
      // TODO: Implement reconnection logic if needed
    }}>
      <ThemeProvider isDark={false}>
        <div
          className={className}
          style={{ ...PANEL_STYLE, ...(style ?? {}) }}
          onKeyDown={onKeyDown}
        >
          <canvas
            ref={setCanvas}
            style={{
              position: "absolute",
              top: 0,
              left: 0,
              width: "100%",
              height: "100%",
              ...(measureActive && { cursor: "crosshair" }),
            }}
          />

          {/* Small control panel in top-left */}
          <div
            style={{
              position: "absolute",
              top: 8,
              left: 8,
              zIndex: 10,
              pointerEvents: "auto",
              background: "rgba(0,0,0,0.7)",
              color: "#fff",
              padding: 8,
              borderRadius: 6,
              fontSize: 12,
              maxWidth: 360,
              maxHeight: "60%",
              overflow: "auto",
            }}
          >
            <div style={{ marginBottom: 8, fontWeight: 600, display: 'flex', alignItems: 'center', gap: '8px' }}>
              3D Controls
              <ConnectionSettingsTrigger />
            </div>

            {frames.length > 0 && (
              <div style={{ marginBottom: 8 }}>
                <label>
                  Frame:{" "}
                  <select
                    value={displayFrame ?? ""}
                    onChange={(e) =>
                      setDisplayFrame(e.target.value || undefined)
                    }
                    style={{ marginLeft: 4, maxWidth: 260 }}
                  >
                    {frames.map((f) => (
                      <option key={f} value={f}>
                        {f}
                      </option>
                    ))}
                  </select>
                </label>
              </div>
            )}

            <div style={{ marginBottom: 8 }}>
              <label style={{ display: "block", marginBottom: 4 }}>
                Point size: <strong>{pointSize}</strong>
              </label>
              <input
                type="range"
                min={1}
                max={10}
                step={1}
                value={pointSize}
                onChange={(e) => setPointSize(Number(e.target.value))}
                style={{ width: "100%" }}
              />
            </div>

            <div style={{ marginBottom: 8 }}>
              <label>
                Decay (s):{" "}
                <input
                  type="number"
                  min={0}
                  step={0.1}
                  value={decayTime}
                  onChange={(e) => {
                    const v = Number(e.target.value);
                    setDecayTime(Number.isFinite(v) && v >= 0 ? v : 0);
                  }}
                  style={{ width: 70, marginLeft: 4 }}
                />
              </label>
            </div>

            <div>
              <div style={{ marginBottom: 4, fontWeight: 500 }}>Topics</div>
              {topics.length === 0 && (
                <div style={{ fontStyle: "italic", opacity: 0.7 }}>
                  No topics yet…
                </div>
              )}
              {topics.map((t) => (
                <label
                  key={t.name}
                  style={{ display: "flex", alignItems: "center", marginBottom: 2 }}
                >
                  <input
                    type="checkbox"
                    checked={isTopicVisible(t.name)}
                    onChange={() => toggleTopicVisibility(t.name)}
                    style={{ marginRight: 6 }}
                  />
                  <span
                    style={{
                      overflow: "hidden",
                      textOverflow: "ellipsis",
                      whiteSpace: "nowrap",
                    }}
                  >
                    {t.name}
                  </span>
                </label>
              ))}
            </div>
          </div>

          <RendererContext.Provider value={renderer}>
            <RendererOverlay
              interfaceMode={interfaceMode}
              canvas={canvas}
              addPanel={addPanel as any}
              enableStats={false}
              perspective={perspective}
              onTogglePerspective={onTogglePerspective}
              measureActive={measureActive}
              onClickMeasure={onClickMeasure}
              canPublish={false}
              publishActive={false}
              onClickPublish={() => {
                /* publishing disabled in this standalone view */
              }}
              onShowTopicSettings={() => {
                /* no settings sidebar here */
              }}
              publishClickType={"point"}
              onChangePublishClickType={() => {
                /* publishing disabled */
              }}
              timezone={undefined}
            />
          </RendererContext.Provider>
        </div>
      </ThemeProvider>
    </ConnectionSettingsProvider>
  );
};

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

function estimateSizeInBytes(obj: any): number {
  try {
    const json = JSON.stringify(obj);
    return typeof json === "string" ? json.length : 0;
  } catch {
    return 0;
  }
}
