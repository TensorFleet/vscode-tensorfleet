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
const LS_SHOW_TF_KEY = "sensor3d.showTf";
const LS_FOLLOW_FRAME_KEY = "sensor3d.followFrame";
const LS_PANEL_COLLAPSED_KEY = "sensor3d.panelCollapsed";
const LS_LIDAR_COLOR_FIELD_KEY = "sensor3d.lidar.colorField";
const LS_LIDAR_COLOR_MAP_KEY = "sensor3d.lidar.colorMap";
const LS_LIDAR_POINT_SHAPE_KEY = "sensor3d.lidar.pointShape";
const LS_LIDAR_OPACITY_KEY = "sensor3d.lidar.opacity";
const AUTO_VISIBLE_SCHEMA_NAMES = new Set([
  "sensor_msgs/LaserScan",
  "sensor_msgs/msg/LaserScan",
  "foxglove.LaserScan",
  "sensor_msgs/PointCloud2",
  "sensor_msgs/msg/PointCloud2",
  "foxglove.PointCloud",
]);
const TF_SCHEMA_NAMES = new Set(["tf2_msgs/TFMessage", "tf2_msgs/msg/TFMessage"]);
const PANEL_LIST_SCHEMA_HINTS = [
  "LaserScan",
  "PointCloud",
  "Marker",
  "Pose",
  "Path",
  "Polygon",
  "Grid",
  "SceneEntity",
];
const WORLD_FRAME_NAMES = new Set(["map", "odom", "world", "earth"]);

const PANEL_COLORS = {
  canvasBackground: "#09111f",
  cardBackground: "rgba(10, 18, 32, 0.78)",
  cardBorder: "rgba(142, 197, 255, 0.18)",
  cardShadow: "0 18px 48px rgba(0, 0, 0, 0.35)",
  textPrimary: "#f5f7fb",
  textSecondary: "rgba(229, 238, 255, 0.72)",
  accent: "#69b7ff",
  accentMuted: "rgba(105, 183, 255, 0.18)",
  inputBackground: "rgba(255, 255, 255, 0.08)",
  inputBorder: "rgba(255, 255, 255, 0.12)",
};

function shouldAutoShowTopic(topic: Pick<Topic, "name" | "schemaName">): boolean {
  if (AUTO_VISIBLE_SCHEMA_NAMES.has(topic.schemaName)) {
    return true;
  }
  return (
    topic.name === "/scan" ||
    topic.name === "/x500/scan" ||
    topic.name.endsWith("/points")
  );
}

function isLaserScanTopic(topic: Pick<Topic, "name" | "schemaName">): boolean {
  return (
    topic.schemaName.includes("LaserScan") ||
    topic.name === "/scan" ||
    topic.name === "/x500/scan"
  );
}

function shouldListTopicInPanel(topic: Pick<Topic, "name" | "schemaName">): boolean {
  if (shouldAutoShowTopic(topic)) {
    return true;
  }
  return PANEL_LIST_SCHEMA_HINTS.some((hint) => topic.schemaName.includes(hint));
}

function choosePreferredFrame(frameNames: string[], current?: string): string | undefined {
  if (current && frameNames.includes(current)) {
    return current;
  }

  const exactPriority = [
    "odom",
    "map",
    "world",
    "base_link",
    "simple_bot/base_link",
    "x500_0",
  ];
  for (const preferred of exactPriority) {
    if (frameNames.includes(preferred)) {
      return preferred;
    }
  }

  const suffixPriority = ["/odom", "/map", "/world", "/base_link"];
  for (const suffix of suffixPriority) {
    const match = frameNames.find((name) => name.endsWith(suffix));
    if (match) {
      return match;
    }
  }

  return frameNames[0];
}

function isWorldLikeFrame(frameName: string): boolean {
  if (WORLD_FRAME_NAMES.has(frameName)) {
    return true;
  }
  return (
    frameName.endsWith("/map") ||
    frameName.endsWith("/odom") ||
    frameName.endsWith("/world")
  );
}

function isSensorLikeFrame(frameName: string): boolean {
  const lower = frameName.toLowerCase();
  return [
    "camera",
    "lidar",
    "laser",
    "imu",
    "gps",
    "rotor",
    "propeller",
    "gimbal",
    "sensor",
    "optical",
  ].some((part) => lower.includes(part));
}

function choosePoseFrames(frameNames: string[], displayFrame?: string): string[] {
  const preferred: string[] = [];
  const tryAdd = (frameName: string | undefined) => {
    if (!frameName || preferred.includes(frameName)) {
      return;
    }
    if (isWorldLikeFrame(frameName) || isSensorLikeFrame(frameName)) {
      return;
    }
    preferred.push(frameName);
  };

  tryAdd(displayFrame);

  const exactMatches = [
    "base_link",
    "base_footprint",
    "simple_bot/base_link",
    "simple_bot",
    "x500_0",
  ];
  for (const frameName of exactMatches) {
    if (frameNames.includes(frameName)) {
      tryAdd(frameName);
    }
  }

  for (const frameName of frameNames) {
    if (
      frameName.endsWith("/base_link") ||
      frameName.endsWith("/base_footprint") ||
      /^x500_\d+$/.test(frameName)
    ) {
      tryAdd(frameName);
    }
  }

  if (preferred.length === 0 && displayFrame && !isWorldLikeFrame(displayFrame)) {
    preferred.push(displayFrame);
  }

  return preferred.slice(0, 3);
}

function isTfTopic(topic: Pick<Topic, "name" | "schemaName">): boolean {
  return (
    topic.name === "/tf" ||
    topic.name === "/tf_static" ||
    TF_SCHEMA_NAMES.has(topic.schemaName)
  );
}

function sortTopicsForPanel(a: Topic, b: Topic): number {
  const aPriority = shouldAutoShowTopic(a) ? 0 : 1;
  const bPriority = shouldAutoShowTopic(b) ? 0 : 1;
  if (aPriority !== bPriority) {
    return aPriority - bPriority;
  }
  return a.name.localeCompare(b.name);
}

function defaultTopicSettings(
  topic: Pick<Topic, "name" | "schemaName">,
  pointSize: number,
  decayTime: number,
) {
  const base = {
    visible: true,
    pointSize,
    decayTime,
  };

  if (topic.schemaName.includes("LaserScan")) {
    return {
      ...base,
      pointShape: "circle" as const,
      colorMode: "colormap" as const,
      colorField: "range" as const,
      colorMap: "turbo" as const,
      explicitAlpha: 0.92,
    };
  }

  return base;
}

export const Sensor3DViewPanel: React.FC<Sensor3DViewPanelProps> = (props) => {
  const interfaceMode: InterfaceMode = "3d";

  const [canvas, setCanvas] = useState<HTMLCanvasElement | null>(null);
  const [renderer, setRenderer] = useState<Renderer | undefined>(undefined);

  // Simple, fixed config – no external panel state
  const initialConfigRef = useRef<RendererConfig>({
    cameraState: {
      ...DEFAULT_CAMERA_STATE,
      distance: 16,
      phi: 58,
      thetaOffset: -35,
      near: 0.05,
    },
    followMode: "follow-none",
    followTf: undefined,
    scene: {
      backgroundColor: PANEL_COLORS.canvasBackground,
      labelScaleFactor: 0.85,
      transforms: {
        showLabel: false,
        axisScale: 0,
        lineWidth: 0,
        lineColor: "#3c7ec9",
      },
    },
    transforms: {},
    topics: {},
    layers: {
      grid: {
        layerId: "foxglove.Grid",
        frameId: undefined,
        size: 24,
        divisions: 24,
        lineWidth: 1,
        color: "#245fb0",
        position: [0, 0, 0],
        rotation: [0, 0, 0],
        label: "Ground",
        visible: true,
      },
    },
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
  const [showTf, setShowTf] = useState<boolean>(() => {
    if (typeof window === "undefined") return false;
    return window.localStorage.getItem(LS_SHOW_TF_KEY) === "true";
  });
  const [followFrame, setFollowFrame] = useState<boolean>(() => {
    if (typeof window === "undefined") return false;
    return window.localStorage.getItem(LS_FOLLOW_FRAME_KEY) === "true";
  });
  const [panelCollapsed, setPanelCollapsed] = useState<boolean>(() => {
    if (typeof window === "undefined") return false;
    return window.localStorage.getItem(LS_PANEL_COLLAPSED_KEY) === "true";
  });
  const [lidarColorField, setLidarColorField] = useState<"range" | "intensity">(() => {
    if (typeof window === "undefined") return "range";
    const stored = window.localStorage.getItem(LS_LIDAR_COLOR_FIELD_KEY);
    return stored === "intensity" ? "intensity" : "range";
  });
  const [lidarColorMap, setLidarColorMap] = useState<"turbo" | "rainbow">(() => {
    if (typeof window === "undefined") return "turbo";
    return window.localStorage.getItem(LS_LIDAR_COLOR_MAP_KEY) === "rainbow"
      ? "rainbow"
      : "turbo";
  });
  const [lidarPointShape, setLidarPointShape] = useState<"circle" | "square">(() => {
    if (typeof window === "undefined") return "circle";
    return window.localStorage.getItem(LS_LIDAR_POINT_SHAPE_KEY) === "square"
      ? "square"
      : "circle";
  });
  const [lidarOpacity, setLidarOpacity] = useState<number>(() => {
    if (typeof window === "undefined") return 0.92;
    const stored = window.localStorage.getItem(LS_LIDAR_OPACITY_KEY);
    const n = stored != null ? Number(stored) : NaN;
    return Number.isFinite(n) && n > 0 && n <= 1 ? n : 0.92;
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
    r.setColorScheme("dark", PANEL_COLORS.canvasBackground);

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
      setDisplayFrame((prev) => choosePreferredFrame(arr, prev));
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

      const topicsNeedingDefaults = topicObjects.filter((topic) => {
        if (!shouldAutoShowTopic(topic)) {
          return false;
        }
        const prev = (renderer.config as any).topics?.[topic.name];
        return prev?.visible === undefined;
      });

      if (topicsNeedingDefaults.length > 0) {
        renderer.updateConfig((draft) => {
          draft.topics ??= {};
          for (const topic of topicsNeedingDefaults) {
            const prev = draft.topics[topic.name] ?? {};
            draft.topics[topic.name] = {
              ...prev,
              ...defaultTopicSettings(topic, pointSize, decayTime),
              ...prev,
              visible: prev.visible ?? true,
              pointSize: prev.pointSize ?? pointSize,
              decayTime: prev.decayTime ?? decayTime,
            };
          }
        });
      }

      renderer.setTopics(topicObjects);
      const desired = computeDesiredSubscriptions(topicsList, renderer);
      const visibleTopics = topicObjects
        .filter(
          (topic) =>
            desired.some((item) => item.topic === topic.name) &&
            !isTfTopic(topic) &&
            shouldListTopicInPanel(topic),
        )
        .sort(sortTopicsForPanel);
      setTopics(visibleTopics);
      reconcileSubscriptions(desired, renderer);
      if (topicsNeedingDefaults.length > 0) {
        renderer.queueAnimationFrame();
      }
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
  }, [renderer, updateFramesFromTfMessage, pointSize, decayTime]);

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

  useEffect(() => {
    if (typeof window !== "undefined") {
      try {
        window.localStorage.setItem(LS_SHOW_TF_KEY, String(showTf));
      } catch {
        // ignore
      }
    }
    if (!renderer) return;
    renderer.updateConfig((draft) => {
      draft.scene ??= {};
      draft.scene.transforms ??= {};
      draft.scene.transforms.showLabel = false;
      draft.scene.transforms.labelSize = 0.12;
      draft.scene.transforms.axisScale = showTf ? 0.14 : 0.3;
      draft.scene.transforms.lineWidth = showTf ? 0.8 : 1.2;
      draft.scene.transforms.lineColor = "#3c7ec9";

      const poseFrames = choosePoseFrames(frames, displayFrame);
      const visibleFrameSet = new Set(showTf ? frames : poseFrames);
      draft.transforms ??= {};
      for (const frameName of frames) {
        const frameKey = `frame:${frameName}`;
        draft.transforms[frameKey] = {
          ...(draft.transforms[frameKey] ?? {}),
          visible: visibleFrameSet.has(frameName),
        };
      }
    });
    renderer.queueAnimationFrame();
  }, [renderer, showTf, frames, displayFrame]);

  useEffect(() => {
    if (typeof window !== "undefined") {
      try {
        window.localStorage.setItem(LS_FOLLOW_FRAME_KEY, String(followFrame));
      } catch {
        // ignore
      }
    }
    if (!renderer) return;
    renderer.config = {
      ...renderer.config,
      followMode: followFrame ? "follow-position" : "follow-none",
    };
    renderer.queueAnimationFrame();
  }, [renderer, followFrame]);

  useEffect(() => {
    if (typeof window !== "undefined") {
      try {
        window.localStorage.setItem(LS_PANEL_COLLAPSED_KEY, String(panelCollapsed));
      } catch {
        // ignore
      }
    }
  }, [panelCollapsed]);

  useEffect(() => {
    if (typeof window !== "undefined") {
      try {
        window.localStorage.setItem(LS_LIDAR_COLOR_FIELD_KEY, lidarColorField);
        window.localStorage.setItem(LS_LIDAR_COLOR_MAP_KEY, lidarColorMap);
        window.localStorage.setItem(LS_LIDAR_POINT_SHAPE_KEY, lidarPointShape);
        window.localStorage.setItem(LS_LIDAR_OPACITY_KEY, String(lidarOpacity));
      } catch {
        // ignore
      }
    }
    if (!renderer) return;
    const lidarTopics = topics.filter(isLaserScanTopic);
    if (lidarTopics.length === 0) return;
    renderer.updateConfig((draft) => {
      draft.topics ??= {};
      for (const topic of lidarTopics) {
        const prev = draft.topics[topic.name] ?? {};
        draft.topics[topic.name] = {
          ...prev,
          colorMode: "colormap",
          colorField: lidarColorField,
          colorMap: lidarColorMap,
          pointShape: lidarPointShape,
          explicitAlpha: lidarOpacity,
        };
      }
    });
    renderer.queueAnimationFrame();
  }, [renderer, topics, lidarColorField, lidarColorMap, lidarPointShape, lidarOpacity]);

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
      if (cfg?.visible !== undefined) return !!cfg.visible;
      const topic = topics.find((candidate) => candidate.name === topicName);
      if (!topic) return false;
      return shouldAutoShowTopic(topic);
    },
    [renderer, topics],
  );

  const toggleTopicVisibility = useCallback(
    (topicName: string) => {
      if (!renderer) return;
      renderer.updateConfig((draft) => {
        draft.topics ??= {};
        draft.topics[topicName] ??= {};
        const current = draft.topics[topicName]?.visible ?? isTopicVisible(topicName);
        draft.topics[topicName]!.visible = !current;
      });
      renderer.queueAnimationFrame();
    },
    [renderer, isTopicVisible],
  );

  // ---- addPanel stub (RendererOverlay expects it) ---------------------------------

  const addPanel = useCallback((_params: any) => {
    // No-op in standalone mode
  }, []);

  // ---- Render ---------------------------------------------------------------------

  const { className, style } = props;
  const poseFrames = choosePoseFrames(frames, displayFrame);

  return (
    <ConnectionSettingsProvider onSettingsChange={(settings) => {
      // Handle connection settings changes - could trigger reconnection
      console.log('Connection settings changed:', settings);
      // TODO: Implement reconnection logic if needed
    }}>
      <ThemeProvider isDark={true}>
        <div
          className={className}
          style={{
            ...PANEL_STYLE,
            ...(style ?? {}),
            background:
              "radial-gradient(circle at top, rgba(25, 48, 78, 0.85) 0%, rgba(10, 17, 31, 0.96) 58%, rgba(6, 11, 20, 1) 100%)",
          }}
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
              top: 14,
              left: 14,
              zIndex: 10,
              pointerEvents: "auto",
              background: PANEL_COLORS.cardBackground,
              color: PANEL_COLORS.textPrimary,
              padding: 14,
              borderRadius: 14,
              border: `1px solid ${PANEL_COLORS.cardBorder}`,
              boxShadow: PANEL_COLORS.cardShadow,
              backdropFilter: "blur(14px)",
              fontSize: 12,
              width: panelCollapsed ? 220 : 320,
              maxWidth: "calc(100% - 28px)",
              maxHeight: panelCollapsed ? undefined : "68%",
              overflow: "auto",
            }}
          >
            <div
              style={{
                marginBottom: panelCollapsed ? 0 : 12,
                display: "flex",
                alignItems: "center",
                justifyContent: "space-between",
                gap: 10,
              }}
            >
              <div>
                <div
                  style={{
                    fontSize: 14,
                    fontWeight: 700,
                    letterSpacing: "0.04em",
                    textTransform: "uppercase",
                  }}
                >
                  Sensor View
                </div>
                <div style={{ marginTop: 3, color: PANEL_COLORS.textSecondary }}>
                  Clean 3D scan view with world-frame controls
                </div>
              </div>
              <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                <button
                  type="button"
                  onClick={() => setPanelCollapsed((prev) => !prev)}
                  title={panelCollapsed ? "Expand controls" : "Collapse controls"}
                  style={{
                    border: `1px solid ${PANEL_COLORS.cardBorder}`,
                    background: panelCollapsed ? "rgba(105, 183, 255, 0.18)" : "rgba(255,255,255,0.06)",
                    color: PANEL_COLORS.textPrimary,
                    borderRadius: 8,
                    width: 30,
                    height: 30,
                    cursor: "pointer",
                    fontSize: 16,
                    lineHeight: 1,
                  }}
                >
                  {panelCollapsed ? "+" : "-"}
                </button>
                {!panelCollapsed && <ConnectionSettingsTrigger />}
              </div>
            </div>

            {panelCollapsed && (
              <div style={{ marginTop: 10, color: PANEL_COLORS.textSecondary }}>
                <div>Frame: {displayFrame ?? "none"}</div>
                <div>Topics: {topics.length}</div>
              </div>
            )}

            {!panelCollapsed && poseFrames.length > 0 && (
              <div style={{ marginBottom: 12 }}>
                <div style={{ marginBottom: 6, color: PANEL_COLORS.textSecondary }}>
                  Pose frames
                </div>
                <div style={{ display: "flex", flexWrap: "wrap", gap: 6 }}>
                  {poseFrames.map((frameName) => (
                    <button
                      key={frameName}
                      type="button"
                      onClick={() => setDisplayFrame(frameName)}
                      style={{
                        border: "none",
                        borderRadius: 999,
                        padding: "6px 10px",
                        cursor: "pointer",
                        background:
                          frameName === displayFrame
                            ? "rgba(105, 183, 255, 0.22)"
                            : "rgba(255, 255, 255, 0.06)",
                        color: PANEL_COLORS.textPrimary,
                        fontSize: 11,
                        fontFamily:
                          'ui-monospace, SFMono-Regular, SF Mono, Menlo, Consolas, monospace',
                      }}
                    >
                      {frameName}
                    </button>
                  ))}
                </div>
              </div>
            )}

            {!panelCollapsed && frames.length > 0 && (
              <div style={{ marginBottom: 12 }}>
                <label style={{ display: "block", marginBottom: 5, color: PANEL_COLORS.textSecondary }}>
                  Display frame
                </label>
                <div
                  style={{
                    display: "flex",
                    gap: 8,
                    alignItems: "center",
                  }}
                >
                  <select
                    value={displayFrame ?? ""}
                    onChange={(e) =>
                      setDisplayFrame(e.target.value || undefined)
                    }
                    style={{
                      flex: 1,
                      maxWidth: "100%",
                      background: PANEL_COLORS.inputBackground,
                      color: PANEL_COLORS.textPrimary,
                      border: `1px solid ${PANEL_COLORS.inputBorder}`,
                      borderRadius: 8,
                      padding: "8px 10px",
                    }}
                  >
                    {frames.map((f) => (
                      <option key={f} value={f}>
                        {f}
                      </option>
                    ))}
                  </select>
                </div>
              </div>
            )}

            {!panelCollapsed && (
            <div
              style={{
                display: "grid",
                gridTemplateColumns: "1fr auto",
                gap: 12,
                marginBottom: 14,
                padding: 10,
                borderRadius: 10,
                background: PANEL_COLORS.accentMuted,
                border: `1px solid ${PANEL_COLORS.cardBorder}`,
              }}
            >
              <label
                style={{
                  display: "flex",
                  alignItems: "center",
                  gap: 8,
                  color: PANEL_COLORS.textSecondary,
                }}
              >
                <input
                  type="checkbox"
                  checked={showTf}
                  onChange={(e) => setShowTf(e.target.checked)}
                />
                Show TF frames
              </label>
              <div style={{ color: PANEL_COLORS.textSecondary }}>
                {topics.length} visual topic{topics.length === 1 ? "" : "s"}
              </div>
            </div>
            )}

            {!panelCollapsed && (
            <div
              style={{
                display: "grid",
                gridTemplateColumns: "1fr auto",
                gap: 12,
                marginBottom: 14,
                padding: 10,
                borderRadius: 10,
                background: "rgba(255,255,255,0.04)",
                border: `1px solid ${PANEL_COLORS.cardBorder}`,
              }}
            >
              <label
                style={{
                  display: "flex",
                  alignItems: "center",
                  gap: 8,
                  color: PANEL_COLORS.textSecondary,
                }}
              >
                <input
                  type="checkbox"
                  checked={followFrame}
                  onChange={(e) => setFollowFrame(e.target.checked)}
                  disabled={!displayFrame}
                />
                Follow selected frame
              </label>
              <div style={{ color: PANEL_COLORS.textSecondary }}>
                {displayFrame ?? "no frame"}
              </div>
            </div>
            )}

            {!panelCollapsed && (
            <div style={{ marginBottom: 12 }}>
              <label style={{ display: "block", marginBottom: 6, color: PANEL_COLORS.textSecondary }}>
                Point size <strong style={{ color: PANEL_COLORS.textPrimary }}>{pointSize}</strong>
              </label>
              <input
                type="range"
                min={1}
                max={10}
                step={1}
                value={pointSize}
                onChange={(e) => setPointSize(Number(e.target.value))}
                style={{ width: "100%", accentColor: PANEL_COLORS.accent }}
              />
            </div>
            )}

            {!panelCollapsed && (
            <div
              style={{
                marginBottom: 14,
                padding: 10,
                borderRadius: 10,
                background: "rgba(255,255,255,0.04)",
                border: `1px solid ${PANEL_COLORS.cardBorder}`,
              }}
            >
              <div style={{ marginBottom: 8, fontWeight: 600 }}>Lidar Style</div>

              <div style={{ marginBottom: 8 }}>
                <label style={{ display: "block", marginBottom: 5, color: PANEL_COLORS.textSecondary }}>
                  Color by
                </label>
                <select
                  value={lidarColorField}
                  onChange={(e) => setLidarColorField(e.target.value as "range" | "intensity")}
                  style={{
                    width: "100%",
                    background: PANEL_COLORS.inputBackground,
                    color: PANEL_COLORS.textPrimary,
                    border: `1px solid ${PANEL_COLORS.inputBorder}`,
                    borderRadius: 8,
                    padding: "8px 10px",
                  }}
                >
                  <option value="range">Range</option>
                  <option value="intensity">Intensity</option>
                </select>
              </div>

              <div style={{ marginBottom: 8 }}>
                <label style={{ display: "block", marginBottom: 5, color: PANEL_COLORS.textSecondary }}>
                  Palette
                </label>
                <select
                  value={lidarColorMap}
                  onChange={(e) => setLidarColorMap(e.target.value as "turbo" | "rainbow")}
                  style={{
                    width: "100%",
                    background: PANEL_COLORS.inputBackground,
                    color: PANEL_COLORS.textPrimary,
                    border: `1px solid ${PANEL_COLORS.inputBorder}`,
                    borderRadius: 8,
                    padding: "8px 10px",
                  }}
                >
                  <option value="turbo">Turbo</option>
                  <option value="rainbow">Rainbow</option>
                </select>
              </div>

              <div style={{ marginBottom: 8 }}>
                <label style={{ display: "block", marginBottom: 5, color: PANEL_COLORS.textSecondary }}>
                  Point shape
                </label>
                <select
                  value={lidarPointShape}
                  onChange={(e) => setLidarPointShape(e.target.value as "circle" | "square")}
                  style={{
                    width: "100%",
                    background: PANEL_COLORS.inputBackground,
                    color: PANEL_COLORS.textPrimary,
                    border: `1px solid ${PANEL_COLORS.inputBorder}`,
                    borderRadius: 8,
                    padding: "8px 10px",
                  }}
                >
                  <option value="circle">Circle</option>
                  <option value="square">Square</option>
                </select>
              </div>

              <div>
                <label style={{ display: "block", marginBottom: 6, color: PANEL_COLORS.textSecondary }}>
                  Opacity <strong style={{ color: PANEL_COLORS.textPrimary }}>{lidarOpacity.toFixed(2)}</strong>
                </label>
                <input
                  type="range"
                  min={0.1}
                  max={1}
                  step={0.05}
                  value={lidarOpacity}
                  onChange={(e) => setLidarOpacity(Number(e.target.value))}
                  style={{ width: "100%", accentColor: PANEL_COLORS.accent }}
                />
              </div>
            </div>
            )}

            {!panelCollapsed && (
            <div style={{ marginBottom: 14 }}>
              <label style={{ display: "block", marginBottom: 6, color: PANEL_COLORS.textSecondary }}>
                Scan persistence
              </label>
              <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                <input
                  type="number"
                  min={0}
                  step={0.1}
                  value={decayTime}
                  onChange={(e) => {
                    const v = Number(e.target.value);
                    setDecayTime(Number.isFinite(v) && v >= 0 ? v : 0);
                  }}
                  style={{
                    width: 76,
                    background: PANEL_COLORS.inputBackground,
                    color: PANEL_COLORS.textPrimary,
                    border: `1px solid ${PANEL_COLORS.inputBorder}`,
                    borderRadius: 8,
                    padding: "8px 10px",
                  }}
                />
                <span style={{ color: PANEL_COLORS.textSecondary }}>
                  seconds
                </span>
              </div>
            </div>
            )}

            {!panelCollapsed && (
            <div>
              <div
                style={{
                  marginBottom: 8,
                  fontWeight: 600,
                  display: "flex",
                  justifyContent: "space-between",
                  alignItems: "center",
                }}
              >
                <span>Visual Topics</span>
                <span style={{ color: PANEL_COLORS.textSecondary, fontWeight: 500 }}>
                  scan and cloud sources
                </span>
              </div>
              {topics.length === 0 && (
                <div
                  style={{
                    fontStyle: "italic",
                    color: PANEL_COLORS.textSecondary,
                    padding: "10px 12px",
                    background: "rgba(255,255,255,0.04)",
                    borderRadius: 10,
                  }}
                >
                  No visual topics discovered yet.
                </div>
              )}
              <div style={{ display: "grid", gap: 6 }}>
                {topics.map((t) => (
                  <label
                    key={t.name}
                    style={{
                      display: "flex",
                      alignItems: "center",
                      gap: 10,
                      padding: "9px 10px",
                      borderRadius: 10,
                      background: isTopicVisible(t.name)
                        ? "rgba(105, 183, 255, 0.12)"
                        : "rgba(255, 255, 255, 0.04)",
                      border: `1px solid ${
                        isTopicVisible(t.name)
                          ? "rgba(105, 183, 255, 0.28)"
                          : "rgba(255, 255, 255, 0.08)"
                      }`,
                    }}
                  >
                    <input
                      type="checkbox"
                      checked={isTopicVisible(t.name)}
                      onChange={() => toggleTopicVisibility(t.name)}
                    />
                    <span
                      style={{
                        overflow: "hidden",
                        textOverflow: "ellipsis",
                        whiteSpace: "nowrap",
                        fontFamily:
                          'ui-monospace, SFMono-Regular, SF Mono, Menlo, Consolas, monospace',
                        fontSize: 11.5,
                      }}
                    >
                      {t.name}
                    </span>
                  </label>
                ))}
              </div>
            </div>
            )}
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
