// Sensor3DViewPanel.tsx
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

export type Sensor3DViewPanelProps = {
  className?: string;
  style?: React.CSSProperties;
};

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
    // imageMode isn't used in 3d mode, but must satisfy type
    imageMode: {} as Partial<ImageModeConfig> as ImageModeConfig,
  });

  const [measureActive, setMeasureActive] = useState(false);
  const [perspective, setPerspective] = useState(
    initialConfigRef.current.cameraState.perspective,
  );

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
        // You can hook this into your snackbar/toast system if you want
        // eslint-disable-next-line no-console
        console.error("[Sensor3DViewPanel] temporary error:", msg);
      },
      testOptions: {},
      customCameraModels: new Map(),
    });

    // Tell renderer we're on a ROS data source so frame IDs get normalized
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

  // ---- ROS2Bridge wiring: available topics + live messages -----------------------

  useEffect(() => {
    if (!renderer) return;

    const liveSubs = liveSubsRef.current;

    const computeDesiredSubscriptions = (
      topics: Ros2BridgeSubscription[],
      r: Renderer,
    ): Ros2BridgeSubscription[] => {
      const desired: Ros2BridgeSubscription[] = [];

      const schemaSubs = r.schemaSubscriptions as Map<string, RendererSubscription[]>;
      const topicSubs = r.topicSubscriptions as Map<string, RendererSubscription[]>;

      for (const t of topics) {
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

    const handleTopicsChanged = (topics: Ros2BridgeSubscription[]) => {
      // Convert ros2Bridge topics to minimal Topic[] for renderer
      const topicObjects: Topic[] = topics.map(
        (t) =>
          ({
            name: t.topic,
            schemaName: t.type,
            // Datatype field is used in some places; set it to the same string
            datatype: t.type,
          } as unknown as Topic),
      );

      renderer.setTopics(topicObjects);

      const desired = computeDesiredSubscriptions(topics, renderer);
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
  }, [renderer]);

  // ---- addPanel stub (RendererOverlay expects it) ---------------------------------

  const addPanel = useCallback((_params: any) => {
    // No-op in standalone mode
  }, []);

  // ---- Render ---------------------------------------------------------------------

  const { className, style } = props;

  return (
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
