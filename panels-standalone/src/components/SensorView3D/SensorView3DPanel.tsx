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
import type { MessageEvent, SettingsTreeAction, Topic } from "@lichtblick/suite";

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
import {
  DEFAULT_SCENE_EXTENSION_CONFIG,
  type SceneExtensionConfig,
} from "@lichtblick/suite-base/panels/ThreeDeeRender/SceneExtensionConfig";
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
const LS_SHOW_DIAGNOSTICS_KEY = "sensor3d.showDiagnostics";
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
  "nav_msgs/OccupancyGrid",
  "nav_msgs/msg/OccupancyGrid",
  "ros.nav_msgs.OccupancyGrid",
]);
const TF_SCHEMA_NAMES = new Set([
  "tf2_msgs/TFMessage",
  "tf2_msgs/msg/TFMessage",
  "ros.tf2_msgs.TFMessage",
]);
const ODOMETRY_SCHEMA_NAMES = new Set([
  "nav_msgs/Odometry",
  "nav_msgs/msg/Odometry",
  "ros.nav_msgs.Odometry",
]);
const OCCUPANCY_GRID_SCHEMA_NAMES = new Set([
  "nav_msgs/OccupancyGrid",
  "nav_msgs/msg/OccupancyGrid",
  "ros.nav_msgs.OccupancyGrid",
]);
const POLYGON_STAMPED_SCHEMA_NAME = "geometry_msgs/msg/PolygonStamped";
const BARE_POLYGON_SCHEMA_NAMES = new Set([
  "geometry_msgs/Polygon",
  "geometry_msgs/msg/Polygon",
  "ros.geometry_msgs.Polygon",
]);
const TURTLEBOT4_AUTO_VISIBLE_TOPIC_NAMES = new Set([
  "/scan",
  "/map",
  "/local_costmap/costmap",
  "/global_costmap/costmap",
]);
const STANDALONE_3D_HIDDEN_BY_DEFAULT_TOPIC_NAMES = new Set([
  "/oakd/rgb/preview/depth/points",
]);
const TURTLEBOT4_SENSOR_TOPIC_DEFAULTS: Ros2BridgeSubscription[] = [
  { topic: "/scan", type: "sensor_msgs/msg/LaserScan" },
  { topic: "/odom", type: "nav_msgs/msg/Odometry" },
  { topic: "/tf", type: "tf2_msgs/msg/TFMessage" },
  { topic: "/tf_static", type: "tf2_msgs/msg/TFMessage" },
  { topic: "/oakd/rgb/preview/depth/points", type: "sensor_msgs/msg/PointCloud2" },
  { topic: "/map", type: "nav_msgs/msg/OccupancyGrid" },
  { topic: "/local_costmap/costmap", type: "nav_msgs/msg/OccupancyGrid" },
  { topic: "/global_costmap/costmap", type: "nav_msgs/msg/OccupancyGrid" },
];
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
const ZERO_TRANSLATION = { x: 0, y: 0, z: 0 };
const IDENTITY_ROTATION = { x: 0, y: 0, z: 0, w: 1 };
const SIMPLE_ROBOT_CAMERA_TRANSLATION = { x: 0.25, y: 0, z: 0.15 };
const SIMPLE_ROBOT_LIDAR_TRANSLATION = { x: 0, y: 0, z: 0.3 };
const CAMERA_OPTICAL_ROTATION = { x: -0.5, y: 0.5, z: -0.5, w: 0.5 };
const STANDALONE_3D_SCENE_EXTENSION_CONFIG: SceneExtensionConfig = {
  ...DEFAULT_SCENE_EXTENSION_CONFIG,
  extensionsById: Object.fromEntries(
    Object.entries(DEFAULT_SCENE_EXTENSION_CONFIG.extensionsById).filter(
      ([extensionId]) => extensionId !== "foxglove.Images",
    ),
  ),
};

function getActiveVmConfigId(): string {
  return (typeof window !== "undefined" ? (window as any).TENSORFLEET_VM_CONFIG_ID : "") ?? "";
}

function mergeTopicDefaults(
  topicsList: Ros2BridgeSubscription[],
  defaults: Ros2BridgeSubscription[],
): Ros2BridgeSubscription[] {
  const merged = new Map<string, Ros2BridgeSubscription>();
  for (const sub of defaults) {
    merged.set(sub.topic, sub);
  }
  for (const sub of topicsList) {
    merged.set(sub.topic, sub);
  }
  return Array.from(merged.values());
}

type TopicLike = Partial<Pick<Topic, "name" | "schemaName">>;

function shouldAutoShowTopic(topic: TopicLike): boolean {
  const name = topic.name ?? "";
  const schemaName = topic.schemaName ?? "";
  if (STANDALONE_3D_HIDDEN_BY_DEFAULT_TOPIC_NAMES.has(name)) {
    return false;
  }
  if (AUTO_VISIBLE_SCHEMA_NAMES.has(schemaName)) {
    return true;
  }
  if (TURTLEBOT4_AUTO_VISIBLE_TOPIC_NAMES.has(name)) {
    return true;
  }
  return (
    name === "/scan" ||
    name === "/x500/scan" ||
    name.endsWith("/points")
  );
}

function isLaserScanTopic(topic: TopicLike): boolean {
  const name = topic.name ?? "";
  const schemaName = topic.schemaName ?? "";
  return (
    schemaName.includes("LaserScan") ||
    name === "/scan" ||
    name === "/scan" ||
    name === "/x500/scan"
  );
}

function shouldListTopicInPanel(topic: TopicLike): boolean {
  const name = topic.name ?? "";
  const schemaName = topic.schemaName ?? "";
  if (STANDALONE_3D_HIDDEN_BY_DEFAULT_TOPIC_NAMES.has(name)) {
    return true;
  }
  if (shouldAutoShowTopic(topic)) {
    return true;
  }
  return PANEL_LIST_SCHEMA_HINTS.some((hint) => schemaName.includes(hint));
}

function shouldSubscribeTopicInStandalone3d(topic: TopicLike): boolean {
  return shouldListTopicInPanel(topic) || isTfTopic(topic) || isOdometryTopic(topic);
}

function isTopicVisibleInRenderer(
  renderer: Renderer | undefined,
  topic: Pick<Topic, "name" | "schemaName">,
): boolean {
  const configuredVisibility = (renderer?.config as any)?.topics?.[topic.name]?.visible;
  if (configuredVisibility !== undefined) {
    return Boolean(configuredVisibility);
  }
  return shouldAutoShowTopic(topic);
}

function shouldKeepTopicRegisteredWithRenderer(
  renderer: Renderer | undefined,
  topic: Pick<Topic, "name" | "schemaName">,
): boolean {
  return (
    isTfTopic(topic) ||
    isOdometryTopic(topic) ||
    isTopicVisibleInRenderer(renderer, topic)
  );
}

type TopicRenderableLike = {
  userData?: {
    settings?: {
      visible?: boolean;
    };
  };
  visible?: boolean;
};

function syncExistingRenderableVisibility(
  renderer: Renderer,
  topicName: string,
  visible: boolean,
): void {
  for (const extension of renderer.sceneExtensions.values()) {
    const renderable = (extension.renderables as Map<string, TopicRenderableLike> | undefined)?.get(
      topicName,
    );
    if (!renderable) {
      continue;
    }
    if (renderable.userData?.settings) {
      renderable.userData.settings.visible = visible;
    }
    renderable.visible = visible;
  }
}

function applyTopicVisibilityChange(
  renderer: Renderer,
  topicName: string,
  visible: boolean,
): boolean {
  let handled = false;

  for (const extension of renderer.sceneExtensions.values()) {
    for (const node of extension.settingsNodes()) {
      if (node.path[0] !== "topics" || node.path[1] !== topicName) {
        continue;
      }
      extension.handleSettingsAction({
        action: "update",
        payload: {
          path: [...node.path, "visible"],
          input: "boolean",
          value: visible,
        },
      } as SettingsTreeAction);
      handled = true;
    }
  }

  // The standalone panel mutates config directly, so existing renderables need
  // their live visibility state updated immediately instead of waiting for a
  // message or a renderer-wide reset.
  syncExistingRenderableVisibility(renderer, topicName, visible);
  return handled;
}

function choosePreferredFrame(frameNames: string[], current?: string): string | undefined {
  const exactPriority = [
    "odom",
    "map",
    "world",
    "base_link",
    "base_footprint",
    "simple_bot/base_link",
    "x500_0",
    "lidar",
    "laser",
  ];
  for (const preferred of exactPriority) {
    if (frameNames.includes(preferred)) {
      return preferred;
    }
  }

  const suffixPriority = [
    "/odom",
    "/map",
    "/world",
    "/base_link",
    "/base_footprint",
    "/lidar",
    "/laser",
  ];
  for (const suffix of suffixPriority) {
    const match = frameNames.find((name) => name.endsWith(suffix));
    if (match) {
      return match;
    }
  }

  if (current && frameNames.includes(current)) {
    return current;
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

function isTfTopic(topic: TopicLike): boolean {
  const name = topic.name ?? "";
  const schemaName = topic.schemaName ?? "";
  return (
    name === "/tf" ||
    name === "/tf_static" ||
    name.endsWith("/tf") ||
    name.endsWith("/tf_static") ||
    TF_SCHEMA_NAMES.has(schemaName)
  );
}

function isOdometryTopic(topic: TopicLike): boolean {
  const name = topic.name ?? "";
  const schemaName = topic.schemaName ?? "";
  return (
    name === "/odom" ||
    name.endsWith("/odom") ||
    ODOMETRY_SCHEMA_NAMES.has(schemaName)
  );
}

function isOccupancyGridTopic(topic: TopicLike): boolean {
  return OCCUPANCY_GRID_SCHEMA_NAMES.has(topic.schemaName ?? "");
}

function isBarePolygonSchemaName(schemaName: string | undefined): boolean {
  return BARE_POLYGON_SCHEMA_NAMES.has(schemaName ?? "");
}

function getRenderableSchemaName(schemaName: string): string {
  return isBarePolygonSchemaName(schemaName) ? POLYGON_STAMPED_SCHEMA_NAME : schemaName;
}

function getOccupancyGridColorMode(topicName: string): "map" | "costmap" {
  return topicName.includes("costmap") ? "costmap" : "map";
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

  if (isOccupancyGridTopic(topic)) {
    return {
      ...base,
      colorMode: getOccupancyGridColorMode(topic.name),
      alpha: topic.name.includes("costmap") ? 0.7 : 1,
    };
  }

  if (STANDALONE_3D_HIDDEN_BY_DEFAULT_TOPIC_NAMES.has(topic.name)) {
    return {
      ...base,
      visible: false,
      explicitAlpha: 0.25,
    };
  }

  return base;
}

function getTopicKindLabel(topic: Pick<Topic, "name" | "schemaName">): string {
  if (topic.schemaName.includes("LaserScan")) {
    return "Lidar";
  }
  if (topic.schemaName.includes("PointCloud")) {
    return topic.name.toLowerCase().includes("depth") ? "Depth points" : "Point cloud";
  }
  const parts = topic.schemaName.split("/");
  return parts[parts.length - 1] ?? topic.schemaName;
}

function normalizeFrameIdForMatching(frameId: string | undefined): string | undefined {
  if (!frameId) {
    return undefined;
  }
  return frameId.startsWith("/") ? frameId.slice(1) : frameId;
}

function isOpticalFrameId(frameId: string | undefined): boolean {
  return normalizeFrameIdForMatching(frameId)?.toLowerCase().includes("optical") ?? false;
}

function getSimpleRobotFramePrefix(frameId: string | undefined): string | undefined {
  const normalized = normalizeFrameIdForMatching(frameId);
  if (!normalized) {
    return undefined;
  }

  for (const suffix of [
    "lidar_link/lidar",
    "rplidar_link/rplidar",
    "camera_link/camera",
    "camera_link",
    "oakd_rgb_camera_frame",
    "oakd_rgb_camera_optical_frame",
    "oakd_rgb_camera_frame/rgbd_camera",
    "base_link",
  ]) {
    if (normalized.endsWith(suffix)) {
      return normalized.slice(0, normalized.length - suffix.length);
    }
  }
  return undefined;
}

function toRendererMessageEvent(
  topic: string,
  schemaName: string,
  msg: any,
): { event: MessageEvent<any>; receiveNs: bigint } {
  const header = msg?.header as
    | { stamp?: { sec?: number; nanosec?: number; nsec?: number } }
    | undefined;

  let publishNs: bigint | undefined;
  if (header?.stamp) {
    const sec = header.stamp.sec ?? 0;
    const nanosec = header.stamp.nanosec ?? (header.stamp as any).nsec ?? 0;
    publishNs = BigInt(sec) * 1_000_000_000n + BigInt(nanosec);
  }

  const nowNs = BigInt(Date.now()) * 1_000_000n;
  const receiveNs = publishNs ?? nowNs;

  return {
    receiveNs,
    event: {
      topic,
      schemaName,
      receiveTime: fromNanoSec(receiveNs),
      publishTime: publishNs ? fromNanoSec(publishNs) : undefined,
      message: msg,
      sizeInBytes: estimateSizeInBytes(msg),
    },
  };
}

function inferSyntheticPolygonFrameId(
  topicName: string,
  displayFrame: string | undefined,
  frameNames: string[],
): string | undefined {
  if (topicName.includes("/global_costmap/")) {
    return (
      frameNames.find((name) => name === "map" || name.endsWith("/map")) ??
      choosePreferredFrame(frameNames, displayFrame) ??
      displayFrame
    );
  }

  if (topicName.includes("/local_costmap/")) {
    return (
      frameNames.find((name) => name === "odom" || name.endsWith("/odom")) ??
      choosePreferredFrame(frameNames, displayFrame) ??
      displayFrame
    );
  }

  return displayFrame ?? choosePreferredFrame(frameNames, displayFrame);
}

function toSyntheticPolygonStamped(
  topicName: string,
  msg: any,
  displayFrame: string | undefined,
  frameNames: string[],
): any {
  if (!msg || typeof msg !== "object") {
    return msg;
  }

  return {
    header: {
      stamp: { sec: 0, nanosec: 0 },
      frame_id: inferSyntheticPolygonFrameId(topicName, displayFrame, frameNames) ?? "",
    },
    polygon: msg,
  };
}

function rewriteTurtlebot4DepthPointCloudFrameId(topicName: string, msg: any): any {
  if (
    topicName !== "/oakd/rgb/preview/depth/points" ||
    !msg ||
    typeof msg !== "object"
  ) {
    return msg;
  }

  const frameId = normalizeFrameIdForMatching(msg?.header?.frame_id as string | undefined);
  if (frameId !== "oakd_rgb_camera_optical_frame") {
    return msg;
  }

  // Live TurtleBot4 samples publish this cloud with an optical-frame header,
  // but the XYZ values already behave like camera-frame coordinates
  // (forward/lateral/up). Treating them as optical points adds the
  // camera->optical rotation a second time and destroys registration.
  return {
    ...msg,
    header: {
      ...(msg.header ?? {}),
      frame_id: "oakd_rgb_camera_frame",
    },
  };
}

function getHeaderStampNs(msg: any): bigint | undefined {
  const header = msg?.header as
    | { stamp?: { sec?: number; nanosec?: number; nsec?: number } }
    | undefined;
  if (!header?.stamp) {
    return undefined;
  }

  const sec = header.stamp.sec ?? 0;
  const nanosec = header.stamp.nanosec ?? (header.stamp as any).nsec ?? 0;
  return BigInt(sec) * 1_000_000_000n + BigInt(nanosec);
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
  const [showDiagnostics, setShowDiagnostics] = useState<boolean>(() => {
    if (typeof window === "undefined") return false;
    return window.localStorage.getItem(LS_SHOW_DIAGNOSTICS_KEY) === "true";
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
  const [topicVisibilityVersion, setTopicVisibilityVersion] = useState(0);

  // Frames discovered from TF messages
  const framesSetRef = useRef<Set<string>>(new Set());
  const tfFramesSetRef = useRef<Set<string>>(new Set());
  const aliasFramesRef = useRef<Map<string, string>>(new Map());
  const fallbackFramesRef = useRef<Set<string>>(new Set());
  const [frames, setFrames] = useState<string[]>([]);
  const [displayFrame, setDisplayFrame] = useState<string | undefined>(undefined);
  const displayFrameRef = useRef<string | undefined>(undefined);
  const topicMessageCountsRef = useRef<Record<string, number>>({});
  const topicFrameIdsRef = useRef<Map<string, string>>(new Map());
  const [, setDiagnosticsTick] = useState(0);

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
      sceneExtensionConfig: STANDALONE_3D_SCENE_EXTENSION_CONFIG,
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
    const tfSet = tfFramesSetRef.current;
    let changed = false;

    for (const t of msg.transforms) {
      const parent = normalizeFrameIdForMatching(t?.header?.frame_id as string | undefined);
      const child = normalizeFrameIdForMatching(t?.child_frame_id as string | undefined);
      if (parent && !set.has(parent)) {
        set.add(parent);
        tfSet.add(parent);
        changed = true;
      }
      if (child && !set.has(child)) {
        set.add(child);
        tfSet.add(child);
        changed = true;
      }
    }

    if (changed) {
      const arr = Array.from(set).sort();
      setFrames(arr);
      setDisplayFrame((prev) => choosePreferredFrame(arr, prev));
    }
  }, []);

  const updateFramesFromOdometryMessage = useCallback((msg: any) => {
    if (!msg || typeof msg !== "object") {
      return;
    }

    const set = framesSetRef.current;
    const tfSet = tfFramesSetRef.current;
    let changed = false;

    const parent = normalizeFrameIdForMatching(msg?.header?.frame_id as string | undefined);
    const child = normalizeFrameIdForMatching(msg?.child_frame_id as string | undefined);

    for (const frame of [parent, child]) {
      if (!frame) {
        continue;
      }
      if (!set.has(frame)) {
        set.add(frame);
        changed = true;
      }
      tfSet.add(frame);
    }

    if (changed) {
      const arr = Array.from(set).sort();
      setFrames(arr);
      setDisplayFrame((prev) => choosePreferredFrame(arr, prev));
    }
  }, []);

  const resolveFrameId = useCallback((frameId: string | undefined): string | undefined => {
    const normalized = normalizeFrameIdForMatching(frameId);
    if (!normalized) {
      return undefined;
    }

    const tfFrames = Array.from(tfFramesSetRef.current);

    // If the exact frame exists in TF, trust that path and do not collapse it
    // onto a shorter suffix alias. Suffix aliasing is only a fallback for
    // messages whose frame ids differ from the TF tree by a namespace prefix.
    if (tfFrames.includes(normalized)) {
      return normalized;
    }

    let bestMatch: string | undefined;
    for (const candidate of tfFrames) {
      if (normalized.endsWith(`/${candidate}`)) {
        if (!bestMatch || candidate.length > bestMatch.length) {
          bestMatch = candidate;
        }
      }
    }
    if (bestMatch) {
      return bestMatch;
    }

    return normalized;
  }, []);

  const ensureAliasedFrame = useCallback((frameId: string | undefined): string | undefined => {
    const normalized = normalizeFrameIdForMatching(frameId);
    if (!normalized) {
      return undefined;
    }

    const canonical = resolveFrameId(normalized);
    if (!renderer || !canonical || canonical === normalized) {
      return canonical ?? normalized;
    }

    const existingFrame = renderer.transformTree.frame(normalized);
    if (existingFrame?.parent() != undefined) {
      return normalized;
    }

    const aliases = aliasFramesRef.current;
    if (aliases.get(normalized) === canonical) {
      return canonical;
    }

    renderer.addCoordinateFrame(canonical);
    renderer.addCoordinateFrame(normalized);
    renderer.addTransform(
      canonical,
      normalized,
      0n,
      ZERO_TRANSLATION,
      IDENTITY_ROTATION,
      ["transforms", `frame:${normalized}`],
    );
    aliases.set(normalized, canonical);
    return canonical;
  }, [renderer, resolveFrameId]);

  const ensureSimpleRobotFallbackFrames = useCallback((frameId: string | undefined) => {
    const normalized = normalizeFrameIdForMatching(frameId);
    const prefix = getSimpleRobotFramePrefix(normalized);
    if (!renderer || !prefix) {
      return;
    }

    const fallbackKey = `${prefix}base_link`;
    if (fallbackFramesRef.current.has(fallbackKey)) {
      return;
    }

    const baseFrame = `${prefix}base_link`;
    const lidarLinkFrame = `${prefix}lidar_link`;
    const lidarFrame = `${prefix}lidar_link/lidar`;
    const rplidarLinkFrame = `${prefix}rplidar_link`;
    const rplidarFrame = `${prefix}rplidar_link/rplidar`;
    const cameraLinkFrame = `${prefix}camera_link`;
    const cameraFrame = `${prefix}camera_link/camera`;
    const oakdCameraFrame = `${prefix}oakd_rgb_camera_frame`;
    const oakdOpticalFrame = `${prefix}oakd_rgb_camera_optical_frame`;
    const rgbdCameraFrame = `${prefix}oakd_rgb_camera_frame/rgbd_camera`;

    const opticalFrames = new Set<string>([oakdOpticalFrame]);
    if (isOpticalFrameId(normalized)) {
      opticalFrames.add(normalized);
    }

    const addFallbackTransformIfMissing = (
      parentFrameId: string,
      childFrameId: string,
      translation: typeof ZERO_TRANSLATION,
      rotation: typeof IDENTITY_ROTATION,
    ) => {
      renderer.addCoordinateFrame(parentFrameId);
      renderer.addCoordinateFrame(childFrameId);

      const childFrame = renderer.transformTree.frame(childFrameId);
      const existingParentId = childFrame?.parent()?.id;
      if (existingParentId != undefined && existingParentId !== parentFrameId) {
        return;
      }

      renderer.addTransform(parentFrameId, childFrameId, 0n, translation, rotation);
    };

    renderer.addCoordinateFrame(baseFrame);
    renderer.addCoordinateFrame(lidarLinkFrame);
    renderer.addCoordinateFrame(lidarFrame);
    renderer.addCoordinateFrame(rplidarLinkFrame);
    renderer.addCoordinateFrame(rplidarFrame);
    renderer.addCoordinateFrame(cameraLinkFrame);
    renderer.addCoordinateFrame(cameraFrame);
    renderer.addCoordinateFrame(oakdCameraFrame);
    renderer.addCoordinateFrame(rgbdCameraFrame);
    for (const opticalFrame of opticalFrames) {
      renderer.addCoordinateFrame(opticalFrame);
    }
    addFallbackTransformIfMissing(
      baseFrame,
      lidarLinkFrame,
      SIMPLE_ROBOT_LIDAR_TRANSLATION,
      IDENTITY_ROTATION,
    );
    addFallbackTransformIfMissing(lidarLinkFrame, lidarFrame, ZERO_TRANSLATION, IDENTITY_ROTATION);
    addFallbackTransformIfMissing(
      baseFrame,
      rplidarLinkFrame,
      SIMPLE_ROBOT_LIDAR_TRANSLATION,
      IDENTITY_ROTATION,
    );
    addFallbackTransformIfMissing(
      rplidarLinkFrame,
      rplidarFrame,
      ZERO_TRANSLATION,
      IDENTITY_ROTATION,
    );
    addFallbackTransformIfMissing(
      baseFrame,
      cameraLinkFrame,
      SIMPLE_ROBOT_CAMERA_TRANSLATION,
      IDENTITY_ROTATION,
    );
    addFallbackTransformIfMissing(cameraLinkFrame, cameraFrame, ZERO_TRANSLATION, IDENTITY_ROTATION);
    addFallbackTransformIfMissing(
      baseFrame,
      oakdCameraFrame,
      SIMPLE_ROBOT_CAMERA_TRANSLATION,
      IDENTITY_ROTATION,
    );
    addFallbackTransformIfMissing(
      oakdCameraFrame,
      rgbdCameraFrame,
      ZERO_TRANSLATION,
      IDENTITY_ROTATION,
    );
    for (const opticalFrame of opticalFrames) {
      addFallbackTransformIfMissing(
        oakdCameraFrame,
        opticalFrame,
        ZERO_TRANSLATION,
        CAMERA_OPTICAL_ROTATION,
      );
    }

    const set = framesSetRef.current;
    for (const synthesizedFrame of [
      baseFrame,
      lidarLinkFrame,
      lidarFrame,
      rplidarLinkFrame,
      rplidarFrame,
      cameraLinkFrame,
      cameraFrame,
      oakdCameraFrame,
      rgbdCameraFrame,
      ...opticalFrames,
    ]) {
      set.add(synthesizedFrame);
    }
    fallbackFramesRef.current.add(fallbackKey);
    const arr = Array.from(set).sort();
    setFrames(arr);
    setDisplayFrame((prev) => choosePreferredFrame(arr, prev));
  }, [renderer]);

  const updateFramesFromMessage = useCallback((msg: any) => {
    if (!msg || typeof msg !== "object") {
      return;
    }

    const set = framesSetRef.current;
    let changed = false;

    const parent = resolveFrameId(msg?.header?.frame_id as string | undefined);
    const child = resolveFrameId(msg?.child_frame_id as string | undefined);

    if (parent && !set.has(parent)) {
      set.add(parent);
      changed = true;
    }
    if (child && !set.has(child)) {
      set.add(child);
      changed = true;
    }

    if (changed) {
      const arr = Array.from(set).sort();
      setFrames(arr);
      setDisplayFrame((prev) => choosePreferredFrame(arr, prev));
    }
  }, [resolveFrameId]);

  const normalizeMessageFrameIds = useCallback((msg: any): any => {
    if (!msg || typeof msg !== "object") {
      return msg;
    }

    let nextMsg = msg;

    const normalizedHeaderFrame =
      ensureAliasedFrame(msg?.header?.frame_id as string | undefined) ??
      resolveFrameId(msg?.header?.frame_id as string | undefined);
    if (normalizedHeaderFrame && normalizedHeaderFrame !== msg?.header?.frame_id) {
      nextMsg = {
        ...nextMsg,
        header: {
          ...(nextMsg.header ?? {}),
          frame_id: normalizedHeaderFrame,
        },
      };
    }

    const normalizedChildFrame =
      ensureAliasedFrame(msg?.child_frame_id as string | undefined) ??
      resolveFrameId(msg?.child_frame_id as string | undefined);
    if (normalizedChildFrame && normalizedChildFrame !== nextMsg?.child_frame_id) {
      nextMsg = {
        ...nextMsg,
        child_frame_id: normalizedChildFrame,
      };
    }

    return nextMsg;
  }, [ensureAliasedFrame, resolveFrameId]);

  const applyOdometryTransform = useCallback((msg: any, receiveNs: bigint) => {
    if (!renderer || !msg || typeof msg !== "object") {
      return;
    }

    const parentFrame = normalizeFrameIdForMatching(msg?.header?.frame_id as string | undefined);
    const childFrame = normalizeFrameIdForMatching(msg?.child_frame_id as string | undefined);
    const position = msg?.pose?.pose?.position;
    const orientation = msg?.pose?.pose?.orientation;

    if (!parentFrame || !childFrame || !position || !orientation) {
      return;
    }

    renderer.addCoordinateFrame(parentFrame);
    renderer.addCoordinateFrame(childFrame);
    renderer.addTransform(
      parentFrame,
      childFrame,
      getHeaderStampNs(msg) ?? receiveNs,
      {
        x: typeof position.x === "number" ? position.x : 0,
        y: typeof position.y === "number" ? position.y : 0,
        z: typeof position.z === "number" ? position.z : 0,
      },
      {
        x: typeof orientation.x === "number" ? orientation.x : 0,
        y: typeof orientation.y === "number" ? orientation.y : 0,
        z: typeof orientation.z === "number" ? orientation.z : 0,
        w: typeof orientation.w === "number" ? orientation.w : 1,
      },
      ["transforms", `frame:${childFrame}`],
    );
  }, [renderer]);

  // ---- ROS2Bridge wiring: available topics + live messages -----------------------

  useEffect(() => {
    if (!renderer) return;

    const liveSubs = liveSubsRef.current;
    const tfSubscriptions: Ros2BridgeSubscription[] = [
      { topic: "/tf", type: "tf2_msgs/msg/TFMessage" },
      { topic: "/tf_static", type: "tf2_msgs/msg/TFMessage" },
    ];
    if (getActiveVmConfigId() === "turtlebot4") {
      tfSubscriptions.push(
        { topic: "/tf", type: "tf2_msgs/msg/TFMessage" },
        { topic: "/tf_static", type: "tf2_msgs/msg/TFMessage" },
      );
    }

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
        const renderableSchemaName = getRenderableSchemaName(schemaName);
        const topicInfo = { name: topicName, schemaName: renderableSchemaName };
        if (!shouldSubscribeTopicInStandalone3d(topicInfo)) {
          continue;
        }

        if (
          !isTfTopic(topicInfo) &&
          !isOdometryTopic(topicInfo) &&
          !isTopicVisibleInRenderer(r, topicInfo)
        ) {
          continue;
        }

        const subsForTopic = topicSubs.get(topicName) ?? [];
        const subsForSchema = schemaSubs.get(renderableSchemaName) ?? [];

        const allSubs = subsForTopic.concat(subsForSchema);
        if (allSubs.length === 0) {
          if (isBarePolygonSchemaName(schemaName)) {
            desired.push({ topic: topicName, type: schemaName });
          }
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
      const requiredSubscriptions = new Map<string, Ros2BridgeSubscription>();
      for (const sub of [...tfSubscriptions, ...desired]) {
        requiredSubscriptions.set(sub.topic, sub);
      }
      const desiredTopics = new Set(requiredSubscriptions.keys());

      // Unsubscribe from topics no longer needed
      for (const [topic, unsubscribe] of current.entries()) {
        if (!desiredTopics.has(topic)) {
          unsubscribe();
          current.delete(topic);
        }
      }

      // Subscribe to new topics
      for (const d of requiredSubscriptions.values()) {
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

            if (isBarePolygonSchemaName(d.type)) {
              msg = toSyntheticPolygonStamped(
                d.topic,
                msg,
                displayFrameRef.current,
                Array.from(framesSetRef.current),
              );
            }

            msg = rewriteTurtlebot4DepthPointCloudFrameId(d.topic, msg);

            const isOdometryMessage = isOdometryTopic({
              name: d.topic,
              schemaName: getRenderableSchemaName(d.type),
            });

            if (isOdometryMessage) {
              updateFramesFromOdometryMessage(msg);
            }

            msg = normalizeMessageFrameIds(msg);
            ensureSimpleRobotFallbackFrames(msg?.header?.frame_id as string | undefined);
            updateFramesFromMessage(msg);
            const topicFrameId = normalizeFrameIdForMatching(
              msg?.header?.frame_id as string | undefined,
            );
            if (topicFrameId) {
              topicFrameIdsRef.current.set(d.topic, topicFrameId);
            }
            topicMessageCountsRef.current[d.topic] = (topicMessageCountsRef.current[d.topic] ?? 0) + 1;

            // If this is a TF message, harvest frames
            if (isTfTopic({ name: d.topic, schemaName: d.type })) {
              updateFramesFromTfMessage(msg);
            }
            const { event, receiveNs } = toRendererMessageEvent(
              d.topic,
              getRenderableSchemaName(d.type),
              msg,
            );
            if (isOdometryMessage) {
              applyOdometryTransform(msg, receiveNs);
            }

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
      const effectiveTopicsList =
        getActiveVmConfigId() === "turtlebot4"
          ? mergeTopicDefaults(topicsList, TURTLEBOT4_SENSOR_TOPIC_DEFAULTS)
          : topicsList;

      const topicObjects: Topic[] = effectiveTopicsList.map(
        (t) =>
          ({
            name: t.topic,
            schemaName: getRenderableSchemaName(t.type),
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

      const desired = computeDesiredSubscriptions(effectiveTopicsList, renderer);
      const odometrySubscriptions = effectiveTopicsList.filter((topic) =>
        isOdometryTopic({ name: topic.topic, schemaName: topic.type }),
      );
      const panelTopics = topicObjects
        .filter((topic) => !isTfTopic(topic) && shouldListTopicInPanel(topic))
        .sort(sortTopicsForPanel);
      const rendererTopics = topicObjects.filter((topic) =>
        shouldKeepTopicRegisteredWithRenderer(renderer, topic),
      );
      renderer.setTopics(rendererTopics);
      for (const topic of topicObjects) {
        syncExistingRenderableVisibility(
          renderer,
          topic.name,
          isTopicVisibleInRenderer(renderer, topic),
        );
      }
      setTopics(panelTopics);
      reconcileSubscriptions(desired.concat(odometrySubscriptions), renderer);
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
  }, [renderer, updateFramesFromTfMessage, updateFramesFromOdometryMessage, updateFramesFromMessage, normalizeMessageFrameIds, ensureSimpleRobotFallbackFrames, applyOdometryTransform, pointSize, decayTime, topicVisibilityVersion]);

  useEffect(() => {
    aliasFramesRef.current.clear();
    fallbackFramesRef.current.clear();
    topicFrameIdsRef.current.clear();
    topicMessageCountsRef.current = {};
    setDiagnosticsTick((prev) => prev + 1);
  }, [renderer]);

  useEffect(() => {
    displayFrameRef.current = displayFrame;
  }, [displayFrame]);

  useEffect(() => {
    if (!renderer || !showDiagnostics || typeof window === "undefined") {
      return;
    }

    const intervalId = window.setInterval(() => {
      setDiagnosticsTick((prev) => prev + 1);
    }, 500);

    return () => {
      window.clearInterval(intervalId);
    };
  }, [renderer, showDiagnostics]);

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
    renderer.updateConfig((draft) => {
      draft.topics ??= {};
      for (const name of Object.keys(draft.topics)) {
        draft.topics[name] = {
          ...(draft.topics[name] ?? {}),
          pointSize,
        };
      }
    });
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
    renderer.updateConfig((draft) => {
      draft.topics ??= {};
      for (const name of Object.keys(draft.topics)) {
        draft.topics[name] = {
          ...(draft.topics[name] ?? {}),
          decayTime,
        };
      }
    });
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
    renderer.updateConfig((draft) => {
      draft.followMode = followFrame ? "follow-position" : "follow-none";
    });
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
        window.localStorage.setItem(LS_SHOW_DIAGNOSTICS_KEY, String(showDiagnostics));
      } catch {
        // ignore
      }
    }
  }, [showDiagnostics]);

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
    if (!renderer) return;
    if (!displayFrame) {
      renderer.updateConfig((draft) => {
        draft.followTf = undefined;
      });
      renderer.setFollowFrameId(undefined);
      renderer.queueAnimationFrame();
      return;
    }

    ensureAliasedFrame(displayFrame);
    renderer.addCoordinateFrame(displayFrame);
    renderer.updateConfig((draft) => {
      draft.followTf = displayFrame;
    });
    renderer.setFollowFrameId(displayFrame);
    renderer.queueAnimationFrame();
  }, [renderer, displayFrame, ensureAliasedFrame]);

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

  useEffect(() => {
    const visibleLaserTopic = topics.find(
      (topic) => isTopicVisible(topic.name) && isLaserScanTopic(topic),
    );
    if (!visibleLaserTopic) {
      return;
    }

    const laserFrame = topicFrameIdsRef.current.get(visibleLaserTopic.name);
    if (!laserFrame) {
      return;
    }

    const simpleRobotPrefix = getSimpleRobotFramePrefix(laserFrame);
    const preferredLaserRenderFrame =
      simpleRobotPrefix != undefined ? `${simpleRobotPrefix}base_link` : laserFrame;

    const currentFrame = normalizeFrameIdForMatching(displayFrame);
    const currentIsCamera =
      currentFrame != undefined &&
      currentFrame.toLowerCase().includes("camera") &&
      !currentFrame.toLowerCase().includes("lidar") &&
      !currentFrame.toLowerCase().includes("laser");

    if (!displayFrame || currentIsCamera) {
      setDisplayFrame((prev) =>
        prev === preferredLaserRenderFrame ? prev : preferredLaserRenderFrame,
      );
    }
  }, [topics, displayFrame, isTopicVisible]);

  const toggleTopicVisibility = useCallback(
    (topicName: string) => {
      if (!renderer) return;
      const nextVisible = !isTopicVisible(topicName);
      const handled = applyTopicVisibilityChange(renderer, topicName, nextVisible);
      if (!handled) {
        renderer.updateConfig((draft) => {
          draft.topics ??= {};
          draft.topics[topicName] ??= {};
          draft.topics[topicName]!.visible = nextVisible;
        });
        syncExistingRenderableVisibility(renderer, topicName, nextVisible);
      }
      setTopicVisibilityVersion((prev) => prev + 1);
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
  const currentFollowFrameId = renderer?.followFrameId;
  const visibleTopicCount = topics.filter((topic) => isTopicVisible(topic.name)).length;
  const rendererDiagnostics = showDiagnostics && renderer
    ? {
        followFrameId: currentFollowFrameId,
        fixedFrameId: renderer.fixedFrameId,
        hasDisplayFrame: displayFrame ? renderer.transformTree.hasFrame(displayFrame) : false,
        followError: renderer.settings.errors.errors.errorAtPath(["general", "followTf"]),
        displayFrameError: displayFrame
          ? renderer.settings.errors.errors.errorAtPath(["transforms", `frame:${displayFrame}`])
          : undefined,
      }
    : undefined;
  const scanRenderableDiagnostics = (() => {
    if (!showDiagnostics || !renderer) {
      return undefined;
    }
    const diagnosticLaserTopic =
      topics.find((topic) => isLaserScanTopic(topic) && isTopicVisible(topic.name))?.name ??
      topics.find((topic) => isLaserScanTopic(topic))?.name ??
      "/scan";
    const extension = renderer.sceneExtensions.get("foxglove.LaserScans") as
      | { renderables?: Map<string, any> }
      | undefined;
    const renderable = extension?.renderables?.get(diagnosticLaserTopic);
    if (!renderable) {
      return undefined;
    }

    const child = Array.isArray(renderable.children) ? renderable.children[0] : undefined;
    const positionAttr = child?.geometry?.attributes?.position;

    return {
      topic: diagnosticLaserTopic,
      frameId: renderable.userData?.frameId as string | undefined,
      visible: Boolean(renderable.visible),
      childVisible: child ? Boolean(child.visible) : undefined,
      pointCount:
        typeof positionAttr?.count === "number"
          ? positionAttr.count
          : Array.isArray(positionAttr?.array)
            ? positionAttr.array.length
            : undefined,
      childCount: Array.isArray(renderable.children) ? renderable.children.length : undefined,
    };
  })();
  const topicDiagnostics = showDiagnostics
    ? topics.map((topic) => ({
        name: topic.name,
        visible: isTopicVisible(topic.name),
        kind: getTopicKindLabel(topic),
        messageCount: topicMessageCountsRef.current[topic.name] ?? 0,
        error: renderer?.settings.errors.errors.errorAtPath(["topics", topic.name]),
      }))
    : [];

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
                  Live lidar, depth, and point-cloud view
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
                <div>Visible topics: {visibleTopicCount}/{topics.length}</div>
                <div>Follow: {currentFollowFrameId ?? "none"}</div>
              </div>
            )}

            {!panelCollapsed && (
            <div
              style={{
                marginBottom: 14,
                padding: 12,
                borderRadius: 12,
                background: "linear-gradient(180deg, rgba(105, 183, 255, 0.16) 0%, rgba(255,255,255,0.04) 100%)",
                border: `1px solid ${PANEL_COLORS.cardBorder}`,
              }}
            >
              <div
                style={{
                  display: "grid",
                  gridTemplateColumns: "repeat(3, minmax(0, 1fr))",
                  gap: 10,
                }}
              >
                <div>
                  <div style={{ color: PANEL_COLORS.textSecondary, marginBottom: 3 }}>Frame</div>
                  <div style={{ fontWeight: 600, wordBreak: "break-word" }}>{displayFrame ?? "auto"}</div>
                </div>
                <div>
                  <div style={{ color: PANEL_COLORS.textSecondary, marginBottom: 3 }}>Sources</div>
                  <div style={{ fontWeight: 600 }}>{visibleTopicCount}/{topics.length} visible</div>
                </div>
                <div>
                  <div style={{ color: PANEL_COLORS.textSecondary, marginBottom: 3 }}>TF</div>
                  <div style={{ fontWeight: 600 }}>{showTf ? "Full tree" : "Pose only"}</div>
                </div>
              </div>
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
                display: "flex",
                justifyContent: "space-between",
                alignItems: "center",
                gap: 10,
                marginBottom: 14,
              }}
            >
              <div>
                <div style={{ fontWeight: 600 }}>Diagnostics</div>
                <div style={{ color: PANEL_COLORS.textSecondary }}>
                  Keep renderer internals out of the main control flow until you need them.
                </div>
              </div>
              <button
                type="button"
                onClick={() => setShowDiagnostics((prev) => !prev)}
                style={{
                  border: `1px solid ${PANEL_COLORS.cardBorder}`,
                  background: showDiagnostics ? "rgba(105, 183, 255, 0.18)" : "rgba(255,255,255,0.06)",
                  color: PANEL_COLORS.textPrimary,
                  borderRadius: 8,
                  padding: "7px 11px",
                  cursor: "pointer",
                  whiteSpace: "nowrap",
                }}
              >
                {showDiagnostics ? "Hide" : "Show"}
              </button>
            </div>
            )}

            {!panelCollapsed && showDiagnostics && (
            <div
              style={{
                marginBottom: 14,
                padding: 10,
                borderRadius: 10,
                background: "rgba(255,255,255,0.04)",
                border: `1px solid ${PANEL_COLORS.cardBorder}`,
              }}
            >
              <div style={{ marginBottom: 8, fontWeight: 600 }}>Renderer Status</div>
              <div style={{ color: PANEL_COLORS.textSecondary, lineHeight: 1.45 }}>
                <div>Selected frame: {displayFrame ?? "none"}</div>
                <div>Renderer follow frame: {rendererDiagnostics?.followFrameId ?? "none"}</div>
                <div>Fixed frame: {rendererDiagnostics?.fixedFrameId ?? "none"}</div>
                <div>Display frame exists: {rendererDiagnostics?.hasDisplayFrame ? "yes" : "no"}</div>
                <div>Scan renderable: {scanRenderableDiagnostics ? "present" : "missing"}</div>
                {scanRenderableDiagnostics && (
                  <>
                    <div>Scan frame: {scanRenderableDiagnostics.frameId ?? "none"}</div>
                    <div>Scan topic: {scanRenderableDiagnostics.topic}</div>
                    <div>Scan visible: {scanRenderableDiagnostics.visible ? "yes" : "no"}</div>
                    <div>Scan child visible: {scanRenderableDiagnostics.childVisible ? "yes" : "no"}</div>
                    <div>Scan points: {scanRenderableDiagnostics.pointCount ?? 0}</div>
                    <div>Scan children: {scanRenderableDiagnostics.childCount ?? 0}</div>
                  </>
                )}
                {rendererDiagnostics?.followError && <div>Follow error: {rendererDiagnostics.followError}</div>}
                {rendererDiagnostics?.displayFrameError && (
                  <div>Frame error: {rendererDiagnostics.displayFrameError}</div>
                )}
              </div>
            </div>
            )}

            {!panelCollapsed && showDiagnostics && topicDiagnostics.length > 0 && (
            <div
              style={{
                marginBottom: 14,
                padding: 10,
                borderRadius: 10,
                background: "rgba(255,255,255,0.04)",
                border: `1px solid ${PANEL_COLORS.cardBorder}`,
              }}
            >
              <div style={{ marginBottom: 8, fontWeight: 600 }}>Topic Diagnostics</div>
              <div style={{ display: "grid", gap: 6, color: PANEL_COLORS.textSecondary }}>
                {topicDiagnostics
                  .filter((topic) => topic.visible || topic.messageCount > 0 || topic.error)
                  .map((topic) => (
                    <div key={topic.name}>
                      {topic.name} | {topic.kind} | msgs: {topic.messageCount}
                      {topic.error ? ` | ${topic.error}` : ""}
                    </div>
                  ))}
              </div>
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
                    <div style={{ minWidth: 0, flex: 1 }}>
                      <div
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
                      </div>
                      <div
                        style={{
                          marginTop: 2,
                          color: PANEL_COLORS.textSecondary,
                          fontSize: 10.5,
                        }}
                      >
                        {getTopicKindLabel(t)}
                      </div>
                    </div>
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
  if (!obj || typeof obj !== "object") {
    return 0;
  }

  const data = obj.data;
  if (typeof obj.point_step === "number" && typeof obj.width === "number") {
    return obj.point_step * obj.width * (typeof obj.height === "number" ? obj.height : 1);
  }
  if (ArrayBuffer.isView(data)) {
    return data.byteLength;
  }
  if (data instanceof ArrayBuffer) {
    return data.byteLength;
  }

  const rangesLength =
    ArrayBuffer.isView(obj.ranges) ? obj.ranges.byteLength : Array.isArray(obj.ranges) ? obj.ranges.length * 4 : 0;
  const intensitiesLength =
    ArrayBuffer.isView(obj.intensities)
      ? obj.intensities.byteLength
      : Array.isArray(obj.intensities)
        ? obj.intensities.length * 4
        : 0;
  if (rangesLength || intensitiesLength) {
    return rangesLength + intensitiesLength;
  }

  return 0;
}
