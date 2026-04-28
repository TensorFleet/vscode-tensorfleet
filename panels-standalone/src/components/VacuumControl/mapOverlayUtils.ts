import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { ObjectPool } from "@lichtblick/den/collection";
import { ros2Bridge, type Subscription } from "../../ros2-bridge";
import {
  getRecordEntry,
  normalizeRosMessage,
} from "../Nav2/runtime/nav2RuntimeUtils";
import type { PoseCoordinates } from "../Nav2/runtime/nav2RuntimeTypes";
import {
  DEFAULT_MAX_CAPACITY_PER_FRAME,
  Transform,
  TransformTree,
  makePose,
  quatFromValues,
  vec3FromValues,
  type Pose,
  type Time,
} from "../../../packages/suite-base/src/panels/ThreeDeeRender/transforms";

export type MapOverlayKey =
  | "map"
  | "globalCostmap"
  | "localCostmap"
  | "plan"
  | "lidar"
  | "depthObstacles";

export type OverlayState = "live" | "waiting" | "no-tf";

export type OverlayVisibility = Record<MapOverlayKey, boolean>;
export type OverlayAvailability = Record<MapOverlayKey, OverlayState>;

export type ProjectedMapPoint = {
  x: number;
  y: number;
  z: number;
};

export type OverlayDefinition = {
  key: MapOverlayKey;
  label: string;
  swatchClassName: string;
};

type SensorProjection = {
  points: ProjectedMapPoint[];
  status: OverlayState;
};

type ParsedLaserScan = {
  frameId: string;
  timestampNs: Time;
  points: ProjectedMapPoint[];
};

type ParsedPointCloud2 = {
  frameId: string;
  timestampNs: Time;
  points: ProjectedMapPoint[];
};

const TF_SUBSCRIPTIONS = [
  { topic: "/tf", type: "tf2_msgs/msg/TFMessage" },
  { topic: "/tf_static", type: "tf2_msgs/msg/TFMessage" },
] as const;

const ROS_POINT_CLOUD_SCHEMA_NAMES = new Set([
  "sensor_msgs/PointCloud2",
  "sensor_msgs/msg/PointCloud2",
]);

const LASER_SCAN_SCHEMA_NAMES = new Set([
  "sensor_msgs/LaserScan",
  "sensor_msgs/msg/LaserScan",
  "foxglove.LaserScan",
]);

const MAP_FRAME_ID = "map";
const LIDAR_TOPIC_PREFERENCE = "/scan";
const DEPTH_TOPIC_PREFERENCE = "/oakd/rgb/preview/depth/points";
const MAX_LIDAR_POINTS = 900;
const MAX_DEPTH_POINTS = 1_500;
const MAX_DEPTH_DISTANCE_M = 6;
const ROBOT_POSE_FRAME_CANDIDATES = [
  "base_footprint",
  "base_link",
  "turtlebot4/base_footprint",
  "turtlebot4/base_link",
];
const LIDAR_FRAME_FALLBACKS = [
  "rplidar_link",
  "turtlebot4/rplidar_link",
  "base_scan",
  "laser",
];

export const DEFAULT_OVERLAY_VISIBILITY: OverlayVisibility = {
  map: true,
  globalCostmap: true,
  localCostmap: true,
  plan: true,
  lidar: false,
  depthObstacles: false,
};

export const MAP_OVERLAY_DEFINITIONS: OverlayDefinition[] = [
  { key: "map", label: "Map", swatchClassName: "vacuum-map-layer-picker__swatch--map" },
  {
    key: "globalCostmap",
    label: "Global costmap",
    swatchClassName: "vacuum-map-layer-picker__swatch--globalCostmap",
  },
  {
    key: "localCostmap",
    label: "Local costmap",
    swatchClassName: "vacuum-map-layer-picker__swatch--localCostmap",
  },
  { key: "plan", label: "Plan", swatchClassName: "vacuum-map-layer-picker__swatch--plan" },
  { key: "lidar", label: "Lidar", swatchClassName: "vacuum-map-layer-picker__swatch--lidar" },
  {
    key: "depthObstacles",
    label: "Depth obstacles",
    swatchClassName: "vacuum-map-layer-picker__swatch--depthObstacles",
  },
];

function nowNs(): Time {
  return BigInt(Date.now()) * 1_000_000n;
}

function normalizeFrameId(frameId: unknown): string | null {
  if (typeof frameId !== "string") {
    return null;
  }
  const normalized = frameId.startsWith("/") ? frameId.slice(1) : frameId;
  return normalized.length > 0 ? normalized : null;
}

function toFiniteNumber(value: unknown): number | null {
  const numeric = typeof value === "string" ? Number(value) : value;
  return typeof numeric === "number" && Number.isFinite(numeric) ? numeric : null;
}

function toNumericArray(value: unknown): number[] | null {
  if (Array.isArray(value)) {
    return value.map((entry) => Number(entry));
  }
  if (typeof ArrayBuffer !== "undefined" && ArrayBuffer.isView(value) && !(value instanceof DataView)) {
    return Array.from(value as unknown as ArrayLike<number>, (entry) => Number(entry));
  }
  return null;
}

function toByteArray(value: unknown): Uint8Array | null {
  if (value instanceof Uint8Array) {
    return value;
  }
  if (Array.isArray(value)) {
    return Uint8Array.from(value, (entry) => Number(entry));
  }
  if (typeof ArrayBuffer !== "undefined" && ArrayBuffer.isView(value)) {
    return new Uint8Array(value.buffer, value.byteOffset, value.byteLength);
  }
  if (typeof value === "string") {
    const binary = atob(value);
    const bytes = new Uint8Array(binary.length);
    for (let index = 0; index < binary.length; index += 1) {
      bytes[index] = binary.charCodeAt(index);
    }
    return bytes;
  }
  return null;
}

function quaternionToYawDegrees(orientation: Pose["orientation"]): number {
  const { x, y, z, w } = orientation;
  const sinyCosp = 2 * (w * z + x * y);
  const cosyCosp = 1 - 2 * (y * y + z * z);
  return (Math.atan2(sinyCosp, cosyCosp) * 180) / Math.PI;
}

function getMessageFrameId(message: Record<string, unknown> | null): string | null {
  if (!message) {
    return null;
  }
  const header = getRecordEntry(message, "header");
  const headerRecord =
    header && typeof header === "object" ? (header as Record<string, unknown>) : null;
  return normalizeFrameId(headerRecord ? getRecordEntry(headerRecord, "frame_id") : null);
}

function extractTimestampNs(value: unknown): Time {
  if (!value || typeof value !== "object") {
    return nowNs();
  }
  const header =
    getRecordEntry(value as Record<string, unknown>, "header") ??
    getRecordEntry(value as Record<string, unknown>, "goal_info");
  const stampedRecord =
    header && typeof header === "object" ? (header as Record<string, unknown>) : (value as Record<string, unknown>);
  const stamp = getRecordEntry(stampedRecord, "stamp");
  if (!stamp || typeof stamp !== "object") {
    return nowNs();
  }
  const sec = Number(getRecordEntry(stamp as Record<string, unknown>, "sec") ?? 0);
  const nanosec = Number(
    getRecordEntry(stamp as Record<string, unknown>, "nanosec") ??
      getRecordEntry(stamp as Record<string, unknown>, "nsec") ??
      0,
  );
  if (!Number.isFinite(sec) || !Number.isFinite(nanosec)) {
    return nowNs();
  }
  return BigInt(Math.trunc(sec)) * 1_000_000_000n + BigInt(Math.trunc(nanosec));
}

function updateTransformTree(tree: TransformTree, message: Record<string, unknown> | null): Time | null {
  const transforms = getRecordEntry(message ?? {}, "transforms");
  if (!Array.isArray(transforms)) {
    return null;
  }

  let latestTimestampNs: Time | null = null;
  for (const transformEntry of transforms) {
    if (!transformEntry || typeof transformEntry !== "object") {
      continue;
    }

    const transformRecord = transformEntry as Record<string, unknown>;
    const header = getRecordEntry(transformRecord, "header");
    const headerRecord =
      header && typeof header === "object" ? (header as Record<string, unknown>) : null;
    const parentFrame = normalizeFrameId(headerRecord ? getRecordEntry(headerRecord, "frame_id") : null);
    const childFrame = normalizeFrameId(getRecordEntry(transformRecord, "child_frame_id"));
    const transform = getRecordEntry(transformRecord, "transform");
    const transformValue =
      transform && typeof transform === "object" ? (transform as Record<string, unknown>) : null;
    const translation =
      transformValue && typeof getRecordEntry(transformValue, "translation") === "object"
        ? (getRecordEntry(transformValue, "translation") as Record<string, unknown>)
        : null;
    const rotation =
      transformValue && typeof getRecordEntry(transformValue, "rotation") === "object"
        ? (getRecordEntry(transformValue, "rotation") as Record<string, unknown>)
        : null;

    if (!parentFrame || !childFrame || !translation || !rotation) {
      continue;
    }

    const tx = toFiniteNumber(getRecordEntry(translation, "x"));
    const ty = toFiniteNumber(getRecordEntry(translation, "y"));
    const tz = toFiniteNumber(getRecordEntry(translation, "z"));
    const qx = toFiniteNumber(getRecordEntry(rotation, "x"));
    const qy = toFiniteNumber(getRecordEntry(rotation, "y"));
    const qz = toFiniteNumber(getRecordEntry(rotation, "z"));
    const qw = toFiniteNumber(getRecordEntry(rotation, "w"));
    if ([tx, ty, tz, qx, qy, qz, qw].some((entry) => entry == null)) {
      continue;
    }

    const frameTransform = Transform.Empty().setPositionRotation(
      vec3FromValues(tx!, ty!, tz!),
      quatFromValues(qx!, qy!, qz!, qw!),
    );
    const timestampNs = extractTimestampNs(transformRecord);
    tree.addTransform(childFrame, parentFrame, timestampNs, frameTransform);
    if (latestTimestampNs == null || timestampNs > latestTimestampNs) {
      latestTimestampNs = timestampNs;
    }
  }

  return latestTimestampNs;
}

function applyPointToMap(
  tree: TransformTree,
  outputPose: Pose,
  inputPose: Pose,
  sourceFrameId: string,
  timestampNs: Time,
): Pose | undefined {
  return tree.apply(
    outputPose,
    inputPose,
    MAP_FRAME_ID,
    undefined,
    sourceFrameId,
    timestampNs,
    timestampNs,
  );
}

function projectPointToMap(
  tree: TransformTree,
  point: ProjectedMapPoint,
  sourceFrameId: string | null,
  latestTransformTimestampNs: Time | null,
): ProjectedMapPoint | null {
  const normalizedSource = normalizeFrameId(sourceFrameId);
  if (!normalizedSource || !tree.hasFrame(MAP_FRAME_ID) || !tree.hasFrame(normalizedSource)) {
    return null;
  }
  if (normalizedSource === MAP_FRAME_ID) {
    return point;
  }

  const inputPose = makePose();
  const outputPose = makePose();
  inputPose.position.x = point.x;
  inputPose.position.y = point.y;
  inputPose.position.z = point.z;
  inputPose.orientation.x = 0;
  inputPose.orientation.y = 0;
  inputPose.orientation.z = 0;
  inputPose.orientation.w = 1;

  const projectedPose = applyPointToMap(
    tree,
    outputPose,
    inputPose,
    normalizedSource,
    latestTransformTimestampNs ?? nowNs(),
  );
  return projectedPose
    ? {
        x: projectedPose.position.x,
        y: projectedPose.position.y,
        z: projectedPose.position.z,
      }
    : null;
}

function getLidarSourceFrameCandidates(frameId: string): string[] {
  const candidates = [frameId];
  const normalized = normalizeFrameId(frameId);
  if (normalized) {
    const parts = normalized.split("/");
    for (let count = parts.length - 1; count >= 1; count -= 1) {
      candidates.push(parts.slice(0, count).join("/"));
    }
  }
  candidates.push(...LIDAR_FRAME_FALLBACKS);
  return Array.from(new Set(candidates));
}

function projectPointsToMap(
  tree: TransformTree,
  sourceFrameIds: string | string[],
  timestampNs: Time,
  points: ProjectedMapPoint[],
  latestTransformTimestampNs: Time | null,
): SensorProjection {
  if (points.length === 0) {
    return { points: [], status: "waiting" };
  }

  const sourceFrameCandidates = Array.isArray(sourceFrameIds) ? sourceFrameIds : [sourceFrameIds];
  const normalizedSource = sourceFrameCandidates
    .map((sourceFrameId) => normalizeFrameId(sourceFrameId))
    .find((sourceFrameId): sourceFrameId is string =>
      !!sourceFrameId && tree.hasFrame(MAP_FRAME_ID) && tree.hasFrame(sourceFrameId),
    );
  if (!normalizedSource || !tree.hasFrame(MAP_FRAME_ID) || !tree.hasFrame(normalizedSource)) {
    return { points: [], status: "no-tf" };
  }

  if (normalizedSource === MAP_FRAME_ID) {
    return { points, status: "live" };
  }

  const inputPose = makePose();
  const outputPose = makePose();
  const projectedPoints: ProjectedMapPoint[] = [];
  const fallbackTimestampNs = latestTransformTimestampNs ?? nowNs();

  for (const point of points) {
    inputPose.position.x = point.x;
    inputPose.position.y = point.y;
    inputPose.position.z = point.z;
    inputPose.orientation.x = 0;
    inputPose.orientation.y = 0;
    inputPose.orientation.z = 0;
    inputPose.orientation.w = 1;

    const projectedPose =
      applyPointToMap(tree, outputPose, inputPose, normalizedSource, timestampNs) ??
      applyPointToMap(tree, outputPose, inputPose, normalizedSource, fallbackTimestampNs);
    if (!projectedPose) {
      continue;
    }

    projectedPoints.push({
      x: projectedPose.position.x,
      y: projectedPose.position.y,
      z: projectedPose.position.z,
    });
  }

  if (projectedPoints.length === 0) {
    return { points: [], status: "no-tf" };
  }

  return {
    points: projectedPoints,
    status: "live",
  };
}

function projectFramePoseToMap(
  tree: TransformTree,
  sourceFrameIds: string[],
  latestTransformTimestampNs: Time | null,
): PoseCoordinates | null {
  if (!tree.hasFrame(MAP_FRAME_ID)) {
    return null;
  }

  const inputPose = makePose();
  const outputPose = makePose();
  const timestampNs = latestTransformTimestampNs ?? nowNs();
  for (const sourceFrameId of sourceFrameIds) {
    const normalizedSource = normalizeFrameId(sourceFrameId);
    if (!normalizedSource || !tree.hasFrame(normalizedSource)) {
      continue;
    }

    const projectedPose = applyPointToMap(
      tree,
      outputPose,
      inputPose,
      normalizedSource,
      timestampNs,
    );
    if (!projectedPose) {
      continue;
    }

    return {
      x: projectedPose.position.x,
      y: projectedPose.position.y,
      yaw: quaternionToYawDegrees(projectedPose.orientation),
    };
  }

  return null;
}

function projectPointsFromRobotPose(
  points: ProjectedMapPoint[],
  robotPose: PoseCoordinates | null,
): SensorProjection {
  if (points.length === 0) {
    return { points: [], status: "waiting" };
  }
  if (!robotPose) {
    return { points: [], status: "no-tf" };
  }

  const yawRadians = ((robotPose.yaw ?? 0) * Math.PI) / 180;
  const cosYaw = Math.cos(yawRadians);
  const sinYaw = Math.sin(yawRadians);

  return {
    points: points.map((point) => ({
      x: robotPose.x + point.x * cosYaw - point.y * sinYaw,
      y: robotPose.y + point.x * sinYaw + point.y * cosYaw,
      z: point.z,
    })),
    status: "live",
  };
}

function parseLaserScan(message: Record<string, unknown> | null): ParsedLaserScan | null {
  if (!message) {
    return null;
  }

  const frameId = getMessageFrameId(message);
  const ranges = toNumericArray(getRecordEntry(message, "ranges"));
  const angleMin = toFiniteNumber(getRecordEntry(message, "angle_min"));
  const angleIncrement = toFiniteNumber(getRecordEntry(message, "angle_increment"));
  const rangeMin = toFiniteNumber(getRecordEntry(message, "range_min")) ?? 0;
  const rangeMax = toFiniteNumber(getRecordEntry(message, "range_max")) ?? Number.POSITIVE_INFINITY;

  if (!frameId || !ranges || angleMin == null || angleIncrement == null) {
    return null;
  }

  const step = Math.max(2, Math.ceil(ranges.length / MAX_LIDAR_POINTS));
  const points: ProjectedMapPoint[] = [];
  for (let index = 0; index < ranges.length; index += step) {
    const range = Number(ranges[index]);
    if (!Number.isFinite(range) || range < rangeMin || range > rangeMax) {
      continue;
    }

    const angle = angleMin + angleIncrement * index;
    points.push({
      x: Math.cos(angle) * range,
      y: Math.sin(angle) * range,
      z: 0,
    });
  }

  return {
    frameId,
    timestampNs: extractTimestampNs(message),
    points,
  };
}

function getPointCloudFieldMap(
  fieldsValue: unknown,
): Map<string, { offset: number; datatype: number }> | null {
  if (!Array.isArray(fieldsValue)) {
    return null;
  }

  const fieldMap = new Map<string, { offset: number; datatype: number }>();
  for (const entry of fieldsValue) {
    if (!entry || typeof entry !== "object") {
      continue;
    }
    const record = entry as Record<string, unknown>;
    const name = getRecordEntry(record, "name");
    const offset = toFiniteNumber(getRecordEntry(record, "offset"));
    const datatype = toFiniteNumber(getRecordEntry(record, "datatype"));
    if (typeof name !== "string" || offset == null || datatype == null) {
      continue;
    }
    fieldMap.set(name, { offset, datatype });
  }
  return fieldMap;
}

function readPointField(
  view: DataView,
  pointOffset: number,
  field: { offset: number; datatype: number },
  littleEndian: boolean,
): number | null {
  const offset = pointOffset + field.offset;
  if (offset < 0 || offset >= view.byteLength) {
    return null;
  }
  if (field.datatype === 1 && offset + 1 <= view.byteLength) {
    return view.getInt8(offset);
  }
  if (field.datatype === 2 && offset + 1 <= view.byteLength) {
    return view.getUint8(offset);
  }
  if (field.datatype === 3 && offset + 2 <= view.byteLength) {
    return view.getInt16(offset, littleEndian);
  }
  if (field.datatype === 4 && offset + 2 <= view.byteLength) {
    return view.getUint16(offset, littleEndian);
  }
  if (field.datatype === 5 && offset + 4 <= view.byteLength) {
    return view.getInt32(offset, littleEndian);
  }
  if (field.datatype === 6 && offset + 4 <= view.byteLength) {
    return view.getUint32(offset, littleEndian);
  }
  if (field.datatype === 7 && offset + 4 <= view.byteLength) {
    return view.getFloat32(offset, littleEndian);
  }
  if (field.datatype === 8 && offset + 8 <= view.byteLength) {
    return view.getFloat64(offset, littleEndian);
  }
  return null;
}

function parsePointCloud2(message: Record<string, unknown> | null): ParsedPointCloud2 | null {
  if (!message) {
    return null;
  }

  const header = getRecordEntry(message, "header");
  const headerRecord =
    header && typeof header === "object" ? (header as Record<string, unknown>) : null;
  const frameId = normalizeFrameId(headerRecord ? getRecordEntry(headerRecord, "frame_id") : null);
  const pointStep = toFiniteNumber(getRecordEntry(message, "point_step"));
  const rowStepValue = toFiniteNumber(getRecordEntry(message, "row_step"));
  const widthValue = toFiniteNumber(getRecordEntry(message, "width"));
  const heightValue = toFiniteNumber(getRecordEntry(message, "height"));
  const data = toByteArray(getRecordEntry(message, "data"));
  const fields = getPointCloudFieldMap(getRecordEntry(message, "fields"));
  const isBigEndian = Boolean(getRecordEntry(message, "is_bigendian"));

  if (!frameId || !data || !fields || pointStep == null || pointStep <= 0) {
    return null;
  }

  const xField = fields.get("x");
  const yField = fields.get("y");
  const zField = fields.get("z");
  if (!xField || !yField || !zField) {
    return null;
  }

  const width = widthValue != null && widthValue > 0 ? Math.trunc(widthValue) : null;
  const height = heightValue != null && heightValue > 0 ? Math.trunc(heightValue) : null;
  const rowStep =
    rowStepValue != null && rowStepValue >= pointStep
      ? Math.trunc(rowStepValue)
      : width != null
        ? width * pointStep
        : data.byteLength;
  const rows = width != null && height != null ? height : 1;
  const columns = width ?? Math.floor(data.byteLength / pointStep);
  const pointCount = rows * columns;
  if (pointCount <= 0 || rowStep <= 0) {
    return null;
  }

  const stride = Math.max(1, Math.ceil(pointCount / MAX_DEPTH_POINTS));
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const points: ProjectedMapPoint[] = [];
  const littleEndian = !isBigEndian;

  for (let row = 0; row < rows; row += 1) {
    for (let column = 0; column < columns; column += 1) {
      const pointIndex = row * columns + column;
      if (pointIndex % stride !== 0) {
        continue;
      }

      const baseOffset = row * rowStep + column * pointStep;
      if (baseOffset < 0 || baseOffset + pointStep > view.byteLength) {
        continue;
      }

      const x = readPointField(view, baseOffset, xField, littleEndian);
      const y = readPointField(view, baseOffset, yField, littleEndian);
      const z = readPointField(view, baseOffset, zField, littleEndian);
      if ([x, y, z].some((value) => value == null || !Number.isFinite(value))) {
        continue;
      }

      points.push({ x: x!, y: y!, z: z! });
    }
  }

  return {
    frameId,
    timestampNs: extractTimestampNs(message),
    points,
  };
}

function getCompactTopicLabel(topic: string): string {
  if (topic.length <= 18) {
    return topic;
  }

  const parts = topic.split("/").filter(Boolean);
  if (parts.length >= 2) {
    return `.../${parts.slice(-2).join("/")}`;
  }
  return topic.slice(0, 18);
}

export function getOverlayStatusLabel(state: OverlayState, sourceTopic?: string | null): string {
  const topicLabel = sourceTopic ? ` ${getCompactTopicLabel(sourceTopic)}` : "";
  if (state === "live") {
    return `Live${topicLabel}`;
  }
  if (state === "no-tf") {
    return `No TF${topicLabel}`;
  }
  return `Waiting${topicLabel}`;
}

export function getOverlayDisabled(_state: OverlayState): boolean {
  return false;
}

export function selectLaserScanTopic(topics: Subscription[]): Subscription | null {
  const candidates = topics.filter((topic) => LASER_SCAN_SCHEMA_NAMES.has(topic.type));
  if (candidates.length === 0) {
    return null;
  }

  return (
    candidates.find((topic) => topic.topic === LIDAR_TOPIC_PREFERENCE) ??
    candidates.find((topic) => topic.topic.toLowerCase().includes("scan")) ??
    candidates[0] ??
    null
  );
}

export function selectDepthPointCloudTopic(topics: Subscription[]): Subscription | null {
  const candidates = topics.filter((topic) => ROS_POINT_CLOUD_SCHEMA_NAMES.has(topic.type));
  if (candidates.length === 0) {
    return null;
  }

  return (
    candidates.find((topic) => topic.topic === DEPTH_TOPIC_PREFERENCE) ??
    candidates.find((topic) => {
      const lowered = topic.topic.toLowerCase();
      return lowered.includes("depth") && lowered.includes("points");
    }) ??
    candidates.find((topic) => topic.topic.toLowerCase().includes("depth")) ??
    null
  );
}

export function useProjectedSensorOverlays(currentPose: PoseCoordinates | null): {
  lidarTopic: Subscription | null;
  depthTopic: Subscription | null;
  robotPose: PoseCoordinates | null;
  projectPointToMap: (point: ProjectedMapPoint, sourceFrameId: string | null) => ProjectedMapPoint | null;
  lidarPoints: ProjectedMapPoint[];
  depthObstaclePoints: ProjectedMapPoint[];
  overlayStatus: Pick<OverlayAvailability, "lidar" | "depthObstacles">;
} {
  const treeRef = useRef(
    new TransformTree(
      new ObjectPool(Transform.Empty, {
        maxCapacity: 5 * DEFAULT_MAX_CAPACITY_PER_FRAME,
      }),
    ),
  );
  const latestTransformTimestampRef = useRef<Time | null>(null);
  const [tfRevision, setTfRevision] = useState(0);
  const [lidarTopic, setLidarTopic] = useState<Subscription | null>(() =>
    selectLaserScanTopic(ros2Bridge.getAvailableTopics()),
  );
  const [laserScanMessage, setLaserScanMessage] = useState<Record<string, unknown> | null>(null);
  const [depthTopic, setDepthTopic] = useState<Subscription | null>(() =>
    selectDepthPointCloudTopic(ros2Bridge.getAvailableTopics()),
  );
  const [depthCloudMessage, setDepthCloudMessage] = useState<Record<string, unknown> | null>(null);

  useEffect(() => {
    const unsubscribes = TF_SUBSCRIPTIONS.map((subscription) =>
      ros2Bridge.subscribe(subscription, (message) => {
        const latestTimestampNs = updateTransformTree(treeRef.current, normalizeRosMessage(message));
        if (
          latestTimestampNs != null &&
          (latestTransformTimestampRef.current == null || latestTimestampNs > latestTransformTimestampRef.current)
        ) {
          latestTransformTimestampRef.current = latestTimestampNs;
        }
        setTfRevision((current) => current + 1);
      }),
    );
    return () => {
      unsubscribes.forEach((unsubscribe) => unsubscribe());
    };
  }, []);

  useEffect(() => {
    const updateSensorTopics = (topics: Subscription[]) => {
      setLidarTopic(selectLaserScanTopic(topics));
      setDepthTopic(selectDepthPointCloudTopic(topics));
    };

    const unsubscribe = ros2Bridge.onAvailableTopicsChanged(updateSensorTopics);
    updateSensorTopics(ros2Bridge.getAvailableTopics());
    return unsubscribe;
  }, []);

  useEffect(() => {
    setLaserScanMessage(null);
    if (!lidarTopic) {
      return;
    }

    const unsubscribe = ros2Bridge.subscribe(lidarTopic, (message) => {
      setLaserScanMessage(normalizeRosMessage(message));
    });
    return unsubscribe;
  }, [lidarTopic]);

  useEffect(() => {
    setDepthCloudMessage(null);
    if (!depthTopic) {
      return;
    }

    const unsubscribe = ros2Bridge.subscribe(depthTopic, (message) => {
      setDepthCloudMessage(normalizeRosMessage(message));
    });
    return unsubscribe;
  }, [depthTopic]);

  const robotPose = useMemo<PoseCoordinates | null>(() => {
    const scanFrameId = getMessageFrameId(laserScanMessage);
    const sourceFrames = scanFrameId
      ? [...ROBOT_POSE_FRAME_CANDIDATES, scanFrameId]
      : ROBOT_POSE_FRAME_CANDIDATES;
    return projectFramePoseToMap(
      treeRef.current,
      sourceFrames,
      latestTransformTimestampRef.current,
    );
  }, [laserScanMessage, tfRevision]);
  const filterPose = currentPose ?? robotPose;
  const projectSinglePointToMap = useCallback(
    (point: ProjectedMapPoint, sourceFrameId: string | null) =>
      projectPointToMap(
        treeRef.current,
        point,
        sourceFrameId,
        latestTransformTimestampRef.current,
      ),
    [tfRevision],
  );

  const lidarProjection = useMemo<SensorProjection>(() => {
    if (!lidarTopic) {
      return { points: [], status: "waiting" };
    }

    const parsedScan = parseLaserScan(laserScanMessage);
    if (!parsedScan) {
      return { points: [], status: "waiting" };
    }
    const projected = projectPointsToMap(
      treeRef.current,
      getLidarSourceFrameCandidates(parsedScan.frameId),
      parsedScan.timestampNs,
      parsedScan.points,
      latestTransformTimestampRef.current,
    );
    return projected.status === "live" ? projected : projectPointsFromRobotPose(parsedScan.points, robotPose);
  }, [laserScanMessage, lidarTopic, robotPose, tfRevision]);

  const depthProjection = useMemo<SensorProjection>(() => {
    if (!depthTopic || !ROS_POINT_CLOUD_SCHEMA_NAMES.has(depthTopic.type)) {
      return { points: [], status: "waiting" };
    }

    const parsedCloud = parsePointCloud2(depthCloudMessage);
    if (!parsedCloud) {
      return { points: [], status: "waiting" };
    }

    const projected = projectPointsToMap(
      treeRef.current,
      parsedCloud.frameId,
      parsedCloud.timestampNs,
      parsedCloud.points,
      latestTransformTimestampRef.current,
    );
    if (projected.status !== "live") {
      return projected;
    }

    const filteredPoints = projected.points.filter((point) => {
      if (!Number.isFinite(point.x) || !Number.isFinite(point.y) || !Number.isFinite(point.z)) {
        return false;
      }
      if (point.z < 0.03 || point.z > 0.45) {
        return false;
      }
      if (!filterPose) {
        return true;
      }
      return Math.hypot(point.x - filterPose.x, point.y - filterPose.y) <= MAX_DEPTH_DISTANCE_M;
    });

    return {
      points: filteredPoints,
      status: "live",
    };
  }, [depthCloudMessage, depthTopic, filterPose, tfRevision]);

  return {
    lidarTopic,
    depthTopic,
    robotPose,
    projectPointToMap: projectSinglePointToMap,
    lidarPoints: lidarProjection.points,
    depthObstaclePoints: depthProjection.points,
    overlayStatus: {
      lidar: lidarProjection.status,
      depthObstacles: depthProjection.status,
    },
  };
}
