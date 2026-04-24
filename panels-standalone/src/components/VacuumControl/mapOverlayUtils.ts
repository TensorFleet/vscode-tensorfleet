import { useEffect, useMemo, useRef, useState } from "react";
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

const POINT_CLOUD_SCHEMA_NAMES = new Set([
  "sensor_msgs/PointCloud2",
  "sensor_msgs/msg/PointCloud2",
  "foxglove.PointCloud",
]);

const ROS_POINT_CLOUD_SCHEMA_NAMES = new Set([
  "sensor_msgs/PointCloud2",
  "sensor_msgs/msg/PointCloud2",
]);

const MAP_FRAME_ID = "map";
const LIDAR_TOPIC = "/scan";
const LIDAR_SCHEMA_NAME = "sensor_msgs/msg/LaserScan";
const DEPTH_TOPIC_PREFERENCE = "/oakd/rgb/preview/depth/points";
const MAX_LIDAR_POINTS = 900;
const MAX_DEPTH_POINTS = 1_500;
const MAX_DEPTH_DISTANCE_M = 6;

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

function projectPointsToMap(
  tree: TransformTree,
  sourceFrameId: string,
  timestampNs: Time,
  points: ProjectedMapPoint[],
  latestTransformTimestampNs: Time | null,
): SensorProjection {
  if (points.length === 0) {
    return { points: [], status: "waiting" };
  }

  const normalizedSource = normalizeFrameId(sourceFrameId);
  if (!normalizedSource || !tree.hasFrame(MAP_FRAME_ID) || !tree.hasFrame(normalizedSource)) {
    return { points: [], status: "no-tf" };
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

function parseLaserScan(message: Record<string, unknown> | null): ParsedLaserScan | null {
  if (!message) {
    return null;
  }

  const header = getRecordEntry(message, "header");
  const headerRecord =
    header && typeof header === "object" ? (header as Record<string, unknown>) : null;
  const frameId = normalizeFrameId(headerRecord ? getRecordEntry(headerRecord, "frame_id") : null);
  const rangesValue = getRecordEntry(message, "ranges");
  const ranges = Array.isArray(rangesValue)
    ? rangesValue
    : typeof ArrayBuffer !== "undefined" && ArrayBuffer.isView(rangesValue)
      ? Array.from(rangesValue as ArrayLike<number>)
      : null;
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
): number | null {
  const offset = pointOffset + field.offset;
  if (offset < 0 || offset >= view.byteLength) {
    return null;
  }
  if (field.datatype === 7 && offset + 4 <= view.byteLength) {
    return view.getFloat32(offset, true);
  }
  if (field.datatype === 8 && offset + 8 <= view.byteLength) {
    return view.getFloat64(offset, true);
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
  const data = toByteArray(getRecordEntry(message, "data"));
  const fields = getPointCloudFieldMap(getRecordEntry(message, "fields"));
  const isBigEndian = Boolean(getRecordEntry(message, "is_bigendian"));

  if (!frameId || !data || !fields || pointStep == null || pointStep <= 0 || isBigEndian) {
    return null;
  }

  const xField = fields.get("x");
  const yField = fields.get("y");
  const zField = fields.get("z");
  if (!xField || !yField || !zField) {
    return null;
  }

  const pointCount = Math.floor(data.byteLength / pointStep);
  if (pointCount <= 0) {
    return null;
  }

  const stride = Math.max(1, Math.ceil(pointCount / MAX_DEPTH_POINTS));
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const points: ProjectedMapPoint[] = [];

  for (let pointIndex = 0; pointIndex < pointCount; pointIndex += stride) {
    const baseOffset = pointIndex * pointStep;
    const x = readPointField(view, baseOffset, xField);
    const y = readPointField(view, baseOffset, yField);
    const z = readPointField(view, baseOffset, zField);
    if ([x, y, z].some((value) => value == null || !Number.isFinite(value))) {
      continue;
    }

    points.push({ x: x!, y: y!, z: z! });
  }

  return {
    frameId,
    timestampNs: extractTimestampNs(message),
    points,
  };
}

export function getOverlayStatusLabel(state: OverlayState): string {
  if (state === "live") {
    return "Live";
  }
  if (state === "no-tf") {
    return "No TF";
  }
  return "Waiting";
}

export function getOverlayDisabled(state: OverlayState): boolean {
  return state !== "live";
}

export function selectDepthPointCloudTopic(topics: Subscription[]): Subscription | null {
  const candidates = topics.filter((topic) => POINT_CLOUD_SCHEMA_NAMES.has(topic.type));
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
  depthTopic: Subscription | null;
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
    const unsubscribe = ros2Bridge.subscribe(
      { topic: LIDAR_TOPIC, type: LIDAR_SCHEMA_NAME },
      (message) => {
        setLaserScanMessage(normalizeRosMessage(message));
      },
    );
    return unsubscribe;
  }, []);

  useEffect(() => {
    const updateDepthTopic = (topics: Subscription[]) => {
      setDepthTopic(selectDepthPointCloudTopic(topics));
    };

    const unsubscribe = ros2Bridge.onAvailableTopicsChanged(updateDepthTopic);
    updateDepthTopic(ros2Bridge.getAvailableTopics());
    return unsubscribe;
  }, []);

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

  const lidarProjection = useMemo<SensorProjection>(() => {
    const parsedScan = parseLaserScan(laserScanMessage);
    if (!parsedScan) {
      return { points: [], status: "waiting" };
    }
    return projectPointsToMap(
      treeRef.current,
      parsedScan.frameId,
      parsedScan.timestampNs,
      parsedScan.points,
      latestTransformTimestampRef.current,
    );
  }, [laserScanMessage, tfRevision]);

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
      if (!currentPose) {
        return true;
      }
      return Math.hypot(point.x - currentPose.x, point.y - currentPose.y) <= MAX_DEPTH_DISTANCE_M;
    });

    return {
      points: filteredPoints,
      status: "live",
    };
  }, [currentPose, depthCloudMessage, depthTopic, tfRevision]);

  return {
    depthTopic,
    lidarPoints: lidarProjection.points,
    depthObstaclePoints: depthProjection.points,
    overlayStatus: {
      lidar: lidarProjection.status,
      depthObstacles: depthProjection.status,
    },
  };
}
