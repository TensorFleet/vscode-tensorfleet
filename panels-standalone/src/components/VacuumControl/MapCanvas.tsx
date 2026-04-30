import React, { useEffect, useMemo, useRef, useState } from "react";
import { ros2Bridge } from "../../ros2-bridge";
import {
  getRecordEntry,
  normalizeRosMessage,
} from "../Nav2/runtime/nav2RuntimeUtils";
import type { PoseCoordinates } from "../Nav2/runtime/nav2RuntimeTypes";
import {
  DEFAULT_OVERLAY_VISIBILITY,
  MAP_OVERLAY_DEFINITIONS,
  getOverlayDisabled,
  getOverlayStatusLabel,
  useProjectedSensorOverlays,
  type MapOverlayKey,
  type OverlayAvailability,
  type OverlayVisibility,
  type ProjectedMapPoint,
} from "./mapOverlayUtils";
import { CameraOverlay } from "./CameraOverlay";

export type MapCanvasTarget = {
  x: number;
  y: number;
  yaw: number;
};

export type MapViewMode = "fit" | "manual" | "follow_robot";

export type MappingSessionState =
  | "not_started"
  | "mapping"
  | "paused"
  | "review"
  | "saved"
  | "discarded"
  | "error";

export type MapCanvasMetadata = {
  hasMap: boolean;
  width: number;
  height: number;
  resolution: number;
  freeCells: number;
  occupiedCells: number;
  unknownCells: number;
  knownCells: number;
  totalCells: number;
  freeRatio: number;
  occupiedRatio: number;
  unknownRatio: number;
  knownRatio: number;
  knownAreaSqM: number;
  lastUpdateAt: number | null;
  poseAvailable: boolean;
  readiness: "No map" | "Map active" | "Map mostly unknown" | "Map review ready" | "Map saved";
};

type MapPoint = {
  x: number;
  y: number;
};

type OccupancyMap = {
  width: number;
  height: number;
  resolution: number;
  originX: number;
  originY: number;
  originYaw: number;
  frameId: string | null;
  data: number[];
};

type MapBounds = {
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
  width: number;
  height: number;
  hasLiveMap: boolean;
};

type CanvasSize = {
  width: number;
  height: number;
};

type MapViewport = {
  viewMode: MapViewMode;
  scale: number;
  offsetX: number;
  offsetY: number;
};

type PointerDragState = {
  pointerId: number;
  startX: number;
  startY: number;
  lastX: number;
  lastY: number;
  isPanning: boolean;
};

export type RouteVisualState = "idle" | "staged" | "active" | "completed" | "failed" | "canceled";

type RasterLayerKey = "map" | "globalCostmap" | "localCostmap";
type RasterLayerMode = "map" | "global-costmap" | "local-costmap";

type RasterLayerConfig = {
  key: RasterLayerKey;
  topic: string;
  type: string;
};

type PointOverlayStyle = {
  fill: string;
  haloFill?: string;
  radius: number;
};

export type MapCanvasProps = {
  currentPose: PoseCoordinates | null;
  planPoints: MapPoint[] | null;
  draftTarget: MapCanvasTarget | null;
  sentTarget: MapCanvasTarget | null;
  routeVisualState: RouteVisualState;
  isGoalActive: boolean;
  mappingState: MappingSessionState;
  disableTargetSelection?: boolean;
  targetDistance: number | null;
  onTargetStart: (target: MapCanvasTarget) => void;
  onTargetRotate: (yaw: number) => void;
  onMapMetadataChange?: (metadata: MapCanvasMetadata) => void;
};

const MAP_FREE_COLOR = { r: 232, g: 230, b: 224 };
const MAP_OCCUPIED_COLOR = { r: 58, g: 58, b: 58 };
const MAP_UNKNOWN_COLOR = { r: 200, g: 196, b: 188 };
const COSTMAP_LOW_THRESHOLD = 5;
const FIT_PADDING_PX = 42;
const MIN_ZOOM_RATIO = 0.5;
const MAX_ZOOM_RATIO = 8;
const POINTER_PAN_THRESHOLD_PX = 5;
const RASTER_LAYER_CONFIGS: RasterLayerConfig[] = [
  {
    key: "map",
    topic: "/map",
    type: "nav_msgs/msg/OccupancyGrid",
  },
  {
    key: "globalCostmap",
    topic: "/global_costmap/costmap",
    type: "nav_msgs/msg/OccupancyGrid",
  },
  {
    key: "localCostmap",
    topic: "/local_costmap/costmap",
    type: "nav_msgs/msg/OccupancyGrid",
  },
];

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
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

function mixChannel(start: number, end: number, amount: number): number {
  return Math.round(start + (end - start) * amount);
}

function quaternionToYawDegrees(orientation: Record<string, unknown> | null): number {
  if (!orientation) {
    return 0;
  }
  const x = toFiniteNumber(getRecordEntry(orientation, "x")) ?? 0;
  const y = toFiniteNumber(getRecordEntry(orientation, "y")) ?? 0;
  const z = toFiniteNumber(getRecordEntry(orientation, "z")) ?? 0;
  const w = toFiniteNumber(getRecordEntry(orientation, "w")) ?? 1;
  const sinyCosp = 2 * (w * z + x * y);
  const cosyCosp = 1 - 2 * (y * y + z * z);
  return (Math.atan2(sinyCosp, cosyCosp) * 180) / Math.PI;
}

function colorizeCostmapCell(
  cell: number,
  mode: Exclude<RasterLayerMode, "map">,
): { r: number; g: number; b: number; a: number } {
  if (cell < 0 || cell <= COSTMAP_LOW_THRESHOLD) {
    return { r: 0, g: 0, b: 0, a: 0 };
  }

  const intensity = clamp(cell, 0, 100) / 100;
  if (mode === "global-costmap") {
    return {
      r: mixChannel(246, 255, intensity),
      g: mixChannel(208, 92, intensity),
      b: mixChannel(92, 52, intensity),
      a: mixChannel(44, 188, intensity),
    };
  }

  return {
    r: mixChannel(82, 26, intensity),
    g: mixChannel(212, 156, intensity),
    b: mixChannel(255, 255, intensity),
    a: mixChannel(48, 196, intensity),
  };
}

function parseOccupancyMap(message: Record<string, unknown> | null): OccupancyMap | null {
  if (!message) {
    return null;
  }

  const info = getRecordEntry(message, "info");
  if (!info || typeof info !== "object") {
    return null;
  }

  const width = toFiniteNumber(getRecordEntry(info as Record<string, unknown>, "width"));
  const height = toFiniteNumber(getRecordEntry(info as Record<string, unknown>, "height"));
  const resolution = toFiniteNumber(getRecordEntry(info as Record<string, unknown>, "resolution"));
  const origin = getRecordEntry(info as Record<string, unknown>, "origin");
  const position =
    origin && typeof origin === "object"
      ? getRecordEntry(origin as Record<string, unknown>, "position")
      : null;
  const originX =
    position && typeof position === "object"
      ? toFiniteNumber(getRecordEntry(position as Record<string, unknown>, "x"))
      : null;
  const originY =
    position && typeof position === "object"
      ? toFiniteNumber(getRecordEntry(position as Record<string, unknown>, "y"))
      : null;
  const orientation =
    origin && typeof origin === "object" && typeof getRecordEntry(origin as Record<string, unknown>, "orientation") === "object"
      ? (getRecordEntry(origin as Record<string, unknown>, "orientation") as Record<string, unknown>)
      : null;
  const data = toNumericArray(getRecordEntry(message, "data"));
  const header = getRecordEntry(message, "header");
  const frameId =
    header && typeof header === "object"
      ? getRecordEntry(header as Record<string, unknown>, "frame_id")
      : null;

  if (
    width == null ||
    height == null ||
    resolution == null ||
    originX == null ||
    originY == null ||
    data == null
  ) {
    return null;
  }

  return {
    width,
    height,
    resolution,
    originX,
    originY,
    originYaw: quaternionToYawDegrees(orientation),
    frameId: typeof frameId === "string" && frameId.length > 0 ? frameId : null,
    data,
  };
}

function drawPlaceholderMap(canvas: HTMLCanvasElement): void {
  const size = 320;
  const context = canvas.getContext("2d");
  if (!context) {
    return;
  }

  canvas.width = size;
  canvas.height = size;

  const image = context.createImageData(size, size);
  for (let y = 0; y < size; y += 1) {
    for (let x = 0; x < size; x += 1) {
      const index = (y * size + x) * 4;
      const mainCorridor = Math.sin((x + 28) * 0.024) + Math.cos((y - 34) * 0.028);
      const sideWalls = Math.sin((x - y) * 0.06) + Math.cos((x + y) * 0.04);
      const occupied =
        mainCorridor > 1.33 ||
        sideWalls > 1.46 ||
        x < 12 ||
        y < 12 ||
        x > size - 12 ||
        y > size - 12;

      const r = occupied ? MAP_OCCUPIED_COLOR.r : MAP_FREE_COLOR.r;
      const g = occupied ? MAP_OCCUPIED_COLOR.g : MAP_FREE_COLOR.g;
      const b = occupied ? MAP_OCCUPIED_COLOR.b : MAP_FREE_COLOR.b;

      image.data[index] = r;
      image.data[index + 1] = g;
      image.data[index + 2] = b;
      image.data[index + 3] = 255;
    }
  }

  context.putImageData(image, 0, 0);
}

function clearCanvas(canvas: HTMLCanvasElement): void {
  const context = canvas.getContext("2d");
  if (!context) {
    return;
  }

  canvas.width = 1;
  canvas.height = 1;
  context.clearRect(0, 0, 1, 1);
}

function drawOccupancyMap(
  canvas: HTMLCanvasElement,
  map: OccupancyMap | null,
  mode: RasterLayerMode,
): void {
  if (!map) {
    if (mode === "map") {
      drawPlaceholderMap(canvas);
    } else {
      clearCanvas(canvas);
    }
    return;
  }

  const context = canvas.getContext("2d");
  if (!context) {
    return;
  }

  canvas.width = map.width;
  canvas.height = map.height;

  const image = context.createImageData(map.width, map.height);
  for (let y = 0; y < map.height; y += 1) {
    for (let x = 0; x < map.width; x += 1) {
      const sourceIndex = x + (map.height - 1 - y) * map.width;
      const cell = Number(map.data[sourceIndex] ?? -1);
      const pixelIndex = (y * map.width + x) * 4;

      let r = 44;
      let g = 50;
      let b = 58;
      let a = 255;

      if (mode !== "map") {
        const color = colorizeCostmapCell(cell, mode);
        r = color.r;
        g = color.g;
        b = color.b;
        a = color.a;
      } else if (cell < 0) {
        r = MAP_UNKNOWN_COLOR.r;
        g = MAP_UNKNOWN_COLOR.g;
        b = MAP_UNKNOWN_COLOR.b;
      } else if (cell <= 15) {
        r = MAP_FREE_COLOR.r;
        g = MAP_FREE_COLOR.g;
        b = MAP_FREE_COLOR.b;
      } else {
        const occupancy = clamp(cell, 0, 100) / 100;
        r = mixChannel(MAP_FREE_COLOR.r, MAP_OCCUPIED_COLOR.r, occupancy);
        g = mixChannel(MAP_FREE_COLOR.g, MAP_OCCUPIED_COLOR.g, occupancy);
        b = mixChannel(MAP_FREE_COLOR.b, MAP_OCCUPIED_COLOR.b, occupancy);
      }

      image.data[pixelIndex] = r;
      image.data[pixelIndex + 1] = g;
      image.data[pixelIndex + 2] = b;
      image.data[pixelIndex + 3] = a;
    }
  }

  context.putImageData(image, 0, 0);
}

function deriveBounds(map: OccupancyMap | null, pose: PoseCoordinates | null): MapBounds {
  if (map) {
    return {
      minX: map.originX,
      maxX: map.originX + map.width * map.resolution,
      minY: map.originY,
      maxY: map.originY + map.height * map.resolution,
      width: map.width * map.resolution,
      height: map.height * map.resolution,
      hasLiveMap: true,
    };
  }

  const centerX = pose?.x ?? 0;
  const centerY = pose?.y ?? 0;
  const span = 8;
  return {
    minX: centerX - span / 2,
    maxX: centerX + span / 2,
    minY: centerY - span / 2,
    maxY: centerY + span / 2,
    width: span,
    height: span,
    hasLiveMap: false,
  };
}

function getMapSignature(map: OccupancyMap | null): string | null {
  if (!map) {
    return null;
  }
  return [
    map.width,
    map.height,
    map.resolution,
    map.originX,
    map.originY,
    map.originYaw,
  ].join(":");
}

function getFitScale(bounds: MapBounds, canvasSize: CanvasSize): number {
  const availableWidth = Math.max(1, canvasSize.width - FIT_PADDING_PX * 2);
  const availableHeight = Math.max(1, canvasSize.height - FIT_PADDING_PX * 2);
  if (bounds.width <= 0 || bounds.height <= 0) {
    return 1;
  }
  return Math.max(0.001, Math.min(availableWidth / bounds.width, availableHeight / bounds.height));
}

function makeFitViewport(bounds: MapBounds, canvasSize: CanvasSize, viewMode: MapViewMode = "fit"): MapViewport {
  const scale = getFitScale(bounds, canvasSize);
  return {
    viewMode,
    scale,
    offsetX: (canvasSize.width - bounds.width * scale) / 2,
    offsetY: (canvasSize.height - bounds.height * scale) / 2,
  };
}

function centerViewportOnPoint(
  point: MapPoint,
  bounds: MapBounds,
  canvasSize: CanvasSize,
  scale: number,
  viewMode: MapViewMode,
): MapViewport {
  return {
    viewMode,
    scale,
    offsetX: canvasSize.width / 2 - (point.x - bounds.minX) * scale,
    offsetY: canvasSize.height / 2 - (bounds.maxY - point.y) * scale,
  };
}

function worldToScreen(point: MapPoint, bounds: MapBounds, viewport: MapViewport): MapPoint {
  return {
    x: viewport.offsetX + (point.x - bounds.minX) * viewport.scale,
    y: viewport.offsetY + (bounds.maxY - point.y) * viewport.scale,
  };
}

function screenToWorld(point: MapPoint, bounds: MapBounds, viewport: MapViewport): MapPoint {
  return {
    x: bounds.minX + (point.x - viewport.offsetX) / viewport.scale,
    y: bounds.maxY - (point.y - viewport.offsetY) / viewport.scale,
  };
}

function getMapLayerStyle(bounds: MapBounds, viewport: MapViewport): React.CSSProperties {
  return {
    left: `${viewport.offsetX}px`,
    top: `${viewport.offsetY}px`,
    width: `${bounds.width * viewport.scale}px`,
    height: `${bounds.height * viewport.scale}px`,
  };
}

function choosePrimaryRasterMap(
  map: OccupancyMap | null,
  globalCostmap: OccupancyMap | null,
  localCostmap: OccupancyMap | null,
): OccupancyMap | null {
  return map ?? globalCostmap ?? localCostmap;
}

function getRasterLayerStyle(
  map: OccupancyMap | null,
  bounds: MapBounds,
  viewport: MapViewport,
  mode: RasterLayerMode,
  projectPointToMap?: (point: ProjectedMapPoint, sourceFrameId: string | null) => ProjectedMapPoint | null,
): React.CSSProperties | undefined {
  if (!map || mode === "map") {
    return undefined;
  }

  const widthMeters = map.width * map.resolution;
  const heightMeters = map.height * map.resolution;
  const yawRadians = (map.originYaw * Math.PI) / 180;
  const cosYaw = Math.cos(yawRadians);
  const sinYaw = Math.sin(yawRadians);
  const localCorners = [
    { x: 0, y: heightMeters },
    { x: widthMeters, y: heightMeters },
    { x: widthMeters, y: 0 },
    { x: 0, y: 0 },
  ];
  const worldCorners = localCorners.map((corner) => ({
    x: map.originX + corner.x * cosYaw - corner.y * sinYaw,
    y: map.originY + corner.x * sinYaw + corner.y * cosYaw,
    z: 0,
  }));
  const projectedCorners = worldCorners.map((corner) =>
    map.frameId && map.frameId !== "map" && projectPointToMap
      ? projectPointToMap(corner, map.frameId)
      : corner,
  );

  if (projectedCorners.some((corner) => corner == null)) {
    return { display: "none" };
  }

  const cornerScreens = (projectedCorners as ProjectedMapPoint[]).map((corner) =>
    worldToScreen(corner, bounds, viewport),
  );
  const left = Math.min(...cornerScreens.map((corner) => corner.x));
  const right = Math.max(...cornerScreens.map((corner) => corner.x));
  const top = Math.min(...cornerScreens.map((corner) => corner.y));
  const bottom = Math.max(...cornerScreens.map((corner) => corner.y));

  return {
    left: `${left}px`,
    top: `${top}px`,
    width: `${right - left}px`,
    height: `${bottom - top}px`,
  };
}

function getMapPrompt(routeVisualState: RouteVisualState, hasTarget: boolean, mappingState: MappingSessionState): string {
  if (mappingState === "mapping") {
    return "Mapping in progress";
  }
  if (mappingState === "review") {
    return "Review map";
  }
  if (mappingState === "paused") {
    return "Mapping paused";
  }
  if (routeVisualState === "completed") {
    return "Destination reached";
  }
  if (routeVisualState === "active") {
    return "Heading to destination";
  }
  if (routeVisualState === "failed") {
    return "Run needs attention";
  }
  if (routeVisualState === "canceled") {
    return "Run canceled";
  }
  return hasTarget ? "Destination selected" : "Click to choose destination";
}

function formatDistanceShort(distance: number | null): string | null {
  if (distance == null || !Number.isFinite(distance)) {
    return null;
  }
  return distance < 10 ? `${distance.toFixed(1)} m` : `${distance.toFixed(0)} m`;
}

function getTargetLabel(routeVisualState: RouteVisualState, distance: number | null): string {
  const distStr = formatDistanceShort(distance);
  if (routeVisualState === "completed") {
    return "Done";
  }
  if (routeVisualState === "active") {
    return distStr ?? "Active";
  }
  if (routeVisualState === "failed") {
    return "Retry";
  }
  if (routeVisualState === "canceled") {
    return "Canceled";
  }
  return distStr ?? "Selected";
}

function buildMapMetadata(args: {
  map: OccupancyMap | null;
  lastUpdateAt: number | null;
  poseAvailable: boolean;
  mappingState: MappingSessionState;
}): MapCanvasMetadata {
  const { map, lastUpdateAt, poseAvailable, mappingState } = args;
  if (!map || map.width <= 0 || map.height <= 0) {
    return {
      hasMap: false,
      width: 0,
      height: 0,
      resolution: 0,
      freeCells: 0,
      occupiedCells: 0,
      unknownCells: 0,
      knownCells: 0,
      totalCells: 0,
      freeRatio: 0,
      occupiedRatio: 0,
      unknownRatio: 1,
      knownRatio: 0,
      knownAreaSqM: 0,
      lastUpdateAt,
      poseAvailable,
      readiness: "No map",
    };
  }

  let freeCells = 0;
  let occupiedCells = 0;
  let unknownCells = 0;
  for (const cell of map.data) {
    if (cell < 0) {
      unknownCells += 1;
    } else if (cell <= 15) {
      freeCells += 1;
    } else {
      occupiedCells += 1;
    }
  }

  const totalCells = Math.max(1, map.width * map.height);
  const knownCells = freeCells + occupiedCells;
  const knownRatio = knownCells / totalCells;
  const unknownRatio = unknownCells / totalCells;
  const readiness: MapCanvasMetadata["readiness"] =
    mappingState === "saved"
      ? "Map saved"
      : mappingState === "review"
        ? "Map review ready"
        : knownRatio < 0.18
          ? "Map mostly unknown"
          : "Map active";

  return {
    hasMap: true,
    width: map.width,
    height: map.height,
    resolution: map.resolution,
    freeCells,
    occupiedCells,
    unknownCells,
    knownCells,
    totalCells,
    freeRatio: freeCells / totalCells,
    occupiedRatio: occupiedCells / totalCells,
    unknownRatio,
    knownRatio,
    knownAreaSqM: knownCells * map.resolution * map.resolution,
    lastUpdateAt,
    poseAvailable,
    readiness,
  };
}

function drawPointOverlay(
  canvas: HTMLCanvasElement,
  points: ProjectedMapPoint[],
  bounds: MapBounds,
  viewport: MapViewport,
  style: PointOverlayStyle,
): void {
  const context = canvas.getContext("2d");
  if (!context) {
    return;
  }

  const width = Math.max(1, Math.round(canvas.clientWidth));
  const height = Math.max(1, Math.round(canvas.clientHeight));
  const devicePixelRatio = window.devicePixelRatio || 1;

  canvas.width = Math.max(1, Math.round(width * devicePixelRatio));
  canvas.height = Math.max(1, Math.round(height * devicePixelRatio));
  context.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  context.clearRect(0, 0, width, height);

  if (points.length === 0) {
    return;
  }

  const glowRadius = style.radius * 5;

  for (const point of points) {
    const screen = worldToScreen(point, bounds, viewport);
    if (!Number.isFinite(screen.x) || !Number.isFinite(screen.y)) {
      continue;
    }

    const x = screen.x;
    const y = screen.y;
    if (x < -glowRadius - 4 || y < -glowRadius - 4 || x > width + glowRadius + 4 || y > height + glowRadius + 4) {
      continue;
    }

    if (style.haloFill) {
      const gradient = context.createRadialGradient(x, y, style.radius * 0.4, x, y, glowRadius);
      gradient.addColorStop(0, style.haloFill);
      gradient.addColorStop(1, "rgba(0,0,0,0)");
      context.fillStyle = gradient;
      context.beginPath();
      context.arc(x, y, glowRadius, 0, Math.PI * 2);
      context.fill();
    }

    context.fillStyle = style.fill;
    context.beginPath();
    context.arc(x, y, style.radius, 0, Math.PI * 2);
    context.fill();
  }
}

const DEPTH_Z_LOW = 0.08;
const DEPTH_Z_HIGH = 0.55;

function drawDepthPointOverlay(
  canvas: HTMLCanvasElement,
  points: ProjectedMapPoint[],
  bounds: MapBounds,
  viewport: MapViewport,
): void {
  const context = canvas.getContext("2d");
  if (!context) {
    return;
  }

  const width = Math.max(1, Math.round(canvas.clientWidth));
  const height = Math.max(1, Math.round(canvas.clientHeight));
  const devicePixelRatio = window.devicePixelRatio || 1;

  canvas.width = Math.max(1, Math.round(width * devicePixelRatio));
  canvas.height = Math.max(1, Math.round(height * devicePixelRatio));
  context.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  context.clearRect(0, 0, width, height);

  if (points.length === 0) {
    return;
  }

  const HALO_R = 4.5;
  const CORE_R = 1.45;

  for (const point of points) {
    const screen = worldToScreen(point, bounds, viewport);
    if (!Number.isFinite(screen.x) || !Number.isFinite(screen.y)) {
      continue;
    }

    const x = screen.x;
    const y = screen.y;
    if (x < -HALO_R - 2 || y < -HALO_R - 2 || x > width + HALO_R + 2 || y > height + HALO_R + 2) {
      continue;
    }

    // Map obstacle height to color while keeping the overlay subordinate to the map and route.
    const zNorm = Math.min(1, Math.max(0, (point.z - DEPTH_Z_LOW) / (DEPTH_Z_HIGH - DEPTH_Z_LOW)));
    const r = 245;
    const g = Math.round(166 - zNorm * 92);
    const b = Math.round(54 - zNorm * 28);
    const coreAlpha = 0.42 + zNorm * 0.18;
    const haloAlpha = 0.08 + zNorm * 0.05;

    const gradient = context.createRadialGradient(x, y, 0, x, y, HALO_R);
    gradient.addColorStop(0, `rgba(${r},${g},${b},${haloAlpha.toFixed(3)})`);
    gradient.addColorStop(1, `rgba(${r},${g},${b},0)`);
    context.fillStyle = gradient;
    context.beginPath();
    context.arc(x, y, HALO_R, 0, Math.PI * 2);
    context.fill();

    context.fillStyle = `rgba(${r},${g},${b},${coreAlpha.toFixed(3)})`;
    context.beginPath();
    context.arc(x, y, CORE_R, 0, Math.PI * 2);
    context.fill();
  }
}

function ZoomInIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 20 20">
      <circle cx="9" cy="9" r="5" />
      <path d="m13 13 4 4" />
      <path d="M9 6.5v5" />
      <path d="M6.5 9h5" />
    </svg>
  );
}

function ZoomOutIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 20 20">
      <circle cx="9" cy="9" r="5" />
      <path d="m13 13 4 4" />
      <path d="M6.5 9h5" />
    </svg>
  );
}

function CenterIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 20 20">
      <circle cx="10" cy="10" r="5.5" />
      <circle cx="10" cy="10" r="1.2" />
      <path d="M10 2.5v2.5" />
      <path d="M10 15v2.5" />
      <path d="M2.5 10H5" />
      <path d="M15 10h2.5" />
    </svg>
  );
}

function LayersIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 20 20">
      <path d="m10 4.2 6.5 3.3L10 10.8 3.5 7.5z" />
      <path d="m5 10.2 5 2.6 5-2.6" />
      <path d="m6.8 13.4 3.2 1.7 3.2-1.7" />
    </svg>
  );
}

export function MapCanvas(props: MapCanvasProps) {
  const mapCanvasRef = useRef<HTMLCanvasElement | null>(null);
  const globalCostmapCanvasRef = useRef<HTMLCanvasElement | null>(null);
  const localCostmapCanvasRef = useRef<HTMLCanvasElement | null>(null);
  const lidarCanvasRef = useRef<HTMLCanvasElement | null>(null);
  const depthCanvasRef = useRef<HTMLCanvasElement | null>(null);
  const stageRef = useRef<HTMLDivElement | null>(null);
  const layerButtonRef = useRef<HTMLButtonElement | null>(null);
  const layerPopoverRef = useRef<HTMLDivElement | null>(null);
  const dragStateRef = useRef<PointerDragState | null>(null);
  const hasFitInitialMapRef = useRef(false);
  const previousMappingStateRef = useRef<MappingSessionState>(props.mappingState);
  const [stageSize, setStageSize] = useState<CanvasSize>({ width: 1, height: 1 });
  const [viewport, setViewport] = useState<MapViewport>({
    viewMode: "fit",
    scale: 1,
    offsetX: 0,
    offsetY: 0,
  });
  const [mapMessage, setMapMessage] = useState<Record<string, unknown> | null>(null);
  const [mapLastUpdateAt, setMapLastUpdateAt] = useState<number | null>(null);
  const [globalCostmapMessage, setGlobalCostmapMessage] = useState<Record<string, unknown> | null>(null);
  const [localCostmapMessage, setLocalCostmapMessage] = useState<Record<string, unknown> | null>(null);
  const [visibleLayers, setVisibleLayers] = useState<OverlayVisibility>(() => ({
    ...DEFAULT_OVERLAY_VISIBILITY,
  }));
  const [isLayerPickerOpen, setIsLayerPickerOpen] = useState(false);

  useEffect(() => {
    const unsubscribes = RASTER_LAYER_CONFIGS.map((layer) =>
      ros2Bridge.subscribe({ topic: layer.topic, type: layer.type }, (message) => {
        const normalizedMessage = normalizeRosMessage(message);
        if (layer.key === "map") {
          setMapMessage(normalizedMessage);
          setMapLastUpdateAt(Date.now());
        } else if (layer.key === "globalCostmap") {
          setGlobalCostmapMessage(normalizedMessage);
        } else {
          setLocalCostmapMessage(normalizedMessage);
        }
      }),
    );
    return () => {
      unsubscribes.forEach((unsubscribe) => unsubscribe());
    };
  }, []);

  const occupancyMap = useMemo(() => parseOccupancyMap(mapMessage), [mapMessage]);
  const globalCostmap = useMemo(() => parseOccupancyMap(globalCostmapMessage), [globalCostmapMessage]);
  const localCostmap = useMemo(() => parseOccupancyMap(localCostmapMessage), [localCostmapMessage]);
  const routePoints = useMemo<MapPoint[]>(
    () =>
      (props.planPoints ?? []).filter(
        (point): point is MapPoint => Number.isFinite(point.x) && Number.isFinite(point.y),
      ),
    [props.planPoints],
  );
  const primaryRasterMap = useMemo(
    () => choosePrimaryRasterMap(occupancyMap, globalCostmap, localCostmap),
    [occupancyMap, globalCostmap, localCostmap],
  );
  const projectedSensorOverlays = useProjectedSensorOverlays(props.currentPose);
  const displayedRobotPose = props.currentPose ?? projectedSensorOverlays.robotPose;
  const bounds = useMemo(
    () => deriveBounds(primaryRasterMap, displayedRobotPose),
    [primaryRasterMap, displayedRobotPose],
  );
  const mapSignature = useMemo(() => getMapSignature(occupancyMap), [occupancyMap]);
  const fitScale = useMemo(() => getFitScale(bounds, stageSize), [bounds, stageSize]);
  const zoomRatio = fitScale > 0 ? viewport.scale / fitScale : 1;
  const mapLayerStyle = useMemo(() => getMapLayerStyle(bounds, viewport), [bounds, viewport]);
  const mapMetadata = useMemo(
    () =>
      buildMapMetadata({
        map: occupancyMap,
        lastUpdateAt: mapLastUpdateAt,
        poseAvailable: props.currentPose != null,
        mappingState: props.mappingState,
      }),
    [occupancyMap, mapLastUpdateAt, props.currentPose, props.mappingState],
  );

  useEffect(() => {
    props.onMapMetadataChange?.(mapMetadata);
  }, [mapMetadata, props.onMapMetadataChange]);

  useEffect(() => {
    if (!stageRef.current) {
      return undefined;
    }
    const element = stageRef.current;
    const updateSize = () => {
      const rect = element.getBoundingClientRect();
      setStageSize({
        width: Math.max(1, Math.round(rect.width)),
        height: Math.max(1, Math.round(rect.height)),
      });
    };
    updateSize();
    const observer = new ResizeObserver(updateSize);
    observer.observe(element);
    return () => observer.disconnect();
  }, []);

  useEffect(() => {
    setViewport((current) => {
      if (current.viewMode !== "fit" && hasFitInitialMapRef.current) {
        return current;
      }
      if (!bounds.hasLiveMap && hasFitInitialMapRef.current) {
        return current;
      }
      if (bounds.hasLiveMap) {
        hasFitInitialMapRef.current = true;
      }
      return makeFitViewport(bounds, stageSize, "fit");
    });
  }, [bounds, mapSignature, stageSize]);

  useEffect(() => {
    if (viewport.viewMode !== "follow_robot" || !displayedRobotPose) {
      return;
    }
    setViewport((current) =>
      centerViewportOnPoint(displayedRobotPose, bounds, stageSize, current.scale, "follow_robot"),
    );
  }, [bounds, displayedRobotPose, stageSize, viewport.viewMode]);

  useEffect(() => {
    const previous = previousMappingStateRef.current;
    previousMappingStateRef.current = props.mappingState;
    if (previous === props.mappingState) {
      return;
    }

    if (props.mappingState === "mapping") {
      setViewport((current) =>
        displayedRobotPose
          ? centerViewportOnPoint(displayedRobotPose, bounds, stageSize, current.scale, "follow_robot")
          : makeFitViewport(bounds, stageSize, "fit"),
      );
      return;
    }

    if (props.mappingState === "review") {
      setViewport(makeFitViewport(bounds, stageSize, "fit"));
    }
  }, [bounds, displayedRobotPose, props.mappingState, stageSize]);

  useEffect(() => {
    if (!mapCanvasRef.current) {
      return;
    }
    drawOccupancyMap(mapCanvasRef.current, occupancyMap, "map");
  }, [occupancyMap]);

  useEffect(() => {
    if (!globalCostmapCanvasRef.current) {
      return;
    }
    drawOccupancyMap(globalCostmapCanvasRef.current, globalCostmap, "global-costmap");
  }, [globalCostmap]);

  useEffect(() => {
    if (!localCostmapCanvasRef.current) {
      return;
    }
    drawOccupancyMap(localCostmapCanvasRef.current, localCostmap, "local-costmap");
  }, [localCostmap]);

  useEffect(() => {
    if (!lidarCanvasRef.current) {
      return;
    }
    drawPointOverlay(
      lidarCanvasRef.current,
      visibleLayers.lidar && projectedSensorOverlays.overlayStatus.lidar === "live"
        ? projectedSensorOverlays.lidarPoints
        : [],
      bounds,
      viewport,
      { fill: "rgba(96, 224, 255, 0.92)", haloFill: "rgba(88, 217, 255, 0.18)", radius: 1.5 },
    );
  }, [
    bounds,
    projectedSensorOverlays.lidarPoints,
    projectedSensorOverlays.overlayStatus.lidar,
    visibleLayers.lidar,
    viewport,
  ]);

  useEffect(() => {
    if (!depthCanvasRef.current) {
      return;
    }
    drawDepthPointOverlay(
      depthCanvasRef.current,
      visibleLayers.depthObstacles && projectedSensorOverlays.overlayStatus.depthObstacles === "live"
        ? projectedSensorOverlays.depthObstaclePoints
        : [],
      bounds,
      viewport,
    );
  }, [
    bounds,
    projectedSensorOverlays.depthObstaclePoints,
    projectedSensorOverlays.overlayStatus.depthObstacles,
    visibleLayers.depthObstacles,
    viewport,
  ]);

  useEffect(() => {
    if (!isLayerPickerOpen) {
      return;
    }

    const handlePointerDown = (event: PointerEvent) => {
      const target = event.target;
      if (!(target instanceof Node)) {
        return;
      }
      if (layerButtonRef.current?.contains(target) || layerPopoverRef.current?.contains(target)) {
        return;
      }
      setIsLayerPickerOpen(false);
    };

    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        setIsLayerPickerOpen(false);
      }
    };

    window.addEventListener("pointerdown", handlePointerDown);
    window.addEventListener("keydown", handleKeyDown);
    return () => {
      window.removeEventListener("pointerdown", handlePointerDown);
      window.removeEventListener("keydown", handleKeyDown);
    };
  }, [isLayerPickerOpen]);

  const hasTarget = props.draftTarget != null;
  const displayedTarget = props.sentTarget ?? props.draftTarget;
  const targetSelectionDisabled = props.disableTargetSelection || props.mappingState === "mapping" || props.mappingState === "paused" || props.mappingState === "review";
  const previewLine =
    props.routeVisualState === "staged" && displayedRobotPose && props.draftTarget
      ? [
          worldToScreen(displayedRobotPose, bounds, viewport),
          worldToScreen(props.draftTarget, bounds, viewport),
        ]
      : null;
  const routePointString = useMemo(
    () =>
      routePoints
        .map((point) => {
          const position = worldToScreen(point, bounds, viewport);
          return `${position.x},${position.y}`;
        })
        .join(" "),
    [bounds, routePoints, viewport],
  );
  const mapPrompt = getMapPrompt(props.routeVisualState, hasTarget, props.mappingState);
  const activeTargetLabel = getTargetLabel(props.routeVisualState, props.targetDistance);
  const overlayStatus: OverlayAvailability = {
    map: occupancyMap ? "live" : "waiting",
    globalCostmap: globalCostmap ? "live" : "waiting",
    localCostmap: localCostmap ? "live" : "waiting",
    plan: routePoints.length > 1 ? "live" : "waiting",
    lidar: projectedSensorOverlays.overlayStatus.lidar,
    depthObstacles: projectedSensorOverlays.overlayStatus.depthObstacles,
  };
  const globalCostmapStyle = useMemo(
    () => getRasterLayerStyle(globalCostmap, bounds, viewport, "global-costmap", projectedSensorOverlays.projectPointToMap),
    [bounds, globalCostmap, projectedSensorOverlays.projectPointToMap, viewport],
  );
  const localCostmapStyle = useMemo(
    () => getRasterLayerStyle(localCostmap, bounds, viewport, "local-costmap", projectedSensorOverlays.projectPointToMap),
    [bounds, localCostmap, projectedSensorOverlays.projectPointToMap, viewport],
  );
  const viewLabel = (() => {
    if (!occupancyMap) {
      return "Waiting for map";
    }
    if (mapMetadata.readiness === "Map mostly unknown") {
      return "Map mostly unexplored";
    }
    if (viewport.viewMode === "follow_robot") {
      return `Following robot · ${Math.round(zoomRatio * 100)}%`;
    }
    if (viewport.viewMode === "manual") {
      return `Manual view · ${Math.round(zoomRatio * 100)}%`;
    }
    return "Full known map";
  })();
  const mappingBanner = (() => {
    if (props.mappingState === "mapping") {
      return `Mapping in progress · ${Math.round(mapMetadata.knownRatio * 100)}% known`;
    }
    if (props.mappingState === "review") {
      return `Review map · ${Math.round(mapMetadata.knownRatio * 100)}% known`;
    }
    if (props.mappingState === "saved") {
      return "Current map · ready for navigation";
    }
    if (props.mappingState === "paused") {
      return "Mapping paused";
    }
    return null;
  })();

  function toggleLayer(layerKey: MapOverlayKey): void {
    setVisibleLayers((current) => ({
      ...current,
      [layerKey]: !current[layerKey],
    }));
  }

  function zoomBy(factor: number): void {
    setViewport((current) => {
      const minScale = fitScale * MIN_ZOOM_RATIO;
      const maxScale = fitScale * MAX_ZOOM_RATIO;
      const nextScale = clamp(current.scale * factor, minScale, maxScale);
      const anchor = { x: stageSize.width / 2, y: stageSize.height / 2 };
      const anchorWorld = screenToWorld(anchor, bounds, current);
      return {
        viewMode: "manual",
        scale: nextScale,
        offsetX: anchor.x - (anchorWorld.x - bounds.minX) * nextScale,
        offsetY: anchor.y - (bounds.maxY - anchorWorld.y) * nextScale,
      };
    });
  }

  function fitMap(): void {
    setViewport(makeFitViewport(bounds, stageSize, "fit"));
  }

  function followRobot(): void {
    if (!displayedRobotPose) {
      return;
    }
    setViewport((current) =>
      centerViewportOnPoint(displayedRobotPose, bounds, stageSize, current.scale, "follow_robot"),
    );
  }

  function stageTargetFromPointer(clientX: number, clientY: number): void {
    if (!stageRef.current) {
      return;
    }
    const rect = stageRef.current.getBoundingClientRect();
    const worldPoint = screenToWorld(
      {
        x: clientX - rect.left,
        y: clientY - rect.top,
      },
      bounds,
      viewport,
    );
    props.onTargetStart({
      x: worldPoint.x,
      y: worldPoint.y,
      yaw: displayedRobotPose?.yaw ?? 0,
    });
  }

  function onPointerDown(event: React.PointerEvent<HTMLDivElement>): void {
    if (props.isGoalActive) {
      return;
    }
    dragStateRef.current = {
      pointerId: event.pointerId,
      startX: event.clientX,
      startY: event.clientY,
      lastX: event.clientX,
      lastY: event.clientY,
      isPanning: false,
    };
    event.currentTarget.setPointerCapture(event.pointerId);
  }

  function onPointerMove(event: React.PointerEvent<HTMLDivElement>): void {
    const dragState = dragStateRef.current;
    if (!dragState || dragState.pointerId !== event.pointerId || props.isGoalActive) {
      return;
    }
    const totalDistance = Math.hypot(event.clientX - dragState.startX, event.clientY - dragState.startY);
    if (!dragState.isPanning && totalDistance < POINTER_PAN_THRESHOLD_PX) {
      return;
    }

    const deltaX = event.clientX - dragState.lastX;
    const deltaY = event.clientY - dragState.lastY;
    dragState.isPanning = true;
    dragState.lastX = event.clientX;
    dragState.lastY = event.clientY;
    setViewport((current) => ({
      ...current,
      viewMode: "manual",
      offsetX: current.offsetX + deltaX,
      offsetY: current.offsetY + deltaY,
    }));
  }

  function onPointerEnd(event: React.PointerEvent<HTMLDivElement>): void {
    const dragState = dragStateRef.current;
    if (!dragState || dragState.pointerId !== event.pointerId) {
      return;
    }
    dragStateRef.current = null;
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
    if (!dragState.isPanning && !props.isGoalActive && !targetSelectionDisabled) {
      stageTargetFromPointer(event.clientX, event.clientY);
    }
  }

  return (
    <div className="vacuum-map-card">
      <div
        ref={stageRef}
        className={`vacuum-map-stage vacuum-map-stage--${props.routeVisualState} ${!props.isGoalActive ? "vacuum-map-stage--interactive" : ""} ${targetSelectionDisabled ? "vacuum-map-stage--mapping" : ""}`}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerEnd}
        onPointerCancel={onPointerEnd}
      >
        <div
          className="vacuum-map-controls vacuum-map-controls--float"
          role="group"
          aria-label="Map controls"
          onPointerDown={(event) => {
            event.stopPropagation();
          }}
          onClick={(event) => {
            event.stopPropagation();
          }}
        >
          <button
            type="button"
            className="vacuum-map-controls__button"
            onClick={() => zoomBy(1.2)}
            disabled={zoomRatio >= MAX_ZOOM_RATIO}
            title="Zoom in"
            aria-label="Zoom in"
          >
            <ZoomInIcon className="vacuum-map-controls__icon" />
          </button>
          <button
            type="button"
            className="vacuum-map-controls__button"
            onClick={() => zoomBy(1 / 1.2)}
            disabled={zoomRatio <= MIN_ZOOM_RATIO}
            title="Zoom out"
            aria-label="Zoom out"
          >
            <ZoomOutIcon className="vacuum-map-controls__icon" />
          </button>
          <span className="vacuum-map-controls__readout" aria-live="polite">
            {Math.round(zoomRatio * 100)}%
          </span>
          <button
            type="button"
            className="vacuum-map-controls__button"
            onClick={fitMap}
            disabled={viewport.viewMode === "fit" && Math.abs(zoomRatio - 1) < 0.01}
            title="Fit Map"
            aria-label="Fit Map"
          >
            <CenterIcon className="vacuum-map-controls__icon" />
          </button>
          <button
            type="button"
            className={`vacuum-map-controls__button ${viewport.viewMode === "follow_robot" ? "vacuum-map-controls__button--active" : ""}`}
            onClick={followRobot}
            disabled={!displayedRobotPose}
            title="Follow Robot"
            aria-label="Follow Robot"
            aria-pressed={viewport.viewMode === "follow_robot"}
          >
            <span className="vacuum-map-controls__follow-dot" />
          </button>
        </div>

        <div
          className="vacuum-map-stage__topline"
          onPointerDown={(event) => {
            event.stopPropagation();
          }}
        >
          <div className="vacuum-map-stage__labels">
            <span className="vacuum-map-stage__prompt-text">{mapPrompt}</span>
            <span className="vacuum-map-stage__view-label">{viewLabel}</span>
          </div>
          <div className="vacuum-map-layer-picker">
            <button
              ref={layerButtonRef}
              type="button"
              className={`vacuum-map-layer-picker__trigger ${isLayerPickerOpen ? "vacuum-map-layer-picker__trigger--open" : ""}`}
              aria-haspopup="dialog"
              aria-expanded={isLayerPickerOpen}
              aria-label="Map layers"
              onClick={() => setIsLayerPickerOpen((current) => !current)}
              onPointerDown={(event) => {
                event.stopPropagation();
              }}
            >
              <LayersIcon className="vacuum-map-layer-picker__trigger-icon" />
              <span>Layers</span>
            </button>

            {isLayerPickerOpen ? (
              <div
                ref={layerPopoverRef}
                className="vacuum-map-layer-picker__popover"
                role="dialog"
                aria-label="Map layers"
                onPointerDown={(event) => {
                  event.stopPropagation();
                }}
              >
                <div className="vacuum-map-layer-picker__popover-head">
                  <strong>Map layers</strong>
                  <span>Toggle live overlays</span>
                </div>
                <div className="vacuum-map-layer-picker__list" role="group" aria-label="Layer options">
                  {MAP_OVERLAY_DEFINITIONS.map((layer) => {
                    const isVisible = visibleLayers[layer.key];
                    const status = overlayStatus[layer.key];
                    const isDisabled = !isVisible && getOverlayDisabled(status);
                    return (
                      <button
                        key={layer.key}
                        type="button"
                        className={`vacuum-map-layer-picker__item ${isVisible ? "vacuum-map-layer-picker__item--active" : ""} vacuum-map-layer-picker__item--${status}`}
                        aria-pressed={isVisible}
                        disabled={isDisabled}
                        onClick={() => toggleLayer(layer.key)}
                      >
                        <span className="vacuum-map-layer-picker__item-main">
                          <span
                            className={`vacuum-map-layer-picker__swatch ${layer.swatchClassName}`}
                          />
                          <span className="vacuum-map-layer-picker__checkbox" aria-hidden="true">
                            <span className="vacuum-map-layer-picker__checkbox-mark" />
                          </span>
                          <span className="vacuum-map-layer-picker__item-copy">
                            <span className="vacuum-map-layer-picker__item-label">{layer.label}</span>
                            <span className="vacuum-map-layer-picker__item-status">
                              {getOverlayStatusLabel(
                                status,
                                layer.key === "lidar"
                                  ? projectedSensorOverlays.lidarTopic?.topic
                                  : layer.key === "depthObstacles"
                                    ? projectedSensorOverlays.depthTopic?.topic
                                    : null,
                              )}
                            </span>
                          </span>
                        </span>
                      </button>
                    );
                  })}
                </div>
              </div>
            ) : null}
          </div>
        </div>

        {mappingBanner ? (
          <div
            className={`vacuum-map-stage__mapping-banner vacuum-map-stage__mapping-banner--${props.mappingState}`}
            onPointerDown={(event) => {
              event.stopPropagation();
            }}
          >
            {mappingBanner}
          </div>
        ) : null}

        <div className="vacuum-map-stage__viewport">
          <canvas
            ref={mapCanvasRef}
            className={`vacuum-map-stage__canvas vacuum-map-stage__canvas--map ${visibleLayers.map ? "vacuum-map-stage__canvas--visible" : "vacuum-map-stage__canvas--hidden"}`}
            style={mapLayerStyle}
          />
          <canvas
            ref={globalCostmapCanvasRef}
            className={`vacuum-map-stage__canvas vacuum-map-stage__canvas--global-costmap ${visibleLayers.globalCostmap ? "vacuum-map-stage__canvas--visible" : "vacuum-map-stage__canvas--hidden"}`}
            style={globalCostmapStyle}
          />
          <canvas
            ref={localCostmapCanvasRef}
            className={`vacuum-map-stage__canvas vacuum-map-stage__canvas--local-costmap ${visibleLayers.localCostmap ? "vacuum-map-stage__canvas--visible" : "vacuum-map-stage__canvas--hidden"}`}
            style={localCostmapStyle}
          />
          <div className="vacuum-map-stage__grid" style={mapLayerStyle} />
          <div className="vacuum-map-stage__map-boundary" style={mapLayerStyle} />
          <svg
            className="vacuum-map-stage__overlay vacuum-map-stage__overlay--plan"
            viewBox={`0 0 ${stageSize.width} ${stageSize.height}`}
            preserveAspectRatio="none"
          >
            <defs>
              <linearGradient id="vacuum-route-gradient" x1="0%" y1="0%" x2="100%" y2="100%">
                <stop offset="0%" stopColor="#63b7df" />
                <stop offset="100%" stopColor="#397fc4" />
              </linearGradient>
              <linearGradient id="vacuum-preview-gradient" x1="0%" y1="0%" x2="100%" y2="100%">
                <stop offset="0%" stopColor="#62d0c6" />
                <stop offset="100%" stopColor="#4b9ed2" />
              </linearGradient>
            </defs>

            {visibleLayers.plan && routePoints.length > 1 ? (
              <>
                <polyline className="vacuum-map-path-casing" points={routePointString} vectorEffect="non-scaling-stroke" />
                <polyline
                  className={`vacuum-map-path vacuum-map-path--${props.routeVisualState}`}
                  points={routePointString}
                  vectorEffect="non-scaling-stroke"
                />
              </>
            ) : null}
            {visibleLayers.plan && previewLine ? (
              <>
                <line
                  className="vacuum-map-preview-line-casing"
                  x1={previewLine[0].x}
                  y1={previewLine[0].y}
                  x2={previewLine[1].x}
                  y2={previewLine[1].y}
                  vectorEffect="non-scaling-stroke"
                />
                <line
                  className={`vacuum-map-preview-line vacuum-map-preview-line--${props.routeVisualState}`}
                  x1={previewLine[0].x}
                  y1={previewLine[0].y}
                  x2={previewLine[1].x}
                  y2={previewLine[1].y}
                  vectorEffect="non-scaling-stroke"
                />
              </>
            ) : null}
          </svg>
          <canvas
            ref={lidarCanvasRef}
            className="vacuum-map-stage__overlay-canvas vacuum-map-stage__overlay-canvas--lidar"
          />
          <canvas
            ref={depthCanvasRef}
            className="vacuum-map-stage__overlay-canvas vacuum-map-stage__overlay-canvas--depth"
          />

          {displayedRobotPose ? (() => {
            const position = worldToScreen(displayedRobotPose, bounds, viewport);
            return (
              <div
                className="vacuum-marker vacuum-marker--robot"
                style={{ left: `${position.x}px`, top: `${position.y}px` }}
              >
                <span className="vacuum-marker__robot-halo" />
                <span
                  className="vacuum-marker__robot-body"
                  style={{ transform: `translate(-50%, -50%) rotate(${displayedRobotPose.yaw ?? 0}deg)` }}
                >
                  <svg viewBox="0 0 32 32" aria-hidden="true">
                    <circle cx="16" cy="16" r="11" className="vacuum-marker__robot-disc" />
                    <path d="M16 4 L22 14 L16 11 L10 14 Z" className="vacuum-marker__robot-arrow-shape" />
                    <circle cx="16" cy="16" r="2.2" className="vacuum-marker__robot-dot" />
                  </svg>
                </span>
                <span className="vacuum-marker__label vacuum-marker__label--robot">Robot</span>
              </div>
            );
          })() : null}

          {displayedTarget ? (() => {
            const position = worldToScreen(displayedTarget, bounds, viewport);
            return (
              <div
                className={`vacuum-marker vacuum-marker--target vacuum-marker--target-${props.routeVisualState}`}
                style={{ left: `${position.x}px`, top: `${position.y}px` }}
              >
                <span className="vacuum-marker__target-ring" />
                <span className="vacuum-marker__target-pin" />
                <span className="vacuum-marker__target-dot" />
                <span className="vacuum-marker__label">{activeTargetLabel}</span>
              </div>
            );
          })() : null}
        </div>

        {!props.draftTarget && !props.isGoalActive && !targetSelectionDisabled ? (
          <div className="vacuum-map-stage__center-prompt">
            <strong>Choose destination</strong>
            <span>{bounds.hasLiveMap ? "Click the map to stage a run" : "Live map unavailable, placeholder ready"}</span>
          </div>
        ) : null}

        <div className="vacuum-map-north" aria-label="North indicator" title="North">
          <svg
            className="vacuum-map-north__svg"
            viewBox="0 0 20 20"
            aria-hidden="true"
            style={occupancyMap && occupancyMap.originYaw !== 0 ? { transform: `rotate(${-occupancyMap.originYaw}deg)` } : undefined}
          >
            <path
              d="M10 3 L10 17"
              stroke="rgba(240, 237, 230, 0.3)"
              strokeWidth="1.4"
              strokeLinecap="round"
            />
            <path
              d="M10 3 L7 9"
              stroke="rgba(115, 182, 242, 0.9)"
              strokeWidth="1.6"
              strokeLinecap="round"
            />
            <path
              d="M10 3 L13 9"
              stroke="rgba(115, 182, 242, 0.9)"
              strokeWidth="1.6"
              strokeLinecap="round"
            />
            <text
              x="10"
              y="19"
              textAnchor="middle"
              fontSize="6"
              fontWeight="800"
              fill="rgba(115, 182, 242, 0.82)"
              letterSpacing="0.04em"
            >
              N
            </text>
          </svg>
        </div>

        <div
          className="vacuum-map-stage__legend"
          onPointerDown={(event) => {
            event.stopPropagation();
          }}
        >
          <span>
            <i className="vacuum-legend vacuum-legend--robot" />
            Robot
          </span>
          <span>
            <i
              className={`vacuum-legend vacuum-legend--${props.routeVisualState === "idle" ? "staged" : props.routeVisualState}`}
            />
            Route
          </span>
          <span>
            <i className="vacuum-legend vacuum-legend--target" />
            Destination
          </span>
        </div>

        <CameraOverlay />
      </div>
    </div>
  );
}
