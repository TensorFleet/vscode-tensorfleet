import React, { useEffect, useMemo, useRef, useState } from "react";
import { ros2Bridge } from "../../ros2-bridge";
import {
  extractStampedPose,
  getRecordEntry,
  normalizeRosMessage,
} from "../Nav2/runtime/nav2RuntimeUtils";
import type { PoseCoordinates } from "../Nav2/runtime/nav2RuntimeTypes";

export type MapCanvasTarget = {
  x: number;
  y: number;
  yaw: number;
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

export type RouteVisualState = "idle" | "staged" | "active" | "completed" | "failed" | "canceled";

export type MapCanvasProps = {
  currentPose: PoseCoordinates | null;
  planMessage: Record<string, unknown> | null;
  draftTarget: MapCanvasTarget | null;
  sentTarget: MapCanvasTarget | null;
  routeVisualState: RouteVisualState;
  isGoalActive: boolean;
  onTargetStart: (target: MapCanvasTarget) => void;
  onTargetRotate: (yaw: number) => void;
};

const MAP_FREE_COLOR = { r: 232, g: 230, b: 224 };
const MAP_OCCUPIED_COLOR = { r: 58, g: 58, b: 58 };
const MAP_UNKNOWN_COLOR = { r: 200, g: 196, b: 188 };

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
  if (typeof ArrayBuffer !== "undefined" && ArrayBuffer.isView(value)) {
    return Array.from(value as ArrayLike<number>, (entry) => Number(entry));
  }
  return null;
}

function mixChannel(start: number, end: number, amount: number): number {
  return Math.round(start + (end - start) * amount);
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
  const dataValue = getRecordEntry(message, "data");
  const data = toNumericArray(dataValue);

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

function drawOccupancyMap(canvas: HTMLCanvasElement, map: OccupancyMap | null): void {
  if (!map) {
    drawPlaceholderMap(canvas);
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

      if (cell < 0) {
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
      image.data[pixelIndex + 3] = 255;
    }
  }

  context.putImageData(image, 0, 0);
}

function extractPathPoints(message: Record<string, unknown> | null): MapPoint[] {
  const poses = getRecordEntry(message ?? {}, "poses");
  if (!Array.isArray(poses)) {
    return [];
  }

  return poses
    .map((poseEntry) => {
      if (!poseEntry || typeof poseEntry !== "object") {
        return null;
      }
      const pose = extractStampedPose(poseEntry as Record<string, unknown>);
      if (!pose) {
        return null;
      }
      const position = getRecordEntry(pose, "position");
      if (!position || typeof position !== "object") {
        return null;
      }
      const x = toFiniteNumber(getRecordEntry(position as Record<string, unknown>, "x"));
      const y = toFiniteNumber(getRecordEntry(position as Record<string, unknown>, "y"));
      return x == null || y == null ? null : { x, y };
    })
    .filter((point): point is MapPoint => point != null);
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

function worldToPercent(point: MapPoint, bounds: MapBounds): { left: number; top: number } {
  return {
    left: clamp(((point.x - bounds.minX) / bounds.width) * 100, -30, 130),
    top: clamp((1 - (point.y - bounds.minY) / bounds.height) * 100, -30, 130),
  };
}

function percentToWorld(leftPercent: number, topPercent: number, bounds: MapBounds): MapPoint {
  return {
    x: bounds.minX + bounds.width * leftPercent,
    y: bounds.minY + bounds.height * (1 - topPercent),
  };
}

function getMapPrompt(routeVisualState: RouteVisualState, hasTarget: boolean): string {
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
  return hasTarget ? "Drag to refine destination" : "Click to choose destination";
}

function getTargetLabel(routeVisualState: RouteVisualState): string {
  if (routeVisualState === "completed") {
    return "Done";
  }
  if (routeVisualState === "active") {
    return "Active";
  }
  if (routeVisualState === "failed") {
    return "Retry";
  }
  if (routeVisualState === "canceled") {
    return "Canceled";
  }
  return "Selected";
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

export function MapCanvas(props: MapCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const stageRef = useRef<HTMLDivElement | null>(null);
  const dragPointerIdRef = useRef<number | null>(null);
  const [zoom, setZoom] = useState(1);
  const [mapMessage, setMapMessage] = useState<Record<string, unknown> | null>(null);

  useEffect(() => {
    const unsubscribe = ros2Bridge.subscribe(
      { topic: "/map", type: "nav_msgs/msg/OccupancyGrid" },
      (message) => setMapMessage(normalizeRosMessage(message)),
    );
    return unsubscribe;
  }, []);

  const occupancyMap = useMemo(() => parseOccupancyMap(mapMessage), [mapMessage]);
  const routePoints = useMemo(() => extractPathPoints(props.planMessage), [props.planMessage]);
  const bounds = useMemo(
    () => deriveBounds(occupancyMap, props.currentPose),
    [occupancyMap, props.currentPose],
  );

  useEffect(() => {
    if (!canvasRef.current) {
      return;
    }
    drawOccupancyMap(canvasRef.current, occupancyMap);
  }, [occupancyMap]);

  const hasTarget = props.draftTarget != null;
  const displayedTarget = props.sentTarget ?? props.draftTarget;
  const previewLine =
    props.routeVisualState === "staged" && props.currentPose && props.draftTarget
      ? [
          worldToPercent(props.currentPose, bounds),
          worldToPercent(props.draftTarget, bounds),
        ]
      : null;
  const mapPrompt = getMapPrompt(props.routeVisualState, hasTarget);
  const activeTargetLabel = getTargetLabel(props.routeVisualState);

  function updateTargetFromPointer(clientX: number, clientY: number, mode: "start" | "rotate"): void {
    if (!stageRef.current) {
      return;
    }
    const rect = stageRef.current.getBoundingClientRect();
    const rawLeft = (clientX - rect.left) / rect.width;
    const rawTop = (clientY - rect.top) / rect.height;
    const left = clamp(0.5 + (rawLeft - 0.5) / zoom, 0, 1);
    const top = clamp(0.5 + (rawTop - 0.5) / zoom, 0, 1);
    const worldPoint = percentToWorld(left, top, bounds);

    if (mode === "start") {
      props.onTargetStart({
        x: worldPoint.x,
        y: worldPoint.y,
        yaw: props.currentPose?.yaw ?? 0,
      });
      return;
    }

    if (!props.draftTarget) {
      return;
    }

    const yaw = (Math.atan2(worldPoint.y - props.draftTarget.y, worldPoint.x - props.draftTarget.x) * 180) / Math.PI;
    props.onTargetRotate(yaw);
  }

  function onPointerDown(event: React.PointerEvent<HTMLDivElement>): void {
    if (props.isGoalActive) {
      return;
    }
    dragPointerIdRef.current = event.pointerId;
    event.currentTarget.setPointerCapture(event.pointerId);
    updateTargetFromPointer(event.clientX, event.clientY, "start");
  }

  function onPointerMove(event: React.PointerEvent<HTMLDivElement>): void {
    if (dragPointerIdRef.current !== event.pointerId || props.isGoalActive) {
      return;
    }
    updateTargetFromPointer(event.clientX, event.clientY, "rotate");
  }

  function onPointerEnd(event: React.PointerEvent<HTMLDivElement>): void {
    if (dragPointerIdRef.current !== event.pointerId) {
      return;
    }
    dragPointerIdRef.current = null;
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
  }

  return (
    <div className="vacuum-map-card">
      <div
        ref={stageRef}
        className={`vacuum-map-stage vacuum-map-stage--${props.routeVisualState} ${!props.isGoalActive ? "vacuum-map-stage--interactive" : ""}`}
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
        >
          <button
            type="button"
            className="vacuum-map-controls__button"
            onClick={() => setZoom((current) => clamp(current + 0.15, 0.8, 2.4))}
            title="Zoom in"
            aria-label="Zoom in"
          >
            <ZoomInIcon className="vacuum-map-controls__icon" />
          </button>
          <button
            type="button"
            className="vacuum-map-controls__button"
            onClick={() => setZoom((current) => clamp(current - 0.15, 0.8, 2.4))}
            title="Zoom out"
            aria-label="Zoom out"
          >
            <ZoomOutIcon className="vacuum-map-controls__icon" />
          </button>
          <button
            type="button"
            className="vacuum-map-controls__button"
            onClick={() => setZoom(1)}
            title="Center map"
            aria-label="Center map"
          >
            <CenterIcon className="vacuum-map-controls__icon" />
          </button>
        </div>

        <div
          className="vacuum-map-stage__topline"
          onPointerDown={(event) => {
            event.stopPropagation();
          }}
        >
          <span className="vacuum-map-stage__prompt-text">{mapPrompt}</span>
        </div>

        <div className="vacuum-map-stage__viewport" style={{ transform: `scale(${zoom})` }}>
          <canvas ref={canvasRef} className="vacuum-map-stage__canvas" />
          <div className="vacuum-map-stage__grid" />

          <svg className="vacuum-map-stage__overlay" viewBox="0 0 100 100" preserveAspectRatio="none">
            <defs>
              <linearGradient id="vacuum-route-gradient" x1="0%" y1="0%" x2="100%" y2="100%">
                <stop offset="0%" stopColor="#4ec7ff" />
                <stop offset="100%" stopColor="#2d8df0" />
              </linearGradient>
              <linearGradient id="vacuum-preview-gradient" x1="0%" y1="0%" x2="100%" y2="100%">
                <stop offset="0%" stopColor="#58d9ff" />
                <stop offset="100%" stopColor="#38c8b0" />
              </linearGradient>
            </defs>

            {routePoints.length > 1 ? (
              <polyline
                className={`vacuum-map-path vacuum-map-path--${props.routeVisualState}`}
                points={routePoints
                  .map((point) => {
                    const position = worldToPercent(point, bounds);
                    return `${position.left},${position.top}`;
                  })
                  .join(" ")}
              />
            ) : null}
            {previewLine ? (
              <line
                className={`vacuum-map-preview-line vacuum-map-preview-line--${props.routeVisualState}`}
                x1={previewLine[0].left}
                y1={previewLine[0].top}
                x2={previewLine[1].left}
                y2={previewLine[1].top}
              />
            ) : null}
          </svg>

          {props.currentPose ? (() => {
            const position = worldToPercent(props.currentPose, bounds);
            return (
              <div
                className="vacuum-marker vacuum-marker--robot"
                style={{ left: `${position.left}%`, top: `${position.top}%` }}
              >
                <span className="vacuum-marker__robot-halo" />
                <span
                  className="vacuum-marker__robot-body"
                  style={{ transform: `translate(-50%, -50%) rotate(${props.currentPose.yaw ?? 0}deg)` }}
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
            const position = worldToPercent(displayedTarget, bounds);
            return (
              <div
                className={`vacuum-marker vacuum-marker--target vacuum-marker--target-${props.routeVisualState}`}
                style={{ left: `${position.left}%`, top: `${position.top}%` }}
              >
                <span className="vacuum-marker__target-ring" />
                <span className="vacuum-marker__target-pin" />
                <span className="vacuum-marker__target-dot" />
                <span className="vacuum-marker__label">{activeTargetLabel}</span>
              </div>
            );
          })() : null}
        </div>

        {!props.draftTarget && !props.isGoalActive ? (
          <div className="vacuum-map-stage__center-prompt">
            <strong>Choose destination</strong>
            <span>{bounds.hasLiveMap ? "Click anywhere on the map" : "Live map unavailable, placeholder ready"}</span>
          </div>
        ) : null}

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
      </div>
    </div>
  );
}
