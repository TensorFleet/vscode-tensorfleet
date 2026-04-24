import React, { useEffect, useMemo, useRef, useState } from "react";
import { ros2Bridge } from "../../ros2-bridge";
import { useConnectionSettings } from "../ConnectionSettingsProvider";
import { useNav2Runtime } from "../Nav2/runtime/useNav2Runtime";
import {
  extractStampedPose,
  formatRosDuration,
  getRecordEntry,
  normalizeRosMessage,
} from "../Nav2/runtime/nav2RuntimeUtils";
import type { GoalState, PoseCoordinates, TopicHealth } from "../Nav2/runtime/nav2RuntimeTypes";
import "./VacuumControlPanel.css";

type DraftTarget = {
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

type OperatorTone = "ready" | "warning" | "success" | "danger" | "info";
type StatusChipTone = "success" | "active" | "inactive";

type OperatorState = {
  key:
    | "disconnected"
    | "waiting-map"
    | "waiting-localization"
    | "checking"
    | "ready"
    | "navigating"
    | "completed"
    | "failed"
    | "canceled";
  title: string;
  detail: string;
  badge: string;
  tone: OperatorTone;
};

type RouteVisualState = "idle" | "staged" | "active" | "completed" | "failed" | "canceled";

const ACTIVE_GOAL_STATES = new Set<GoalState>(["sending", "accepted", "executing", "canceling"]);
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

  if (
    width == null ||
    height == null ||
    resolution == null ||
    originX == null ||
    originY == null ||
    !Array.isArray(dataValue)
  ) {
    return null;
  }

  return {
    width,
    height,
    resolution,
    originX,
    originY,
    data: dataValue.map((cell) => Number(cell)),
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
      const occupied = mainCorridor > 1.33 || sideWalls > 1.46 || x < 12 || y < 12 || x > size - 12 || y > size - 12;

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

function formatDistance(distance: number | null): string {
  if (distance == null || !Number.isFinite(distance)) {
    return "n/a";
  }
  return distance < 10 ? `${distance.toFixed(1)} m` : `${distance.toFixed(0)} m`;
}

function formatEta(value: unknown): string {
  const numeric = toFiniteNumber(value);
  return numeric != null ? formatRosDuration(numeric) : formatRosDuration(value);
}

function distanceBetween(a: MapPoint | null, b: MapPoint | null): number | null {
  if (!a || !b) {
    return null;
  }
  return Math.hypot(b.x - a.x, b.y - a.y);
}

function headingLabel(yaw: number): string {
  const headings = ["East", "North-East", "North", "North-West", "West", "South-West", "South", "South-East"];
  const normalized = ((yaw % 360) + 360) % 360;
  return headings[Math.round(normalized / 45) % headings.length]!;
}

function getTopicState(topicHealth: TopicHealth[], topic: string): TopicHealth["status"] | null {
  return topicHealth.find((entry) => entry.topic === topic)?.status ?? null;
}

function getOperatorState(args: {
  connectionStatus: "connected" | "connecting" | "disconnected";
  mapStatus: TopicHealth["status"] | null;
  poseAvailable: boolean;
  preflightReady: boolean;
  goalState: GoalState;
  targetSelected: boolean;
}): OperatorState {
  if (args.connectionStatus !== "connected") {
    return {
      key: "disconnected",
      title: args.connectionStatus === "connecting" ? "Connecting" : "Disconnected",
      detail: "Connection needed.",
      badge: args.connectionStatus === "connecting" ? "Connecting" : "Offline",
      tone: "warning",
    };
  }

  if (args.mapStatus !== "receiving") {
    return {
      key: "waiting-map",
      title: "Waiting for map",
      detail: "Map not ready yet.",
      badge: "Map",
      tone: "warning",
    };
  }

  if (!args.poseAvailable) {
    return {
      key: "waiting-localization",
      title: "Positioning robot",
      detail: "Robot position is still settling.",
      badge: "Locating",
      tone: "warning",
    };
  }

  if (!args.preflightReady) {
    return {
      key: "checking",
      title: "Almost ready",
      detail: "Final checks are still running.",
      badge: "Checking",
      tone: "info",
    };
  }

  if (ACTIVE_GOAL_STATES.has(args.goalState)) {
    return {
      key: "navigating",
      title: args.goalState === "canceling" ? "Stopping" : "On the way",
      detail: args.goalState === "canceling" ? "Stopping this run." : "Moving to the selected destination.",
      badge: args.goalState === "canceling" ? "Stopping" : "Active",
      tone: "info",
    };
  }

  if (args.goalState === "succeeded") {
    return {
      key: "completed",
      title: "Completed",
      detail: "Destination reached.",
      badge: "Done",
      tone: "success",
    };
  }

  if (args.goalState === "canceled") {
    return {
      key: "canceled",
      title: "Canceled",
      detail: "Run stopped.",
      badge: "Stopped",
      tone: "warning",
    };
  }

  if (args.goalState === "aborted" || args.goalState === "rejected" || args.goalState === "unknown") {
    return {
      key: "failed",
      title: "Needs attention",
      detail: "Could not complete the run.",
      badge: "Issue",
      tone: "danger",
    };
  }

  return {
    key: "ready",
    title: args.targetSelected ? "Ready to send" : "Ready",
    detail: args.targetSelected ? "Destination selected." : "Choose a destination on the map.",
    badge: args.targetSelected ? "Selected" : "Ready",
    tone: "ready",
  };
}

function getRouteVisualState(goalState: GoalState, hasTarget: boolean): RouteVisualState {
  if (goalState === "succeeded") {
    return "completed";
  }
  if (goalState === "canceled") {
    return "canceled";
  }
  if (goalState === "aborted" || goalState === "rejected" || goalState === "unknown") {
    return "failed";
  }
  if (ACTIVE_GOAL_STATES.has(goalState)) {
    return "active";
  }
  return hasTarget ? "staged" : "idle";
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

function GearIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
      <path d="M10.3 2.7h3.4l.5 2.1c.4.1.9.3 1.3.5l1.9-1.1 2.4 2.4-1.1 1.9c.2.4.4.9.5 1.3l2.1.5v3.4l-2.1.5c-.1.4-.3.9-.5 1.3l1.1 1.9-2.4 2.4-1.9-1.1c-.4.2-.9.4-1.3.5l-.5 2.1h-3.4l-.5-2.1c-.4-.1-.9-.3-1.3-.5l-1.9 1.1-2.4-2.4 1.1-1.9c-.2-.4-.4-.9-.5-1.3l-2.1-.5v-3.4l2.1-.5c.1-.4.3-.9.5-1.3L4.2 6.6 6.6 4.2l1.9 1.1c.4-.2.9-.4 1.3-.5z" />
      <circle cx="12" cy="12" r="3.3" />
    </svg>
  );
}

function VacuumMark(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 40 40" fill="none">
      <defs>
        <linearGradient id="vacuum-mark-grad" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#58e4cc" />
          <stop offset="100%" stopColor="#2d8df0" />
        </linearGradient>
      </defs>
      <circle cx="20" cy="20" r="15" stroke="url(#vacuum-mark-grad)" strokeWidth="2.2" />
      <circle cx="20" cy="20" r="8" stroke="url(#vacuum-mark-grad)" strokeWidth="1.6" opacity="0.65" />
      <circle cx="20" cy="20" r="2.6" fill="url(#vacuum-mark-grad)" />
    </svg>
  );
}

function SidebarIcon(props: { className?: string; kind: "navigation" | "history" | "settings" }) {
  if (props.kind === "navigation") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
        <path d="M4.5 18.5 9 6.5l10.5-2-4.5 12z" />
        <path d="m9 6.5 5.5 5.5" />
      </svg>
    );
  }

  if (props.kind === "history") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
        <path d="M5 5v5h5" />
        <path d="M7.7 16.3A7 7 0 1 0 5 10" />
        <path d="M12 8v4l2.5 1.5" />
      </svg>
    );
  }

  return <GearIcon className={props.className} />;
}

function StatusChipIcon(props: { className?: string; kind: "connected" | "map" | "localized" | "ready" | "target" }) {
  if (props.kind === "connected") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 20 20">
        <path d="M3.5 7.5A9.5 9.5 0 0 1 10 5a9.5 9.5 0 0 1 6.5 2.5" />
        <path d="M6 10a6 6 0 0 1 8 0" />
        <path d="M8.5 12.5a2.5 2.5 0 0 1 3 0" />
        <circle cx="10" cy="15.5" r="1.2" />
      </svg>
    );
  }

  if (props.kind === "map") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 20 20">
        <path d="M3.5 5.5 8 4l4 1.5L16.5 4v10.5L12 16l-4-1.5L3.5 16z" />
        <path d="M8 4v10.5" />
        <path d="M12 5.5V16" />
      </svg>
    );
  }

  if (props.kind === "localized") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 20 20">
        <circle cx="10" cy="10" r="5.5" />
        <circle cx="10" cy="10" r="1.7" />
        <path d="M10 2.5v2" />
        <path d="M10 15.5v2" />
        <path d="M2.5 10h2" />
        <path d="M15.5 10h2" />
      </svg>
    );
  }

  if (props.kind === "ready") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 20 20">
        <path d="m4.5 10.5 3.2 3.2 7.8-7.8" />
      </svg>
    );
  }

  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 20 20">
      <path d="M10 16.8s4.8-4.7 4.8-8.3A4.8 4.8 0 1 0 5.2 8.5c0 3.6 4.8 8.3 4.8 8.3Z" />
      <circle cx="10" cy="8.4" r="1.6" />
    </svg>
  );
}

function StateIcon(props: { className?: string; stateKey: OperatorState["key"] }) {
  if (props.stateKey === "ready" || props.stateKey === "waiting-map" || props.stateKey === "waiting-localization" || props.stateKey === "checking") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
        <path d="m5 12.5 4.2 4.2L19 7" />
      </svg>
    );
  }

  if (props.stateKey === "navigating") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
        <path d="M5 18 19 6" />
        <path d="M10 6h9v9" />
      </svg>
    );
  }

  if (props.stateKey === "completed") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
        <path d="M6 18V7.5h10l-1.7 3.2L16 14H6" />
        <path d="m8 16 2.4 2.4L18.5 10" />
      </svg>
    );
  }

  if (props.stateKey === "failed") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
        <path d="M12 4 3.5 19h17z" />
        <path d="M12 9v4.5" />
        <circle cx="12" cy="17" r="1" />
      </svg>
    );
  }

  if (props.stateKey === "canceled" || props.stateKey === "disconnected") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
        <circle cx="12" cy="12" r="8" />
        <path d="M9 9l6 6" />
      </svg>
    );
  }

  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
      <circle cx="12" cy="12" r="8" />
      <path d="M12 8v5" />
      <circle cx="12" cy="16.5" r="0.8" />
    </svg>
  );
}

function DirectionIcon(props: { className?: string; direction?: number }) {
  return (
    <svg
      aria-hidden="true"
      className={props.className}
      viewBox="0 0 24 24"
      style={props.direction == null ? undefined : { transform: `rotate(${props.direction}deg)` }}
    >
      <path d="M12 4 17.5 18l-5.5-2.8L6.5 18z" />
    </svg>
  );
}

function ConnectionPillIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 12 12">
      <circle cx="6" cy="6" r="4.5" />
    </svg>
  );
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

function DestinationEmptyIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 32 32">
      <path d="M16 4.5C11 4.5 7 8.3 7 13.1c0 5.3 5.6 11.3 8.2 13.8a1.2 1.2 0 0 0 1.6 0C19.4 24.4 25 18.4 25 13.1 25 8.3 21 4.5 16 4.5Z" />
      <circle cx="16" cy="12.8" r="3.2" />
    </svg>
  );
}

function CompassIcon(props: { className?: string; direction?: number }) {
  return (
    <svg
      aria-hidden="true"
      className={props.className}
      viewBox="0 0 24 24"
      style={props.direction == null ? undefined : { transform: `rotate(${props.direction}deg)` }}
    >
      <circle cx="12" cy="12" r="9" />
      <path d="M12 5.5 16 12l-4 6.5L8 12z" />
    </svg>
  );
}

function SendIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 20 20">
      <path d="M17.5 10 2.5 3l2.5 7-2.5 7 15-7z" />
    </svg>
  );
}

function getChipTone(isActive: boolean, tone: Exclude<StatusChipTone, "inactive">): StatusChipTone {
  return isActive ? tone : "inactive";
}

function getProgressLabel(routeVisualState: RouteVisualState): string {
  if (routeVisualState === "completed") {
    return "Destination reached";
  }
  if (routeVisualState === "failed") {
    return "Run needs attention";
  }
  if (routeVisualState === "canceled") {
    return "Run canceled";
  }
  if (routeVisualState === "active") {
    return "En route to destination";
  }
  return "Awaiting run start";
}


export function VacuumControlPanel() {
  const runtime = useNav2Runtime();
  const { openOverlay } = useConnectionSettings();
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const stageRef = useRef<HTMLDivElement | null>(null);
  const dragPointerIdRef = useRef<number | null>(null);
  const [draftTarget, setDraftTarget] = useState<DraftTarget | null>(null);
  const [sentTarget, setSentTarget] = useState<DraftTarget | null>(null);
  const [initialRouteDistance, setInitialRouteDistance] = useState<number | null>(null);
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
  const routePoints = useMemo(() => extractPathPoints(runtime.planMessage), [runtime.planMessage]);
  const currentPose = runtime.currentMapCoordinates;
  const bounds = useMemo(() => deriveBounds(occupancyMap, currentPose), [occupancyMap, currentPose]);
  const mapStatus = getTopicState(runtime.topicHealth, "/map");
  const isGoalActive = ACTIVE_GOAL_STATES.has(runtime.goalState);
  const hasTarget = draftTarget != null;
  const routeVisualState = getRouteVisualState(runtime.goalState, hasTarget);
  const operatorState = getOperatorState({
    connectionStatus: runtime.connectionStatus,
    mapStatus,
    poseAvailable: currentPose != null,
    preflightReady: runtime.preflightStatus.state === "ready",
    goalState: runtime.goalState,
    targetSelected: hasTarget,
  });

  useEffect(() => {
    if (!canvasRef.current) {
      return;
    }
    drawOccupancyMap(canvasRef.current, occupancyMap);
  }, [occupancyMap]);

  const displayedTarget = sentTarget ?? draftTarget;
  const destinationDistance = distanceBetween(currentPose, displayedTarget);
  const routeDistanceRemaining =
    toFiniteNumber(runtime.feedbackDistanceRemaining) ?? (isGoalActive ? destinationDistance : null);
  const showProgressMetric =
    routeVisualState === "active" ||
    routeVisualState === "completed" ||
    routeVisualState === "failed" ||
    routeVisualState === "canceled";
  const routeProgress = (() => {
    if (runtime.goalState === "succeeded") {
      return 1;
    }
    const remaining = toFiniteNumber(runtime.feedbackDistanceRemaining);
    if (remaining == null || initialRouteDistance == null || initialRouteDistance <= 0) {
      return displayedTarget && isGoalActive ? 0.12 : 0;
    }
    return clamp(1 - remaining / initialRouteDistance, 0, 0.98);
  })();
  const systemChips = [
    {
      label: "Map Live",
      icon: "map" as const,
      state: getChipTone(mapStatus === "receiving", "success"),
    },
    {
      label: "Localized",
      icon: "localized" as const,
      state: getChipTone(Boolean(currentPose), "success"),
    },
  ] as const;
  const taskChips = [
    {
      label: "Ready",
      icon: "ready" as const,
      state: getChipTone(
        runtime.preflightStatus.state === "ready" && Boolean(currentPose) && mapStatus === "receiving",
        "success",
      ),
    },
    {
      label: "Target Selected",
      icon: "target" as const,
      state: getChipTone(hasTarget, "active"),
    },
  ] as const;

  const previewLine =
    routeVisualState === "staged" && currentPose && draftTarget
      ? [worldToPercent(currentPose, bounds), worldToPercent(draftTarget, bounds)]
      : null;
  const activeTargetLabel = getTargetLabel(routeVisualState);
  const mapPrompt = getMapPrompt(routeVisualState, hasTarget);
  const targetDistanceLabel = draftTarget ? formatDistance(distanceBetween(currentPose, draftTarget)) : null;
  const targetHeadingLabel = draftTarget ? headingLabel(draftTarget.yaw) : null;
  const progressLabel = getProgressLabel(routeVisualState);
  const progressBarWidth = (() => {
    if (!showProgressMetric) {
      return 0;
    }
    if (routeVisualState === "completed") {
      return 100;
    }
    return Math.max(routeProgress * 100, 6);
  })();

  async function handleSend(): Promise<void> {
    if (!draftTarget) {
      return;
    }
    setSentTarget(draftTarget);
    setInitialRouteDistance(distanceBetween(currentPose, draftTarget));
    await runtime.sendGoal(draftTarget);
  }

  async function handleCancel(): Promise<void> {
    await runtime.cancelGoal();
  }

  function handleClear(): void {
    if (isGoalActive) {
      return;
    }
    setDraftTarget(null);
    setSentTarget(null);
    setInitialRouteDistance(null);
  }

  function updateTargetFromPointer(clientX: number, clientY: number, mode: "start" | "rotate"): void {
    if (!stageRef.current) {
      return;
    }
    const rect = stageRef.current.getBoundingClientRect();
    const rawLeft = (clientX - rect.left) / rect.width;
    const rawTop = (clientY - rect.top) / rect.height;
    // The viewport is CSS-scaled around the stage center. Invert the transform to
    // convert stage-relative click fractions into canvas-content fractions.
    const left = clamp(0.5 + (rawLeft - 0.5) / zoom, 0, 1);
    const top = clamp(0.5 + (rawTop - 0.5) / zoom, 0, 1);
    const worldPoint = percentToWorld(left, top, bounds);

    if (mode === "start") {
      setSentTarget(null);
      setInitialRouteDistance(null);
      setDraftTarget({
        x: worldPoint.x,
        y: worldPoint.y,
        yaw: currentPose?.yaw ?? 0,
      });
      return;
    }

    setDraftTarget((current) => {
      if (!current) {
        return current;
      }
      const yaw = (Math.atan2(worldPoint.y - current.y, worldPoint.x - current.x) * 180) / Math.PI;
      return { ...current, yaw };
    });
  }

  function onPointerDown(event: React.PointerEvent<HTMLDivElement>): void {
    if (isGoalActive) {
      return;
    }
    dragPointerIdRef.current = event.pointerId;
    event.currentTarget.setPointerCapture(event.pointerId);
    updateTargetFromPointer(event.clientX, event.clientY, "start");
  }

  function onPointerMove(event: React.PointerEvent<HTMLDivElement>): void {
    if (dragPointerIdRef.current !== event.pointerId || isGoalActive) {
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
    <div className="vacuum-shell">
      <aside className="vacuum-rail">
        <div className="vacuum-rail__brand" title="Vacuum Control">
          <VacuumMark className="vacuum-rail__mark" />
        </div>

        <nav className="vacuum-rail__nav" aria-label="Panel navigation">
          <button className="vacuum-rail__item vacuum-rail__item--active" type="button">
            <span className="vacuum-rail__item-bar" />
            <SidebarIcon className="vacuum-rail__icon" kind="navigation" />
            <span>Navigation</span>
          </button>
          <button className="vacuum-rail__item" type="button">
            <span className="vacuum-rail__item-bar" />
            <SidebarIcon className="vacuum-rail__icon" kind="history" />
            <span>History</span>
          </button>
        </nav>

        <button className="vacuum-rail__item vacuum-rail__item--settings" type="button" onClick={openOverlay} title="Settings">
          <span className="vacuum-rail__item-bar" />
          <SidebarIcon className="vacuum-rail__icon" kind="settings" />
          <span>Settings</span>
        </button>
      </aside>

      <main className="vacuum-main">
        <header className="vacuum-header">
          <div className="vacuum-header__left">
            <h1 className="vacuum-header__title">Vacuum Control</h1>
            <span className="vacuum-header__breadcrumb">Navigation</span>
          </div>
          <div className="vacuum-header__right">
            <span className={`vacuum-pill vacuum-pill--${runtime.connectionStatus}`}>
              <ConnectionPillIcon className="vacuum-pill__icon" />
              <span>
                {runtime.connectionStatus === "connected"
                  ? "Connected"
                  : runtime.connectionStatus === "connecting"
                    ? "Connecting"
                    : "Disconnected"}
              </span>
            </span>
            <button className="vacuum-icon-button" type="button" onClick={openOverlay} title="Settings" aria-label="Settings">
              <GearIcon className="vacuum-icon-button__icon" />
            </button>
          </div>
        </header>

        <section className="vacuum-status-strip" aria-label="Readiness status">
          <span className="vacuum-status-group__label">System</span>
          {systemChips.map((chip) => (
            <div key={chip.label} className={`vacuum-status-chip vacuum-status-chip--${chip.state}`}>
              <StatusChipIcon className="vacuum-status-chip__icon" kind={chip.icon} />
              <span>{chip.label}</span>
            </div>
          ))}
          <span className="vacuum-status-divider" aria-hidden="true" />
          <span className="vacuum-status-group__label">Task</span>
          {taskChips.map((chip) => (
            <div key={chip.label} className={`vacuum-status-chip vacuum-status-chip--${chip.state}`}>
              <StatusChipIcon className="vacuum-status-chip__icon" kind={chip.icon} />
              <span>{chip.label}</span>
            </div>
          ))}
        </section>

        <section className="vacuum-layout">
          <div className="vacuum-map-card">
            <div
              ref={stageRef}
              className={`vacuum-map-stage vacuum-map-stage--${routeVisualState} ${!isGoalActive ? "vacuum-map-stage--interactive" : ""}`}
              onPointerDown={onPointerDown}
              onPointerMove={onPointerMove}
              onPointerUp={onPointerEnd}
              onPointerCancel={onPointerEnd}
            >
              <div
                className="vacuum-map-controls vacuum-map-controls--float"
                role="group"
                aria-label="Map controls"
                onPointerDown={(e) => { e.stopPropagation(); }}
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
              <div className="vacuum-map-stage__topline" onPointerDown={(e) => { e.stopPropagation(); }}>
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
                      className={`vacuum-map-path vacuum-map-path--${routeVisualState}`}
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
                      className={`vacuum-map-preview-line vacuum-map-preview-line--${routeVisualState}`}
                      x1={previewLine[0].left}
                      y1={previewLine[0].top}
                      x2={previewLine[1].left}
                      y2={previewLine[1].top}
                    />
                  ) : null}
                </svg>

                {currentPose ? (() => {
                  const position = worldToPercent(currentPose, bounds);
                  return (
                    <div
                      className="vacuum-marker vacuum-marker--robot"
                      style={{ left: `${position.left}%`, top: `${position.top}%` }}
                    >
                      <span className="vacuum-marker__robot-halo" />
                      <span
                        className="vacuum-marker__robot-body"
                        style={{ transform: `translate(-50%, -50%) rotate(${currentPose.yaw ?? 0}deg)` }}
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
                      className={`vacuum-marker vacuum-marker--target vacuum-marker--target-${routeVisualState}`}
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

              {!draftTarget && !isGoalActive ? (
                <div className="vacuum-map-stage__center-prompt">
                  <strong>Choose destination</strong>
                  <span>Click anywhere on the map</span>
                </div>
              ) : null}

              <div className="vacuum-map-stage__legend" onPointerDown={(e) => { e.stopPropagation(); }}>
                <span><i className="vacuum-legend vacuum-legend--robot" />Robot</span>
                <span><i className={`vacuum-legend vacuum-legend--${routeVisualState === "idle" ? "staged" : routeVisualState}`} />Route</span>
                <span><i className="vacuum-legend vacuum-legend--target" />Destination</span>
              </div>
            </div>
          </div>

          <div className="vacuum-sidebar">

            {/* ── Card 1: Current State ── */}
            <section className="vacuum-panel-card">
              <div className="vacuum-panel-card__head">
                <p className="vacuum-panel-card__eyebrow">Current State</p>
                <span className={`vacuum-panel-indicator vacuum-panel-indicator--${operatorState.tone}`} />
              </div>
              <div className="vacuum-state-row">
                <div className={`vacuum-state-icon vacuum-state-icon--${operatorState.tone}`}>
                  <StateIcon className="vacuum-state-icon__svg" stateKey={operatorState.key} />
                </div>
                <div className="vacuum-state-row__text">
                  <p className="vacuum-state-row__title">{operatorState.title}</p>
                  <p className="vacuum-state-row__detail">{operatorState.detail}</p>
                </div>
                <VacuumMark className="vacuum-state-row__disc" aria-hidden="true" />
              </div>
            </section>

            {/* ── Card 2: Selected Destination ── */}
            <section className="vacuum-panel-card">
              <div className="vacuum-panel-card__head">
                <p className="vacuum-panel-card__eyebrow">Selected Destination</p>
              </div>
              {draftTarget ? (
                <div className="vacuum-dest-row">
                  <div className="vacuum-dest-row__icon-wrap">
                    <DirectionIcon className="vacuum-dest-row__icon" direction={draftTarget.yaw} />
                  </div>
                  <div className="vacuum-dest-row__text">
                    <p className="vacuum-dest-row__title">Destination selected</p>
                    <p className="vacuum-dest-row__sub">{targetDistanceLabel} · {targetHeadingLabel}</p>
                  </div>
                  <CompassIcon className="vacuum-compass vacuum-compass--sm" direction={draftTarget.yaw} />
                </div>
              ) : (
                <div className="vacuum-dest-row vacuum-dest-row--empty">
                  <div className="vacuum-dest-row__icon-wrap vacuum-dest-row__icon-wrap--muted">
                    <DestinationEmptyIcon className="vacuum-dest-row__icon vacuum-dest-row__icon--empty" />
                  </div>
                  <div className="vacuum-dest-row__text">
                    <p className="vacuum-dest-row__title vacuum-dest-row__title--muted">No destination</p>
                    <p className="vacuum-dest-row__sub">Click the map to pick one</p>
                  </div>
                </div>
              )}
            </section>

            {/* ── Card 3: Progress (conditional) ── */}
            {showProgressMetric ? (
              <section className="vacuum-panel-card">
                <div className="vacuum-panel-card__head">
                  <p className="vacuum-panel-card__eyebrow">Progress</p>
                  <strong className={`vacuum-progress-pct vacuum-progress-pct--${routeVisualState}`}>
                    {Math.round(routeProgress * 100)}%
                  </strong>
                </div>
                <p className="vacuum-progress-label">{progressLabel}</p>
                <div className="vacuum-progress">
                  <div
                    className={`vacuum-progress__bar vacuum-progress__bar--${routeVisualState}`}
                    style={{ width: `${displayedTarget ? progressBarWidth : 0}%` }}
                  />
                </div>
                {routeVisualState === "active" ? (
                  <div className="vacuum-stats vacuum-stats--progress">
                    <div>
                      <span>Distance remaining</span>
                      <strong>{formatDistance(routeDistanceRemaining)}</strong>
                    </div>
                    <div>
                      <span>Est. time</span>
                      <strong>{formatEta(runtime.feedbackEta)}</strong>
                    </div>
                  </div>
                ) : null}
              </section>
            ) : null}

            {/* ── Card 4: Actions ── */}
            <section className="vacuum-panel-card">
              <div className="vacuum-panel-card__head">
                <p className="vacuum-panel-card__eyebrow">Actions</p>
              </div>
              <div className="vacuum-actions">
                {isGoalActive ? (
                  <button
                    className="vacuum-action vacuum-action--danger"
                    type="button"
                    onClick={() => void handleCancel()}
                    disabled={runtime.isCancelingGoal}
                  >
                    {runtime.isCancelingGoal ? "Canceling…" : "Cancel run"}
                  </button>
                ) : (
                  <button
                    className="vacuum-action vacuum-action--primary"
                    type="button"
                    onClick={() => void handleSend()}
                    disabled={!draftTarget || runtime.preflightStatus.state !== "ready" || runtime.isSendingGoal}
                  >
                    <SendIcon className="vacuum-action__icon" />
                    {runtime.isSendingGoal ? "Sending…" : "Send"}
                  </button>
                )}
                <button
                  className="vacuum-action vacuum-action--ghost"
                  type="button"
                  onClick={handleClear}
                  disabled={!draftTarget || isGoalActive}
                >
                  Clear destination
                </button>
              </div>
            </section>

          </div>
        </section>
      </main>
    </div>
  );
}
