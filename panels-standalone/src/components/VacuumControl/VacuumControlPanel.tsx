import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useConnectionSettings } from "../ConnectionSettingsProvider";
import { ros2Bridge } from "tensorfleet-ros";
import {
  formatDuration,
  useVacuumAdapter,
  type VacuumCommandResult,
  type VacuumMappingStatus,
  type VacuumNavigationState,
  type VacuumPoseCoordinates,
  type VacuumSavedMapSummary,
} from "../../vacuum-adapter";
import {
  MapCanvas,
  type CleanAreaRect,
  type CleanAreaValidation,
  type CleanAreaVisualState,
  type MapCanvasMetadata,
  type MapCanvasTarget,
  type MappingSessionState,
  type RouteVisualState,
} from "./MapCanvas";
import { TeleopCard } from "./TeleopCard";
import {
  buildCleanAreaCoverageSnapshot,
  buildCleanAreaCoverageTarget,
  markCleanAreaCoveredCells,
  type CleanAreaCoverageTarget,
  type CleanAreaCoverageSnapshot,
} from "./cleanAreaCoverage";
import { buildLawnmowerWaypoints } from "./cleanAreaPlanner";
import "./VacuumControlPanel.css";

type DraftTarget = MapCanvasTarget;
type CleanAreaMissionState =
  | "idle"
  | "editing"
  | "confirmed"
  | "preparing"
  | "running"
  | "paused"
  | "canceling"
  | "completed"
  | "failed"
  | "canceled";

type StatusChipTone = "success" | "active" | "inactive";
type SavedMapSummary = {
  name: string;
  createdAt: number;
  metadata: MapCanvasMetadata;
  notes: string;
};

const CLEAN_AREA_SWATH_WIDTH_M = 0.3;
const CLEAN_AREA_OVERLAP_RATIO = 0.6;
const CLEAN_AREA_GOAL_TOLERANCE_COMPENSATION_M = 0.28;
const CLEAN_AREA_COMPLETE_COVERAGE_THRESHOLD = 0.95;

function mapAdapterMappingState(state: VacuumMappingStatus["state"]): MappingSessionState {
  if (state === "auto_mapping" || state === "manual_mapping") {
    return "mapping";
  }
  if (state === "paused" || state === "needs_assistance") {
    return "paused";
  }
  if (state === "review") {
    return "review";
  }
  if (state === "accepted") {
    return "saved";
  }
  if (state === "discarded") {
    return "discarded";
  }
  if (state === "error") {
    return "error";
  }
  return "not_started";
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function toFiniteNumber(value: unknown): number | null {
  const numeric = typeof value === "string" ? Number(value) : value;
  return typeof numeric === "number" && Number.isFinite(numeric) ? numeric : null;
}

function formatDistance(distance: number | null): string {
  if (distance == null || !Number.isFinite(distance)) {
    return "n/a";
  }
  return distance < 10 ? `${distance.toFixed(1)} m` : `${distance.toFixed(0)} m`;
}

function formatCoordinate(value: number | null | undefined): string {
  return typeof value === "number" && Number.isFinite(value) ? value.toFixed(2) : "n/a";
}

function formatPercent(value: number): string {
  return `${Math.round(clamp(value, 0, 1) * 100)}%`;
}

function formatArea(value: number): string {
  return Number.isFinite(value) ? `${value.toFixed(1)} m²` : "n/a";
}

function formatDimension(value: number): string {
  return Number.isFinite(value) ? `${value.toFixed(1)} m` : "n/a";
}

function formatResolution(value: number): string {
  return value > 0 ? `${value.toFixed(3)} m/cell` : "n/a";
}

function formatMapAge(lastUpdateAt: number | null, now: number): string {
  if (lastUpdateAt == null) {
    return "n/a";
  }
  const seconds = Math.max(0, Math.round((now - lastUpdateAt) / 1000));
  if (seconds < 2) {
    return "just now";
  }
  if (seconds < 60) {
    return `${seconds}s ago`;
  }
  return `${Math.round(seconds / 60)}m ago`;
}

function formatSavedMapTime(timestamp: number | null): string {
  if (timestamp == null) {
    return "unknown";
  }
  return new Date(timestamp).toLocaleString();
}

function formatRecoveries(value: number | null): string {
  return value == null ? "0" : String(Math.max(0, Math.round(value)));
}

function distanceBetween(
  a: { x: number; y: number } | null,
  b: { x: number; y: number } | null,
): number | null {
  if (!a || !b) {
    return null;
  }
  return Math.hypot(b.x - a.x, b.y - a.y);
}

function bearingBetween(
  a: { x: number; y: number } | null,
  b: { x: number; y: number } | null,
): number | null {
  if (!a || !b) {
    return null;
  }
  return (Math.atan2(b.y - a.y, b.x - a.x) * 180) / Math.PI;
}

function headingLabel(yaw: number): string {
  const headings = ["East", "North-East", "North", "North-West", "West", "South-West", "South", "South-East"];
  const normalized = ((yaw % 360) + 360) % 360;
  return headings[Math.round(normalized / 45) % headings.length]!;
}

function getCleanAreaSize(rect: CleanAreaRect | null): { width: number; height: number; area: number } {
  if (!rect) {
    return { width: 0, height: 0, area: 0 };
  }
  const width = Math.max(0, rect.maxX - rect.minX);
  const height = Math.max(0, rect.maxY - rect.minY);
  return { width, height, area: width * height };
}

function getCleanAreaSpacing(mapMetadata: MapCanvasMetadata | null): number {
  const resolution = mapMetadata?.resolution ?? 0;
  const overlapSpacing = CLEAN_AREA_SWATH_WIDTH_M * (1 - CLEAN_AREA_OVERLAP_RATIO);
  const cellAwareSpacing = resolution > 0 ? resolution * 1.5 : 0;
  return clamp(Math.max(overlapSpacing, cellAwareSpacing), 0.08, CLEAN_AREA_SWATH_WIDTH_M * 0.6);
}

function getPathDistance(points: Array<{ x: number; y: number }>): number {
  let distance = 0;
  for (let index = 1; index < points.length; index += 1) {
    const previous = points[index - 1]!;
    const current = points[index]!;
    distance += Math.hypot(current.x - previous.x, current.y - previous.y);
  }
  return distance;
}

function getCleanAreaMetrics(points: DraftTarget[], currentIndex: number): {
  passCount: number;
  totalDistance: number;
  remainingDistance: number;
} {
  const clampedIndex = clamp(currentIndex, 0, points.length);
  const remainingPoints = points.slice(Math.max(0, clampedIndex));
  return {
    passCount: Math.ceil(points.length / 2),
    totalDistance: getPathDistance(points),
    remainingDistance: getPathDistance(remainingPoints),
  };
}

function getCleanAreaVisualState(state: CleanAreaMissionState, toolActive: boolean): CleanAreaVisualState {
  if (toolActive) {
    return "editing";
  }
  return state;
}

function getRouteVisualState(
  navigationState: VacuumNavigationState,
  active: boolean,
  hasDraftTarget: boolean,
  hasTarget: boolean,
): RouteVisualState {
  if (!active && hasDraftTarget) {
    return "staged";
  }
  if (navigationState === "completed") {
    return "completed";
  }
  if (navigationState === "canceled") {
    return "canceled";
  }
  if (navigationState === "failed" || navigationState === "blocked" || navigationState === "unknown") {
    return "failed";
  }
  if (active) {
    return "active";
  }
  return hasTarget ? "staged" : "idle";
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

function ConnectionPillIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 12 12">
      <circle cx="6" cy="6" r="4.5" />
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

function StopIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 20 20">
      <rect x="5" y="5" width="10" height="10" rx="2" />
    </svg>
  );
}

function ClearIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 20 20">
      <path d="M5 5l10 10" />
      <path d="M15 5 5 15" />
    </svg>
  );
}

function SpinnerIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={`${props.className ?? ""} vacuum-spinner`} viewBox="0 0 20 20">
      <circle cx="10" cy="10" r="7" strokeDasharray="22 22" strokeLinecap="round" />
    </svg>
  );
}

function ChevronIcon(props: { expanded: boolean; className?: string }) {
  return (
    <svg className={props.className} viewBox="0 0 16 16" aria-hidden="true">
      <path d={props.expanded ? "M4 10l4-4 4 4" : "M4 6l4 4 4-4"} />
    </svg>
  );
}

function MappingModeIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
      <path d="M4 6.5 8.5 5l7 2L20 5.5v11.5L15.5 18.5l-7-2L4 18z" />
      <path d="M8.5 5v13" />
      <path d="M15.5 7v11.5" />
    </svg>
  );
}

function NavigateModeIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
      <path d="M5 18.5 9.5 6 20 4 15.5 16.5z" />
      <path d="m9.5 6 6 6" />
    </svg>
  );
}

function CleanModeIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
      <rect x="3" y="3" width="18" height="18" rx="2.5" />
      <path d="M7 8h10" />
      <path d="M7 12h10" />
      <path d="M7 16h6" />
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

function MappingCard(props: {
  mappingState: MappingSessionState;
  mappingStatus: VacuumMappingStatus;
  metadata: MapCanvasMetadata | null;
  savedMap: SavedMapSummary | null;
  savedMaps: VacuumSavedMapSummary[];
  commandError: string | null;
  mapName: string;
  now: number;
  onMapNameChange: (value: string) => void;
  autoSupported: boolean;
  manualSupported: boolean;
  onStartAuto: () => void;
  onStartManual: () => void;
  onPause: () => void;
  onContinue: () => void;
  onFinish: () => void;
  onDiscard: () => void;
  onUseMap: () => void;
  onLoadMap: (name: string) => void;
  onImproveMap: (name: string) => void;
  onRemap: () => void;
}): JSX.Element {
  const [isExpanded, setIsExpanded] = useState(true);
  const metadata = props.metadata;
  const hasMap = metadata?.hasMap ?? false;
  const stateCopy: Record<MappingSessionState, { title: string; detail: string; badge: string }> = {
    not_started: {
      title: "Mapping",
      detail: "No mapping session active. Start auto mapping to let the robot explore.",
      badge: "Idle",
    },
    mapping: {
      title: "Mapping",
      detail:
        props.mappingStatus.mode === "auto"
          ? "Auto mapping in progress. Pause if the robot needs manual help."
          : "Manual mapping in progress. Drive slowly around the space.",
      badge: props.mappingStatus.mode === "auto" ? "Auto" : "Manual",
    },
    paused: {
      title: "Mapping",
      detail:
        props.mappingStatus.state === "needs_assistance"
          ? props.mappingStatus.stateReason
          : "Mapping paused. Use manual control if needed, then resume or finish.",
      badge: props.mappingStatus.state === "needs_assistance" ? "Assist" : "Paused",
    },
    review: {
      title: "Review Map",
      detail: "Check that the known map looks complete enough before using it.",
      badge: "Review",
    },
    saved: {
      title: "Current Map",
      detail: "Map accepted for navigation and future coverage planning.",
      badge: "Ready",
    },
    discarded: {
      title: "Mapping",
      detail: "Mapping run discarded. Start a new mapping run when ready.",
      badge: "Discarded",
    },
    error: {
      title: "Mapping",
      detail: "Mapping error. No valid /map data is available yet.",
      badge: "Issue",
    },
  };
  const copy = stateCopy[props.mappingState];
  const savedLabel = props.savedMap?.name || "Current map";

  return (
    <section className={`vacuum-panel-card vacuum-panel-card--mapping vacuum-panel-card--mapping-${props.mappingState}`}>
      <button
        className="vacuum-panel-card__head vacuum-collapsible-card-head"
        type="button"
        onClick={() => { setIsExpanded((value) => !value); }}
        aria-expanded={isExpanded}
      >
        <p className="vacuum-panel-card__eyebrow">{copy.title}</p>
        <div className="vacuum-collapsible-card-head__right">
          <span className="vacuum-mapping-badge">{copy.badge}</span>
          <ChevronIcon className="vacuum-collapsible-card-chevron" expanded={isExpanded} />
        </div>
      </button>

      {isExpanded ? (
        <>
          <p className="vacuum-mapping-copy">{copy.detail}</p>

          {props.commandError ? (
            <div className="vacuum-mapping-error" role="status">
              {props.commandError}
            </div>
          ) : null}

          {props.mappingState === "saved" ? (
            <div className="vacuum-current-map">
              <strong>{savedLabel}</strong>
              <span>
                {props.mappingStatus.loadedMapPath
                  ? `Loaded from ${props.mappingStatus.loadedMapPath}`
                  : props.mappingStatus.savedMapPath
                    ? `Saved at ${props.mappingStatus.savedMapPath}`
                  : props.mappingStatus.saveError
                    ? `Accepted for this session. Save failed: ${props.mappingStatus.saveError}`
                    : props.mappingStatus.loadError
                      ? `Load failed: ${props.mappingStatus.loadError}`
                      : "Accepted for this session."}
              </span>
            </div>
          ) : null}

          {props.savedMaps.length > 0 ? (
            <div className="vacuum-saved-map-list">
              <div className="vacuum-saved-map-list__head">
                <span>Saved maps</span>
                {props.mappingStatus.activeMapName ? <strong>{props.mappingStatus.activeMapName}</strong> : null}
              </div>
              {props.savedMaps.map((savedMap) => (
                <div
                  key={savedMap.id}
                  className={`vacuum-saved-map${savedMap.active ? " vacuum-saved-map--active" : ""}`}
                >
                  <button className="vacuum-saved-map__main" type="button" onClick={() => props.onLoadMap(savedMap.name)}>
                    <span>{savedMap.name}</span>
                    <small>{formatSavedMapTime(savedMap.modifiedAt)}</small>
                  </button>
                  <button className="vacuum-saved-map__action" type="button" onClick={() => props.onLoadMap(savedMap.name)}>
                    Use
                  </button>
                  <button className="vacuum-saved-map__action" type="button" onClick={() => props.onImproveMap(savedMap.name)}>
                    Improve
                  </button>
                </div>
              ))}
            </div>
          ) : null}

          <div className="vacuum-map-metadata-grid">
            <div>
              <span>Map</span>
              <strong>{hasMap && metadata ? `${metadata.width} × ${metadata.height}` : "Waiting"}</strong>
            </div>
            <div>
              <span>Resolution</span>
              <strong>{metadata ? formatResolution(metadata.resolution) : "n/a"}</strong>
            </div>
            <div>
              <span>Known</span>
              <strong>{formatPercent(props.mappingStatus.knownRatio || metadata?.knownRatio || 0)}</strong>
            </div>
            <div>
              <span>Unknown</span>
              <strong>{formatPercent(props.mappingStatus.unknownRatio || metadata?.unknownRatio || 0)}</strong>
            </div>
            <div>
              <span>Known area</span>
              <strong>{metadata ? formatArea(metadata.knownAreaSqM) : "n/a"}</strong>
            </div>
            <div>
              <span>Last update</span>
              <strong>{metadata ? formatMapAge(metadata.lastUpdateAt, props.now) : "n/a"}</strong>
            </div>
            <div>
              <span>Pose</span>
              <strong>{metadata?.poseAvailable ? "Available" : "Missing"}</strong>
            </div>
            <div>
              <span>Frontiers</span>
              <strong>{props.mappingStatus.frontierCount}</strong>
            </div>
            <div>
              <span>Goals</span>
              <strong>{props.mappingStatus.visitedGoalCount} / {props.mappingStatus.failedGoalCount}</strong>
            </div>
          </div>

          {props.mappingState === "review" ? (
            <label className="vacuum-map-name-field">
              <span>Map name</span>
              <input
                value={props.mapName}
                onChange={(event) => props.onMapNameChange(event.target.value)}
                placeholder="Lab Mapping Run 1"
              />
            </label>
          ) : null}

          <div className="vacuum-mapping-actions">
            {props.mappingState === "not_started" || props.mappingState === "discarded" || props.mappingState === "error" ? (
              <>
                <button
                  className="vacuum-action vacuum-action--primary"
                  type="button"
                  onClick={props.onStartAuto}
                  disabled={!props.autoSupported}
                >
                  Start auto mapping
                </button>
                <button
                  className="vacuum-action vacuum-action--ghost"
                  type="button"
                  onClick={props.onStartManual}
                  disabled={!props.manualSupported}
                >
                  Manual mapping
                </button>
              </>
            ) : null}
            {props.mappingState === "mapping" ? (
              <>
                <button className="vacuum-action vacuum-action--primary" type="button" onClick={props.onFinish}>
                  Finish & review
                </button>
                <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onPause}>
                  Pause
                </button>
                <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onDiscard}>
                  Discard Session
                </button>
              </>
            ) : null}
            {props.mappingState === "paused" ? (
              <>
                <button className="vacuum-action vacuum-action--primary" type="button" onClick={props.onContinue}>
                  Resume auto mapping
                </button>
                <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onFinish}>
                  Finish & review
                </button>
                <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onDiscard}>
                  Discard Session
                </button>
              </>
            ) : null}
            {props.mappingState === "review" ? (
              <>
                <button className="vacuum-action vacuum-action--primary" type="button" onClick={props.onUseMap} disabled={!hasMap}>
                  Save Map
                </button>
                <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onContinue}>
                  Continue Mapping
                </button>
                <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onDiscard}>
                  Discard Session
                </button>
              </>
            ) : null}
            {props.mappingState === "saved" ? (
              <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onRemap}>
                Improve Current Map
              </button>
            ) : null}
          </div>
        </>
      ) : null}
    </section>
  );
}

function CleanAreaCard(props: {
  state: CleanAreaMissionState;
  toolActive: boolean;
  rect: CleanAreaRect | null;
  validation: CleanAreaValidation | null;
  coverage: CleanAreaCoverageSnapshot | null;
  waypointCount: number;
  currentWaypointIndex: number;
  passCount: number;
  estimatedDistance: number;
  distanceRemaining: number;
  commandError: string | null;
  canStart: boolean;
  canCancel: boolean;
  canPause: boolean;
  onActivateTool: () => void;
  onConfirm: () => void;
  onStart: () => void;
  onPause: () => void;
  onRetry: () => void;
  onSkip: () => void;
  onCancel: () => void;
  onClear: () => void;
}): JSX.Element {
  const size = getCleanAreaSize(props.rect);
  const waypointProgress =
    props.state === "completed"
      ? 1
      : props.waypointCount > 0
        ? clamp(props.currentWaypointIndex / props.waypointCount, 0, 0.98)
        : 0;
  const coverageProgress = props.coverage?.progress ?? 0;
  const progress = props.coverage ? coverageProgress : waypointProgress;
  const coverageActive = props.coverage != null && props.coverage.targetCells > 0;
  const coverageMeetsCompletion =
    !props.coverage ||
    props.coverage.targetCells === 0 ||
    props.coverage.progress >= CLEAN_AREA_COMPLETE_COVERAGE_THRESHOLD;
  const completedWithCoverageGap = props.state === "completed" && !coverageMeetsCompletion;
  const progressBarTone =
    props.state === "completed"
      ? coverageMeetsCompletion ? "completed" : "paused"
      : props.state === "paused"
        ? "paused"
        : "active";
  const stateCopy: Record<CleanAreaMissionState, { badge: string; detail: string }> = {
    idle: {
      badge: "Idle",
      detail: "Select a bounded free-space region before starting an area clean.",
    },
    editing: {
      badge: "Editing",
      detail: props.validation?.message ?? "Draw or adjust the clean area on the map.",
    },
    confirmed: {
      badge: "Preview",
      detail: "Clean Area Preview",
    },
    preparing: {
      badge: "Preparing",
      detail: "Preparing first waypoint.",
    },
    running: {
      badge: "Cleaning",
      detail: `Waypoint ${Math.min(props.currentWaypointIndex + 1, props.waypointCount)} / ${props.waypointCount}.`,
    },
    paused: {
      badge: "Paused",
      detail: `Paused at waypoint ${Math.min(props.currentWaypointIndex + 1, props.waypointCount)} / ${props.waypointCount}.`,
    },
    canceling: {
      badge: "Canceling",
      detail: "Stopping the current navigation goal.",
    },
    completed: {
      badge: completedWithCoverageGap ? "Route done" : "Done",
      detail: completedWithCoverageGap
        ? `Waypoint route finished with ${Math.round(coverageProgress * 100)}% coverage. Uncovered cleanable cells remain.`
        : "Area clean completed.",
    },
    failed: {
      badge: "Issue",
      detail: props.commandError ?? `Could not reach waypoint ${Math.min(props.currentWaypointIndex + 1, props.waypointCount)}.`,
    },
    canceled: {
      badge: "Stopped",
      detail: "Area clean canceled.",
    },
  };
  const copy = stateCopy[props.state];

  return (
    <section className={`vacuum-panel-card vacuum-panel-card--clean-area vacuum-panel-card--clean-area-${props.state}`}>
      <div className="vacuum-panel-card__head">
        <p className="vacuum-panel-card__eyebrow">Clean Area</p>
        <span className="vacuum-clean-area-badge">{copy.badge}</span>
      </div>
      <p className="vacuum-clean-area-copy">{copy.detail}</p>

      {props.commandError && props.state !== "failed" ? (
        <div className="vacuum-mapping-error" role="status">
          {props.commandError}
        </div>
      ) : null}

      <div className="vacuum-clean-area-stats">
        <div>
          <span>Size</span>
          <strong>
            {props.rect ? `${formatDimension(size.width)} × ${formatDimension(size.height)}` : "No area"}
          </strong>
        </div>
        <div>
          <span>Area</span>
          <strong>{props.rect ? formatArea(size.area) : "n/a"}</strong>
        </div>
        <div>
          <span>Cleanable</span>
          <strong>{props.coverage ? formatArea(props.coverage.cleanableAreaSqM) : props.validation ? formatPercent(props.validation.freeRatio) : "n/a"}</strong>
        </div>
        <div>
          <span>Skipped</span>
          <strong>{props.coverage ? formatArea(props.coverage.skippedAreaSqM) : "n/a"}</strong>
        </div>
        <div>
          <span>Passes</span>
          <strong>{props.passCount || "n/a"}</strong>
        </div>
        <div>
          <span>Distance</span>
          <strong>{props.estimatedDistance > 0 ? formatDistance(props.estimatedDistance) : "n/a"}</strong>
        </div>
      </div>

      {props.state === "confirmed" ? (
        <div className="vacuum-clean-area-preview">
          <strong>Estimated passes: {props.passCount || "n/a"}</strong>
          <span>Estimated distance: {props.estimatedDistance > 0 ? formatDistance(props.estimatedDistance) : "n/a"}</span>
          {props.coverage ? (
            <span>
              Target cleanable area: {formatArea(props.coverage.cleanableAreaSqM)} · skipped occupied/unknown/out-of-bounds: {formatArea(props.coverage.skippedAreaSqM)}
            </span>
          ) : null}
        </div>
      ) : null}

      {coverageActive || props.state === "preparing" || props.state === "running" || props.state === "paused" || props.state === "canceling" ? (
        <>
          <div className="vacuum-progress">
            <div
              className={`vacuum-progress__bar vacuum-progress__bar--${progressBarTone}`}
              style={{ width: `${progress <= 0 ? (props.state === "preparing" ? 3 : 0) : Math.max(progress * 100, 6)}%` }}
            />
          </div>
          <div className="vacuum-clean-area-progress-label">
            Coverage: {Math.round(progress * 100)}%
            {props.coverage
              ? ` · ${formatArea(props.coverage.coveredAreaSqM)} covered · ${formatArea(props.coverage.remainingAreaSqM)} remaining`
              : ` · Distance remaining: ${formatDistance(props.distanceRemaining)}`}
          </div>
          {props.coverage ? (
            <div className="vacuum-clean-area-coverage-grid">
              <div>
                <span>Target cleanable</span>
                <strong>{formatArea(props.coverage.cleanableAreaSqM)}</strong>
              </div>
              <div>
                <span>Covered area</span>
                <strong>{formatArea(props.coverage.coveredAreaSqM)}</strong>
              </div>
              <div>
                <span>Remaining area</span>
                <strong>{formatArea(props.coverage.remainingAreaSqM)}</strong>
              </div>
              <div>
                <span>Skipped cells</span>
                <strong>{props.coverage.occupiedCells} occupied · {props.coverage.unknownCells} unknown · {props.coverage.outOfBoundsCells} out</strong>
              </div>
              <div>
                <span>Waypoint progress</span>
                <strong>{Math.min(props.currentWaypointIndex, props.waypointCount)} / {props.waypointCount}</strong>
              </div>
              <div>
                <span>Swath width</span>
                <strong>{formatDimension(props.coverage.swathWidth)}</strong>
              </div>
            </div>
          ) : null}
        </>
      ) : null}

      {props.state === "failed" ? (
        <div className="vacuum-clean-area-failure-actions">
          <span>Options</span>
          <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onRetry} disabled={!props.canStart}>
            Retry waypoint
          </button>
          <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onSkip}>
            Skip waypoint
          </button>
          <button className="vacuum-action vacuum-action--danger" type="button" onClick={props.onCancel}>
            Cancel cleaning
          </button>
        </div>
      ) : null}

      <div className="vacuum-actions">
        {props.state === "preparing" || props.state === "running" || props.state === "paused" || props.state === "canceling" ? (
          <>
            {props.state === "paused" ? (
              <button className="vacuum-action vacuum-action--primary" type="button" onClick={props.onStart} disabled={!props.canStart}>
                Resume
              </button>
            ) : (
              <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onPause} disabled={!props.canPause}>
                Pause
              </button>
            )}
            <button className="vacuum-action vacuum-action--danger" type="button" onClick={props.onCancel} disabled={!props.canCancel}>
              <StopIcon className="vacuum-action__icon vacuum-action__icon--stroke" />
              Cancel
            </button>
          </>
        ) : (
          <>
            <button className="vacuum-action vacuum-action--primary" type="button" onClick={props.onActivateTool}>
              {props.toolActive ? "Drawing area" : props.rect ? "Edit area" : "Clean Area"}
            </button>
            {props.toolActive || props.state === "editing" ? (
              <button
                className="vacuum-action vacuum-action--ghost"
                type="button"
                onClick={props.onConfirm}
                disabled={!props.rect || !props.validation?.ok}
              >
                Preview Path
              </button>
            ) : null}
            {props.state === "confirmed" || props.state === "completed" || props.state === "canceled" ? (
              <button className="vacuum-action vacuum-action--primary" type="button" onClick={props.onStart} disabled={!props.canStart}>
                {props.state === "confirmed" ? "Start Cleaning" : "Start cleaning"}
              </button>
            ) : null}
            <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onClear} disabled={!props.rect}>
              Clear area
            </button>
          </>
        )}
      </div>
    </section>
  );
}

export function VacuumControlPanel() {
  const adapter = useVacuumAdapter();
  const snapshot = adapter.snapshot;
  const { openOverlay } = useConnectionSettings();
  const [draftTarget, setDraftTarget] = useState<DraftTarget | null>(null);
  const [sentTarget, setSentTarget] = useState<DraftTarget | null>(null);
  const [wallClockElapsed, setWallClockElapsed] = useState<number | null>(null);
  const [metadataClock, setMetadataClock] = useState(() => Date.now());
  const [mapMetadata, setMapMetadata] = useState<MapCanvasMetadata | null>(null);
  const [mapName, setMapName] = useState("Lab Mapping Run 1");
  const [savedMap, setSavedMap] = useState<SavedMapSummary | null>(null);
  const [mappingCommandError, setMappingCommandError] = useState<string | null>(null);
  const [cleanAreaToolActive, setCleanAreaToolActive] = useState(false);
  const [cleanAreaRect, setCleanAreaRect] = useState<CleanAreaRect | null>(null);
  const [cleanAreaValidation, setCleanAreaValidation] = useState<CleanAreaValidation | null>(null);
  const [cleanAreaState, setCleanAreaState] = useState<CleanAreaMissionState>("idle");
  const [cleanAreaWaypoints, setCleanAreaWaypoints] = useState<DraftTarget[]>([]);
  const [cleanAreaCurrentIndex, setCleanAreaCurrentIndex] = useState(0);
  const [cleanAreaCommandError, setCleanAreaCommandError] = useState<string | null>(null);
  const [cleanAreaCoveredCellKeys, setCleanAreaCoveredCellKeys] = useState<Set<string>>(() => new Set());
  const [cleanAreaMissionTarget, setCleanAreaMissionTarget] = useState<CleanAreaCoverageTarget | null>(null);
  const [activeMode, setActiveMode] = useState<"mapping" | "navigation" | "clean">("navigation");
  const goalStartTimeRef = useRef<number | null>(null);
  const cleanAreaCancelRequestedRef = useRef(false);
  const cleanAreaSawActiveRef = useRef(false);
  const previousCoveragePoseRef = useRef<VacuumPoseCoordinates | null>(null);

  const currentPose = snapshot.pose.coordinates;
  const availability = snapshot.availability.status;
  const isMapReceiving = snapshot.map.receiving;
  const poseReady = snapshot.pose.available;
  const readinessReady = snapshot.readiness.ready;
  const navigationState = snapshot.navigation.state;
  const isGoalActive = snapshot.navigation.active;
  const isSendingGoal = snapshot.navigation.isSending;
  const isCancelingGoal = snapshot.navigation.isCanceling;
  const goToLocationSupported = snapshot.capabilities.go_to_location.supported;
  const cancelNavigationSupported = snapshot.capabilities.cancel_navigation.supported;
  const mappingStatus = snapshot.mapping;
  const mappingState = mapAdapterMappingState(mappingStatus.state);
  const autoMappingSupported = snapshot.capabilities.auto_mapping.supported;
  const mappingSessionSupported = snapshot.capabilities.mapping_session.supported;
  const liveCleanAreaCoverageTarget = useMemo(
    () => buildCleanAreaCoverageTarget(cleanAreaRect, snapshot.map.grid),
    [cleanAreaRect, snapshot.map.grid],
  );
  const cleanAreaCoverageTarget = cleanAreaMissionTarget ?? liveCleanAreaCoverageTarget;
  const cleanAreaCoverage = useMemo(
    () =>
      buildCleanAreaCoverageSnapshot({
        target: cleanAreaCoverageTarget,
        coveredCellKeys: cleanAreaCoveredCellKeys,
        swathWidth: CLEAN_AREA_SWATH_WIDTH_M,
      }),
    [cleanAreaCoverageTarget, cleanAreaCoveredCellKeys],
  );

  const displayedTarget = sentTarget ?? draftTarget;
  const hasTarget = displayedTarget != null;
  const routeVisualState = getRouteVisualState(navigationState, isGoalActive, draftTarget != null, hasTarget);
  const displayedPlanPoints =
    sentTarget != null && routeVisualState !== "staged" && routeVisualState !== "canceled"
      ? snapshot.navigation.planPath
      : null;

  const destinationDistance = distanceBetween(currentPose, displayedTarget);
  const destinationBearing = bearingBetween(currentPose, displayedTarget);
  const feedbackDistanceRemaining = snapshot.navigation.progress.distanceRemaining;
  const initialRouteDistance = snapshot.navigation.progress.initialDistance;
  const routeDistanceRemaining =
    routeVisualState === "completed"
      ? 0
      : feedbackDistanceRemaining ?? (isGoalActive ? destinationDistance : null);
  const showProgressMetric =
    routeVisualState === "active" ||
    routeVisualState === "completed" ||
    routeVisualState === "failed" ||
    routeVisualState === "canceled";
  const isIndeterminate = isGoalActive && feedbackDistanceRemaining == null;
  const routeProgress = (() => {
    if (navigationState === "completed") {
      return 1;
    }
    const remaining = feedbackDistanceRemaining;
    if (remaining == null || initialRouteDistance == null || initialRouteDistance <= 0) {
      return 0;
    }
    return clamp(1 - remaining / initialRouteDistance, 0, 0.98);
  })();
  const systemChips = [
    {
      label: "Connected",
      icon: "connected" as const,
      state: getChipTone(availability === "online", "success"),
      pulsing: false,
    },
    {
      label: "Map Live",
      icon: "map" as const,
      state: getChipTone(isMapReceiving, "success"),
      pulsing: isMapReceiving,
    },
    {
      label: "Localized",
      icon: "localized" as const,
      state: getChipTone(poseReady, "success"),
      pulsing: false,
    },
  ] as const;
  const taskChips = [
    {
      label: "Ready",
      icon: "ready" as const,
      state: getChipTone(readinessReady && poseReady, "success"),
    },
    {
      label: "Target Selected",
      icon: "target" as const,
      state: getChipTone(hasTarget, "active"),
    },
  ] as const;
  const readinessIssue = snapshot.readiness.blockingReasons[0] ?? null;
  const targetDistanceLabel = displayedTarget ? formatDistance(destinationDistance) : null;
  const targetHeadingLabel = displayedTarget ? headingLabel(displayedTarget.yaw) : null;
  const targetBearingLabel =
    destinationBearing == null ? "n/a" : `${Math.round(((destinationBearing % 360) + 360) % 360)}°`;
  const destinationBadgeLabel = (() => {
    if (sentTarget != null && isGoalActive) {
      return "Sent";
    }
    if (routeVisualState === "completed") {
      return "Reached";
    }
    return "Selected";
  })();
  const primaryActionLabel = (() => {
    if (isSendingGoal) {
      return "Sending...";
    }
    if (routeVisualState === "failed") {
      return "Retry run";
    }
    if (routeVisualState === "completed" || routeVisualState === "canceled") {
      return "Run again";
    }
    return "Start run";
  })();
  const actionHint = (() => {
    if (isGoalActive) {
      return isCancelingGoal
        ? "Stop request sent. Waiting for the robot to confirm."
        : "Robot is moving. Stop the run before changing destination.";
    }
    if (!hasTarget) {
      return "Click the map to choose a destination.";
    }
    return readinessIssue ?? "Destination is ready to send.";
  })();
  const progressLabel = getProgressLabel(routeVisualState);
  const progressStatusLabel = (() => {
    if (routeVisualState === "completed") {
      return "Complete";
    }
    if (routeVisualState === "failed") {
      return "Attention";
    }
    if (routeVisualState === "canceled") {
      return "Stopped";
    }
    return "Running";
  })();
  const progressBarWidth = (() => {
    if (!showProgressMetric || isIndeterminate) {
      return 0;
    }
    if (routeVisualState === "completed") {
      return 100;
    }
    return Math.max(routeProgress * 100, 6);
  })();
  const elapsedLabel = snapshot.navigation.progress.navigationTime != null
    ? formatDuration(snapshot.navigation.progress.navigationTime)
    : wallClockElapsed != null
      ? formatDuration(wallClockElapsed)
      : "n/a";
  const recoveryCount = toFiniteNumber(snapshot.navigation.progress.recoveries);
  const recoveryLabel = formatRecoveries(recoveryCount);
  const recoveryWarning = recoveryCount != null && recoveryCount > 0;
  const modeBreadcrumb =
    activeMode === "mapping" ? "Mapping" : activeMode === "clean" ? "Clean Area" : "Navigate";

  const isMappingWorkflowActive =
    mappingState === "mapping" || mappingState === "paused" || mappingState === "review";
  const isCleanAreaRunning =
    cleanAreaState === "preparing" || cleanAreaState === "running" || cleanAreaState === "canceling";
  const isCleanAreaActive = isCleanAreaRunning || cleanAreaState === "paused";
  const isCleanAreaModeLocked = isCleanAreaActive || cleanAreaToolActive;
  const cleanAreaVisualState = getCleanAreaVisualState(cleanAreaState, cleanAreaToolActive);
  const cleanAreaSpacing = getCleanAreaSpacing(mapMetadata);
  const cleanAreaPreviewPoints =
    cleanAreaWaypoints.length > 0
      ? cleanAreaWaypoints
      : cleanAreaRect
        ? buildLawnmowerWaypoints({
            rect: cleanAreaRect,
            spacing: cleanAreaSpacing,
            swathWidth: CLEAN_AREA_SWATH_WIDTH_M,
            goalCompletionTolerance: CLEAN_AREA_GOAL_TOLERANCE_COMPENSATION_M,
            target: cleanAreaCoverageTarget,
          })
        : null;
  const cleanAreaMetrics = getCleanAreaMetrics(cleanAreaPreviewPoints ?? [], cleanAreaCurrentIndex);

  useEffect(() => {
    setCleanAreaCoveredCellKeys(new Set());
    previousCoveragePoseRef.current = null;
  }, [cleanAreaCoverageTarget?.signature]);

  useEffect(() => {
    const shouldTrackCoverage =
      cleanAreaState === "preparing" || cleanAreaState === "running" || cleanAreaState === "canceling";
    if (!shouldTrackCoverage || !currentPose || !cleanAreaCoverageTarget) {
      previousCoveragePoseRef.current = shouldTrackCoverage ? previousCoveragePoseRef.current : null;
      return;
    }

    const previousPose = previousCoveragePoseRef.current;
    setCleanAreaCoveredCellKeys((current) =>
      markCleanAreaCoveredCells({
        target: cleanAreaCoverageTarget,
        coveredCellKeys: current,
        previousPose,
        currentPose,
        swathWidth: CLEAN_AREA_SWATH_WIDTH_M,
      }),
    );
    previousCoveragePoseRef.current = currentPose;
  }, [cleanAreaCoverageTarget, cleanAreaState, currentPose]);

  useEffect(() => {
    if (isGoalActive) {
      goalStartTimeRef.current = Date.now();
      setWallClockElapsed(0);
      const interval = setInterval(() => {
        if (goalStartTimeRef.current != null) {
          setWallClockElapsed((Date.now() - goalStartTimeRef.current) / 1000);
        }
      }, 1000);
      return () => clearInterval(interval);
    }
    return undefined;
  }, [isGoalActive]);

  useEffect(() => {
    const interval = setInterval(() => setMetadataClock(Date.now()), 1000);
    return () => clearInterval(interval);
  }, []);

  const isTerminalState =
    routeVisualState === "completed" || routeVisualState === "failed" || routeVisualState === "canceled";
  const runTarget = draftTarget ?? (isTerminalState ? sentTarget : null);
  const canSendRun =
    Boolean(runTarget) && goToLocationSupported && readinessReady && !isSendingGoal && !isCleanAreaRunning;
  const canCancelRun = cancelNavigationSupported && !isCancelingGoal;
  const canStartCleanArea =
    Boolean(cleanAreaRect) &&
    Boolean(cleanAreaValidation?.ok) &&
    Boolean(cleanAreaCoverageTarget && cleanAreaCoverageTarget.cleanableCells.length > 0) &&
    cleanAreaWaypoints.length > 0 &&
    goToLocationSupported &&
    readinessReady &&
    !isSendingGoal &&
    !isGoalActive &&
    !isMappingWorkflowActive;

  const handleCleanAreaChange = useCallback((rect: CleanAreaRect, validation: CleanAreaValidation): void => {
    if (!cleanAreaToolActive && cleanAreaState !== "idle" && cleanAreaState !== "editing") {
      setCleanAreaValidation(validation);
      return;
    }

    const nextTarget = buildCleanAreaCoverageTarget(rect, snapshot.map.grid);
    setCleanAreaRect(rect);
    setCleanAreaValidation(validation);
    setCleanAreaMissionTarget(null);
    setCleanAreaWaypoints(
      validation.ok
        ? buildLawnmowerWaypoints({
            rect,
            spacing: getCleanAreaSpacing(mapMetadata),
            swathWidth: CLEAN_AREA_SWATH_WIDTH_M,
            goalCompletionTolerance: CLEAN_AREA_GOAL_TOLERANCE_COMPENSATION_M,
            target: nextTarget,
          })
        : [],
    );
    setCleanAreaCommandError(validation.ok ? null : validation.message);
    setCleanAreaState((current) => {
      return current === "idle" || current === "editing" ? "editing" : current;
    });
  }, [cleanAreaState, cleanAreaToolActive, mapMetadata, snapshot.map.grid]);

  async function handleSend(overrideTarget?: DraftTarget): Promise<void> {
    const target = overrideTarget ?? runTarget;
    if (!target) {
      return;
    }
    setSentTarget(target);
    await adapter.sendCommand({ command: "go_to_location", target });
  }

  async function handleCancel(): Promise<void> {
    await adapter.sendCommand({ command: "cancel_navigation" });
  }

  function handleClear(): void {
    if (isGoalActive) {
      return;
    }
    setDraftTarget(null);
    setSentTarget(null);
  }

  function handleTargetStart(target: DraftTarget): void {
    if (isMappingWorkflowActive || cleanAreaToolActive || isCleanAreaActive) {
      return;
    }
    setSentTarget(null);
    setDraftTarget(target);
  }

  function handleTargetRotate(yaw: number): void {
    if (isMappingWorkflowActive || cleanAreaToolActive || isCleanAreaActive) {
      return;
    }
    setDraftTarget((current) => {
      if (!current) {
        return current;
      }
      return { ...current, yaw };
    });
  }

  function stopManualMotion(): void {
    if (ros2Bridge.isConnected()) {
      ros2Bridge.publish("/cmd_vel_raw", "geometry_msgs/msg/Twist", {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
    }
  }

  async function handleStartAutoMapping(): Promise<void> {
    setMappingCommandError(null);
    const result = await adapter.sendCommand({ command: "start_mapping", mode: "auto", name: mapName });
    handleMappingCommandResult(result);
    setDraftTarget(null);
    setSentTarget(null);
  }

  async function handleStartManualMapping(): Promise<void> {
    setMappingCommandError(null);
    const result = await adapter.sendCommand({ command: "start_mapping", mode: "manual", name: mapName });
    handleMappingCommandResult(result);
    setDraftTarget(null);
    setSentTarget(null);
  }

  async function handlePauseMapping(): Promise<void> {
    stopManualMotion();
    setMappingCommandError(null);
    handleMappingCommandResult(await adapter.sendCommand({ command: "pause_mapping" }));
  }

  async function handleContinueMapping(): Promise<void> {
    setMappingCommandError(null);
    handleMappingCommandResult(await adapter.sendCommand({ command: "resume_mapping" }));
  }

  async function handleFinishMapping(): Promise<void> {
    stopManualMotion();
    setMappingCommandError(null);
    handleMappingCommandResult(await adapter.sendCommand({ command: "finish_mapping" }));
  }

  async function handleDiscardMapping(): Promise<void> {
    stopManualMotion();
    setMappingCommandError(null);
    handleMappingCommandResult(await adapter.sendCommand({ command: "discard_mapping" }));
    setDraftTarget(null);
    setSentTarget(null);
  }

  function handleMappingCommandResult(result: VacuumCommandResult): boolean {
    if (result.ok) {
      setMappingCommandError(null);
      return true;
    }
    setMappingCommandError(result.error.message);
    return false;
  }

  async function handleUseMap(): Promise<void> {
    if (!mapMetadata?.hasMap) {
      return;
    }
    setMappingCommandError(null);
    const result = await adapter.sendCommand({ command: "accept_map", name: mapName });
    if (!handleMappingCommandResult(result)) {
      return;
    }
    setSavedMap({
      name: mapName.trim() || "Current map",
      createdAt: Date.now(),
      metadata: mapMetadata,
      notes: "Accepted in UI for navigation and future coverage planning.",
    });
  }

  async function handleRemap(): Promise<void> {
    setMappingCommandError(null);
    const result = await adapter.sendCommand({ command: "start_mapping", mode: autoMappingSupported ? "auto" : "manual", name: mapName });
    handleMappingCommandResult(result);
    setDraftTarget(null);
    setSentTarget(null);
  }

  async function handleLoadMap(name: string): Promise<void> {
    setMappingCommandError(null);
    const result = await adapter.sendCommand({ command: "load_map", name });
    if (!handleMappingCommandResult(result)) {
      return;
    }
    setMapName(name);
    setSavedMap({
      name,
      createdAt: Date.now(),
      metadata: mapMetadata ?? {
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
        knownRatio: 0,
        unknownRatio: 1,
        knownAreaSqM: 0,
        lastUpdateAt: null,
        poseAvailable: false,
        readiness: "No map",
      },
      notes: "Loaded saved map for navigation and future coverage planning.",
    });
    setDraftTarget(null);
    setSentTarget(null);
  }

  async function handleImproveMap(name: string): Promise<void> {
    setMappingCommandError(null);
    const loadResult = await adapter.sendCommand({ command: "load_map", name });
    if (!handleMappingCommandResult(loadResult)) {
      return;
    }
    const startResult = await adapter.sendCommand({
      command: "start_mapping",
      mode: autoMappingSupported ? "auto" : "manual",
      name,
    });
    if (!handleMappingCommandResult(startResult)) {
      return;
    }
    setMapName(name);
    setSavedMap(null);
    setDraftTarget(null);
    setSentTarget(null);
  }

  function handleActivateCleanAreaTool(): void {
    if (isGoalActive || isMappingWorkflowActive || isCleanAreaActive) {
      return;
    }
    setCleanAreaToolActive(true);
    setCleanAreaState("editing");
    setCleanAreaCommandError(null);
    setDraftTarget(null);
    setSentTarget(null);
  }

  function handleConfirmCleanArea(): void {
    if (!cleanAreaRect || !cleanAreaValidation?.ok) {
      return;
    }
    const target = buildCleanAreaCoverageTarget(cleanAreaRect, snapshot.map.grid);
    if (!target || target.cleanableCells.length === 0) {
      setCleanAreaCommandError("Area has no cleanable cells.");
      return;
    }
    const waypoints = buildLawnmowerWaypoints({
      rect: cleanAreaRect,
      spacing: cleanAreaSpacing,
      swathWidth: CLEAN_AREA_SWATH_WIDTH_M,
      goalCompletionTolerance: CLEAN_AREA_GOAL_TOLERANCE_COMPENSATION_M,
      target,
    });
    if (waypoints.length === 0) {
      setCleanAreaCommandError("No reachable clean-area waypoints could be planned.");
      return;
    }
    setCleanAreaToolActive(false);
    setCleanAreaMissionTarget(target);
    setCleanAreaWaypoints(waypoints);
    setCleanAreaCurrentIndex(0);
    setCleanAreaCommandError(null);
    setCleanAreaState("confirmed");
  }

  function clearCleanAreaNavigationTarget(): void {
    setDraftTarget(null);
    setSentTarget(null);
  }

  function handleClearCleanArea(): void {
    if (isCleanAreaActive) {
      return;
    }
    setCleanAreaToolActive(false);
    setCleanAreaRect(null);
    setCleanAreaValidation(null);
    setCleanAreaWaypoints([]);
    setCleanAreaCurrentIndex(0);
    setCleanAreaCommandError(null);
    setCleanAreaCoveredCellKeys(new Set());
    setCleanAreaMissionTarget(null);
    previousCoveragePoseRef.current = null;
    setCleanAreaState("idle");
  }

  async function dispatchCleanAreaWaypoint(index: number): Promise<void> {
    const target = cleanAreaWaypoints[index];
    if (!target) {
      clearCleanAreaNavigationTarget();
      setCleanAreaState("completed");
      return;
    }
    cleanAreaSawActiveRef.current = false;
    setCleanAreaCurrentIndex(index);
    setDraftTarget(null);
    setSentTarget(target);
    const result = await adapter.sendCommand({ command: "go_to_location", target });
    if (!result.ok) {
      setCleanAreaCommandError(result.error.message);
      setCleanAreaState("failed");
      return;
    }
    setCleanAreaState("running");
  }

  function handleStartCleanArea(): void {
    if (!canStartCleanArea) {
      return;
    }
    const isResuming = cleanAreaState === "paused";
    const startIndex = isResuming ? cleanAreaCurrentIndex : 0;
    cleanAreaCancelRequestedRef.current = false;
    cleanAreaSawActiveRef.current = false;
    if (!isResuming) {
      setCleanAreaCoveredCellKeys(new Set());
      previousCoveragePoseRef.current = null;
    }
    setCleanAreaToolActive(false);
    setCleanAreaCommandError(null);
    setCleanAreaCurrentIndex(startIndex);
    setCleanAreaState("preparing");
    void dispatchCleanAreaWaypoint(startIndex);
  }

  async function handlePauseCleanArea(): Promise<void> {
    if (cleanAreaState !== "running" && cleanAreaState !== "preparing") {
      return;
    }
    cleanAreaCancelRequestedRef.current = false;
    if (isGoalActive || navigationState === "active" || navigationState === "sending") {
      const result = await adapter.sendCommand({ command: "cancel_navigation" });
      if (!result.ok) {
        setCleanAreaCommandError(result.error.message);
        setCleanAreaState("failed");
        return;
      }
    }
    clearCleanAreaNavigationTarget();
    setCleanAreaState("paused");
  }

  async function handleCancelCleanArea(): Promise<void> {
    cleanAreaCancelRequestedRef.current = true;
    setCleanAreaState("canceling");
    if (isGoalActive || navigationState === "active" || navigationState === "sending") {
      const result = await adapter.sendCommand({ command: "cancel_navigation" });
      if (!result.ok) {
        setCleanAreaCommandError(result.error.message);
        setCleanAreaState("failed");
      }
      return;
    }
    clearCleanAreaNavigationTarget();
    setCleanAreaState("canceled");
  }

  function handleRetryCleanAreaWaypoint(): void {
    if (!canStartCleanArea) {
      return;
    }
    cleanAreaCancelRequestedRef.current = false;
    setCleanAreaCommandError(null);
    setCleanAreaState("preparing");
    void dispatchCleanAreaWaypoint(cleanAreaCurrentIndex);
  }

  function handleSkipCleanAreaWaypoint(): void {
    const nextIndex = cleanAreaCurrentIndex + 1;
    cleanAreaCancelRequestedRef.current = false;
    setCleanAreaCommandError(null);
    if (nextIndex >= cleanAreaWaypoints.length) {
      setCleanAreaCurrentIndex(cleanAreaWaypoints.length);
      clearCleanAreaNavigationTarget();
      setCleanAreaState("completed");
      return;
    }
    setCleanAreaState("preparing");
    void dispatchCleanAreaWaypoint(nextIndex);
  }

  useEffect(() => {
    if (cleanAreaState !== "running" && cleanAreaState !== "canceling") {
      return;
    }
    if (isGoalActive || navigationState === "active" || navigationState === "sending" || navigationState === "canceling") {
      cleanAreaSawActiveRef.current = true;
      return;
    }
    if (!cleanAreaSawActiveRef.current) {
      return;
    }
    if (navigationState === "completed") {
      const nextIndex = cleanAreaCurrentIndex + 1;
      if (nextIndex >= cleanAreaWaypoints.length) {
        setCleanAreaCurrentIndex(cleanAreaWaypoints.length);
        clearCleanAreaNavigationTarget();
        setCleanAreaState("completed");
        return;
      }
      void dispatchCleanAreaWaypoint(nextIndex);
      return;
    }
    if (navigationState === "canceled") {
      if (cleanAreaCancelRequestedRef.current || cleanAreaState === "canceling") {
        clearCleanAreaNavigationTarget();
        setCleanAreaState("canceled");
        setCleanAreaCommandError(null);
        return;
      }
      clearCleanAreaNavigationTarget();
      setCleanAreaState("failed");
      setCleanAreaCommandError("Clean area run was canceled outside the area workflow.");
      return;
    }
    if (navigationState === "failed" || navigationState === "blocked" || navigationState === "unknown") {
      clearCleanAreaNavigationTarget();
      setCleanAreaState("failed");
      setCleanAreaCommandError(snapshot.navigation.detail ?? "Navigation failed while cleaning the selected area.");
    }
  }, [cleanAreaCurrentIndex, cleanAreaState, cleanAreaWaypoints.length, isGoalActive, navigationState, snapshot.navigation.detail]);

  useEffect(() => {
    if (isMappingWorkflowActive) {
      setActiveMode("mapping");
    } else if (isCleanAreaActive || cleanAreaToolActive) {
      setActiveMode("clean");
    } else if (isGoalActive) {
      setActiveMode("navigation");
    }
  }, [isMappingWorkflowActive, isCleanAreaActive, cleanAreaToolActive, isGoalActive]);

  return (
    <div className="vacuum-shell">
      <aside className="vacuum-rail">
        <div className="vacuum-rail__brand" title="Vacuum Control">
          <VacuumMark className="vacuum-rail__mark" />
        </div>

        <nav className="vacuum-rail__nav" aria-label="Panel navigation">
          <button
            className="vacuum-rail__item vacuum-rail__item--active"
            type="button"
            title="Navigation"
            aria-label="Navigation"
          >
            <SidebarIcon className="vacuum-rail__icon" kind="navigation" />
          </button>
        </nav>

        <button
          className="vacuum-rail__item vacuum-rail__item--settings"
          type="button"
          title="Connection settings"
          aria-label="Connection settings"
          onClick={openOverlay}
        >
          <GearIcon className="vacuum-rail__icon" />
        </button>

      </aside>

      <main className="vacuum-main">
        <header className="vacuum-header">
          <div className="vacuum-header__left">
            <h1 className="vacuum-header__title">Vacuum Control</h1>
            <span className="vacuum-header__breadcrumb">{modeBreadcrumb}</span>
          </div>
          <div className="vacuum-header__right">
            {availability === "offline" ? (
              <button
                className="vacuum-pill vacuum-pill--disconnected vacuum-pill--clickable"
                type="button"
                onClick={openOverlay}
                title="Open connection settings"
              >
                <ConnectionPillIcon className="vacuum-pill__icon" />
                <span>Disconnected</span>
              </button>
            ) : (
              <span className={`vacuum-pill vacuum-pill--${availability === "online" ? "connected" : "connecting"}`}>
                <ConnectionPillIcon className="vacuum-pill__icon" />
                <span>{availability === "online" ? "Connected" : "Connecting"}</span>
              </span>
            )}
          </div>
        </header>

        <section className="vacuum-status-strip" aria-label="Readiness status">
          <span className="vacuum-status-group__label">System</span>
          {systemChips.map((chip) => (
            <div
              key={chip.label}
              className={`vacuum-status-chip vacuum-status-chip--${chip.state}${chip.pulsing ? " vacuum-status-chip--pulsing" : ""}`}
            >
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
          <MapCanvas
            currentPose={currentPose}
            planPoints={displayedPlanPoints}
            draftTarget={draftTarget}
            sentTarget={sentTarget}
            cleanAreaRect={cleanAreaRect}
            cleanAreaPreviewPoints={cleanAreaPreviewPoints}
            cleanAreaCurrentIndex={cleanAreaCurrentIndex}
            cleanAreaCoverage={cleanAreaCoverage}
            cleanAreaToolActive={cleanAreaToolActive}
            cleanAreaVisualState={cleanAreaVisualState}
            interactionMode={activeMode}
            routeVisualState={routeVisualState}
            isGoalActive={isGoalActive}
            mappingState={mappingState}
            disableTargetSelection={activeMode !== "navigation" || isMappingWorkflowActive || cleanAreaToolActive || isCleanAreaActive}
            targetDistance={destinationDistance}
            adapterMapGrid={snapshot.map.grid}
            adapterMapMetadata={snapshot.map.metadata}
            onTargetStart={handleTargetStart}
            onTargetRotate={handleTargetRotate}
            onCleanAreaChange={handleCleanAreaChange}
            onMapMetadataChange={setMapMetadata}
          />

          <div className="vacuum-sidebar">

            {/* ── Mode switcher ── */}
            <div className="vacuum-mode-switcher">
              <span className="vacuum-mode-switcher__label">Mode</span>
              <div className="vacuum-mode-switcher__tabs">
                <button
                  type="button"
                  className={`vacuum-mode-tab vacuum-mode-tab--mapping${activeMode === "mapping" ? " vacuum-mode-tab--active" : ""}`}
                  onClick={() => { setActiveMode("mapping"); }}
                  disabled={isCleanAreaModeLocked || (isGoalActive && !isMappingWorkflowActive)}
                  title={isCleanAreaModeLocked ? "Finish clean area first" : isGoalActive && !isMappingWorkflowActive ? "Stop navigation first" : "Mapping"}
                >
                  <MappingModeIcon className="vacuum-mode-tab__icon" />
                  <span>Mapping</span>
                </button>
                <button
                  type="button"
                  className={`vacuum-mode-tab vacuum-mode-tab--navigation${activeMode === "navigation" ? " vacuum-mode-tab--active" : ""}`}
                  onClick={() => { setActiveMode("navigation"); }}
                  disabled={isMappingWorkflowActive || isCleanAreaModeLocked}
                  title={isMappingWorkflowActive ? "Finish mapping first" : isCleanAreaModeLocked ? "Finish clean area first" : "Navigate"}
                >
                  <NavigateModeIcon className="vacuum-mode-tab__icon" />
                  <span>Navigate</span>
                </button>
                <button
                  type="button"
                  className={`vacuum-mode-tab vacuum-mode-tab--clean${activeMode === "clean" ? " vacuum-mode-tab--active" : ""}`}
                  onClick={() => { setActiveMode("clean"); }}
                  disabled={isMappingWorkflowActive || (isGoalActive && !isCleanAreaActive)}
                  title={isMappingWorkflowActive ? "Finish mapping first" : isGoalActive && !isCleanAreaActive ? "Stop navigation first" : "Clean area"}
                >
                  <CleanModeIcon className="vacuum-mode-tab__icon" />
                  <span>Clean area</span>
                </button>
              </div>
            </div>

            {/* ── Mapping mode ── */}
            {activeMode === "mapping" && (
              <div className="vacuum-mode-content">
                <MappingCard
                  mappingState={mappingState}
                  mappingStatus={mappingStatus}
                  metadata={mapMetadata}
                  savedMap={savedMap}
                  savedMaps={mappingStatus.savedMaps}
                  commandError={mappingCommandError}
                  mapName={mapName}
                  now={metadataClock}
                  onMapNameChange={setMapName}
                  autoSupported={autoMappingSupported}
                  manualSupported={mappingSessionSupported}
                  onStartAuto={() => void handleStartAutoMapping()}
                  onStartManual={() => void handleStartManualMapping()}
                  onPause={() => void handlePauseMapping()}
                  onContinue={() => void handleContinueMapping()}
                  onFinish={() => void handleFinishMapping()}
                  onDiscard={() => void handleDiscardMapping()}
                  onUseMap={() => void handleUseMap()}
                  onLoadMap={(name) => void handleLoadMap(name)}
                  onImproveMap={(name) => void handleImproveMap(name)}
                  onRemap={() => void handleRemap()}
                />
                <TeleopCard
                  disabled={mappingStatus.state === "auto_mapping"}
                  disabledReason="Pause auto mapping before using manual control."
                />
              </div>
            )}

            {/* ── Navigate mode ── */}
            {activeMode === "navigation" && (
              <div className="vacuum-mode-content">
                <section className={`vacuum-panel-card vacuum-panel-card--destination ${displayedTarget ? "vacuum-panel-card--destination-selected" : ""}`}>
                  <div className="vacuum-panel-card__head">
                    <p className="vacuum-panel-card__eyebrow">Destination</p>
                    {displayedTarget ? (
                      <span className="vacuum-destination-status">{destinationBadgeLabel}</span>
                    ) : null}
                  </div>
                  {displayedTarget ? (
                    <>
                      <div className="vacuum-dest-row">
                        <div className="vacuum-dest-row__icon-wrap">
                          <CompassIcon className="vacuum-dest-row__icon vacuum-dest-row__icon--compass" direction={displayedTarget.yaw} />
                        </div>
                        <div className="vacuum-dest-row__text">
                          <p className="vacuum-dest-row__title">Destination selected</p>
                          <p className="vacuum-dest-row__sub">{targetDistanceLabel} from robot · bearing {targetBearingLabel}</p>
                          <p className="vacuum-dest-row__sub">coords: {formatCoordinate(displayedTarget.x)}, {formatCoordinate(displayedTarget.y)} · facing {targetHeadingLabel}</p>
                        </div>
                      </div>
                    </>
                  ) : (
                    <div className="vacuum-dest-row vacuum-dest-row--empty">
                      <div className="vacuum-dest-row__icon-wrap vacuum-dest-row__icon-wrap--muted">
                        <DestinationEmptyIcon className="vacuum-dest-row__icon vacuum-dest-row__icon--empty" />
                      </div>
                      <div className="vacuum-dest-row__text">
                        <p className="vacuum-dest-row__title vacuum-dest-row__title--muted">No destination</p>
                        <p className="vacuum-dest-row__sub">
                          {isMappingWorkflowActive ? "Disabled during mapping" : "Click the map to pick one"}
                        </p>
                      </div>
                    </div>
                  )}
                </section>

                {showProgressMetric ? (
                  <section className={`vacuum-panel-card vacuum-panel-card--progress vacuum-panel-card--progress-${routeVisualState}`}>
                    <div className="vacuum-panel-card__head">
                      <p className="vacuum-panel-card__eyebrow">Progress</p>
                      <span className={`vacuum-progress-status vacuum-progress-status--${routeVisualState}`}>
                        {progressStatusLabel}
                      </span>
                    </div>
                    <p className="vacuum-progress-label">{progressLabel}</p>
                    <div className="vacuum-progress">
                      <div
                        className={`vacuum-progress__bar vacuum-progress__bar--${routeVisualState}${isIndeterminate ? " vacuum-progress__bar--indeterminate" : ""}`}
                        style={isIndeterminate ? undefined : { width: `${displayedTarget ? progressBarWidth : 0}%` }}
                      />
                    </div>
                    <div className="vacuum-stats vacuum-stats--progress">
                      <div>
                        <span>Remaining</span>
                        <strong>{formatDistance(routeDistanceRemaining)}</strong>
                      </div>
                      <div>
                        <span>Elapsed</span>
                        <strong>{elapsedLabel}</strong>
                      </div>
                      <div>
                        <span>Recoveries</span>
                        <strong style={recoveryWarning ? { color: "var(--vacuum-warning)" } : undefined}>
                          {recoveryLabel}
                        </strong>
                      </div>
                    </div>
                  </section>
                ) : (
                  <section className="vacuum-panel-card vacuum-panel-card--progress-idle">
                    <p className="vacuum-panel-card__eyebrow">Progress</p>
                    <p className="vacuum-progress-idle-hint">Start a run to see progress.</p>
                  </section>
                )}

                <section className="vacuum-panel-card vacuum-panel-card--actions">
                  <p className="vacuum-action-hint">{actionHint}</p>
                  <div className="vacuum-actions">
                    {availability !== "online" ? (
                      <button
                        className="vacuum-action vacuum-action--primary"
                        type="button"
                        onClick={openOverlay}
                      >
                        <GearIcon className="vacuum-action__icon vacuum-action__icon--stroke" />
                        Connection settings
                      </button>
                    ) : isGoalActive ? (
                      <button
                        className="vacuum-action vacuum-action--danger"
                        type="button"
                        onClick={() => void handleCancel()}
                        disabled={!canCancelRun}
                      >
                        <StopIcon className="vacuum-action__icon vacuum-action__icon--stroke" />
                        {isCancelingGoal ? "Stopping..." : "Stop run"}
                      </button>
                    ) : (
                      <button
                        className="vacuum-action vacuum-action--primary"
                        type="button"
                        onClick={() => void handleSend()}
                        disabled={!canSendRun}
                      >
                        {isSendingGoal ? (
                          <SpinnerIcon className="vacuum-action__icon vacuum-action__icon--stroke" />
                        ) : (
                          <SendIcon className="vacuum-action__icon" />
                        )}
                        {primaryActionLabel}
                      </button>
                    )}
                    <button
                      className="vacuum-action vacuum-action--ghost"
                      type="button"
                      onClick={handleClear}
                      disabled={!hasTarget || isGoalActive}
                    >
                      <ClearIcon className="vacuum-action__icon vacuum-action__icon--stroke" />
                      Clear destination
                    </button>
                  </div>
                </section>
              </div>
            )}

            {/* ── Clean area mode ── */}
            {activeMode === "clean" && (
              <div className="vacuum-mode-content">
                <CleanAreaCard
                  state={cleanAreaState}
                  toolActive={cleanAreaToolActive}
                  rect={cleanAreaRect}
                  validation={cleanAreaValidation}
                  coverage={cleanAreaCoverage}
                  waypointCount={cleanAreaWaypoints.length}
                  currentWaypointIndex={cleanAreaCurrentIndex}
                  passCount={cleanAreaMetrics.passCount}
                  estimatedDistance={cleanAreaMetrics.totalDistance}
                  distanceRemaining={cleanAreaMetrics.remainingDistance}
                  commandError={cleanAreaCommandError}
                  canStart={canStartCleanArea}
                  canCancel={cancelNavigationSupported && !isCancelingGoal}
                  canPause={cancelNavigationSupported && !isCancelingGoal}
                  onActivateTool={handleActivateCleanAreaTool}
                  onConfirm={handleConfirmCleanArea}
                  onStart={handleStartCleanArea}
                  onPause={() => void handlePauseCleanArea()}
                  onRetry={handleRetryCleanAreaWaypoint}
                  onSkip={handleSkipCleanAreaWaypoint}
                  onCancel={() => void handleCancelCleanArea()}
                  onClear={handleClearCleanArea}
                />
                <TeleopCard
                  disabled={isCleanAreaActive}
                  disabledReason="Stop the cleaning run before using manual control."
                />
              </div>
            )}

          </div>
        </section>
      </main>
    </div>
  );
}
