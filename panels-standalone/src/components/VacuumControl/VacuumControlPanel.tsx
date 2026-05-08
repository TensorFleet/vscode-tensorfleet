import { useEffect, useRef, useState, type ReactNode } from "react";
import { useConnectionSettings } from "../ConnectionSettingsProvider";
import { ros2Bridge } from "../../ros2-bridge";
import {
  formatDuration,
  useVacuumAdapter,
  type VacuumCommandResult,
  type VacuumMappingStatus,
  type VacuumMissionState,
  type VacuumNavigationState,
  type VacuumSavedMapSummary,
} from "../../vacuum-adapter";
import {
  MapCanvas,
  type MapCanvasMetadata,
  type MapCanvasTarget,
  type MappingSessionState,
  type RouteVisualState,
} from "./MapCanvas";
import { TeleopCard } from "./TeleopCard";
import "./VacuumControlPanel.css";

type DraftTarget = MapCanvasTarget;

type OperatorTone = "ready" | "warning" | "success" | "danger" | "info";
type StatusChipTone = "success" | "active" | "inactive";
type SavedMapSummary = {
  name: string;
  createdAt: number;
  metadata: MapCanvasMetadata;
  notes: string;
};

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

type OperatorStateKey =
  | "disconnected"
  | "waiting-map"
  | "waiting-localization"
  | "checking"
  | "ready"
  | "mapping"
  | "navigating"
  | "completed"
  | "failed"
  | "canceled";

type OperatorState = {
  key: OperatorStateKey;
  title: string;
  detail: string;
  badge: string;
  tone: OperatorTone;
};

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

function getOperatorState(args: {
  availability: "online" | "connecting" | "offline";
  mapReady: boolean;
  poseAvailable: boolean;
  preflightReady: boolean;
  navigationState: VacuumNavigationState;
  missionState: VacuumMissionState;
  isCanceling: boolean;
  targetSelected: boolean;
}): OperatorState {
  if (args.availability !== "online") {
    return {
      key: "disconnected",
      title: args.availability === "connecting" ? "Connecting" : "Disconnected",
      detail: "Connection needed.",
      badge: args.availability === "connecting" ? "Connecting" : "Offline",
      tone: "warning",
    };
  }

  if (!args.mapReady) {
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

  if (args.missionState === "navigating") {
    const stopping = args.isCanceling || args.navigationState === "canceling";
    return {
      key: "navigating",
      title: stopping ? "Stopping" : "On the way",
      detail: stopping ? "Stopping this run." : "Moving to the selected destination.",
      badge: stopping ? "Stopping" : "Active",
      tone: "info",
    };
  }

  if (args.missionState === "mapping") {
    return {
      key: "mapping",
      title: "Mapping",
      detail: "Mapping workflow active.",
      badge: "Mapping",
      tone: "info",
    };
  }

  if (args.navigationState === "completed") {
    return {
      key: "completed",
      title: "Completed",
      detail: "Destination reached.",
      badge: "Done",
      tone: "success",
    };
  }

  if (args.navigationState === "canceled") {
    return {
      key: "canceled",
      title: "Canceled",
      detail: "Run stopped.",
      badge: "Stopped",
      tone: "warning",
    };
  }

  if (args.navigationState === "failed" || args.navigationState === "unknown") {
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
  if (navigationState === "failed" || navigationState === "unknown") {
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

function StateIcon(props: { className?: string; stateKey: OperatorStateKey }) {
  if (props.stateKey === "ready") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
        <path d="m5 12.5 4.2 4.2L19 7" />
      </svg>
    );
  }

  if (props.stateKey === "waiting-map" || props.stateKey === "waiting-localization" || props.stateKey === "checking") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
        <circle cx="12" cy="12" r="8" />
        <path d="M12 8v4l2.5 2.5" />
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

function CollapsibleGroup(props: {
  title: string;
  status?: string;
  defaultExpanded?: boolean;
  children: ReactNode;
}): JSX.Element {
  const [isExpanded, setIsExpanded] = useState(props.defaultExpanded ?? true);

  return (
    <section className="vacuum-card-group">
      <button
        className="vacuum-card-group__head"
        type="button"
        onClick={() => { setIsExpanded((value) => !value); }}
        aria-expanded={isExpanded}
      >
        <span className="vacuum-card-group__title">{props.title}</span>
        <span className="vacuum-card-group__right">
          {props.status ? <span className="vacuum-card-group__status">{props.status}</span> : null}
          <ChevronIcon className="vacuum-card-group__chevron" expanded={isExpanded} />
        </span>
      </button>
      {isExpanded ? <div className="vacuum-card-group__body">{props.children}</div> : null}
    </section>
  );
}

function CardGroup(props: {
  title: string;
  status?: string;
  children: ReactNode;
}): JSX.Element {
  return (
    <section className="vacuum-card-group">
      <div className="vacuum-card-group__head vacuum-card-group__head--static">
        <span className="vacuum-card-group__title">{props.title}</span>
        {props.status ? (
          <span className="vacuum-card-group__right">
            <span className="vacuum-card-group__status">{props.status}</span>
          </span>
        ) : null}
      </div>
      <div className="vacuum-card-group__body">{props.children}</div>
    </section>
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
  const goalStartTimeRef = useRef<number | null>(null);

  const currentPose = snapshot.pose.coordinates;
  const availability = snapshot.availability.status;
  const mapReady = snapshot.map.readiness === "ready";
  const isMapReceiving = snapshot.map.receiving;
  const poseReady = snapshot.pose.available;
  const readinessReady = snapshot.readiness.ready;
  const preflightBlocking = snapshot.readiness.blockingReasons.some(
    (reason) => reason === "Navigation checks are not ready.",
  );
  const preflightReady = !preflightBlocking;
  const navigationState = snapshot.navigation.state;
  const missionState = snapshot.mission.state;
  const isGoalActive = snapshot.navigation.active;
  const isSendingGoal = snapshot.navigation.isSending;
  const isCancelingGoal = snapshot.navigation.isCanceling;
  const goToLocationSupported = snapshot.capabilities.go_to_location.supported;
  const cancelNavigationSupported = snapshot.capabilities.cancel_navigation.supported;
  const mappingStatus = snapshot.mapping;
  const mappingState = mapAdapterMappingState(mappingStatus.state);
  const autoMappingSupported = snapshot.capabilities.auto_mapping.supported;
  const mappingSessionSupported = snapshot.capabilities.mapping_session.supported;

  const displayedTarget = sentTarget ?? draftTarget;
  const hasTarget = displayedTarget != null;
  const routeVisualState = getRouteVisualState(navigationState, isGoalActive, draftTarget != null, hasTarget);
  const displayedPlanPoints =
    sentTarget != null && routeVisualState !== "staged" && routeVisualState !== "canceled"
      ? snapshot.navigation.planPath
      : null;

  const operatorState = getOperatorState({
    availability,
    mapReady,
    poseAvailable: poseReady,
    preflightReady,
    navigationState,
    missionState,
    isCanceling: isCancelingGoal,
    targetSelected: hasTarget,
  });

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
  const progressPctLabel = isIndeterminate ? "—" : `${Math.round(routeProgress * 100)}%`;
  const elapsedLabel = snapshot.navigation.progress.navigationTime != null
    ? formatDuration(snapshot.navigation.progress.navigationTime)
    : wallClockElapsed != null
      ? formatDuration(wallClockElapsed)
      : "n/a";
  const recoveryCount = toFiniteNumber(snapshot.navigation.progress.recoveries);
  const recoveryLabel = formatRecoveries(recoveryCount);
  const recoveryWarning = recoveryCount != null && recoveryCount > 0;
  const isMappingWorkflowActive =
    mappingState === "mapping" || mappingState === "paused" || mappingState === "review";

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
    Boolean(runTarget) && goToLocationSupported && readinessReady && !isSendingGoal;
  const canCancelRun = cancelNavigationSupported && !isCancelingGoal;

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
    if (isMappingWorkflowActive) {
      return;
    }
    setSentTarget(null);
    setDraftTarget(target);
  }

  function handleTargetRotate(yaw: number): void {
    if (isMappingWorkflowActive) {
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
            <span className="vacuum-header__breadcrumb">Navigation</span>
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
            routeVisualState={routeVisualState}
            isGoalActive={isGoalActive}
            mappingState={mappingState}
            disableTargetSelection={isMappingWorkflowActive}
            targetDistance={destinationDistance}
            adapterMapGrid={snapshot.map.grid}
            adapterMapMetadata={snapshot.map.metadata}
            onTargetStart={handleTargetStart}
            onTargetRotate={handleTargetRotate}
            onMapMetadataChange={setMapMetadata}
          />

          <div className="vacuum-sidebar">

            <section className={`vacuum-panel-card vacuum-panel-card--state vacuum-panel-card--${operatorState.tone}`} aria-live="polite">
              <div className="vacuum-panel-card__head">
                <p className="vacuum-panel-card__eyebrow">Current State</p>
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
              <div className="vacuum-readiness-grid">
                <div className={`vacuum-readiness-item ${availability === "online" ? "vacuum-readiness-item--ready" : ""}`}>
                  <span>Connection</span>
                  <strong>{availability === "online" ? "Online" : availability === "connecting" ? "Connecting" : "Offline"}</strong>
                </div>
                <div className={`vacuum-readiness-item ${isMapReceiving ? "vacuum-readiness-item--ready" : ""}`}>
                  <span>Map</span>
                  <strong>{isMapReceiving ? "Live" : "Waiting"}</strong>
                </div>
                <div className={`vacuum-readiness-item ${poseReady ? "vacuum-readiness-item--ready" : ""}`}>
                  <span>Position</span>
                  <strong>{poseReady ? "Known" : "Settling"}</strong>
                </div>
              </div>
            </section>

            <CardGroup title="Mapping & Manual Control" status={mappingStatus.state === "auto_mapping" ? "Auto mapping" : "Ready"}>
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
            </CardGroup>

            <CollapsibleGroup title="Destination Run" status={hasTarget ? destinationBadgeLabel : "No target"}>
              <section className={`vacuum-panel-card vacuum-panel-card--destination ${displayedTarget ? "vacuum-panel-card--destination-selected" : ""}`}>
                <div className="vacuum-panel-card__head">
                  <p className="vacuum-panel-card__eyebrow">Selected Destination</p>
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
                        <p className="vacuum-dest-row__sub">{targetDistanceLabel} from robot</p>
                      </div>
                    </div>
                    <div className="vacuum-destination-details">
                      <div>
                        <span>Facing</span>
                        <strong>{targetHeadingLabel}</strong>
                      </div>
                      <div>
                        <span>Bearing</span>
                        <strong>{targetBearingLabel}</strong>
                      </div>
                      <div>
                        <span>Map coords</span>
                        <strong>
                          {formatCoordinate(displayedTarget.x)}, {formatCoordinate(displayedTarget.y)}
                        </strong>
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
                  <div className="vacuum-progress-summary">
                    <strong className={`vacuum-progress-pct vacuum-progress-pct--${routeVisualState}${isIndeterminate ? " vacuum-progress-pct--indeterminate" : ""}`}>
                      {progressPctLabel}
                    </strong>
                    <p className="vacuum-progress-label">{progressLabel}</p>
                  </div>
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
                <div className="vacuum-panel-card__head">
                  <p className="vacuum-panel-card__eyebrow">Actions</p>
                </div>
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
            </CollapsibleGroup>

          </div>
        </section>
      </main>
    </div>
  );
}
