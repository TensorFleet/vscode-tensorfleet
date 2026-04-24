import React, { useState } from "react";
import { useConnectionSettings } from "../ConnectionSettingsProvider";
import { useNav2Runtime } from "../Nav2/runtime/useNav2Runtime";
import { formatRosDuration } from "../Nav2/runtime/nav2RuntimeUtils";
import type { GoalState, TopicHealth } from "../Nav2/runtime/nav2RuntimeTypes";
import { MapCanvas, type MapCanvasTarget, type RouteVisualState } from "./MapCanvas";
import "./VacuumControlPanel.css";

type DraftTarget = MapCanvasTarget;

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

const ACTIVE_GOAL_STATES = new Set<GoalState>(["sending", "accepted", "executing", "canceling"]);

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

function formatEta(value: unknown): string {
  const numeric = toFiniteNumber(value);
  return numeric != null ? formatRosDuration(numeric) : formatRosDuration(value);
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
  const [draftTarget, setDraftTarget] = useState<DraftTarget | null>(null);
  const [sentTarget, setSentTarget] = useState<DraftTarget | null>(null);
  const [initialRouteDistance, setInitialRouteDistance] = useState<number | null>(null);
  const currentPose = runtime.currentMapCoordinates;
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

  function handleTargetStart(target: DraftTarget): void {
    setSentTarget(null);
    setInitialRouteDistance(null);
    setDraftTarget(target);
  }

  function handleTargetRotate(yaw: number): void {
    setDraftTarget((current) => {
      if (!current) {
        return current;
      }
      return { ...current, yaw };
    });
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
          <MapCanvas
            currentPose={currentPose}
            planMessage={runtime.planMessage}
            draftTarget={draftTarget}
            sentTarget={sentTarget}
            routeVisualState={routeVisualState}
            isGoalActive={isGoalActive}
            onTargetStart={handleTargetStart}
            onTargetRotate={handleTargetRotate}
          />

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
