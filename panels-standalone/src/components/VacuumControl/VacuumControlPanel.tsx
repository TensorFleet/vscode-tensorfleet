import { useEffect, useMemo, useRef, useState } from "react";
import { useConnectionSettings } from "../ConnectionSettingsProvider";
import { formatRosDuration } from "../Nav2/runtime/nav2RuntimeUtils";
import {
  useVacuumAdapter,
  type VacuumMissionState,
  type VacuumNavigationState,
} from "../../vacuum-adapter";
import { MapCanvas, type MapCanvasTarget, type RouteVisualState } from "./MapCanvas";
import { TeleopCard } from "./TeleopCard";
import "./VacuumControlPanel.css";

type DraftTarget = MapCanvasTarget;

type OperatorTone = "ready" | "warning" | "success" | "danger" | "info";
type StatusChipTone = "success" | "active" | "inactive";

type OperatorStateKey =
  | "disconnected"
  | "waiting-map"
  | "waiting-localization"
  | "checking"
  | "ready"
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
  const adapter = useVacuumAdapter();
  const snapshot = adapter.snapshot;
  const { openOverlay } = useConnectionSettings();
  const [draftTarget, setDraftTarget] = useState<DraftTarget | null>(null);
  const [sentTarget, setSentTarget] = useState<DraftTarget | null>(null);
  const [wallClockElapsed, setWallClockElapsed] = useState<number | null>(null);
  const goalStartTimeRef = useRef<number | null>(null);

  const currentPose = snapshot.pose.coordinates;
  const availability = snapshot.availability.status;
  const mapReady = snapshot.map.readiness === "ready";
  const isMapReceiving = snapshot.map.receiving;
  const poseReady = snapshot.pose.available;
  const readinessReady = snapshot.readiness.ready;
  const preflightBlocking = snapshot.readiness.blockingReasons.some(
    (reason) => reason === "Nav2 preflight checks are not ready.",
  );
  const preflightReady = !preflightBlocking;
  const navigationState = snapshot.navigation.state;
  const missionState = snapshot.mission.state;
  const isGoalActive = snapshot.navigation.active;
  const isSendingGoal = snapshot.navigation.isSending;
  const isCancelingGoal = snapshot.navigation.isCanceling;
  const goToLocationSupported = snapshot.capabilities.go_to_location.supported;
  const cancelNavigationSupported = snapshot.capabilities.cancel_navigation.supported;

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
    ? formatRosDuration(snapshot.navigation.progress.navigationTime)
    : wallClockElapsed != null
      ? formatRosDuration(wallClockElapsed)
      : "n/a";
  const recoveryCount = toFiniteNumber(snapshot.navigation.progress.recoveries);
  const recoveryLabel = formatRecoveries(recoveryCount);
  const recoveryWarning = recoveryCount != null && recoveryCount > 0;

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
    setSentTarget(null);
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
            targetDistance={destinationDistance}
            onTargetStart={handleTargetStart}
            onTargetRotate={handleTargetRotate}
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
                    <p className="vacuum-dest-row__sub">Click the map to pick one</p>
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

            <TeleopCard />

          </div>
        </section>
      </main>
    </div>
  );
}
