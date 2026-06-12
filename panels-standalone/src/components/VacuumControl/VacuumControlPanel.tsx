import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useConnectionSettings } from "../ConnectionSettingsProvider";
import { ros2Bridge } from "tensorfleet-ros";
import {
  formatDuration,
  normalizeVacuumAdapterBackend,
  readConfiguredVacuumAdapterBackend,
  useVacuumAdapter,
  VACUUM_ADAPTER_BACKENDS,
  deriveVacuumPrimaryRobotState,
  type CapabilitySupport,
  type VacuumCommandResult,
  type VacuumAttachmentState,
  type VacuumAttachmentsState,
  type VacuumAdapterBackendId,
  type VacuumAvailability,
  type VacuumBatteryState,
  type VacuumCapabilities,
  type VacuumCleaningSettingState,
  type VacuumCleaningSettingsState,
  type VacuumDockComponentState,
  type VacuumDockStatus,
  type VacuumFaultState,
  type VacuumLayeredMapPreview,
  type VacuumMapEntity,
  type VacuumMapLayer,
  type VacuumMaintenanceState,
  type VacuumRobotActivity,
  type VacuumRuntimeHealth,
  type VacuumSourceState,
  type VacuumStatisticsState,
  type VacuumMapTarget,
  type VacuumMapTargets,
  type VacuumMapAnnotation,
  type VacuumMapAnnotationKind,
  type VacuumMappingStatus,
  type VacuumLegacyMissionStatus,
  type VacuumMissionSnapshot,
  type VacuumNavigationState,
  type VacuumPoseCoordinates,
  type VacuumPrimaryRobotStateSummary,
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
  type CleanAreaCoverageOverlayCell,
  type CleanAreaCoverageTarget,
  type CleanAreaCoverageSnapshot,
} from "./cleanAreaCoverage";
import { buildLawnmowerWaypoints } from "./cleanAreaPlanner";
import {
  DEFAULT_CLEAN_AREA_COVERAGE_PROFILE,
  buildCleanAreaCoverageRuntimeConfig,
  type CleanAreaCoverageRuntimeConfig,
} from "./cleanAreaProfile";
import "./VacuumControlPanel.css";

type DraftTarget = MapCanvasTarget;
type VacuumControlMode = "mapping" | "navigation" | "clean" | "rooms";
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
type RoomZoneTargetStatus = "none" | "cleanable" | "partial" | "invalid";

type StatusChipTone = "success" | "active" | "inactive";
type SavedMapSummary = {
  name: string;
  createdAt: number;
  metadata: MapCanvasMetadata;
  notes: string;
};

function navigationDestinationDismissKey(
  target: DraftTarget | null | undefined,
  missionId: string | null | undefined,
  missionStatus: string | null | undefined,
  updatedAt: number | null | undefined,
): string | null {
  if (!target) {
    return null;
  }
  return [
    missionId ?? "navigation",
    missionStatus ?? "unknown",
    updatedAt ?? "no-update",
    target.x.toFixed(3),
    target.y.toFixed(3),
    target.yaw.toFixed(3),
  ].join(":");
}

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

function formatCurrentStatisticsDuration(value: number | null | undefined): string {
  if (typeof value !== "number" || !Number.isFinite(value) || value < 0) {
    return "n/a";
  }
  const totalSeconds = Math.round(value);
  const hours = Math.floor(totalSeconds / 3600);
  const minutes = Math.floor((totalSeconds % 3600) / 60);
  const seconds = totalSeconds % 60;
  const paddedMinutes = String(minutes).padStart(hours > 0 ? 2 : 1, "0");
  const paddedSeconds = String(seconds).padStart(2, "0");
  return hours > 0 ? `${String(hours).padStart(2, "0")}h ${paddedMinutes}m ${paddedSeconds}s` : `${paddedMinutes}m ${paddedSeconds}s`;
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

function formatMissionTime(timestamp: number | null): string {
  if (timestamp == null) {
    return "unknown";
  }
  return new Date(timestamp).toLocaleString();
}

function getMissionDisplayName(mission: VacuumMissionSnapshot): string {
  const target = missionTargetRecord(mission);
  const annotation = target?.annotation && typeof target.annotation === "object" ? (target.annotation as Record<string, unknown>) : null;
  const annotationName = typeof annotation?.name === "string" ? annotation.name : null;
  if (annotationName) {
    return annotationName;
  }
  if (mission.type === "room_cleaning" || mission.requestedCommand === "start_room_cleaning") {
    return "Room cleaning";
  }
  if (mission.type === "zone_cleaning" || mission.requestedCommand === "start_zone_cleaning") {
    return "Zone cleaning";
  }
  if (mission.type === "coverage") {
    return "Area cleaning";
  }
  if (mission.type === "navigation") {
    return "Navigation";
  }
  if (mission.type === "mapping") {
    return "Mapping";
  }
  return mission.type.replace(/_/g, " ");
}

function getMissionResultLabel(mission: VacuumMissionSnapshot): string {
  const resultStatus = mission.result?.status ?? (isTerminalMissionStatus(mission.status) ? mission.status : null);
  if (resultStatus === "completed") {
    const coverage = coverageMissionCoverageRecord(mission) ?? mission.result?.details ?? null;
    if (coverage && (mission.type === "coverage" || isRoomZoneMissionSnapshot(mission))) {
      const remainingAreaSqM = toFiniteNumber(coverage.remainingAreaSqM) ?? mission.progress.areaRemainingSqM ?? 0;
      const skippedAreaSqM = toFiniteNumber(coverage.skippedAreaSqM) ?? 0;
      const progress =
        toFiniteNumber(coverage.progress) ?? toFiniteNumber(coverage.coverageProgress) ?? mission.progress.percent ?? 1;
      const completionThreshold =
        toFiniteNumber(coverage.completionThreshold) ?? toFiniteNumber(coverage.coverageThreshold) ?? 0.95;
      const coverageThresholdReached =
        typeof coverage.coverageThresholdReached === "boolean" ? coverage.coverageThresholdReached : null;
      if (
        remainingAreaSqM > 0.01 ||
        skippedAreaSqM > 0.01 ||
        coverageThresholdReached === false ||
        (coverageThresholdReached !== true && progress < completionThreshold)
      ) {
        return "Partially cleaned";
      }
      return "Cleaned";
    }
    return "Completed";
  }
  if (resultStatus === "failed") {
    return "Failed";
  }
  if (resultStatus === "canceled") {
    return "Canceled";
  }
  if (resultStatus === "unsupported") {
    return "Unsupported";
  }
  return mission.status;
}

function getMissionResultTone(label: string): string {
  return label.toLowerCase().replace(/\s+/g, "-");
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

function cleanAreaRectToCoverageArea(rect: CleanAreaRect) {
  return {
    shape: "rectangle" as const,
    minX: rect.minX,
    minY: rect.minY,
    maxX: rect.maxX,
    maxY: rect.maxY,
  };
}

function cleanAreaRectToAnnotationArea(rect: CleanAreaRect): VacuumMapAnnotation["area"] {
  return cleanAreaRectToCoverageArea(rect);
}

function annotationAreaToCleanAreaRect(area: VacuumMapAnnotation["area"]): CleanAreaRect | null {
  if (area.shape === "rectangle") {
    return {
      minX: area.minX,
      minY: area.minY,
      maxX: area.maxX,
      maxY: area.maxY,
    };
  }
  return null;
}

function currentMapAnnotationId(snapshot: { mapping: VacuumMappingStatus; map: { grid: unknown } }): string | null {
  return snapshot.mapping.activeMapName ?? snapshot.mapping.loadedMapPath ?? snapshot.mapping.savedMapPath ?? (snapshot.map.grid ? "live-map" : null);
}

function isTerminalMissionStatus(status: VacuumMissionSnapshot["status"]): boolean {
  return status === "completed" || status === "failed" || status === "canceled" || status === "unsupported";
}

function isRoomZoneMissionSnapshot(mission: VacuumMissionSnapshot | null | undefined): boolean {
  return (
    mission?.type === "room_cleaning" ||
    mission?.type === "zone_cleaning" ||
    mission?.requestedCommand === "start_room_cleaning" ||
    mission?.requestedCommand === "start_zone_cleaning"
  );
}

const RECENT_ROOM_ZONE_MODE_RESTORE_MAX_AGE_MS = 2 * 60 * 1000;

const RECENT_ROOM_ZONE_DISMISSED_AT_STORAGE_KEY =
  "tensorfleet:vacuums:vacuum-control:recent-room-zone-missions-dismissed-at";

const ADAPTER_BACKEND_STORAGE_KEY = "tensorfleet:vacuums:adapter-backend";
const ADAPTER_BACKEND_OPTIONS = Object.values(VACUUM_ADAPTER_BACKENDS) as VacuumAdapterBackendId[];

function writeSelectedAdapterBackend(backend: VacuumAdapterBackendId): void {
  if (typeof window === "undefined") {
    return;
  }
  try {
    window.localStorage.setItem(ADAPTER_BACKEND_STORAGE_KEY, backend);
    (window as unknown as { TENSORFLEET_VACUUM_BACKEND?: VacuumAdapterBackendId }).TENSORFLEET_VACUUM_BACKEND = backend;
  } catch {
    // Selection still applies for the current React state even if localStorage is unavailable.
  }
}

function readDismissedRoomZoneMissionTime(): number | null {
  if (typeof window === "undefined") {
    return null;
  }
  try {
    const raw = window.localStorage.getItem(RECENT_ROOM_ZONE_DISMISSED_AT_STORAGE_KEY);
    if (!raw) {
      return null;
    }
    const parsed = Number(raw);
    return Number.isFinite(parsed) ? parsed : null;
  } catch {
    return null;
  }
}

function writeDismissedRoomZoneMissionTime(value: number | null): void {
  if (typeof window === "undefined") {
    return;
  }
  try {
    if (value == null) {
      window.localStorage.removeItem(RECENT_ROOM_ZONE_DISMISSED_AT_STORAGE_KEY);
    } else {
      window.localStorage.setItem(RECENT_ROOM_ZONE_DISMISSED_AT_STORAGE_KEY, String(value));
    }
  } catch {
    // Best-effort persistence; a failed write only affects reload behavior.
  }
}

function nextAnnotationName(kind: VacuumMapAnnotationKind, annotations: VacuumMapAnnotation[]): string {
  const prefix = kind === "room" ? "Room" : "Zone";
  const used = new Set<number>();
  for (const ann of annotations) {
    if (ann.kind === kind) {
      const match = /^(?:Room|Zone) (\d+)$/.exec(ann.name);
      if (match) {
        used.add(Number(match[1]));
      }
    }
  }
  let n = 1;
  while (used.has(n)) n++;
  return `${prefix} ${n}`;
}

function missionTerminalTime(mission: VacuumMissionSnapshot | null | undefined): number | null {
  return mission?.result?.completedAt ?? mission?.updatedAt ?? mission?.startedAt ?? null;
}

function mapCoverageMissionState(mission: VacuumMissionSnapshot | null): CleanAreaMissionState | null {
  if (!mission) {
    return null;
  }
  if (mission.status === "preparing" || mission.status === "resuming") {
    return "preparing";
  }
  if (mission.status === "running") {
    return "running";
  }
  if (mission.status === "paused") {
    return "paused";
  }
  if (mission.status === "needs_assistance") {
    return "failed";
  }
  if (mission.status === "canceling") {
    return "canceling";
  }
  if (mission.status === "completed") {
    return "completed";
  }
  if (mission.status === "canceled") {
    return "canceled";
  }
  if (mission.status === "failed" || mission.status === "unsupported") {
    return "failed";
  }
  return null;
}

function missionTargetRecord(mission: VacuumMissionSnapshot | null): Record<string, unknown> | null {
  return mission?.target && typeof mission.target === "object" ? (mission.target as Record<string, unknown>) : null;
}

function coverageMissionArea(mission: VacuumMissionSnapshot | null): CleanAreaRect | null {
  const target = missionTargetRecord(mission);
  const rawArea = target?.area && typeof target.area === "object" ? (target.area as Record<string, unknown>) : target;
  if (!rawArea || rawArea.shape !== "rectangle") {
    return null;
  }
  const minX = toFiniteNumber(rawArea.minX);
  const minY = toFiniteNumber(rawArea.minY);
  const maxX = toFiniteNumber(rawArea.maxX);
  const maxY = toFiniteNumber(rawArea.maxY);
  if (minX == null || minY == null || maxX == null || maxY == null) {
    return null;
  }
  return { minX, minY, maxX, maxY };
}

function coverageMissionRoute(mission: VacuumMissionSnapshot | null): DraftTarget[] {
  const target = missionTargetRecord(mission);
  const route = Array.isArray(target?.route) ? target.route : [];
  return route.flatMap((entry): DraftTarget[] => {
    if (!entry || typeof entry !== "object") {
      return [];
    }
    const record = entry as Record<string, unknown>;
    const x = toFiniteNumber(record.x);
    const y = toFiniteNumber(record.y);
    const yaw = toFiniteNumber(record.yaw) ?? 0;
    return x == null || y == null ? [] : [{ x, y, yaw }];
  });
}

function coverageMissionCoverageRecord(mission: VacuumMissionSnapshot | null): Record<string, unknown> | null {
  const target = missionTargetRecord(mission);
  return target?.coverage && typeof target.coverage === "object" ? (target.coverage as Record<string, unknown>) : null;
}

function coverageMissionOverlayCells(mission: VacuumMissionSnapshot | null): CleanAreaCoverageOverlayCell[] {
  const coverage = coverageMissionCoverageRecord(mission);
  const overlayCells = Array.isArray(coverage?.overlayCells) ? coverage.overlayCells : [];
  return overlayCells.flatMap((entry): CleanAreaCoverageOverlayCell[] => {
    if (!entry || typeof entry !== "object") {
      return [];
    }
    const record = entry as Record<string, unknown>;
    const key = typeof record.key === "string" ? record.key : null;
    const cellX = toFiniteNumber(record.cellX);
    const cellY = toFiniteNumber(record.cellY);
    const minX = toFiniteNumber(record.minX);
    const minY = toFiniteNumber(record.minY);
    const maxX = toFiniteNumber(record.maxX);
    const maxY = toFiniteNumber(record.maxY);
    const state = typeof record.state === "string" ? record.state : null;
    if (
      !key ||
      cellX == null ||
      cellY == null ||
      minX == null ||
      minY == null ||
      maxX == null ||
      maxY == null ||
      !["remaining", "covered", "occupied", "unknown", "out_of_bounds", "too_small"].includes(state ?? "")
    ) {
      return [];
    }
    return [
      {
        key,
        cellX,
        cellY,
        minX,
        minY,
        maxX,
        maxY,
        state: state as CleanAreaCoverageOverlayCell["state"],
      },
    ];
  });
}

function buildRuntimeCoverageSnapshot(args: {
  mission: VacuumMissionSnapshot | null;
  target: CleanAreaCoverageTarget | null;
  swathWidth: number;
}): CleanAreaCoverageSnapshot | null {
  if (!args.mission) {
    return null;
  }
  const coverage = coverageMissionCoverageRecord(args.mission);
  if (!coverage || !args.target) {
    return null;
  }
  const base = buildCleanAreaCoverageSnapshot({
    target: args.target,
    coveredCellKeys: new Set(),
    swathWidth: args.swathWidth,
  });
  if (!base) {
    return null;
  }
  const overlayCells = coverageMissionOverlayCells(args.mission);
  const targetCells = toFiniteNumber(coverage.targetCells) ?? base.targetCells;
  const coveredCells = toFiniteNumber(coverage.coveredCells) ?? base.coveredCells;
  const remainingCells = toFiniteNumber(coverage.remainingCells) ?? Math.max(0, targetCells - coveredCells);
  const cleanableAreaSqM = toFiniteNumber(coverage.cleanableAreaSqM) ?? base.cleanableAreaSqM;
  const coveredAreaSqM = toFiniteNumber(coverage.coveredAreaSqM) ?? args.mission.progress.areaCoveredSqM ?? base.coveredAreaSqM;
  const remainingAreaSqM =
    toFiniteNumber(coverage.remainingAreaSqM) ??
    args.mission.progress.areaRemainingSqM ??
    Math.max(0, cleanableAreaSqM - coveredAreaSqM);
  const skippedAreaSqM = toFiniteNumber(coverage.skippedAreaSqM) ?? base.skippedAreaSqM;
  const progress = args.mission.progress.percent == null ? base.progress : clamp(args.mission.progress.percent, 0, 1);
  const swathWidth = toFiniteNumber(coverage.swathWidth) ?? args.swathWidth;

  return {
    ...base,
    swathWidth,
    targetCells,
    coveredCells,
    remainingCells,
    cleanableAreaSqM,
    coveredAreaSqM,
    remainingAreaSqM,
    skippedAreaSqM,
    progress,
    overlayCells: overlayCells.length > 0 ? overlayCells : base.overlayCells,
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
  if (!hasTarget) {
    return "idle";
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

function StatusChipIcon(props: { className?: string; kind: "connected" | "map" | "localized" | "ready" | "target" | "battery" | "dock" }) {
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

  if (props.kind === "battery") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 20 20">
        <rect x="3" y="6" width="12" height="8" rx="1.8" />
        <path d="M15 9h2v2h-2" />
      </svg>
    );
  }

  if (props.kind === "dock") {
    return (
      <svg aria-hidden="true" className={props.className} viewBox="0 0 20 20">
        <path d="M4 15.5h12" />
        <path d="M6 15.5v-7l4-3 4 3v7" />
        <path d="M8.5 15.5v-4h3v4" />
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

function RoomsModeIcon(props: { className?: string }) {
  return (
    <svg aria-hidden="true" className={props.className} viewBox="0 0 24 24">
      <path d="M4 5h16v14H4z" />
      <path d="M12 5v14" />
      <path d="M4 12h8" />
      <path d="M12 10h8" />
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
                  <button
                    className="vacuum-saved-map__main"
                    type="button"
                    onClick={() => props.onLoadMap(savedMap.name)}
                    disabled={!savedMap.loadable}
                    title={savedMap.loadUnavailableReason ?? "Load saved map"}
                  >
                    <span>{savedMap.name}</span>
                    <small>{savedMap.loadable ? formatSavedMapTime(savedMap.modifiedAt) : "Needs pose graph"}</small>
                  </button>
                  <button
                    className="vacuum-saved-map__action"
                    type="button"
                    onClick={() => props.onLoadMap(savedMap.name)}
                    disabled={!savedMap.loadable}
                    title={savedMap.loadUnavailableReason ?? "Use saved map"}
                  >
                    {savedMap.loadable ? "Use" : "Unavailable"}
                  </button>
                  <button
                    className="vacuum-saved-map__action"
                    type="button"
                    onClick={() => props.onImproveMap(savedMap.name)}
                    disabled={!savedMap.loadable}
                    title={savedMap.loadUnavailableReason ?? "Improve saved map"}
                  >
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
  coverageConfig: CleanAreaCoverageRuntimeConfig;
  waypointCount: number;
  currentWaypointIndex: number;
  passCount: number;
  estimatedDistance: number;
  distanceRemaining: number;
  commandError: string | null;
  canStart: boolean;
  canCancel: boolean;
  canPause: boolean;
  canRetry: boolean;
  canSkip: boolean;
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
  const selectedAreaLabel = props.coverage
    ? formatArea(props.coverage.cleanableAreaSqM)
    : props.rect
      ? formatArea(size.area)
      : "n/a";
  const cleanedAreaLabel = props.coverage ? formatArea(props.coverage.coveredAreaSqM) : "n/a";
  const remainingAreaLabel = props.coverage ? formatArea(props.coverage.remainingAreaSqM) : "n/a";
  const skippedAreaLabel = props.coverage ? formatArea(props.coverage.skippedAreaSqM) : "n/a";
  const routeStepLabel = props.waypointCount > 0
    ? `${Math.min(props.currentWaypointIndex + 1, props.waypointCount)} of ${props.waypointCount}`
    : "n/a";
  const routeSummaryLabel =
    props.state === "running" || props.state === "preparing" || props.state === "paused" || props.state === "canceling"
      ? `Step ${routeStepLabel}`
      : props.passCount > 0
        ? `${props.passCount} passes`
        : "Ready";
  const coverageMeetsCompletion =
    !props.coverage ||
    props.coverage.targetCells === 0 ||
    props.coverage.progress >= props.coverageConfig.completionThreshold;
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
      badge: "Starting",
      detail: "Starting the area clean.",
    },
    running: {
      badge: "Cleaning",
      detail: "Cleaning the selected area.",
    },
    paused: {
      badge: "Paused",
      detail: "Cleaning is paused.",
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
      detail: props.commandError ?? "The robot could not finish this area.",
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

      {props.state === "confirmed" ? (
        <div className="vacuum-clean-area-preview">
          <strong>Ready to clean {selectedAreaLabel}</strong>
          <span>{props.coverage ? `${skippedAreaLabel} cannot be cleaned in this selection.` : "Review the highlighted area before starting."}</span>
          {props.coverage ? (
            <span>{routeSummaryLabel} · {props.estimatedDistance > 0 ? formatDistance(props.estimatedDistance) : "route ready"}</span>
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
          <div className="vacuum-clean-area-summary">
            <strong>{formatPercent(progress)}</strong>
            <span>
              {props.coverage
                ? `${cleanedAreaLabel} cleaned · ${remainingAreaLabel} left`
                : `Distance left: ${formatDistance(props.distanceRemaining)}`}
            </span>
          </div>
          {props.coverage ? (
            <div className="vacuum-clean-area-quick-stats">
              <div>
                <span>Cleaned</span>
                <strong>{cleanedAreaLabel}</strong>
              </div>
              <div>
                <span>Left</span>
                <strong>{remainingAreaLabel}</strong>
              </div>
              <div>
                <span>Skipped</span>
                <strong>{skippedAreaLabel}</strong>
              </div>
              <div>
                <span>Status</span>
                <strong>{routeSummaryLabel}</strong>
              </div>
            </div>
          ) : null}
        </>
      ) : null}

      {props.state === "failed" ? (
        <div className="vacuum-clean-area-failure-actions">
          <span>Options</span>
          <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onRetry} disabled={!props.canRetry}>
            Retry waypoint
          </button>
          <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onSkip} disabled={!props.canSkip}>
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

function RoomZonesCard(props: {
  annotations: VacuumMapAnnotation[];
  selectedAnnotation: VacuumMapAnnotation | null;
  targetStatus: RoomZoneTargetStatus;
  targetStatusLabel: string;
  targetDetail: string;
  targetCoverage: CleanAreaCoverageSnapshot | null;
  waypointCount: number;
  passCount: number;
  estimatedDistance: number;
  toolActive: boolean;
  draftKind: VacuumMapAnnotationKind;
  draftName: string;
  draftRect: CleanAreaRect | null;
  validation: CleanAreaValidation | null;
  commandError: string | null;
  roomSemanticsSupported: boolean;
  zoneSemanticsSupported: boolean;
  canStartCleaning: boolean;
  canPauseCleaning: boolean;
  canResumeCleaning: boolean;
  canCancelCleaning: boolean;
  canRetryCleaning: boolean;
  canSkipCleaning: boolean;
  cleaningActive: boolean;
  cleaningState: CleanAreaMissionState | null;
  canSave: boolean;
  saveDisabledReason: string | null;
  cleanDisabledReason: string | null;
  onDraftKindChange: (kind: VacuumMapAnnotationKind) => void;
  onDraftNameChange: (name: string) => void;
  onActivateTool: () => void;
  onSave: () => void;
  onStartCleaning: () => void;
  onPauseCleaning: () => void;
  onResumeCleaning: () => void;
  onCancelCleaning: () => void;
  onRetryCleaning: () => void;
  onSkipCleaning: () => void;
  onSelect: (id: string) => void;
  onDelete: () => void;
  onClearDraft: () => void;
}): JSX.Element {
  const [deleteConfirmPending, setDeleteConfirmPending] = useState(false);
  const selectedId = props.selectedAnnotation?.id ?? null;

  useEffect(() => {
    setDeleteConfirmPending(false);
  }, [selectedId]);

  const selected = props.selectedAnnotation;
  const kindLabel = props.draftKind === "room" ? "room" : "zone";
  const hasDraft = Boolean(props.draftRect) || props.toolActive;
  const rect = props.draftRect ?? (selected?.area.shape === "rectangle" ? selected.area : null);
  const size = getCleanAreaSize(rect);
  const targetCoverage = props.targetCoverage;

  const statusLabel = props.toolActive
    ? "Drawing"
    : props.cleaningActive
      ? props.cleaningState === "paused"
        ? "Paused"
        : props.cleaningState === "failed"
          ? "Needs assistance"
        : props.cleaningState === "canceling"
          ? "Canceling"
          : "Cleaning"
      : props.draftRect
        ? "Draft ready"
        : selected
          ? selected.name
          : "No selection";

  const selectionSummary = selected
    ? `${selected.name} (${selected.kind})`
    : props.draftRect
      ? `${kindLabel} draft`
      : "None";

  return (
    <section className="vacuum-panel-card vacuum-panel-card--room-zones">
      <div className="vacuum-panel-card__head">
        <p className="vacuum-panel-card__eyebrow">Rooms / Zones</p>
        <span className="vacuum-clean-area-badge">{statusLabel}</span>
      </div>

      {selected && !hasDraft ? (
        <p className="vacuum-room-zone-selected-name">{selected.name}</p>
      ) : null}

      {props.commandError ? (
        <div className="vacuum-mapping-error" role="status">
          {props.commandError}
        </div>
      ) : null}

      <div className="vacuum-room-zone-form">
        <div className="vacuum-room-zone-kind" role="group" aria-label="Annotation type">
          <button
            type="button"
            className={props.draftKind === "room" ? "vacuum-room-zone-kind__button vacuum-room-zone-kind__button--active" : "vacuum-room-zone-kind__button"}
            onClick={() => props.onDraftKindChange("room")}
            disabled={!props.roomSemanticsSupported}
          >
            Room
          </button>
          <button
            type="button"
            className={props.draftKind === "zone" ? "vacuum-room-zone-kind__button vacuum-room-zone-kind__button--active" : "vacuum-room-zone-kind__button"}
            onClick={() => props.onDraftKindChange("zone")}
            disabled={!props.zoneSemanticsSupported}
          >
            Zone
          </button>
        </div>
        <label className="vacuum-map-name-field">
          <span>Name</span>
          <input
            value={props.draftName}
            onChange={(event) => props.onDraftNameChange(event.target.value)}
            placeholder={props.draftKind === "room" ? "Room 1" : "Zone 1"}
          />
        </label>
      </div>

      <div className="vacuum-clean-area-coverage-grid">
        <div>
          <span>Selection</span>
          <strong>{selectionSummary}</strong>
        </div>
        <div>
          <span>Draft</span>
          <strong>{props.validation?.message ?? (selected ? "Saved annotation." : "Waiting")}</strong>
        </div>
        <div>
          <span>Size</span>
          <strong>{formatDimension(size.width)} × {formatDimension(size.height)}</strong>
        </div>
        <div>
          <span>Saved</span>
          <strong>{props.annotations.length}</strong>
        </div>
      </div>

      {selected ? (
        <div className={`vacuum-room-zone-target vacuum-room-zone-target--${props.targetStatus}`}>
          <div className="vacuum-room-zone-target__head">
            <strong>{props.targetStatusLabel}</strong>
            <span>{selected.kind === "room" ? "Room target" : "Zone target"}</span>
          </div>
          <p>{props.targetDetail}</p>
          {targetCoverage ? (
            <div className="vacuum-clean-area-quick-stats">
              <div>
                <span>Cleanable</span>
                <strong>{formatArea(targetCoverage.cleanableAreaSqM)}</strong>
              </div>
              <div>
                <span>Skipped</span>
                <strong>{formatArea(targetCoverage.skippedAreaSqM)}</strong>
              </div>
              <div>
                <span>Route</span>
                <strong>{props.passCount > 0 ? `${props.passCount} passes` : "n/a"}</strong>
              </div>
              <div>
                <span>Distance</span>
                <strong>{props.estimatedDistance > 0 ? formatDistance(props.estimatedDistance) : "n/a"}</strong>
              </div>
            </div>
          ) : null}
          {props.waypointCount > 0 ? (
            <span className="vacuum-room-zone-target__route">{props.waypointCount} preview waypoints</span>
          ) : null}
        </div>
      ) : null}

      {props.annotations.length > 0 ? (
        <div className="vacuum-room-zone-list">
          {props.annotations.map((annotation) => (
            <button
              key={annotation.id}
              type="button"
              className={`vacuum-room-zone-list__item${annotation.id === selected?.id ? " vacuum-room-zone-list__item--active" : ""}`}
              onClick={() => props.onSelect(annotation.id)}
            >
              <span>{annotation.name}</span>
              <small>{annotation.kind}</small>
            </button>
          ))}
        </div>
      ) : (
        <p className="vacuum-clean-area-copy">No saved rooms or zones on this map.</p>
      )}

      <div className="vacuum-actions">
        {/* Drawing / draft phase */}
        {hasDraft ? (
          <>
            <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onActivateTool}>
              {props.toolActive ? "Finish shape" : "Redraw"}
            </button>
            <button
              className="vacuum-action vacuum-action--primary"
              type="button"
              onClick={props.onSave}
              disabled={!props.canSave}
              title={props.saveDisabledReason ?? undefined}
            >
              Save {kindLabel}
            </button>
            {props.saveDisabledReason ? (
              <p className="vacuum-action-hint vacuum-action-hint--disabled">{props.saveDisabledReason}</p>
            ) : null}
            <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onClearDraft}>
              Clear draft
            </button>
          </>
        ) : selected && props.cleaningActive ? (
          /* Active cleaning controls */
          <>
            {props.cleaningState === "failed" ? (
              <>
                <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onRetryCleaning} disabled={!props.canRetryCleaning}>
                  Retry step
                </button>
                <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onSkipCleaning} disabled={!props.canSkipCleaning}>
                  Skip step
                </button>
              </>
            ) : props.cleaningState === "paused" ? (
              <button className="vacuum-action vacuum-action--primary" type="button" onClick={props.onResumeCleaning} disabled={!props.canResumeCleaning}>
                Resume cleaning
              </button>
            ) : (
              <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onPauseCleaning} disabled={!props.canPauseCleaning}>
                Pause cleaning
              </button>
            )}
            <button className="vacuum-action vacuum-action--danger" type="button" onClick={props.onCancelCleaning} disabled={!props.canCancelCleaning}>
              Cancel cleaning
            </button>
          </>
        ) : selected ? (
          /* Saved annotation selected, not cleaning */
          <>
            <button
              className="vacuum-action vacuum-action--primary"
              type="button"
              onClick={props.onStartCleaning}
              disabled={!props.canStartCleaning}
              title={props.cleanDisabledReason ?? undefined}
            >
              Clean {selected.name}
            </button>
            {props.cleanDisabledReason ? (
              <p className="vacuum-action-hint vacuum-action-hint--disabled">{props.cleanDisabledReason}</p>
            ) : null}
            <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onActivateTool}>
              Draw area
            </button>
            {deleteConfirmPending ? (
              <>
                <button
                  className="vacuum-action vacuum-action--danger"
                  type="button"
                  onClick={() => { setDeleteConfirmPending(false); props.onDelete(); }}
                >
                  Confirm delete
                </button>
                <button
                  className="vacuum-action vacuum-action--ghost"
                  type="button"
                  onClick={() => setDeleteConfirmPending(false)}
                >
                  Keep it
                </button>
              </>
            ) : (
              <>
                <button
                  className="vacuum-action vacuum-action--ghost"
                  type="button"
                  onClick={() => setDeleteConfirmPending(true)}
                >
                  Delete {selected.kind}
                </button>
                <button className="vacuum-action vacuum-action--ghost" type="button" onClick={props.onClearDraft}>
                  Clear selection
                </button>
              </>
            )}
          </>
        ) : (
          /* No selection, no draft */
          <button className="vacuum-action vacuum-action--primary" type="button" onClick={props.onActivateTool}>
            Draw area
          </button>
        )}
      </div>
    </section>
  );
}

function capabilityLabel(capability: CapabilitySupport): string {
  if (capability.supported) {
    return "Available";
  }
  return capability.notes ?? "Unsupported";
}

function formatBatteryState(battery: VacuumBatteryState): string {
  if (battery.percentage == null) {
    return battery.detail ?? "Battery state unavailable";
  }
  const charging = battery.charging == null ? "" : battery.charging ? " · charging" : " · not charging";
  return `${Math.round(battery.percentage)}%${charging}`;
}

function formatBatteryLevel(battery: VacuumBatteryState): string | null {
  return battery.percentage == null ? null : `${Math.round(battery.percentage)}%`;
}

function humanizeStatus(value: string | null | undefined): string {
  if (!value) {
    return "Unknown";
  }
  return value
    .replace(/_/g, " ")
    .replace(/\b\w/g, (letter) => letter.toUpperCase());
}

function humanizeReasonCode(value: string | null | undefined): string | null {
  if (!value) {
    return null;
  }
  const knownReasons: Record<string, string> = {
    invalid_state: "Robot state does not allow this action.",
    source_stale: "Robot state is stale.",
    stale_source: "Robot state is stale.",
    stale: "Robot state is stale.",
    source_unreachable: "Source unreachable.",
    unreachable: "Source unreachable.",
    runtime_offline: "Runtime offline.",
    degraded_runtime: "Runtime degraded.",
    offline: "Runtime offline.",
    unsupported: "Not supported by this backend.",
    unsupported_command: "Not supported by this backend.",
    capability_unavailable: "Not supported by this backend.",
    unavailable: "Currently unavailable.",
    not_supported: "Not supported by this backend.",
    invalid_request: "Invalid command request.",
    invalid_json: "Invalid command payload.",
    missing_command: "Missing command.",
    source_command_failed: "Backend command failed.",
    backend_error: "Backend command failed.",
  };
  return knownReasons[value] ?? `${value.replace(/_/g, " ")}.`;
}

function formatTimestamp(value: number | string | null | undefined): string {
  if (value == null || value === "") {
    return "n/a";
  }
  const timestamp = typeof value === "number" ? value : Date.parse(value);
  if (!Number.isFinite(timestamp)) {
    return String(value);
  }
  return new Date(timestamp).toLocaleString();
}

function formatCapabilityReason(capability: CapabilitySupport, _command?: "start_cleaning" | "pause" | "stop" | "return_to_dock"): string | null {
  const structuredReason = capability.reasons?.[0]?.message ?? null;
  const reasonCode = capability.reasons?.[0]?.code ?? capability.availabilityReason ?? null;
  const readableReason = structuredReason ?? humanizeReasonCode(reasonCode);
  if (!capability.supported) {
    return readableReason ?? capability.notes ?? "Not supported by this backend.";
  }
  if (capability.available === false) {
    return readableReason ?? capability.notes ?? "Currently unavailable.";
  }
  if (capability.status === "unavailable") {
    return readableReason ?? capability.notes ?? "Currently unavailable.";
  }
  if (capability.status === "detected_not_ready") {
    return structuredReason ?? capability.notes ?? "Detected, but this product workflow is not implemented yet.";
  }
  return null;
}

function getActivityTone(activity: VacuumRobotActivity | undefined): "ready" | "warning" | "danger" | "muted" {
  if (!activity) return "muted";
  if (activity.status === "faulted" || activity.status === "unavailable") return "danger";
  if (activity.status === "paused" || activity.status === "unknown") return "warning";
  if (activity.status === "idle" || activity.status === "docked" || activity.status === "charging") return "ready";
  return "warning";
}

function getDockTone(dock: VacuumDockStatus | undefined): "ready" | "warning" | "danger" | "muted" {
  if (!dock?.state) return "muted";
  if (dock.state === "docked" || dock.state === "charging") return "ready";
  if (dock.state === "returning") return "warning";
  if (dock.state === "error") return "danger";
  return "muted";
}

function getBasicActivityLabel(activity: VacuumRobotActivity | undefined): string {
  if (activity?.status === "returning") {
    return "Returning to dock";
  }
  return humanizeStatus(activity?.status ?? "unknown");
}

function formatDockBatterySummary(
  activity: VacuumRobotActivity | undefined,
  dock: VacuumDockStatus | undefined,
  battery: VacuumBatteryState,
  options: { omitDockWhenActivityDuplicates?: boolean } = {},
): string {
  const batteryLevel = formatBatteryLevel(battery);
  const dockLabel = dock?.state && dock.state !== "unknown" ? humanizeStatus(dock.state) : null;
  const activityStatus = activity?.status;
  const dockDuplicatesActivity =
    options.omitDockWhenActivityDuplicates === true &&
    ((activityStatus === "docked" && dock?.state === "docked") ||
      (activityStatus === "charging" && dock?.state === "charging") ||
      (activityStatus === "returning" && (dock?.state === "returning" || dock?.state === "docked")));
  const parts = [dockDuplicatesActivity ? null : dockLabel, batteryLevel].filter(Boolean);
  return parts.length > 0 ? parts.join(" · ") : "Battery unknown";
}

function getPrimaryStateTone(primary: VacuumPrimaryRobotStateSummary): "ready" | "warning" | "danger" | "muted" {
  if (primary.state === "offline" || primary.state === "unavailable" || primary.state === "error") {
    return "danger";
  }
  if (primary.state === "cleaning" || primary.state === "paused" || primary.state === "returning_to_dock") {
    return "warning";
  }
  if (primary.state === "docked" || primary.state === "charging" || primary.state === "idle") {
    return "ready";
  }
  return "muted";
}

function formatCompactStateDetail(primary: VacuumPrimaryRobotStateSummary, fault: VacuumFaultState): string | null {
  if (primary.state === "error") {
    return fault.faults[0] ?? primary.detail;
  }
  if (primary.state === "offline" || primary.state === "unavailable") {
    return primary.detail;
  }
  return null;
}

function MissionLifecycleCard(props: {
  mission: VacuumLegacyMissionStatus;
  battery: VacuumBatteryState;
  capabilities: VacuumCapabilities;
  commandError: string | null;
  onReturnToDock: () => void;
}): JSX.Element {
  const returnToDock = props.capabilities.return_to_dock;
  const dockState = props.capabilities.dock_state;
  const battery = props.capabilities.battery;
  const canUseReturnToDock = returnToDock.supported && returnToDock.available !== false;

  return (
    <section className="vacuum-panel-card vacuum-panel-card--mission-lifecycle">
      <div className="vacuum-panel-card__head">
        <p className="vacuum-panel-card__eyebrow">Mission Lifecycle</p>
        <span className="vacuum-clean-area-badge">{props.mission.state}</span>
      </div>
      <p className="vacuum-clean-area-copy">{props.mission.detail ?? "Adapter mission state is available."}</p>

      {props.commandError ? (
        <div className="vacuum-mapping-error" role="status">
          {props.commandError}
        </div>
      ) : null}

      <div className="vacuum-clean-area-coverage-grid">
        <div>
          <span>Dock state</span>
          <strong>{capabilityLabel(dockState)}</strong>
        </div>
        <div>
          <span>Return to dock</span>
          <strong>{capabilityLabel(returnToDock)}</strong>
        </div>
        <div>
          <span>Battery</span>
          <strong>{battery.supported ? formatBatteryState(props.battery) : capabilityLabel(battery)}</strong>
        </div>
        <div>
          <span>Charging</span>
          <strong>{props.battery.charging == null ? "n/a" : props.battery.charging ? "Yes" : "No"}</strong>
        </div>
      </div>

      <div className="vacuum-actions">
        <button
          className="vacuum-action vacuum-action--ghost"
          type="button"
          onClick={props.onReturnToDock}
          disabled={!canUseReturnToDock}
        >
          Return to dock
        </button>
      </div>
    </section>
  );
}

function BasicControlsCard(props: {
  capabilities: VacuumCapabilities;
  commandError: string | null;
  onStart: () => void;
  onPause: () => void;
  onResume: () => void;
  onStop: () => void;
  onReturnToDock: () => void;
}): JSX.Element | null {
  const controls = [
    {
      key: "start_cleaning",
      label: "Start cleaning",
      className: "vacuum-action vacuum-action--primary",
      capability: props.capabilities.start_cleaning,
      onClick: props.onStart,
    },
    {
      key: "pause",
      label: "Pause",
      className: "vacuum-action vacuum-action--ghost",
      capability: props.capabilities.pause,
      onClick: props.onPause,
    },
    {
      key: "resume",
      label: "Resume",
      className: "vacuum-action vacuum-action--primary",
      capability: props.capabilities.resume,
      onClick: props.onResume,
    },
    {
      key: "stop",
      label: "Stop",
      className: "vacuum-action vacuum-action--danger",
      capability: props.capabilities.stop,
      onClick: props.onStop,
    },
    {
      key: "return_to_dock",
      label: "Return to dock",
      className: "vacuum-action vacuum-action--ghost",
      capability: props.capabilities.return_to_dock,
      onClick: props.onReturnToDock,
    },
  ];
  const supportedControls = controls.filter((control) => control.capability.supported);

  if (supportedControls.length === 0) {
    return null;
  }

  return (
    <section className="vacuum-panel-card vacuum-panel-card--mission-lifecycle">
      <div className="vacuum-panel-card__head">
        <p className="vacuum-panel-card__eyebrow">Basic Cleaning</p>
      </div>
      {props.commandError ? (
        <div className="vacuum-mapping-error" role="status">
          {props.commandError}
        </div>
      ) : null}
      <div className="vacuum-actions vacuum-basic-actions">
        {supportedControls.map((control) => {
          const disabled = control.capability.available === false || control.capability.status === "unavailable";
          return (
            <div key={control.key} className="vacuum-basic-action-row">
              <button
                className={control.className}
                type="button"
                onClick={control.onClick}
                disabled={disabled}
              >
                {control.label}
              </button>
            </div>
          );
        })}
      </div>
    </section>
  );
}

function CleaningSettingControl(props: {
  label: string;
  setting: VacuumCleaningSettingState;
  capability: CapabilitySupport;
  onChange: (value: string) => void;
}): JSX.Element {
  const currentOption = props.setting.options.find((option) => option.value === props.setting.current);
  const currentLabel = currentOption?.label ?? (props.setting.current ? humanizeStatus(props.setting.current) : "Unknown");
  const reason = formatCapabilityReason(props.capability) ?? props.setting.detail ?? null;
  const disabled = props.capability.available === false || props.capability.status === "unavailable" || props.setting.readiness === "unavailable";

  return (
    <div className="vacuum-cleaning-setting">
      <div className="vacuum-cleaning-setting__head">
        <span>{props.label}</span>
        <strong>{currentLabel}</strong>
      </div>
      <div className="vacuum-cleaning-setting__options" role="group" aria-label={props.label}>
        {props.setting.options.map((option) => (
          <button
            key={option.value}
            className={`vacuum-setting-option${option.value === props.setting.current ? " vacuum-setting-option--active" : ""}`}
            type="button"
            onClick={() => props.onChange(option.value)}
            disabled={disabled || option.value === props.setting.current}
            title={disabled && reason ? reason : undefined}
          >
            {option.label}
          </button>
        ))}
      </div>
      {disabled && reason ? (
        <p className="vacuum-action-hint vacuum-action-hint--disabled">{reason}</p>
      ) : null}
    </div>
  );
}

function CleaningSettingsCard(props: {
  settings?: VacuumCleaningSettingsState;
  capabilities: VacuumCapabilities;
  commandError: string | null;
  onSetFanSpeed: (value: string) => void;
  onSetWaterUsage: (value: string) => void;
}): JSX.Element | null {
  const showFanSpeed = props.settings?.fanSpeed && props.capabilities.fan_speed.supported;
  const showWaterUsage = props.settings?.waterUsage && props.capabilities.water_usage.supported;

  if (!showFanSpeed && !showWaterUsage) {
    return null;
  }

  return (
    <section className="vacuum-panel-card vacuum-panel-card--cleaning-settings">
      <div className="vacuum-panel-card__head">
        <p className="vacuum-panel-card__eyebrow">Cleaning Settings</p>
      </div>
      {props.commandError ? (
        <div className="vacuum-mapping-error" role="status">
          {props.commandError}
        </div>
      ) : null}
      {showFanSpeed ? (
        <CleaningSettingControl
          label="Fan speed"
          setting={props.settings!.fanSpeed!}
          capability={props.capabilities.fan_speed}
          onChange={props.onSetFanSpeed}
        />
      ) : null}
      {showWaterUsage ? (
        <CleaningSettingControl
          label="Water usage"
          setting={props.settings!.waterUsage!}
          capability={props.capabilities.water_usage}
          onChange={props.onSetWaterUsage}
        />
      ) : null}
    </section>
  );
}

function formatConsumableValue(item: VacuumMaintenanceState["consumables"][number]): string {
  if (typeof item.remainingPercent === "number") {
    return `${Math.round(item.remainingPercent)}%`;
  }
  if (typeof item.remainingMinutes === "number") {
    if (item.remainingMinutes < 60) {
      return `${item.remainingMinutes} min`;
    }
    const hours = Math.floor(item.remainingMinutes / 60);
    return hours < 48 ? `${hours} h` : `${Math.floor(hours / 24)} d`;
  }
  return "Unknown";
}

function isAttachmentAttentionWorthy(item: VacuumAttachmentState): boolean {
  const { kind, status } = item;
  if (status === "missing" || status === "error") return true;
  if (status === "full" && kind === "dustbin") return true;
  if (status === "empty" && kind === "water_tank") return true;
  if (status === "low" && (kind === "water_tank" || kind === "detergent")) return true;
  return false;
}

function isDockComponentAttentionWorthy(component: VacuumDockComponentState): boolean {
  const { kind, status } = component;
  if (status === "missing" || status === "error") return true;
  if (status === "full" && (kind === "wastewater" || kind === "dustbag")) return true;
  if (status === "empty" && kind === "freshwater") return true;
  if (status === "low" && (kind === "freshwater" || kind === "detergent")) return true;
  return false;
}

function isConsumableAttentionWorthy(consumable: VacuumMaintenanceState["consumables"][number]): boolean {
  return consumable.status === "warning" || consumable.status === "replace_now" || consumable.status === "replace_soon";
}

function formatAttachmentAttentionLabel(item: VacuumAttachmentState): string {
  if (item.status === "full" && item.kind === "dustbin") return "Dustbin full";
  if (item.status === "empty" && item.kind === "water_tank") return "Water tank empty";
  if (item.status === "low" && item.kind === "water_tank") return "Water tank low";
  if (item.status === "low" && item.kind === "detergent") return "Detergent low";
  if (item.status === "missing") return `${item.label} missing`;
  if (item.status === "error") return `${item.label} error`;
  return item.label;
}

function formatDockComponentAttentionLabel(component: VacuumDockComponentState): string {
  if (component.status === "full" && (component.kind === "wastewater" || component.kind === "dustbag")) return `${component.label} full`;
  if (component.status === "empty" && component.kind === "freshwater") return "Freshwater empty";
  if (component.status === "low" && component.kind === "freshwater") return "Freshwater low";
  if (component.status === "low" && component.kind === "detergent") return "Dock detergent low";
  if (component.status === "missing") return `${component.label} missing`;
  if (component.status === "error") return `${component.label} error`;
  return component.label;
}

function derivePeripheralAttentionLine(
  attachments: VacuumAttachmentsState | undefined,
  dockComponents: VacuumDockComponentState[] | undefined,
  consumables: VacuumMaintenanceState["consumables"],
): string | null {
  const items: string[] = [];
  for (const item of (attachments?.items ?? [])) {
    if (isAttachmentAttentionWorthy(item)) {
      items.push(formatAttachmentAttentionLabel(item));
    }
  }
  for (const component of (dockComponents ?? [])) {
    if (isDockComponentAttentionWorthy(component)) {
      items.push(formatDockComponentAttentionLabel(component));
    }
  }
  for (const consumable of consumables) {
    if (isConsumableAttentionWorthy(consumable)) {
      items.push(consumable.label);
    }
  }
  if (items.length === 0) return null;
  if (items.length <= 3) return items.join(" · ");
  return `${items.slice(0, 2).join(" · ")} · +${items.length - 2} more`;
}

function consumableTone(status: VacuumMaintenanceState["consumables"][number]["status"]): string {
  switch (status) {
    case "replace_now":
      return "replace-now";
    case "replace_soon":
      return "replace-soon";
    case "warning":
      return "warning";
    case "ok":
      return "ok";
    default:
      return "unknown";
  }
}

function MaintenanceCard(props: {
  maintenance?: VacuumMaintenanceState;
  capabilities: VacuumCapabilities;
}): JSX.Element | null {
  const consumables = props.maintenance?.consumables ?? [];
  if (!props.capabilities.consumables.supported || consumables.length === 0) {
    return null;
  }
  const reason = props.capabilities.consumables.available === false
    ? formatCapabilityReason(props.capabilities.consumables)
    : null;
  const attentionCount = consumables.filter(isConsumableAttentionWorthy).length;

  return (
    <section className="vacuum-panel-card vacuum-panel-card--maintenance">
      <div className="vacuum-panel-card__head">
        <p className="vacuum-panel-card__eyebrow">Maintenance</p>
        {attentionCount > 0 ? (
          <span className="vacuum-state-badge vacuum-state-badge--warning">{attentionCount}</span>
        ) : null}
      </div>
      {reason ? (
        <p className="vacuum-action-hint vacuum-action-hint--disabled">{reason}</p>
      ) : null}
      <div className="vacuum-maintenance-list">
        {consumables.map((item) => {
          const percent = typeof item.remainingPercent === "number"
            ? Math.max(0, Math.min(100, item.remainingPercent))
            : null;
          const tone = consumableTone(item.status);
          return (
            <div key={item.id} className={`vacuum-consumable vacuum-consumable--${tone}`}>
              <div className="vacuum-consumable__head">
                <strong>{item.label}</strong>
                <span>{formatConsumableValue(item)}</span>
              </div>
              {percent != null ? (
                <div className="vacuum-consumable__track" aria-hidden="true">
                  <span style={{ width: `${percent}%` }} />
                </div>
              ) : null}
              {item.detail ? <p>{item.detail}</p> : null}
            </div>
          );
        })}
      </div>
    </section>
  );
}

function CurrentStatisticsCard(props: {
  statistics?: VacuumStatisticsState;
  capabilities: VacuumCapabilities;
}): JSX.Element | null {
  const current = props.statistics?.current;
  if (!props.capabilities.statistics.supported || !current) {
    return null;
  }
  const hasDuration = typeof current.durationSeconds === "number" && Number.isFinite(current.durationSeconds);
  const hasArea = typeof current.areaSquareMeters === "number" && Number.isFinite(current.areaSquareMeters);
  const areaSquareMeters = hasArea ? current.areaSquareMeters : null;
  if (!hasDuration && !hasArea) {
    return null;
  }

  return (
    <section className="vacuum-panel-card vacuum-panel-card--current-statistics" aria-label="Current statistics">
      <div className="vacuum-panel-card__head">
        <p className="vacuum-panel-card__eyebrow">Current Statistics</p>
      </div>
      <div className="vacuum-current-statistics">
        <div>
          <span>Duration</span>
          <strong>{formatCurrentStatisticsDuration(current.durationSeconds)}</strong>
        </div>
        <div>
          <span>Area</span>
          <strong>{areaSquareMeters == null ? "n/a" : formatArea(areaSquareMeters)}</strong>
        </div>
      </div>
    </section>
  );
}

function formatReadinessRowValue(status: string, levelPercent?: number): string {
  const label = humanizeStatus(status);
  return typeof levelPercent === "number" && Number.isFinite(levelPercent)
    ? `${label} - ${Math.round(Math.max(0, Math.min(100, levelPercent)))}%`
    : label;
}

function ReadinessListCard(props: {
  title: string;
  items: Array<{ id: string; label: string; status: string; levelPercent?: number; detail?: string }>;
  attentionCount?: number;
}): JSX.Element | null {
  if (props.items.length === 0) {
    return null;
  }
  return (
    <section className="vacuum-panel-card vacuum-panel-card--readiness-list" aria-label={props.title}>
      <div className="vacuum-panel-card__head">
        <p className="vacuum-panel-card__eyebrow">{props.title}</p>
        {props.attentionCount != null && props.attentionCount > 0 ? (
          <span className="vacuum-state-badge vacuum-state-badge--warning">{props.attentionCount}</span>
        ) : null}
      </div>
      <div className="vacuum-readiness-list">
        {props.items.map((item) => (
          <div key={item.id} className="vacuum-readiness-row">
            <div>
              <strong>{item.label}</strong>
              {item.detail ? <p>{item.detail}</p> : null}
            </div>
            <span>{formatReadinessRowValue(item.status, item.levelPercent)}</span>
          </div>
        ))}
      </div>
    </section>
  );
}

function AttachmentsCard(props: {
  attachments?: VacuumAttachmentsState;
  capabilities: VacuumCapabilities;
}): JSX.Element | null {
  const items = props.attachments?.items ?? [];
  if (!props.capabilities.attachments.supported || items.length === 0) {
    return null;
  }
  const attentionCount = items.filter(isAttachmentAttentionWorthy).length;
  return <ReadinessListCard title="Attachments" items={items} attentionCount={attentionCount} />;
}

function DockComponentsCard(props: {
  components?: VacuumDockComponentState[];
  capabilities: VacuumCapabilities;
}): JSX.Element | null {
  const components = props.components ?? [];
  if (!props.capabilities.dock_components.supported || components.length === 0) {
    return null;
  }
  const attentionCount = components.filter(isDockComponentAttentionWorthy).length;
  return <ReadinessListCard title="Dock Components" items={components} attentionCount={attentionCount} />;
}

function formatMapTargetKind(kind: VacuumMapTarget["kind"]): string {
  switch (kind) {
    case "room":
      return "Room";
    case "segment":
      return "Segment";
    case "zone":
      return "Zone";
  }
}

function formatMapTargetGeometry(geometry: VacuumMapTarget["geometry"]): string | null {
  if (!geometry) {
    return null;
  }
  if (geometry.type === "polygon") {
    return "Polygon";
  }
  if (geometry.type === "rectangle") {
    return "Rectangle";
  }
  return "Area available";
}

type MapPreviewBounds = {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
};

type MapPreviewRunRect = {
  x: number;
  y: number;
  width: number;
  height: number;
};

type MapPreviewTransform = {
  pixelSize: number;
  viewBox: MapPreviewBounds;
  markerSize: number;
};

type MapPreviewRenderableLayer = {
  layer: VacuumMapLayer;
  originalIndex: number;
  segmentToneIndex: number;
};

const MAP_PREVIEW_LAYER_ORDER: Record<VacuumMapLayer["kind"], number> = {
  floor: 0,
  segment: 1,
  unknown: 2,
  path: 3,
  wall: 4,
};

function expandMapPreviewBounds(bounds: MapPreviewBounds | null, next: MapPreviewBounds): MapPreviewBounds {
  if (!bounds) {
    return next;
  }
  return {
    minX: Math.min(bounds.minX, next.minX),
    minY: Math.min(bounds.minY, next.minY),
    maxX: Math.max(bounds.maxX, next.maxX),
    maxY: Math.max(bounds.maxY, next.maxY),
  };
}

function mapPreviewRunRect(run: NonNullable<VacuumMapLayer["runs"]>[number], pixelSize: number): MapPreviewRunRect {
  return {
    x: run.x * pixelSize,
    y: run.y * pixelSize,
    width: run.count * pixelSize,
    height: pixelSize,
  };
}

function mapPreviewPointBounds(points: Array<{ x: number; y: number }>, padding = 0): MapPreviewBounds | null {
  let bounds: MapPreviewBounds | null = null;
  for (const point of points) {
    if (!Number.isFinite(point.x) || !Number.isFinite(point.y)) {
      continue;
    }
    bounds = expandMapPreviewBounds(bounds, {
      minX: point.x - padding,
      minY: point.y - padding,
      maxX: point.x + padding,
      maxY: point.y + padding,
    });
  }
  return bounds;
}

function mapPreviewLayerBounds(layer: VacuumMapLayer, pixelSize: number): MapPreviewBounds | null {
  let bounds: MapPreviewBounds | null = null;
  for (const run of layer.runs ?? []) {
    const rect = mapPreviewRunRect(run, pixelSize);
    bounds = expandMapPreviewBounds(bounds, {
      minX: rect.x,
      minY: rect.y,
      maxX: rect.x + rect.width,
      maxY: rect.y + rect.height,
    });
  }
  const pointBounds = mapPreviewPointBounds(layer.points ?? []);
  return pointBounds ? expandMapPreviewBounds(bounds, pointBounds) : bounds;
}

function mapPreviewContentBounds(preview: VacuumLayeredMapPreview, pixelSize: number): MapPreviewBounds | null {
  let bounds: MapPreviewBounds | null = null;
  for (const layer of preview.layers) {
    const layerBounds = mapPreviewLayerBounds(layer, pixelSize);
    if (layerBounds) {
      bounds = expandMapPreviewBounds(bounds, layerBounds);
    }
  }
  for (const entity of preview.entities) {
    const pointPadding = entity.kind === "robot" || entity.kind === "charger" ? 12 : 0;
    const entityBounds = mapPreviewPointBounds(entity.points ?? [], pointPadding);
    if (entityBounds) {
      bounds = expandMapPreviewBounds(bounds, entityBounds);
    }
  }
  return bounds && bounds.maxX > bounds.minX && bounds.maxY > bounds.minY ? bounds : null;
}

function clampMapPreviewBounds(bounds: MapPreviewBounds, preview: VacuumLayeredMapPreview): MapPreviewBounds {
  const minX = clamp(bounds.minX, 0, preview.width);
  const minY = clamp(bounds.minY, 0, preview.height);
  const maxX = clamp(bounds.maxX, minX, preview.width);
  const maxY = clamp(bounds.maxY, minY, preview.height);
  return maxX > minX && maxY > minY ? { minX, minY, maxX, maxY } : { minX: 0, minY: 0, maxX: preview.width, maxY: preview.height };
}

function buildMapPreviewTransform(preview: VacuumLayeredMapPreview): MapPreviewTransform {
  const pixelSize = preview.pixelSize && preview.pixelSize > 0 ? preview.pixelSize : 1;
  const fallbackBounds = { minX: 0, minY: 0, maxX: preview.width, maxY: preview.height };
  const contentBounds = mapPreviewContentBounds(preview, pixelSize) ?? fallbackBounds;
  const contentWidth = Math.max(1, contentBounds.maxX - contentBounds.minX);
  const contentHeight = Math.max(1, contentBounds.maxY - contentBounds.minY);
  const padding = Math.max(4, Math.min(16, Math.min(contentWidth, contentHeight) * 0.04));
  const viewBox = clampMapPreviewBounds(
    {
      minX: contentBounds.minX - padding,
      minY: contentBounds.minY - padding,
      maxX: contentBounds.maxX + padding,
      maxY: contentBounds.maxY + padding,
    },
    preview,
  );
  const viewBoxMinDimension = Math.min(viewBox.maxX - viewBox.minX, viewBox.maxY - viewBox.minY);
  return {
    pixelSize,
    viewBox,
    markerSize: clamp(viewBoxMinDimension * 0.04, 6, 14),
  };
}

function mapPreviewLayerClass(layer: VacuumMapLayer, segmentToneIndex: number): string {
  const segmentTone = layer.kind === "segment" ? ` vacuum-map-preview-layer--segment-${segmentToneIndex % 6}` : "";
  return `vacuum-map-preview-layer vacuum-map-preview-layer--${layer.kind}${segmentTone}`;
}

function mapPreviewPointList(points: Array<{ x: number; y: number }>): string {
  return points.map((point) => `${point.x},${point.y}`).join(" ");
}

function mapPreviewViewBox(bounds: MapPreviewBounds): string {
  return `${bounds.minX} ${bounds.minY} ${bounds.maxX - bounds.minX} ${bounds.maxY - bounds.minY}`;
}

function mapPreviewAspectRatio(bounds: MapPreviewBounds): string {
  return `${Math.max(1, bounds.maxX - bounds.minX)} / ${Math.max(1, bounds.maxY - bounds.minY)}`;
}

function prepareMapPreviewLayers(layers: VacuumMapLayer[]): MapPreviewRenderableLayer[] {
  let segmentCount = 0;
  return layers
    .map((layer, originalIndex) => {
      const segmentToneIndex = layer.kind === "segment" ? segmentCount++ : 0;
      return { layer, originalIndex, segmentToneIndex };
    })
    .sort((a, b) => {
      const orderDelta = MAP_PREVIEW_LAYER_ORDER[a.layer.kind] - MAP_PREVIEW_LAYER_ORDER[b.layer.kind];
      return orderDelta !== 0 ? orderDelta : a.originalIndex - b.originalIndex;
    });
}

function mapPreviewLegendItems(preview: VacuumLayeredMapPreview): string[] {
  const present = new Set<string>();
  for (const layer of preview.layers) {
    if (layer.kind === "floor") {
      present.add("Floor");
    } else if (layer.kind === "wall") {
      present.add("Walls");
    } else if (layer.kind === "segment") {
      present.add("Segments");
    } else if (layer.kind === "path") {
      present.add("Path");
    }
  }
  for (const entity of preview.entities) {
    if (entity.kind === "robot") {
      present.add("Robot");
    } else if (entity.kind === "charger") {
      present.add("Charger");
    } else if (entity.kind === "zone") {
      present.add("Zone");
    } else if (entity.kind === "no_go_area") {
      present.add("No-go");
    } else if (entity.kind === "no_mop_area") {
      present.add("No-mop");
    } else if (entity.kind === "virtual_wall") {
      present.add("Virtual wall");
    } else if (entity.kind === "obstacle") {
      present.add("Obstacle");
    } else if (entity.kind === "path") {
      present.add("Path");
    }
  }
  return ["Floor", "Walls", "Segments", "Robot", "Charger", "Zone", "No-go", "No-mop", "Virtual wall", "Obstacle", "Path"]
    .filter((item) => present.has(item));
}

function mapPreviewLegendClass(item: string): string {
  return item.toLowerCase().replace(/[^a-z0-9]+/g, "-").replace(/^-|-$/g, "");
}

function hasRenderableLayeredMapPreview(preview: VacuumLayeredMapPreview | undefined): preview is VacuumLayeredMapPreview {
  return !!preview && preview.width > 0 && preview.height > 0 && (preview.layers.length > 0 || preview.entities.length > 0);
}

function MapPreviewLayer(props: {
  layer: VacuumMapLayer;
  segmentToneIndex: number;
  transform: MapPreviewTransform;
}): JSX.Element {
  const className = mapPreviewLayerClass(props.layer, props.segmentToneIndex);
  return (
    <g className={className}>
      {(props.layer.runs ?? []).map((run, index) => {
        const rect = mapPreviewRunRect(run, props.transform.pixelSize);
        return (
          <rect
            key={`${props.layer.id}:run:${index}`}
            x={rect.x}
            y={rect.y}
            width={rect.width}
            height={rect.height}
          />
        );
      })}
      {props.layer.points && props.layer.points.length >= 2 ? (
        <polyline points={mapPreviewPointList(props.layer.points)} />
      ) : null}
    </g>
  );
}

function MapPreviewEntity(props: {
  entity: VacuumMapEntity;
  markerSize: number;
}): JSX.Element | null {
  const points = props.entity.points ?? [];
  if (points.length === 0) {
    return null;
  }
  if (props.entity.kind === "zone" && points.length >= 3) {
    return <polygon className="vacuum-map-preview-entity vacuum-map-preview-entity--zone" points={mapPreviewPointList(points)} />;
  }
  if (props.entity.kind === "no_go_area" && points.length >= 3) {
    return <polygon className="vacuum-map-preview-entity vacuum-map-preview-entity--no-go-area" points={mapPreviewPointList(points)} />;
  }
  if (props.entity.kind === "no_mop_area" && points.length >= 3) {
    return <polygon className="vacuum-map-preview-entity vacuum-map-preview-entity--no-mop-area" points={mapPreviewPointList(points)} />;
  }
  if (props.entity.kind === "virtual_wall" && points.length >= 2) {
    return <polyline className="vacuum-map-preview-entity vacuum-map-preview-entity--virtual-wall" points={mapPreviewPointList(points)} />;
  }
  if (props.entity.kind === "path" && points.length >= 2) {
    return <polyline className="vacuum-map-preview-entity vacuum-map-preview-entity--path" points={mapPreviewPointList(points)} />;
  }
  const point = points[0]!;
  if (props.entity.kind === "robot") {
    const angle = props.entity.angle ?? 0;
    const radius = props.markerSize;
    return (
      <g className="vacuum-map-preview-entity vacuum-map-preview-entity--robot" transform={`translate(${point.x} ${point.y}) rotate(${angle})`}>
        <circle r={radius} />
        <path d={`M ${radius * 1.35} 0 L ${radius * -0.62} ${radius * -0.75} L ${radius * -0.38} 0 L ${radius * -0.62} ${radius * 0.75} Z`} />
      </g>
    );
  }
  if (props.entity.kind === "charger") {
    const width = props.markerSize * 1.8;
    const height = props.markerSize * 1.28;
    return (
      <g className="vacuum-map-preview-entity vacuum-map-preview-entity--charger" transform={`translate(${point.x} ${point.y})`}>
        <rect x={width / -2} y={height / -2} width={width} height={height} rx={Math.max(1.5, props.markerSize * 0.22)} />
        <path d={`M ${props.markerSize * -0.38} ${height / -2} L ${props.markerSize * -0.38} ${height / -2 - props.markerSize * 0.62} M ${props.markerSize * 0.38} ${height / -2} L ${props.markerSize * 0.38} ${height / -2 - props.markerSize * 0.62}`} />
      </g>
    );
  }
  return <circle className={`vacuum-map-preview-entity vacuum-map-preview-entity--${props.entity.kind}`} cx={point.x} cy={point.y} r={props.markerSize * 0.58} />;
}

function ValetudoLayeredMapSvg(props: {
  preview?: VacuumLayeredMapPreview;
  className?: string;
  ariaLabel: string;
}): JSX.Element | null {
  const preview = props.preview;
  if (!hasRenderableLayeredMapPreview(preview)) {
    return null;
  }
  const transform = buildMapPreviewTransform(preview);
  const renderableLayers = prepareMapPreviewLayers(preview.layers);

  return (
    <svg
      className={props.className ?? "vacuum-map-preview-svg"}
      viewBox={mapPreviewViewBox(transform.viewBox)}
      preserveAspectRatio="xMidYMid meet"
      role="img"
      aria-label={props.ariaLabel}
    >
      <rect className="vacuum-map-preview-bg" x="0" y="0" width={preview.width} height={preview.height} />
      {renderableLayers.map(({ layer, segmentToneIndex }) => (
        <MapPreviewLayer key={layer.id} layer={layer} segmentToneIndex={segmentToneIndex} transform={transform} />
      ))}
      {preview.entities.map((entity) => (
        <MapPreviewEntity key={entity.id} entity={entity} markerSize={transform.markerSize} />
      ))}
    </svg>
  );
}

function MapPreviewCard(props: {
  preview?: VacuumLayeredMapPreview;
}): JSX.Element | null {
  const preview = props.preview;
  if (!hasRenderableLayeredMapPreview(preview)) {
    return null;
  }
  const transform = buildMapPreviewTransform(preview);
  const legendItems = mapPreviewLegendItems(preview);

  return (
    <section className="vacuum-panel-card vacuum-panel-card--map-preview" aria-label="Map preview">
      <div className="vacuum-panel-card__head">
        <p className="vacuum-panel-card__eyebrow">Map Preview</p>
      </div>
      <div className="vacuum-map-preview-frame" style={{ aspectRatio: mapPreviewAspectRatio(transform.viewBox) }}>
        <ValetudoLayeredMapSvg preview={preview} ariaLabel="Static normalized map preview" />
      </div>
      {legendItems.length > 0 ? (
        <div className="vacuum-map-preview-legend" aria-hidden="true">
          {legendItems.map((item) => (
            <span key={item} className={`vacuum-map-preview-legend__item vacuum-map-preview-legend__item--${mapPreviewLegendClass(item)}`}>
              {item}
            </span>
          ))}
        </div>
      ) : null}
      <p className="vacuum-action-hint">Static preview from normalized map data. Cleaning commands are not enabled.</p>
    </section>
  );
}

function ValetudoMainMapPreview(props: {
  preview?: VacuumLayeredMapPreview;
}): JSX.Element | null {
  const preview = props.preview;
  if (!hasRenderableLayeredMapPreview(preview)) {
    return null;
  }
  const transform = buildMapPreviewTransform(preview);

  return (
    <section className="vacuum-map-card vacuum-map-card--layered-preview" aria-label="Main map preview">
      <div className="vacuum-map-stage vacuum-map-stage--layered-preview">
        <div className="vacuum-layered-map-preview-frame" style={{ aspectRatio: mapPreviewAspectRatio(transform.viewBox) }}>
          <ValetudoLayeredMapSvg
            preview={preview}
            className="vacuum-map-preview-svg vacuum-map-preview-svg--main"
            ariaLabel="Large static normalized map preview"
          />
        </div>
      </div>
    </section>
  );
}

function MapTargetSection(props: {
  title: string;
  targets: VacuumMapTarget[];
}): JSX.Element | null {
  if (props.targets.length === 0) {
    return null;
  }
  return (
    <div className="vacuum-map-target-section">
      <p className="vacuum-map-target-section__title">{props.title}</p>
      <div className="vacuum-map-target-list">
        {props.targets.map((target) => {
          const geometryLabel = formatMapTargetGeometry(target.geometry);
          return (
            <div key={target.id} className="vacuum-map-target-row">
              <div>
                <strong>{target.label}</strong>
                {target.detail ? <p>{target.detail}</p> : null}
                {geometryLabel ? <p>{geometryLabel}</p> : null}
              </div>
              <span>{`${formatMapTargetKind(target.kind)} - ${target.available ? "Available" : "Unavailable"}`}</span>
            </div>
          );
        })}
      </div>
    </div>
  );
}

function MapTargetsCard(props: {
  targets?: VacuumMapTargets;
}): JSX.Element | null {
  const segments = props.targets?.segments ?? [];
  const zones = props.targets?.zones ?? [];
  if (segments.length === 0 && zones.length === 0) {
    return null;
  }

  return (
    <section className="vacuum-panel-card vacuum-panel-card--map-targets" aria-label="Map targets">
      <div className="vacuum-panel-card__head">
        <p className="vacuum-panel-card__eyebrow">Map Targets</p>
      </div>
      <p className="vacuum-action-hint">
        Saved map targets discovered from normalized map data. Cleaning commands are not enabled yet.
      </p>
      <MapTargetSection title="Segments / Rooms" targets={segments} />
      <MapTargetSection title="Zones" targets={zones} />
    </section>
  );
}

function NoMapCanvasPlaceholder(props: {
  mapDetail?: string;
}): JSX.Element {
  return (
    <section className="vacuum-map-card" aria-label="Reserved map area">
      <div className="vacuum-map-stage vacuum-map-stage--no-map">
        <span className="vacuum-no-map-stage-note">{props.mapDetail ?? "Map unavailable."}</span>
      </div>
    </section>
  );
}

function RobotOverviewCard(props: {
  identity: { label: string; model?: string };
  availability: VacuumAvailability;
  battery?: VacuumBatteryState;
  dock?: VacuumDockStatus;
  health?: VacuumRuntimeHealth;
  source?: VacuumSourceState;
  activity?: VacuumRobotActivity;
  fault: VacuumFaultState;
  peripheralAttentionLine?: string | null;
}): JSX.Element {
  const primary = deriveVacuumPrimaryRobotState(props);
  const compactDetail = formatCompactStateDetail(primary, props.fault);
  const activityDetail = props.activity?.detail ?? null;

  return (
    <section className="vacuum-panel-card vacuum-panel-card--robot-overview" aria-label="Robot status">
      <div className="vacuum-panel-card__head">
        <p className="vacuum-panel-card__eyebrow">Robot Overview</p>
      </div>
      <div className="vacuum-robot-overview">
        <div className="vacuum-robot-overview__identity">
          <strong>{props.identity.label}</strong>
          {props.identity.model ? <span>{props.identity.model}</span> : null}
        </div>
        <strong className={`vacuum-robot-state vacuum-robot-state--${getPrimaryStateTone(primary)}`}>
          {primary.label.toUpperCase()}
        </strong>
        {compactDetail ? <p>{compactDetail}</p> : null}
        {!compactDetail && activityDetail ? <p>{activityDetail}</p> : null}
        {!compactDetail && props.peripheralAttentionLine ? (
          <p className="vacuum-robot-overview__peripheral-attention">{props.peripheralAttentionLine}</p>
        ) : null}
      </div>
    </section>
  );
}

function BatteryDockCard(props: {
  battery: VacuumBatteryState;
  dock?: VacuumDockStatus;
  capabilities: VacuumCapabilities;
}): JSX.Element | null {
  const batteryCapability = props.capabilities.battery;
  const dockCapability = props.capabilities.dock_state;
  const hasBatteryState = props.battery.percentage != null || props.battery.detail;
  const hasDockState = Boolean(props.dock?.state && props.dock.state !== "unknown") || props.dock?.detail;

  if (!batteryCapability.supported && !dockCapability.supported && !hasBatteryState && !hasDockState) {
    return null;
  }

  const batteryPercent = typeof props.battery.percentage === "number"
    ? Math.max(0, Math.min(100, props.battery.percentage))
    : null;
  const batteryReason = formatCapabilityReason(batteryCapability) ?? props.battery.detail ?? null;
  const dockReason = formatCapabilityReason(dockCapability) ?? props.dock?.detail ?? null;
  const dockLabel = props.dock?.state && props.dock.state !== "unknown" ? humanizeStatus(props.dock.state) : "Unknown";
  const charging =
    props.battery.charging ?? props.dock?.charging ?? null;

  return (
    <section className="vacuum-panel-card vacuum-panel-card--battery-dock" aria-label="Battery and dock">
      <div className="vacuum-panel-card__head">
        <p className="vacuum-panel-card__eyebrow">Battery and Dock</p>
      </div>
      <div className="vacuum-battery-dock-grid">
        <div className="vacuum-battery-dock-row">
          <span>Battery</span>
          <strong>{batteryPercent == null ? "Unknown" : `${Math.round(batteryPercent)}%`}</strong>
        </div>
        <div className="vacuum-robot-battery__track" aria-hidden="true">
          <span style={{ width: `${batteryPercent ?? 0}%` }} />
        </div>
        {batteryReason && batteryPercent == null ? (
          <p className="vacuum-action-hint vacuum-action-hint--disabled">{batteryReason}</p>
        ) : null}
        <div className="vacuum-battery-dock-row">
          <span>Dock</span>
          <strong className={`vacuum-battery-dock-state vacuum-battery-dock-state--${getDockTone(props.dock)}`}>
            {dockLabel}
          </strong>
        </div>
        <div className="vacuum-battery-dock-row">
          <span>Charging</span>
          <strong>{charging == null ? "Unknown" : charging ? "Yes" : "No"}</strong>
        </div>
        {dockReason && dockLabel === "Unknown" ? (
          <p className="vacuum-action-hint vacuum-action-hint--disabled">{dockReason}</p>
        ) : null}
      </div>
    </section>
  );
}

function SourceHealthCard(props: {
  availability: VacuumAvailability;
  health?: VacuumRuntimeHealth;
  source?: VacuumSourceState;
  fault: VacuumFaultState;
}): JSX.Element | null {
  if (!props.health && !props.source && !props.availability.detail && props.fault.faults.length === 0) {
    return null;
  }

  const runtimeLabel = props.health?.runtimeStatus ? humanizeStatus(props.health.runtimeStatus) : "Unknown";
  const sourceStatus = props.source?.stale ? "Stale" : props.source?.status ? humanizeStatus(props.source.status) : "Unknown";
  const sourceDetail = props.source?.reason ?? props.fault.faults[0] ?? props.health?.detail ?? null;

  return (
    <section className="vacuum-panel-card vacuum-panel-card--source-health" aria-label="Source and health">
      <div className="vacuum-panel-card__head">
        <p className="vacuum-panel-card__eyebrow">Source / Health</p>
      </div>
      <div className="vacuum-source-health-grid">
        <div className="vacuum-source-health-row">
          <span>Connection</span>
          <strong>{humanizeStatus(props.availability.status)}</strong>
        </div>
        <div className="vacuum-source-health-row">
          <span>Runtime</span>
          <strong>{runtimeLabel}</strong>
        </div>
        <div className="vacuum-source-health-row">
          <span>Robot source</span>
          <strong>{sourceStatus}</strong>
        </div>
        {sourceDetail ? <p>{sourceDetail}</p> : null}
      </div>
    </section>
  );
}

function UnsupportedFeatureCard(props: { title: string; detail: string }): JSX.Element {
  return (
    <section className="vacuum-panel-card vacuum-panel-card--progress-idle">
      <p className="vacuum-panel-card__eyebrow">{props.title}</p>
      <p className="vacuum-progress-idle-hint">{props.detail}</p>
    </section>
  );
}

function RecentMissionsCard(props: { missions: VacuumMissionSnapshot[] }): JSX.Element | null {
  const missions = props.missions.filter((mission) => isTerminalMissionStatus(mission.status)).slice(0, 4);
  if (missions.length === 0) {
    return null;
  }

  return (
    <section className="vacuum-panel-card vacuum-panel-card--recent-missions">
      <div className="vacuum-panel-card__head">
        <p className="vacuum-panel-card__eyebrow">Recent Missions</p>
        <span className="vacuum-clean-area-badge">{missions.length}</span>
      </div>
      <div className="vacuum-recent-mission-list">
        {missions.map((mission) => {
          const completedAt = mission.result?.completedAt ?? mission.updatedAt ?? mission.startedAt;
          const progress = mission.progress.percent == null ? null : formatPercent(mission.progress.percent);
          const area = mission.progress.areaCoveredSqM == null ? null : formatArea(mission.progress.areaCoveredSqM);
          const resultLabel = getMissionResultLabel(mission);
          return (
            <div key={mission.id} className={`vacuum-recent-mission vacuum-recent-mission--${getMissionResultTone(resultLabel)}`}>
              <div className="vacuum-recent-mission__main">
                <strong>{getMissionDisplayName(mission)}</strong>
                <span>{mission.result?.summary ?? mission.phase}</span>
              </div>
              <div className="vacuum-recent-mission__meta">
                <span>{resultLabel}</span>
                <span>{progress ?? area ?? formatMissionTime(completedAt)}</span>
              </div>
            </div>
          );
        })}
      </div>
    </section>
  );
}

type VacuumControlPanelContentProps = {
  backend: VacuumAdapterBackendId;
  onBackendChange: (backend: VacuumAdapterBackendId) => void;
};

function VacuumControlPanelContent(props: VacuumControlPanelContentProps) {
  const adapter = useVacuumAdapter({ backend: props.backend });
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
  const [missionCommandError, setMissionCommandError] = useState<string | null>(null);
  const [cleanAreaCoveredCellKeys, setCleanAreaCoveredCellKeys] = useState<Set<string>>(() => new Set());
  const [cleanAreaMissionTarget, setCleanAreaMissionTarget] = useState<CleanAreaCoverageTarget | null>(null);
  const [roomZoneToolActive, setRoomZoneToolActive] = useState(false);
  const [roomZoneDraftKind, setRoomZoneDraftKind] = useState<VacuumMapAnnotationKind>("room");
  const [roomZoneDraftName, setRoomZoneDraftName] = useState("Room 1");
  const [roomZoneDraftRect, setRoomZoneDraftRect] = useState<CleanAreaRect | null>(null);
  const [roomZoneValidation, setRoomZoneValidation] = useState<CleanAreaValidation | null>(null);
  const [selectedRoomZoneId, setSelectedRoomZoneId] = useState<string | null>(null);
  const [roomZoneCommandError, setRoomZoneCommandError] = useState<string | null>(null);
  const [activeMode, setActiveMode] = useState<VacuumControlMode>("navigation");
  const [dismissedNavigationTargetKey, setDismissedNavigationTargetKey] = useState<string | null>(null);
  const [dismissedCoverageMissionId, setDismissedCoverageMissionId] = useState<string | null>(null);
  const [recentRoomZoneMissionsDismissedAt, setRecentRoomZoneMissionsDismissedAt] = useState<number | null>(
    () => readDismissedRoomZoneMissionTime(),
  );

  useEffect(() => {
    writeDismissedRoomZoneMissionTime(recentRoomZoneMissionsDismissedAt);
  }, [recentRoomZoneMissionsDismissedAt]);
  const goalStartTimeRef = useRef<number | null>(null);
  const previousCoveragePoseRef = useRef<VacuumPoseCoordinates | null>(null);
  const restoredRoomZoneRecentModeRef = useRef(false);

  const currentPose = snapshot.pose.coordinates;
  const availability = snapshot.availability.status;
  const isMapReceiving = snapshot.map.receiving;
  const poseReady = snapshot.pose.available;
  const readinessReady = snapshot.readiness.ready;
  const navigationState = snapshot.navigation.state;
  const isGoalActive = snapshot.navigation.active;
  const isSendingGoal = snapshot.navigation.isSending;
  const isCancelingGoal = snapshot.navigation.isCanceling;
  const startNavigationSupported = snapshot.capabilities.start_navigation.supported;
  const startCoverageSupported = snapshot.capabilities.start_coverage.supported;
  const cancelMissionSupported = snapshot.capabilities.cancel_mission.supported;
  const cancelNavigationSupported = snapshot.capabilities.cancel_navigation.supported;
  const pauseMissionSupported = snapshot.capabilities.pause_mission.supported;
  const resumeMissionSupported = snapshot.capabilities.resume_mission.supported;
  const retryMissionStepSupported = snapshot.capabilities.retry_mission_step.supported;
  const skipMissionStepSupported = snapshot.capabilities.skip_mission_step.supported;
  const roomSemanticsSupported = snapshot.capabilities.room_semantics.supported;
  const zoneSemanticsSupported = snapshot.capabilities.zone_semantics.supported;
  const roomCleaningSupported = snapshot.capabilities.room_cleaning.supported;
  const zoneCleaningSupported = snapshot.capabilities.zone_cleaning.supported;
  const mappingStatus = snapshot.mapping;
  const mappingState = mapAdapterMappingState(mappingStatus.state);
  const autoMappingSupported = snapshot.capabilities.auto_mapping.supported;
  const mappingSessionSupported = snapshot.capabilities.mapping_session.supported;
  const mapSupported = snapshot.capabilities.map.supported;
  const mapSurfaceAvailable = mapSupported;
  const navigationSupported = startNavigationSupported || snapshot.capabilities.go_to_location.supported;
  const cleanAreaSupported = startCoverageSupported || snapshot.capabilities.coverage_mission.supported;
  const roomsZonesSupported =
    mapSurfaceAvailable && (roomSemanticsSupported || zoneSemanticsSupported || roomCleaningSupported || zoneCleaningSupported);
  const manualControlSupported = snapshot.capabilities.manual_control.supported;
  const isBasicRobotProfile =
    !mapSurfaceAvailable &&
    !navigationSupported &&
    !cleanAreaSupported &&
    !roomsZonesSupported &&
    !mappingSessionSupported &&
    !autoMappingSupported &&
    !manualControlSupported;
  const cleanAreaCoverageConfig = useMemo(
    () =>
      buildCleanAreaCoverageRuntimeConfig({
        profile: DEFAULT_CLEAN_AREA_COVERAGE_PROFILE,
        mapMetadata,
      }),
    [mapMetadata],
  );
  const liveCleanAreaCoverageTarget = useMemo(
    () =>
      buildCleanAreaCoverageTarget(cleanAreaRect, snapshot.map.grid, {
        minimumUsefulCleanableRegionSqM: cleanAreaCoverageConfig.minimumUsefulCleanableRegionSqM,
      }),
    [cleanAreaCoverageConfig.minimumUsefulCleanableRegionSqM, cleanAreaRect, snapshot.map.grid],
  );
  const cleanAreaCoverageTarget = cleanAreaMissionTarget ?? liveCleanAreaCoverageTarget;
  const cleanAreaCoverage = useMemo(
    () =>
      buildCleanAreaCoverageSnapshot({
        target: cleanAreaCoverageTarget,
        coveredCellKeys: cleanAreaCoveredCellKeys,
        swathWidth: cleanAreaCoverageConfig.cleaningSwathWidthM,
      }),
    [cleanAreaCoverageConfig.cleaningSwathWidthM, cleanAreaCoverageTarget, cleanAreaCoveredCellKeys],
  );
  const roomZoneAnnotations = snapshot.map.annotations;
  const currentMapId = currentMapAnnotationId(snapshot);
  const latestRecentRoomZoneMission = useMemo(
    () =>
      snapshot.missions.recent.find((mission) => {
        if (!isRoomZoneMissionSnapshot(mission) || !isTerminalMissionStatus(mission.status)) {
          return false;
        }
        if (recentRoomZoneMissionsDismissedAt == null) {
          return true;
        }
        const terminalTime = missionTerminalTime(mission);
        return terminalTime != null && terminalTime > recentRoomZoneMissionsDismissedAt;
      }) ?? null,
    [recentRoomZoneMissionsDismissedAt, snapshot.missions.recent],
  );
  const latestRecentRoomZoneMissionTime = missionTerminalTime(latestRecentRoomZoneMission);
  const shouldRestoreRecentRoomZoneMode =
    latestRecentRoomZoneMissionTime != null &&
    metadataClock - latestRecentRoomZoneMissionTime >= 0 &&
    metadataClock - latestRecentRoomZoneMissionTime <= RECENT_ROOM_ZONE_MODE_RESTORE_MAX_AGE_MS;
  const activeRoomZoneTarget = missionTargetRecord(snapshot.activeMission);
  const activeRoomZoneMissionRect = coverageMissionArea(snapshot.activeMission);
  const isActiveRoomZoneSnapshot = isRoomZoneMissionSnapshot(snapshot.activeMission);
  const activeRoomZoneAnnotationRecord =
    isActiveRoomZoneSnapshot
      ? activeRoomZoneTarget?.annotation && typeof activeRoomZoneTarget.annotation === "object"
        ? (activeRoomZoneTarget.annotation as Record<string, unknown>)
        : null
      : null;
  const activeRoomZoneAnnotationId =
    typeof activeRoomZoneAnnotationRecord?.id === "string" ? activeRoomZoneAnnotationRecord.id : null;
  const recentRoomZoneTarget = missionTargetRecord(latestRecentRoomZoneMission);
  const recentRoomZoneAnnotationRecord =
    recentRoomZoneTarget?.annotation && typeof recentRoomZoneTarget.annotation === "object"
      ? (recentRoomZoneTarget.annotation as Record<string, unknown>)
      : null;
  const recentRoomZoneAnnotationId =
    typeof recentRoomZoneAnnotationRecord?.id === "string" ? recentRoomZoneAnnotationRecord.id : null;
  const recentRoomZoneAnnotationMapId =
    typeof recentRoomZoneAnnotationRecord?.mapId === "string" ? recentRoomZoneAnnotationRecord.mapId : null;
  const recentRoomZoneAnnotationMatchesMap =
    recentRoomZoneAnnotationRecord != null && (recentRoomZoneAnnotationMapId == null || recentRoomZoneAnnotationMapId === currentMapId);
  const recoveredRoomZoneSelectionId = selectedRoomZoneId ?? activeRoomZoneAnnotationId ?? (recentRoomZoneAnnotationMatchesMap ? recentRoomZoneAnnotationId : null);
  const selectedRoomZone =
    roomZoneAnnotations.find((annotation) => annotation.id === recoveredRoomZoneSelectionId) ??
    (activeRoomZoneAnnotationRecord && isActiveRoomZoneSnapshot && snapshot.activeMission
      ? {
          id: activeRoomZoneAnnotationId ?? snapshot.activeMission.id,
          kind: snapshot.activeMission.type === "room_cleaning" || snapshot.activeMission.requestedCommand === "start_room_cleaning" ? "room" : "zone",
          name:
            typeof activeRoomZoneAnnotationRecord.name === "string"
              ? activeRoomZoneAnnotationRecord.name
            : snapshot.activeMission.type === "room_cleaning" || snapshot.activeMission.requestedCommand === "start_room_cleaning"
                ? "Room"
                : "Zone",
          area: activeRoomZoneMissionRect
            ? cleanAreaRectToAnnotationArea(activeRoomZoneMissionRect)
            : { shape: "rectangle", minX: 0, minY: 0, maxX: 0, maxY: 0 },
          mapId: typeof activeRoomZoneAnnotationRecord.mapId === "string" ? activeRoomZoneAnnotationRecord.mapId : null,
          createdAt: snapshot.activeMission.startedAt ?? Date.now(),
          updatedAt: snapshot.activeMission.updatedAt ?? Date.now(),
        }
      : recentRoomZoneAnnotationRecord && latestRecentRoomZoneMission && recentRoomZoneAnnotationMatchesMap
        ? {
            id: recentRoomZoneAnnotationId ?? latestRecentRoomZoneMission.id,
            kind:
              latestRecentRoomZoneMission.type === "room_cleaning" ||
              latestRecentRoomZoneMission.requestedCommand === "start_room_cleaning"
                ? "room"
                : "zone",
            name:
              typeof recentRoomZoneAnnotationRecord.name === "string"
                ? recentRoomZoneAnnotationRecord.name
                : latestRecentRoomZoneMission.type === "room_cleaning" ||
                    latestRecentRoomZoneMission.requestedCommand === "start_room_cleaning"
                  ? "Room"
                  : "Zone",
            area: coverageMissionArea(latestRecentRoomZoneMission)
              ? cleanAreaRectToAnnotationArea(coverageMissionArea(latestRecentRoomZoneMission)!)
              : { shape: "rectangle", minX: 0, minY: 0, maxX: 0, maxY: 0 },
            mapId: recentRoomZoneAnnotationMapId,
            createdAt: latestRecentRoomZoneMission.startedAt ?? Date.now(),
            updatedAt: latestRecentRoomZoneMission.updatedAt ?? Date.now(),
          }
      : null);
  const selectedRoomZoneRect = selectedRoomZone ? annotationAreaToCleanAreaRect(selectedRoomZone.area) : null;
  const selectedRoomZoneCoverageTarget = useMemo(
    () =>
      buildCleanAreaCoverageTarget(selectedRoomZoneRect, snapshot.map.grid, {
        minimumUsefulCleanableRegionSqM: cleanAreaCoverageConfig.minimumUsefulCleanableRegionSqM,
      }),
    [cleanAreaCoverageConfig.minimumUsefulCleanableRegionSqM, selectedRoomZoneRect, snapshot.map.grid],
  );
  const selectedRoomZoneCoverage = useMemo(
    () =>
      buildCleanAreaCoverageSnapshot({
        target: selectedRoomZoneCoverageTarget,
        coveredCellKeys: new Set(),
        swathWidth: cleanAreaCoverageConfig.cleaningSwathWidthM,
      }),
    [cleanAreaCoverageConfig.cleaningSwathWidthM, selectedRoomZoneCoverageTarget],
  );
  const selectedRoomZoneWaypoints = useMemo(
    () =>
      selectedRoomZoneRect && selectedRoomZoneCoverageTarget && selectedRoomZoneCoverageTarget.cleanableCells.length > 0
        ? buildLawnmowerWaypoints({
            rect: selectedRoomZoneRect,
            spacing: cleanAreaCoverageConfig.laneSpacingM,
            swathWidth: cleanAreaCoverageConfig.cleaningSwathWidthM,
            boundaryExtensionM: cleanAreaCoverageConfig.boundaryExtensionM,
            target: selectedRoomZoneCoverageTarget,
          })
        : [],
    [cleanAreaCoverageConfig, selectedRoomZoneCoverageTarget, selectedRoomZoneRect],
  );
  const selectedRoomZoneMetrics = getCleanAreaMetrics(selectedRoomZoneWaypoints, 0);
  const selectedRoomZoneTargetStatus: RoomZoneTargetStatus = (() => {
    if (!selectedRoomZone) {
      return "none";
    }
    if (!selectedRoomZoneRect || !selectedRoomZoneCoverage || selectedRoomZoneCoverage.targetCells === 0) {
      return "invalid";
    }
    if (
      selectedRoomZoneCoverage.occupiedCells > 0 ||
      selectedRoomZoneCoverage.unknownCells > 0 ||
      selectedRoomZoneCoverage.outOfBoundsCells > 0 ||
      selectedRoomZoneCoverage.skippedSmallRegionCells > 0
    ) {
      return "partial";
    }
    return "cleanable";
  })();
  const selectedRoomZoneTargetLabel = (() => {
    if (selectedRoomZoneTargetStatus === "cleanable") {
      return "Cleanable";
    }
    if (selectedRoomZoneTargetStatus === "partial") {
      return "Partially cleanable";
    }
    if (selectedRoomZoneTargetStatus === "invalid") {
      return "Invalid target";
    }
    return "No target";
  })();
  const selectedRoomZoneTargetDetail = (() => {
    if (!selectedRoomZone) {
      return "Select a saved room or zone to preview it as a cleaning target.";
    }
    if (!selectedRoomZoneRect) {
      return "This saved area shape is not supported by the prototype preview yet.";
    }
    if (!snapshot.map.grid) {
      return "Map data is unavailable, so this target cannot be evaluated.";
    }
    if (!selectedRoomZoneCoverage || selectedRoomZoneCoverage.targetCells === 0) {
      return "No cleanable free-space cells were found in this target.";
    }
    if (selectedRoomZoneTargetStatus === "partial") {
      const parts: string[] = [];
      if (selectedRoomZoneCoverage.occupiedCells > 0) parts.push(`${selectedRoomZoneCoverage.occupiedCells} occupied`);
      if (selectedRoomZoneCoverage.unknownCells > 0) parts.push(`${selectedRoomZoneCoverage.unknownCells} unknown`);
      if (selectedRoomZoneCoverage.outOfBoundsCells > 0) parts.push(`${selectedRoomZoneCoverage.outOfBoundsCells} out of bounds`);
      if (selectedRoomZoneCoverage.skippedSmallRegionCells > 0) parts.push(`${selectedRoomZoneCoverage.skippedSmallRegionCells} too-small region cells`);
      const skipDetail = parts.length > 0 ? ` (${parts.join(", ")})` : "";
      return `Some cells will be skipped${skipDetail}. The cleanable portion is used for preview and coverage.`;
    }
    return "This saved area can be expressed as a full coverage target.";
  })();

  const activeMissionType = snapshot.activeMission?.type ?? null;
  const activeNavigationMission = snapshot.activeMission?.type === "navigation" ? snapshot.activeMission : null;
  const isRuntimeNavigationActive =
    activeNavigationMission != null && ["preparing", "running", "canceling"].includes(activeNavigationMission.status);
  const rawActiveCoverageMission =
    snapshot.activeMission?.type === "coverage" ||
    snapshot.activeMission?.type === "room_cleaning" ||
    snapshot.activeMission?.type === "zone_cleaning"
      ? snapshot.activeMission
      : null;
  const rawCoverageMissionTerminal = rawActiveCoverageMission ? isTerminalMissionStatus(rawActiveCoverageMission.status) : false;
  const activeCoverageMission =
    rawActiveCoverageMission && !(rawCoverageMissionTerminal && rawActiveCoverageMission.id === dismissedCoverageMissionId)
      ? rawActiveCoverageMission
      : null;
  const activeCoverageMissionState = mapCoverageMissionState(activeCoverageMission);
  const activeRoomZoneMission =
    isRoomZoneMissionSnapshot(activeCoverageMission)
      ? activeCoverageMission
      : null;
  const isRoomZoneCleaningRuntimeActive =
    activeRoomZoneMission != null && !isTerminalMissionStatus(activeRoomZoneMission.status);
  const activeCoverageArea = coverageMissionArea(activeCoverageMission);
  const activeCoverageRoute = coverageMissionRoute(activeCoverageMission);
  const activeCoverageTarget = useMemo(
    () =>
      buildCleanAreaCoverageTarget(activeCoverageArea, snapshot.map.grid, {
        minimumUsefulCleanableRegionSqM: cleanAreaCoverageConfig.minimumUsefulCleanableRegionSqM,
      }),
    [activeCoverageArea, cleanAreaCoverageConfig.minimumUsefulCleanableRegionSqM, snapshot.map.grid],
  );
  const activeCoverageSnapshot = useMemo(() => {
    return buildRuntimeCoverageSnapshot({
      mission: activeCoverageMission,
      target: activeCoverageTarget,
      swathWidth: cleanAreaCoverageConfig.cleaningSwathWidthM,
    });
  }, [activeCoverageMission, activeCoverageTarget, cleanAreaCoverageConfig.cleaningSwathWidthM]);
  const displayedCleanAreaCoverage = activeCoverageMission ? activeCoverageSnapshot : cleanAreaCoverage;
  const displayedCleanAreaCoverageTarget = activeCoverageTarget ?? cleanAreaCoverageTarget;
  const snapshotNavigationTarget = snapshot.navigation.currentTarget;
  const isTerminalNavigationSnapshot =
    navigationState === "completed" || navigationState === "canceled" || navigationState === "failed";
  const snapshotNavigationDismissKey = navigationDestinationDismissKey(
    snapshotNavigationTarget,
    activeNavigationMission?.id,
    activeNavigationMission?.status ?? navigationState,
    activeNavigationMission?.updatedAt,
  );
  const isSnapshotNavigationTargetDismissed =
    !isGoalActive &&
    isTerminalNavigationSnapshot &&
    snapshotNavigationDismissKey != null &&
    dismissedNavigationTargetKey === snapshotNavigationDismissKey;
  const snapshotNavigationTargetVisible =
    snapshotNavigationTarget != null &&
    !isSnapshotNavigationTargetDismissed &&
    (activeMissionType === "navigation" ||
      isGoalActive ||
      navigationState === "completed" ||
      navigationState === "canceled" ||
      navigationState === "failed");
  const displayedSentTarget = snapshotNavigationTargetVisible ? snapshotNavigationTarget : sentTarget;
  const displayedDraftTarget = snapshotNavigationTargetVisible ? null : draftTarget;
  const displayedTarget = displayedSentTarget ?? displayedDraftTarget;
  const hasTarget = displayedTarget != null;
  const routeVisualState = getRouteVisualState(navigationState, isGoalActive, displayedDraftTarget != null, hasTarget);
  const displayedPlanPoints =
    displayedSentTarget != null && routeVisualState !== "staged" && routeVisualState !== "canceled"
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
  const systemChips = mapSurfaceAvailable
    ? [
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
      ] as const
    : [
        {
          label: "Connected",
          icon: "connected" as const,
          state: getChipTone(availability === "online", "success"),
          pulsing: false,
        },
        {
          label: "Runtime",
          icon: "ready" as const,
          state: getChipTone(snapshot.health?.runtimeStatus === "online", "success"),
          pulsing: false,
        },
        {
          label: "Source",
          icon: "connected" as const,
          state: getChipTone(Boolean(snapshot.source && !snapshot.source.stale && snapshot.source.status === "reachable"), "success"),
          pulsing: false,
        },
      ] as const;
  const taskChips = mapSurfaceAvailable
    ? [
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
      ] as const
    : [
        {
          label: getBasicActivityLabel(snapshot.activity),
          icon: "ready" as const,
          state: getActivityTone(snapshot.activity) === "danger" ? "inactive" as const : getChipTone(Boolean(snapshot.activity), "active"),
        },
        {
          label: formatDockBatterySummary(snapshot.activity, snapshot.dock, snapshot.battery, {
            omitDockWhenActivityDuplicates: true,
          }),
          icon: "battery" as const,
          state: getChipTone(snapshot.battery.readiness === "ready" || getDockTone(snapshot.dock) === "ready", "success"),
        },
      ] as const;
  const readinessIssue = snapshot.readiness.blockingReasons[0] ?? null;
  const targetDistanceLabel = displayedTarget ? formatDistance(destinationDistance) : null;
  const targetHeadingLabel = displayedTarget ? headingLabel(displayedTarget.yaw) : null;
  const targetBearingLabel =
    destinationBearing == null ? "n/a" : `${Math.round(((destinationBearing % 360) + 360) % 360)}°`;
  const destinationBadgeLabel = (() => {
    if (displayedSentTarget != null && isGoalActive) {
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
    isBasicRobotProfile
      ? "Basic Robot"
      : activeMode === "mapping"
      ? "Mapping"
      : activeMode === "clean"
        ? "Clean Area"
        : activeMode === "rooms"
          ? "Rooms / Zones"
          : "Navigate";

  const isMappingWorkflowActive =
    mappingState === "mapping" || mappingState === "paused" || mappingState === "review";
  const isRuntimeCoverageActive =
    activeCoverageMission != null &&
    ["preparing", "running", "paused", "canceling", "resuming", "needs_assistance"].includes(activeCoverageMission.status);
  const displayedCleanAreaState = activeCoverageMissionState ?? cleanAreaState;
  const displayedCleanAreaRect = activeCoverageArea ?? cleanAreaRect;
  const displayedCleanAreaWaypoints = activeCoverageRoute.length > 0 ? activeCoverageRoute : cleanAreaWaypoints;
  const displayedCleanAreaCurrentIndex = activeCoverageMission?.progress.currentStep != null
    ? Math.max(0, activeCoverageMission.progress.currentStep - 1)
    : cleanAreaCurrentIndex;
  const displayedCleanAreaCommandError =
    activeCoverageMission?.status === "needs_assistance" ||
    activeCoverageMission?.status === "failed" ||
    activeCoverageMission?.status === "unsupported"
      ? activeCoverageMission.error?.message ?? cleanAreaCommandError
      : cleanAreaCommandError;
  const isCleanAreaRunning =
    displayedCleanAreaState === "preparing" || displayedCleanAreaState === "running" || displayedCleanAreaState === "canceling";
  const isCleanAreaActive = isRuntimeCoverageActive || isCleanAreaRunning || displayedCleanAreaState === "paused";
  const hasCleanAreaDraft = Boolean(displayedCleanAreaRect) && displayedCleanAreaState !== "idle";
  const isCleanAreaModeLocked = isCleanAreaActive || cleanAreaToolActive;
  const isRoomZoneModeLocked = roomZoneToolActive || isRoomZoneCleaningRuntimeActive;
  const cleanAreaVisualState = getCleanAreaVisualState(displayedCleanAreaState, cleanAreaToolActive);
  const cleanAreaPreviewPoints =
    displayedCleanAreaWaypoints.length > 0
      ? displayedCleanAreaWaypoints
      : displayedCleanAreaRect
        ? buildLawnmowerWaypoints({
            rect: displayedCleanAreaRect,
            spacing: cleanAreaCoverageConfig.laneSpacingM,
            swathWidth: cleanAreaCoverageConfig.cleaningSwathWidthM,
            boundaryExtensionM: cleanAreaCoverageConfig.boundaryExtensionM,
            target: displayedCleanAreaCoverageTarget,
          })
        : null;
  const cleanAreaMetrics = getCleanAreaMetrics(cleanAreaPreviewPoints ?? [], displayedCleanAreaCurrentIndex);

  useEffect(() => {
    if (activeCoverageMission) {
      return;
    }
    setCleanAreaCoveredCellKeys(new Set());
    previousCoveragePoseRef.current = null;
  }, [activeCoverageMission, cleanAreaCoverageTarget?.signature]);

  useEffect(() => {
    if (activeCoverageMission) {
      previousCoveragePoseRef.current = null;
      return;
    }
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
        swathWidth: cleanAreaCoverageConfig.cleaningSwathWidthM,
      }),
    );
    previousCoveragePoseRef.current = currentPose;
  }, [activeCoverageMission, cleanAreaCoverageConfig.cleaningSwathWidthM, cleanAreaCoverageTarget, cleanAreaState, currentPose]);

  useEffect(() => {
    if (isGoalActive) {
      setDismissedNavigationTargetKey(null);
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

  useEffect(() => {
    if (selectedRoomZoneId && !roomZoneAnnotations.some((annotation) => annotation.id === selectedRoomZoneId)) {
      setSelectedRoomZoneId(null);
    }
  }, [roomZoneAnnotations, selectedRoomZoneId]);

  const isTerminalState =
    routeVisualState === "completed" || routeVisualState === "failed" || routeVisualState === "canceled";
  const runTarget = displayedDraftTarget ?? (isTerminalState ? displayedSentTarget : null);
  const canSendRun =
    Boolean(runTarget) && startNavigationSupported && readinessReady && !isSendingGoal && !isCleanAreaRunning;
  const canCancelRun = (cancelMissionSupported || cancelNavigationSupported) && !isCancelingGoal;
  const canStartCleanArea =
    displayedCleanAreaState === "paused" && activeCoverageMission
      ? resumeMissionSupported
      : Boolean(displayedCleanAreaRect) &&
        Boolean(activeCoverageMission || cleanAreaValidation?.ok) &&
        Boolean(displayedCleanAreaCoverageTarget && displayedCleanAreaCoverageTarget.cleanableCells.length > 0) &&
        startCoverageSupported &&
        readinessReady &&
        !isSendingGoal &&
        !isGoalActive &&
        !isMappingWorkflowActive;
  const canPauseCleanArea = Boolean(activeCoverageMission?.availableActions.includes("pause_mission")) && pauseMissionSupported;
  const canCancelCleanArea = Boolean(activeCoverageMission?.availableActions.includes("cancel_mission")) && cancelMissionSupported;
  const canSaveRoomZone =
    Boolean(roomZoneDraftRect) &&
    Boolean(roomZoneValidation?.ok) &&
    (roomZoneDraftKind === "room" ? roomSemanticsSupported : zoneSemanticsSupported);
  const canStartSelectedRoomZone =
    Boolean(selectedRoomZone) &&
    selectedRoomZoneTargetStatus !== "invalid" &&
    Boolean(selectedRoomZoneCoverageTarget && selectedRoomZoneCoverageTarget.cleanableCells.length > 0) &&
    (selectedRoomZone?.kind === "room" ? roomCleaningSupported : zoneCleaningSupported) &&
    readinessReady &&
    !isSendingGoal &&
    !isGoalActive &&
    !isMappingWorkflowActive &&
    !isCleanAreaActive;
  const canPauseRoomZoneCleaning =
    Boolean(activeRoomZoneMission?.availableActions.includes("pause_mission")) && pauseMissionSupported;
  const canResumeRoomZoneCleaning =
    Boolean(activeRoomZoneMission?.availableActions.includes("resume_mission")) && resumeMissionSupported;
  const canCancelRoomZoneCleaning =
    Boolean(activeRoomZoneMission?.availableActions.includes("cancel_mission")) && cancelMissionSupported;
  const canRetryRoomZoneCleaning =
    Boolean(activeRoomZoneMission?.availableActions.includes("retry_mission_step")) && retryMissionStepSupported;
  const canSkipRoomZoneCleaning =
    Boolean(activeRoomZoneMission?.availableActions.includes("skip_mission_step")) && skipMissionStepSupported;

  const saveDisabledReason: string | null = (() => {
    if (canSaveRoomZone) return null;
    if (!roomZoneDraftRect) return "Draw an area first.";
    if (!roomZoneValidation?.ok) return roomZoneValidation?.message ?? "Area is too small or invalid.";
    if (roomZoneDraftKind === "room" && !roomSemanticsSupported) return "Room semantics not supported.";
    if (roomZoneDraftKind === "zone" && !zoneSemanticsSupported) return "Zone semantics not supported.";
    return "Cannot save yet.";
  })();

  const cleanDisabledReason: string | null = (() => {
    if (isRoomZoneCleaningRuntimeActive) return null;
    if (!selectedRoomZone) return "Select a saved room or zone first.";
    if (selectedRoomZoneTargetStatus === "invalid") return "No cleanable area found in this target.";
    if (!selectedRoomZoneCoverageTarget || selectedRoomZoneCoverageTarget.cleanableCells.length === 0) return "No cleanable cells found in this area.";
    if (selectedRoomZone.kind === "room" && !roomCleaningSupported) return "Room cleaning not supported.";
    if (selectedRoomZone.kind === "zone" && !zoneCleaningSupported) return "Zone cleaning not supported.";
    if (!readinessReady) return snapshot.readiness.blockingReasons[0] ?? "Robot is not ready.";
    if (isSendingGoal || isGoalActive) return "Stop navigation first.";
    if (isMappingWorkflowActive) return "Finish mapping first.";
    if (isCleanAreaActive) return "Stop clean area mission first.";
    return null;
  })();

  const handleCleanAreaChange = useCallback((rect: CleanAreaRect, validation: CleanAreaValidation): void => {
    if (!cleanAreaToolActive && cleanAreaState !== "idle" && cleanAreaState !== "editing") {
      setCleanAreaValidation(validation);
      return;
    }

    const nextTarget = buildCleanAreaCoverageTarget(rect, snapshot.map.grid, {
      minimumUsefulCleanableRegionSqM: cleanAreaCoverageConfig.minimumUsefulCleanableRegionSqM,
    });
    setCleanAreaRect(rect);
    setCleanAreaValidation(validation);
    setCleanAreaMissionTarget(null);
    setCleanAreaWaypoints(
      validation.ok
        ? buildLawnmowerWaypoints({
            rect,
            spacing: cleanAreaCoverageConfig.laneSpacingM,
            swathWidth: cleanAreaCoverageConfig.cleaningSwathWidthM,
            boundaryExtensionM: cleanAreaCoverageConfig.boundaryExtensionM,
            target: nextTarget,
          })
        : [],
    );
    setCleanAreaCommandError(validation.ok ? null : validation.message);
    setCleanAreaState((current) => {
      return current === "idle" || current === "editing" ? "editing" : current;
    });
  }, [cleanAreaCoverageConfig, cleanAreaState, cleanAreaToolActive, snapshot.map.grid]);

  const handleRoomZoneChange = useCallback((rect: CleanAreaRect, validation: CleanAreaValidation): void => {
    setRoomZoneDraftRect(rect);
    setRoomZoneValidation(validation);
    setRoomZoneCommandError(validation.ok ? null : validation.message);
    setSelectedRoomZoneId(null);
  }, []);

  const handleMapAreaChange = useCallback((rect: CleanAreaRect, validation: CleanAreaValidation): void => {
    if (activeMode === "rooms") {
      if (!roomZoneToolActive) {
        return;
      }
      handleRoomZoneChange(rect, validation);
      return;
    }
    handleCleanAreaChange(rect, validation);
  }, [activeMode, handleCleanAreaChange, handleRoomZoneChange, roomZoneToolActive]);

  async function handleSend(overrideTarget?: DraftTarget): Promise<void> {
    const target = overrideTarget ?? runTarget;
    if (!target) {
      return;
    }
    setDismissedNavigationTargetKey(null);
    setSentTarget(target);
    await adapter.sendCommand({ command: "start_navigation", target });
  }

  async function handleCancel(): Promise<void> {
    await adapter.sendCommand({ command: cancelMissionSupported ? "cancel_mission" : "cancel_navigation" });
  }

  function handleClear(): void {
    if (isGoalActive) {
      return;
    }
    if (snapshotNavigationTargetVisible && isTerminalNavigationSnapshot && snapshotNavigationDismissKey) {
      setDismissedNavigationTargetKey(snapshotNavigationDismissKey);
    }
    setDraftTarget(null);
    setSentTarget(null);
  }

  function handleTargetStart(target: DraftTarget): void {
    if (isMappingWorkflowActive || cleanAreaToolActive || roomZoneToolActive || isCleanAreaActive) {
      return;
    }
    setSentTarget(null);
    setDraftTarget(target);
  }

  function handleTargetRotate(yaw: number): void {
    if (isMappingWorkflowActive || cleanAreaToolActive || roomZoneToolActive || isCleanAreaActive) {
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

  function handleActivateRoomZoneTool(): void {
    if (isGoalActive || isMappingWorkflowActive || isCleanAreaActive) {
      return;
    }
    if (roomZoneToolActive) {
      setRoomZoneToolActive(false);
      return;
    }
    setActiveMode("rooms");
    setRoomZoneToolActive(true);
    setRoomZoneCommandError(null);
    setSelectedRoomZoneId(null);
    setRoomZoneDraftName(nextAnnotationName(roomZoneDraftKind, roomZoneAnnotations));
    setDraftTarget(null);
    setSentTarget(null);
  }

  function handleRoomZoneDraftKindChange(kind: VacuumMapAnnotationKind): void {
    setRoomZoneDraftKind(kind);
    if (!roomZoneDraftRect) {
      setRoomZoneDraftName(nextAnnotationName(kind, roomZoneAnnotations));
    }
  }

  async function handleSaveRoomZone(): Promise<void> {
    if (!canSaveRoomZone || !roomZoneDraftRect) {
      return;
    }
    const now = Date.now();
    const trimmedName = roomZoneDraftName.trim();
    const annotation: Omit<VacuumMapAnnotation, "createdAt" | "updatedAt"> = {
      id: `${roomZoneDraftKind}-${now}`,
      kind: roomZoneDraftKind,
      name: trimmedName || (roomZoneDraftKind === "room" ? "Room" : "Zone"),
      area: cleanAreaRectToAnnotationArea(roomZoneDraftRect),
      mapId: currentMapAnnotationId(snapshot),
    };
    setRoomZoneCommandError(null);
    const result = await adapter.sendCommand({ command: "save_map_annotation", annotation });
    if (!result.ok) {
      setRoomZoneCommandError(result.error.message);
      return;
    }
    setSelectedRoomZoneId(annotation.id);
    setRoomZoneDraftRect(null);
    setRoomZoneValidation(null);
    setRoomZoneToolActive(false);
  }

  function handleSelectRoomZone(id: string): void {
    const annotation = roomZoneAnnotations.find((entry) => entry.id === id);
    setSelectedRoomZoneId(id);
    setRoomZoneDraftRect(null);
    setRoomZoneValidation(null);
    setRoomZoneToolActive(false);
    setRoomZoneCommandError(null);
    if (annotation) {
      setRoomZoneDraftKind(annotation.kind);
      setRoomZoneDraftName(annotation.name);
    }
  }

  async function handleDeleteRoomZone(): Promise<void> {
    if (!selectedRoomZone) {
      return;
    }
    setRoomZoneCommandError(null);
    const isSavedAnnotation = roomZoneAnnotations.some((entry) => entry.id === selectedRoomZone.id);
    if (isSavedAnnotation) {
      const result = await adapter.sendCommand({ command: "delete_map_annotation", id: selectedRoomZone.id });
      if (!result.ok) {
        setRoomZoneCommandError(result.error.message);
        return;
      }
    }
    setRecentRoomZoneMissionsDismissedAt(Date.now());
    setSelectedRoomZoneId(null);
  }

  function handleClearRoomZoneDraft(): void {
    setRecentRoomZoneMissionsDismissedAt(Date.now());
    setSelectedRoomZoneId(null);
    setRoomZoneDraftRect(null);
    setRoomZoneValidation(null);
    setRoomZoneToolActive(false);
    setRoomZoneCommandError(null);
  }

  async function handleStartRoomZoneCleaning(): Promise<void> {
    if (!selectedRoomZone || !canStartSelectedRoomZone) {
      return;
    }
    setRoomZoneCommandError(null);
    setDismissedCoverageMissionId(null);
    const result = await adapter.sendCommand({
      command: selectedRoomZone.kind === "room" ? "start_room_cleaning" : "start_zone_cleaning",
      annotation: selectedRoomZone,
      coverage: {
        swathWidth: cleanAreaCoverageConfig.cleaningSwathWidthM,
        laneSpacing: cleanAreaCoverageConfig.laneSpacingM,
        completionThreshold: cleanAreaCoverageConfig.completionThreshold,
        boundaryExtension: cleanAreaCoverageConfig.boundaryExtensionM,
      },
    });
    if (!result.ok) {
      setRoomZoneCommandError(result.error.message);
    }
  }

  async function handlePauseRoomZoneCleaning(): Promise<void> {
    if (!activeRoomZoneMission || !canPauseRoomZoneCleaning) {
      return;
    }
    const result = await adapter.sendCommand({ command: "pause_mission" });
    if (!result.ok) {
      setRoomZoneCommandError(result.error.message);
    }
  }

  async function handleResumeRoomZoneCleaning(): Promise<void> {
    if (!activeRoomZoneMission || !canResumeRoomZoneCleaning) {
      return;
    }
    const result = await adapter.sendCommand({ command: "resume_mission" });
    if (!result.ok) {
      setRoomZoneCommandError(result.error.message);
    }
  }

  async function handleCancelRoomZoneCleaning(): Promise<void> {
    if (!activeRoomZoneMission || !canCancelRoomZoneCleaning) {
      return;
    }
    const result = await adapter.sendCommand({ command: "cancel_mission" });
    if (!result.ok) {
      setRoomZoneCommandError(result.error.message);
    }
  }

  async function handleRetryRoomZoneStep(): Promise<void> {
    if (!activeRoomZoneMission || !canRetryRoomZoneCleaning) {
      return;
    }
    setRoomZoneCommandError(null);
    const result = await adapter.sendCommand({ command: "retry_mission_step" });
    if (!result.ok) {
      setRoomZoneCommandError(result.error.message);
    }
  }

  async function handleSkipRoomZoneStep(): Promise<void> {
    if (!activeRoomZoneMission || !canSkipRoomZoneCleaning) {
      return;
    }
    setRoomZoneCommandError(null);
    const result = await adapter.sendCommand({ command: "skip_mission_step" });
    if (!result.ok) {
      setRoomZoneCommandError(result.error.message);
    }
  }

  function handleConfirmCleanArea(): void {
    if (!cleanAreaRect || !cleanAreaValidation?.ok) {
      return;
    }
    const target = buildCleanAreaCoverageTarget(cleanAreaRect, snapshot.map.grid, {
      minimumUsefulCleanableRegionSqM: cleanAreaCoverageConfig.minimumUsefulCleanableRegionSqM,
    });
    if (!target || target.cleanableCells.length === 0) {
      setCleanAreaCommandError("Area has no cleanable cells.");
      return;
    }
    const waypoints = buildLawnmowerWaypoints({
      rect: cleanAreaRect,
      spacing: cleanAreaCoverageConfig.laneSpacingM,
      swathWidth: cleanAreaCoverageConfig.cleaningSwathWidthM,
      boundaryExtensionM: cleanAreaCoverageConfig.boundaryExtensionM,
      target,
    });
    setCleanAreaToolActive(false);
    setCleanAreaMissionTarget(target);
    setCleanAreaWaypoints(waypoints);
    setCleanAreaCurrentIndex(0);
    setCleanAreaCommandError(null);
    setCleanAreaState("confirmed");
  }

  function handleClearCleanArea(): void {
    if (isCleanAreaActive) {
      return;
    }
    if (activeCoverageMission && isTerminalMissionStatus(activeCoverageMission.status)) {
      setDismissedCoverageMissionId(activeCoverageMission.id);
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

  async function handleStartCleanArea(): Promise<void> {
    if (!canStartCleanArea) {
      return;
    }
    setCleanAreaCommandError(null);
    if (activeCoverageMission && displayedCleanAreaState === "paused") {
      const result = await adapter.sendCommand({ command: "resume_mission" });
      if (!result.ok) {
        setCleanAreaCommandError(result.error.message);
      }
      return;
    }
    if (!displayedCleanAreaRect) {
      return;
    }
    setDismissedCoverageMissionId(null);
    setCleanAreaCoveredCellKeys(new Set());
    previousCoveragePoseRef.current = null;
    setCleanAreaToolActive(false);
    setCleanAreaState("preparing");
    const result = await adapter.sendCommand({
      command: "start_coverage",
      area: cleanAreaRectToCoverageArea(displayedCleanAreaRect),
      coverage: {
        swathWidth: cleanAreaCoverageConfig.cleaningSwathWidthM,
        laneSpacing: cleanAreaCoverageConfig.laneSpacingM,
        completionThreshold: cleanAreaCoverageConfig.completionThreshold,
        boundaryExtension: cleanAreaCoverageConfig.boundaryExtensionM,
      },
    });
    if (!result.ok) {
      setCleanAreaCommandError(result.error.message);
      setCleanAreaState("failed");
    }
  }

  async function handlePauseCleanArea(): Promise<void> {
    if (!activeCoverageMission || !pauseMissionSupported) {
      return;
    }
    const result = await adapter.sendCommand({ command: "pause_mission" });
    if (!result.ok) {
      setCleanAreaCommandError(result.error.message);
    }
  }

  async function handleCancelCleanArea(): Promise<void> {
    if (!activeCoverageMission || !cancelMissionSupported) {
      return;
    }
    const result = await adapter.sendCommand({ command: "cancel_mission" });
    if (!result.ok) {
      setCleanAreaCommandError(result.error.message);
    }
  }

  async function handleReturnToDock(): Promise<void> {
    setMissionCommandError(null);
    const result = await adapter.sendCommand({ command: "return_to_dock" });
    if (!result.ok) {
      setMissionCommandError(result.error.message);
    }
  }

  async function handleBasicCommand(command: "start_cleaning" | "pause" | "resume" | "stop" | "return_to_dock"): Promise<void> {
    setMissionCommandError(null);
    const result = await adapter.sendCommand({ command });
    if (!result.ok) {
      setMissionCommandError(result.error.message);
    }
  }

  async function handleCleaningSettingCommand(command: "set_fan_speed" | "set_water_usage", value: string): Promise<void> {
    setMissionCommandError(null);
    const result = await adapter.sendCommand({ command, value });
    if (!result.ok) {
      setMissionCommandError(result.error.message);
    }
  }

  async function handleRetryCleanAreaWaypoint(): Promise<void> {
    if (!activeCoverageMission || !retryMissionStepSupported) {
      return;
    }
    setCleanAreaCommandError(null);
    const result = await adapter.sendCommand({ command: "retry_mission_step" });
    if (!result.ok) {
      setCleanAreaCommandError(result.error.message);
    }
  }

  async function handleSkipCleanAreaWaypoint(): Promise<void> {
    if (!activeCoverageMission || !skipMissionStepSupported) {
      return;
    }
    setCleanAreaCommandError(null);
    const result = await adapter.sendCommand({ command: "skip_mission_step" });
    if (!result.ok) {
      setCleanAreaCommandError(result.error.message);
    }
  }

  useEffect(() => {
    if (activeMode === "mapping" && !mappingSessionSupported && !autoMappingSupported) {
      setActiveMode(navigationSupported ? "navigation" : cleanAreaSupported ? "clean" : roomsZonesSupported ? "rooms" : "navigation");
      return;
    }
    if (activeMode === "navigation" && !navigationSupported) {
      setActiveMode(cleanAreaSupported ? "clean" : roomsZonesSupported ? "rooms" : mappingSessionSupported || autoMappingSupported ? "mapping" : "navigation");
      return;
    }
    if (activeMode === "clean" && !cleanAreaSupported) {
      setActiveMode(navigationSupported ? "navigation" : roomsZonesSupported ? "rooms" : mappingSessionSupported || autoMappingSupported ? "mapping" : "navigation");
      return;
    }
    if (activeMode === "rooms" && !roomsZonesSupported) {
      setActiveMode(navigationSupported ? "navigation" : cleanAreaSupported ? "clean" : mappingSessionSupported || autoMappingSupported ? "mapping" : "navigation");
      return;
    }
    if (isMappingWorkflowActive) {
      setActiveMode("mapping");
    } else if (activeRoomZoneMission) {
      setActiveMode("rooms");
    } else if (roomZoneToolActive) {
      setActiveMode("rooms");
    } else if (isCleanAreaActive || cleanAreaToolActive || activeCoverageMission) {
      setActiveMode("clean");
    } else if (
      !restoredRoomZoneRecentModeRef.current &&
      latestRecentRoomZoneMission &&
      shouldRestoreRecentRoomZoneMode &&
      activeMode === "navigation" &&
      !hasTarget
    ) {
      restoredRoomZoneRecentModeRef.current = true;
      setActiveMode("rooms");
    } else if (activeMode === "clean" && hasCleanAreaDraft) {
      return;
    } else if (isGoalActive || isRuntimeNavigationActive) {
      setActiveMode("navigation");
    }
  }, [
    activeMissionType,
    activeCoverageMission,
    activeRoomZoneMission,
    activeMode,
    hasCleanAreaDraft,
    hasTarget,
    isRuntimeNavigationActive,
    isMappingWorkflowActive,
    isCleanAreaActive,
    cleanAreaToolActive,
    latestRecentRoomZoneMission,
    shouldRestoreRecentRoomZoneMode,
    roomZoneToolActive,
    isGoalActive,
    autoMappingSupported,
    cleanAreaSupported,
    mappingSessionSupported,
    navigationSupported,
    roomsZonesSupported,
  ]);

  const mapRoomZonePreviewRect = activeRoomZoneMission ? activeCoverageArea : roomZoneDraftRect ?? selectedRoomZoneRect;
  const mapCleanAreaRect = activeMode === "rooms" ? mapRoomZonePreviewRect : displayedCleanAreaRect;
  const mapCleanAreaCoverage = activeMode === "rooms" ? activeRoomZoneMission ? activeCoverageSnapshot : selectedRoomZoneCoverage : displayedCleanAreaCoverage;
  const mapCleanAreaToolActive = activeMode === "rooms" ? roomZoneToolActive : cleanAreaToolActive;
  const mapCleanAreaEditable = activeMode === "rooms" ? roomZoneToolActive : cleanAreaToolActive;
  const mapCleanAreaVisualState = activeMode === "rooms"
    ? roomZoneToolActive ? "editing" : activeRoomZoneMission ? cleanAreaVisualState : mapRoomZonePreviewRect ? "confirmed" : "idle"
    : cleanAreaVisualState;
  const mapCleanAreaPreviewPoints = activeMode === "rooms" ? activeRoomZoneMission ? cleanAreaPreviewPoints : selectedRoomZoneWaypoints : cleanAreaPreviewPoints;

  // ── Sidebar group gate conditions (basic robot profile only) ──
  const sidebarAttachmentItems = snapshot.attachments?.items ?? [];
  const sidebarDockComponentItems = snapshot.dock?.components ?? [];
  const sidebarConsumableItems = snapshot.maintenance?.consumables ?? [];
  const sidebarMapTargetSegments = snapshot.map.targets?.segments ?? [];
  const sidebarMapTargetZones = snapshot.map.targets?.zones ?? [];
  const sidebarShowMapTargets = sidebarMapTargetSegments.length > 0 || sidebarMapTargetZones.length > 0;
  const layeredMapPreviewAvailable = hasRenderableLayeredMapPreview(snapshot.map.layeredPreview);
  const sidebarShowMapPreview = layeredMapPreviewAvailable;
  const sidebarShowAttachments = snapshot.capabilities.attachments.supported && sidebarAttachmentItems.length > 0;
  const sidebarShowDockComponents = snapshot.capabilities.dock_components.supported && sidebarDockComponentItems.length > 0;
  const sidebarShowFanSpeed = !!(snapshot.cleaningSettings?.fanSpeed && snapshot.capabilities.fan_speed.supported);
  const sidebarShowWaterUsage = !!(snapshot.cleaningSettings?.waterUsage && snapshot.capabilities.water_usage.supported);
  const sidebarShowCleaningSettings = sidebarShowFanSpeed || sidebarShowWaterUsage;
  const sidebarStatsCurrentData = snapshot.statistics?.current;
  const sidebarShowCurrentStats =
    snapshot.capabilities.statistics.supported &&
    sidebarStatsCurrentData != null &&
    ((typeof sidebarStatsCurrentData.durationSeconds === "number" && Number.isFinite(sidebarStatsCurrentData.durationSeconds)) ||
      (typeof sidebarStatsCurrentData.areaSquareMeters === "number" && Number.isFinite(sidebarStatsCurrentData.areaSquareMeters)));
  const sidebarShowMaintenance = snapshot.capabilities.consumables.supported && sidebarConsumableItems.length > 0;
  const sidebarShowSourceHealth = !!(snapshot.health ?? snapshot.source ?? snapshot.availability.detail ?? snapshot.fault.faults[0]);
  const sidebarPeripheralAttentionLine = derivePeripheralAttentionLine(
    snapshot.attachments,
    sidebarDockComponentItems,
    sidebarConsumableItems,
  );

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
            <label className="vacuum-backend-select">
              <span>Adapter</span>
              <select
                value={props.backend}
                onChange={(event) => {
                  const nextBackend = normalizeVacuumAdapterBackend(event.target.value);
                  if (nextBackend) {
                    props.onBackendChange(nextBackend);
                  }
                }}
              >
                <option value={ADAPTER_BACKEND_OPTIONS[0]}>Simulation</option>
                <option value={ADAPTER_BACKEND_OPTIONS[1]}>Valetudo runtime</option>
              </select>
            </label>
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
          {mapSurfaceAvailable ? (
            <MapCanvas
              currentPose={currentPose}
              planPoints={displayedPlanPoints}
              draftTarget={displayedDraftTarget}
              sentTarget={displayedSentTarget}
              cleanAreaRect={mapCleanAreaRect}
              cleanAreaPreviewPoints={mapCleanAreaPreviewPoints}
              cleanAreaCurrentIndex={displayedCleanAreaCurrentIndex}
              cleanAreaCoverage={mapCleanAreaCoverage}
              cleanAreaToolActive={mapCleanAreaToolActive}
              cleanAreaEditable={mapCleanAreaEditable}
              cleanAreaVisualState={mapCleanAreaVisualState}
              mapAnnotations={activeMode === "rooms" ? roomZoneAnnotations : []}
              selectedMapAnnotationId={activeMode === "rooms" ? recoveredRoomZoneSelectionId : null}
              interactionMode={activeMode}
              routeVisualState={routeVisualState}
              isGoalActive={isGoalActive}
              mappingState={mappingState}
              disableTargetSelection={activeMode !== "navigation" || isMappingWorkflowActive || cleanAreaToolActive || roomZoneToolActive || isCleanAreaActive}
              targetDistance={destinationDistance}
              adapterMapGrid={snapshot.map.grid}
              adapterMapMetadata={snapshot.map.metadata}
              onTargetStart={handleTargetStart}
              onTargetRotate={handleTargetRotate}
              onCleanAreaChange={handleMapAreaChange}
              onMapAnnotationSelect={handleSelectRoomZone}
              onMapMetadataChange={setMapMetadata}
            />
          ) : layeredMapPreviewAvailable ? (
            <ValetudoMainMapPreview
              preview={snapshot.map.layeredPreview}
            />
          ) : (
            <NoMapCanvasPlaceholder
              mapDetail={snapshot.map.detail}
            />
          )}

          <div className="vacuum-sidebar">
            {isBasicRobotProfile ? (
              <div className="vacuum-mode-content">
                {/* ── Operate ─────────────────────────── */}
                <div className="vacuum-card-group">
                  <div className="vacuum-card-group__head vacuum-card-group__head--static" aria-label="Operate group">
                    <span className="vacuum-card-group__title">Operate</span>
                  </div>
                  <div className="vacuum-card-group__body">
                    <RobotOverviewCard
                      identity={snapshot.identity}
                      availability={snapshot.availability}
                      battery={snapshot.battery}
                      dock={snapshot.dock}
                      health={snapshot.health}
                      source={snapshot.source}
                      activity={snapshot.activity}
                      fault={snapshot.fault}
                      peripheralAttentionLine={sidebarPeripheralAttentionLine}
                    />
                    <BasicControlsCard
                      capabilities={snapshot.capabilities}
                      commandError={missionCommandError}
                      onStart={() => void handleBasicCommand("start_cleaning")}
                      onPause={() => void handleBasicCommand("pause")}
                      onResume={() => void handleBasicCommand("resume")}
                      onStop={() => void handleBasicCommand("stop")}
                      onReturnToDock={() => void handleBasicCommand("return_to_dock")}
                    />
                    {sidebarShowCurrentStats ? (
                      <CurrentStatisticsCard
                        statistics={snapshot.statistics}
                        capabilities={snapshot.capabilities}
                      />
                    ) : null}
                    {sidebarShowMapPreview ? (
                      <MapPreviewCard preview={snapshot.map.layeredPreview} />
                    ) : null}
                    {sidebarShowMapTargets ? (
                      <MapTargetsCard targets={snapshot.map.targets} />
                    ) : null}
                  </div>
                </div>

                {/* ── Readiness ────────────────────────── */}
                <div className="vacuum-card-group">
                  <div className="vacuum-card-group__head vacuum-card-group__head--static" aria-label="Readiness group">
                    <span className="vacuum-card-group__title">Readiness</span>
                  </div>
                  <div className="vacuum-card-group__body">
                    <BatteryDockCard
                      battery={snapshot.battery}
                      dock={snapshot.dock}
                      capabilities={snapshot.capabilities}
                    />
                    {sidebarShowAttachments ? (
                      <AttachmentsCard
                        attachments={snapshot.attachments}
                        capabilities={snapshot.capabilities}
                      />
                    ) : null}
                    {sidebarShowDockComponents ? (
                      <DockComponentsCard
                        components={snapshot.dock?.components}
                        capabilities={snapshot.capabilities}
                      />
                    ) : null}
                  </div>
                </div>

                {/* ── Configure ────────────────────────── */}
                {sidebarShowCleaningSettings ? (
                  <div className="vacuum-card-group">
                    <div className="vacuum-card-group__head vacuum-card-group__head--static" aria-label="Configure group">
                      <span className="vacuum-card-group__title">Configure</span>
                    </div>
                    <div className="vacuum-card-group__body">
                      <CleaningSettingsCard
                        settings={snapshot.cleaningSettings}
                        capabilities={snapshot.capabilities}
                        commandError={missionCommandError}
                        onSetFanSpeed={(value) => void handleCleaningSettingCommand("set_fan_speed", value)}
                        onSetWaterUsage={(value) => void handleCleaningSettingCommand("set_water_usage", value)}
                      />
                    </div>
                  </div>
                ) : null}

                {/* ── Maintain ─────────────────────────── */}
                {sidebarShowMaintenance ? (
                  <div className="vacuum-card-group">
                    <div className="vacuum-card-group__head vacuum-card-group__head--static" aria-label="Maintain group">
                      <span className="vacuum-card-group__title">Maintain</span>
                    </div>
                    <div className="vacuum-card-group__body">
                      <MaintenanceCard
                        maintenance={snapshot.maintenance}
                        capabilities={snapshot.capabilities}
                      />
                    </div>
                  </div>
                ) : null}

                {/* ── Context ──────────────────────────── */}
                {sidebarShowSourceHealth ? (
                  <div className="vacuum-card-group">
                    <div className="vacuum-card-group__head vacuum-card-group__head--static" aria-label="Context group">
                      <span className="vacuum-card-group__title">Context</span>
                    </div>
                    <div className="vacuum-card-group__body">
                      <SourceHealthCard
                        availability={snapshot.availability}
                        health={snapshot.health}
                        source={snapshot.source}
                        fault={snapshot.fault}
                      />
                    </div>
                  </div>
                ) : null}
              </div>
            ) : (
              <>
            {/* ── Mode switcher ── */}
            <div className="vacuum-mode-switcher">
              <span className="vacuum-mode-switcher__label">Mode</span>
              <div className="vacuum-mode-switcher__tabs">
                <button
                  type="button"
                  className={`vacuum-mode-tab vacuum-mode-tab--mapping${activeMode === "mapping" ? " vacuum-mode-tab--active" : ""}`}
                  onClick={() => { setActiveMode("mapping"); }}
                  disabled={(!mappingSessionSupported && !autoMappingSupported) || isCleanAreaModeLocked || isRoomZoneModeLocked || (isGoalActive && !isMappingWorkflowActive)}
                  title={!mappingSessionSupported && !autoMappingSupported ? "Mapping unsupported" : isCleanAreaModeLocked ? "Finish clean area first" : isRoomZoneModeLocked ? "Finish room or zone editing first" : isGoalActive && !isMappingWorkflowActive ? "Stop navigation first" : "Mapping"}
                >
                  <MappingModeIcon className="vacuum-mode-tab__icon" />
                  <span>Mapping</span>
                </button>
                <button
                  type="button"
                  className={`vacuum-mode-tab vacuum-mode-tab--navigation${activeMode === "navigation" ? " vacuum-mode-tab--active" : ""}`}
                  onClick={() => { setActiveMode("navigation"); }}
                  disabled={!navigationSupported || isMappingWorkflowActive || isCleanAreaModeLocked || isRoomZoneModeLocked}
                  title={!navigationSupported ? "Navigation unsupported" : isMappingWorkflowActive ? "Finish mapping first" : isCleanAreaModeLocked ? "Finish clean area first" : isRoomZoneModeLocked ? "Finish room or zone editing first" : "Navigate"}
                >
                  <NavigateModeIcon className="vacuum-mode-tab__icon" />
                  <span>Navigate</span>
                </button>
                <button
                  type="button"
                  className={`vacuum-mode-tab vacuum-mode-tab--clean${activeMode === "clean" ? " vacuum-mode-tab--active" : ""}`}
                  onClick={() => { setActiveMode("clean"); }}
                  disabled={!cleanAreaSupported || isMappingWorkflowActive || isRoomZoneModeLocked || (isGoalActive && !isCleanAreaActive)}
                  title={!cleanAreaSupported ? "Clean Area unsupported" : isMappingWorkflowActive ? "Finish mapping first" : isRoomZoneModeLocked ? "Finish room or zone editing first" : isGoalActive && !isCleanAreaActive ? "Stop navigation first" : "Clean area"}
                >
                  <CleanModeIcon className="vacuum-mode-tab__icon" />
                  <span>Clean area</span>
                </button>
                <button
                  type="button"
                  className={`vacuum-mode-tab vacuum-mode-tab--rooms${activeMode === "rooms" ? " vacuum-mode-tab--active" : ""}`}
                  onClick={() => { setActiveMode("rooms"); }}
                  disabled={!roomsZonesSupported || isMappingWorkflowActive || isCleanAreaModeLocked || (isGoalActive && !isCleanAreaActive)}
                  title={!roomsZonesSupported ? "Rooms and zones unsupported" : isMappingWorkflowActive ? "Finish mapping first" : isCleanAreaModeLocked ? "Finish clean area first" : isGoalActive && !isCleanAreaActive ? "Stop navigation first" : "Rooms and zones"}
                >
                  <RoomsModeIcon className="vacuum-mode-tab__icon" />
                  <span>Rooms</span>
                </button>
              </div>
            </div>

            {/* ── Mapping mode ── */}
            {activeMode === "mapping" && (
              <div className="vacuum-mode-content">
                {mappingSessionSupported || autoMappingSupported ? (
                  <>
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
                    {manualControlSupported ? (
                      <TeleopCard
                        disabled={mappingStatus.state === "auto_mapping"}
                        disabledReason="Pause auto mapping before using manual control."
                      />
                    ) : null}
                  </>
                ) : (
                  <UnsupportedFeatureCard title="Mapping" detail={snapshot.mapping.stateReason} />
                )}
              </div>
            )}

            {/* ── Navigate mode ── */}
            {activeMode === "navigation" && (
              <div className="vacuum-mode-content">
                {!navigationSupported ? (
                  <UnsupportedFeatureCard
                    title="Navigation"
                    detail={snapshot.capabilities.start_navigation.notes ?? "Navigation is unavailable for this backend."}
                  />
                ) : (
                  <>
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
                  </>
                )}
              </div>
            )}

            {/* ── Clean area mode ── */}
            {activeMode === "clean" && (
              <div className="vacuum-mode-content">
                {cleanAreaSupported ? (
                  <CleanAreaCard
                    state={displayedCleanAreaState}
                    toolActive={cleanAreaToolActive}
                    rect={displayedCleanAreaRect}
                    validation={cleanAreaValidation}
                    coverage={displayedCleanAreaCoverage}
                    coverageConfig={cleanAreaCoverageConfig}
                    waypointCount={displayedCleanAreaWaypoints.length}
                    currentWaypointIndex={displayedCleanAreaCurrentIndex}
                    passCount={cleanAreaMetrics.passCount}
                    estimatedDistance={cleanAreaMetrics.totalDistance}
                    distanceRemaining={cleanAreaMetrics.remainingDistance}
                    commandError={displayedCleanAreaCommandError}
                    canStart={canStartCleanArea}
                    canCancel={canCancelCleanArea && !isCancelingGoal}
                    canPause={canPauseCleanArea && !isCancelingGoal}
                    canRetry={Boolean(activeCoverageMission?.availableActions.includes("retry_mission_step")) && retryMissionStepSupported}
                    canSkip={Boolean(activeCoverageMission?.availableActions.includes("skip_mission_step")) && skipMissionStepSupported}
                    onActivateTool={handleActivateCleanAreaTool}
                    onConfirm={handleConfirmCleanArea}
                    onStart={handleStartCleanArea}
                    onPause={() => void handlePauseCleanArea()}
                    onRetry={handleRetryCleanAreaWaypoint}
                    onSkip={handleSkipCleanAreaWaypoint}
                    onCancel={() => void handleCancelCleanArea()}
                    onClear={handleClearCleanArea}
                  />
                ) : (
                  <UnsupportedFeatureCard
                    title="Clean Area"
                    detail={snapshot.capabilities.start_coverage.notes ?? "Clean Area is unavailable for this backend."}
                  />
                )}
                <MissionLifecycleCard
                  mission={snapshot.mission}
                  battery={snapshot.battery}
                  capabilities={snapshot.capabilities}
                  commandError={missionCommandError}
                  onReturnToDock={() => void handleReturnToDock()}
                />
                <RecentMissionsCard missions={snapshot.missions.recent} />
                {manualControlSupported ? (
                  <TeleopCard
                    disabled={isCleanAreaActive}
                    disabledReason="Stop the cleaning run before using manual control."
                  />
                ) : null}
              </div>
            )}

            {/* ── Rooms / Zones mode ── */}
            {activeMode === "rooms" && (
              <div className="vacuum-mode-content">
                {roomsZonesSupported ? (
                  <RoomZonesCard
                  annotations={roomZoneAnnotations}
                  selectedAnnotation={selectedRoomZone}
                  targetStatus={selectedRoomZoneTargetStatus}
                  targetStatusLabel={selectedRoomZoneTargetLabel}
                  targetDetail={selectedRoomZoneTargetDetail}
                  targetCoverage={selectedRoomZoneCoverage}
                  waypointCount={selectedRoomZoneWaypoints.length}
                  passCount={selectedRoomZoneMetrics.passCount}
                  estimatedDistance={selectedRoomZoneMetrics.totalDistance}
                  toolActive={roomZoneToolActive}
                  draftKind={roomZoneDraftKind}
                  draftName={roomZoneDraftName}
                  draftRect={roomZoneDraftRect}
                  validation={roomZoneValidation}
                  commandError={roomZoneCommandError}
                  roomSemanticsSupported={roomSemanticsSupported}
                  zoneSemanticsSupported={zoneSemanticsSupported}
                  canStartCleaning={canStartSelectedRoomZone}
                  canPauseCleaning={canPauseRoomZoneCleaning}
                  canResumeCleaning={canResumeRoomZoneCleaning}
                  canCancelCleaning={canCancelRoomZoneCleaning}
                  canRetryCleaning={canRetryRoomZoneCleaning}
                  canSkipCleaning={canSkipRoomZoneCleaning}
                  cleaningActive={isRoomZoneCleaningRuntimeActive}
                  cleaningState={activeCoverageMissionState}
                  canSave={canSaveRoomZone}
                  saveDisabledReason={saveDisabledReason}
                  cleanDisabledReason={cleanDisabledReason}
                  onDraftKindChange={handleRoomZoneDraftKindChange}
                  onDraftNameChange={setRoomZoneDraftName}
                  onActivateTool={handleActivateRoomZoneTool}
                  onSave={() => void handleSaveRoomZone()}
                  onStartCleaning={() => void handleStartRoomZoneCleaning()}
                  onPauseCleaning={() => void handlePauseRoomZoneCleaning()}
                  onResumeCleaning={() => void handleResumeRoomZoneCleaning()}
                  onCancelCleaning={() => void handleCancelRoomZoneCleaning()}
                  onRetryCleaning={() => void handleRetryRoomZoneStep()}
                  onSkipCleaning={() => void handleSkipRoomZoneStep()}
                  onSelect={handleSelectRoomZone}
                  onDelete={() => void handleDeleteRoomZone()}
                  onClearDraft={handleClearRoomZoneDraft}
                  />
                ) : (
                  <UnsupportedFeatureCard
                    title="Rooms / Zones"
                    detail={snapshot.capabilities.room_semantics.notes ?? "Rooms and zones are unavailable for this backend."}
                  />
                )}
                <MissionLifecycleCard
                  mission={snapshot.mission}
                  battery={snapshot.battery}
                  capabilities={snapshot.capabilities}
                  commandError={missionCommandError}
                  onReturnToDock={() => void handleReturnToDock()}
                />
                <RecentMissionsCard missions={snapshot.missions.recent} />
                {manualControlSupported ? (
                  <TeleopCard disabled={isCleanAreaActive} disabledReason="Stop the active mission before using manual control." />
                ) : null}
              </div>
            )}

              </>
            )}
          </div>
        </section>
      </main>
    </div>
  );
}

export function VacuumControlPanel() {
  const [backend, setBackend] = useState<VacuumAdapterBackendId>(() => readConfiguredVacuumAdapterBackend());

  const handleBackendChange = useCallback((nextBackend: VacuumAdapterBackendId) => {
    writeSelectedAdapterBackend(nextBackend);
    setBackend(nextBackend);
  }, []);

  return (
    <VacuumControlPanelContent
      key={backend}
      backend={backend}
      onBackendChange={handleBackendChange}
    />
  );
}
