import type { VacuumBackendSource, VacuumCapabilities } from "./capabilities";

export type VacuumAvailabilityStatus = "offline" | "connecting" | "online";
export type VacuumReadinessState = "ready" | "waiting" | "degraded" | "unavailable";

export type VacuumNavigationState =
  | "idle"
  | "sending"
  | "active"
  | "canceling"
  | "completed"
  | "canceled"
  | "failed"
  | "blocked"
  | "unknown";

export type VacuumNavigationTerminalState = "completed" | "canceled" | "failed";

export type VacuumMissionState =
  | "idle"
  | "navigating"
  | "mapping"
  | "cleaning"
  | "paused"
  | "returning"
  | "charging";

export type VacuumMissionType =
  | "mapping"
  | "navigation"
  | "coverage"
  | "return_to_dock"
  | "room_cleaning"
  | "zone_cleaning"
  | "hardware_cleaning";

export type VacuumMissionStatus =
  | "idle"
  | "preparing"
  | "running"
  | "paused"
  | "canceling"
  | "returning"
  | "charging"
  | "resuming"
  | "needs_assistance"
  | "completed"
  | "failed"
  | "canceled"
  | "unsupported";

export type VacuumMissionAction =
  | "pause_mapping"
  | "resume_mapping"
  | "finish_mapping"
  | "pause_mission"
  | "resume_mission"
  | "cancel_mission"
  | "retry_mission_step"
  | "skip_mission_step"
  | "return_to_dock"
  | "accept_map"
  | "discard_mapping";

export type VacuumRobotActivityStatus =
  | "unknown"
  | "unavailable"
  | "idle"
  | "cleaning"
  | "paused"
  | "returning"
  | "docked"
  | "charging"
  | "faulted"
  | "mapping"
  | "navigating"
  | "covering";

export type VacuumRobotActivityAction =
  | VacuumMissionAction
  | "start_cleaning"
  | "pause"
  | "resume"
  | "stop"
  | "return_to_dock"
  | "start_navigation"
  | "cancel_navigation"
  | "start_mapping"
  | "start_coverage";

export type VacuumRobotActivity = {
  status: VacuumRobotActivityStatus;
  label?: string;
  updatedAt?: number | string;
  source?: VacuumBackendSource;
  reason?: string;
  availableActions?: VacuumRobotActivityAction[];
  details?: unknown;
};

export type VacuumMissionProgress = {
  percent: number | null;
  currentStep: number | null;
  totalSteps: number | null;
  distanceRemaining: number | null;
  areaCoveredSqM: number | null;
  areaRemainingSqM: number | null;
};

export type VacuumMissionResult = {
  status: Extract<VacuumMissionStatus, "completed" | "failed" | "canceled" | "unsupported">;
  completedAt: number | null;
  summary?: string;
  details?: Record<string, unknown>;
};

export type VacuumMissionError = {
  code: string;
  message: string;
  recoverable: boolean;
};

export type VacuumMissionSnapshot = {
  id: string;
  type: VacuumMissionType;
  status: VacuumMissionStatus;
  backendSource: VacuumBackendSource;
  startedAt: number | null;
  updatedAt: number | null;
  requestedCommand: string;
  phase: string;
  progress: VacuumMissionProgress;
  availableActions: VacuumMissionAction[];
  result: VacuumMissionResult | null;
  error: VacuumMissionError | null;
  target: unknown;
};

export type VacuumMissionCollection = {
  active: VacuumMissionSnapshot | null;
  recent: VacuumMissionSnapshot[];
};

export type VacuumPoseCoordinates = {
  x: number;
  y: number;
  yaw: number | null;
};

export type VacuumGoalCoordinates = {
  x: number;
  y: number;
  yaw: number;
};

export type VacuumPathPoint = {
  x: number;
  y: number;
};

export type VacuumBackendIdentity = {
  id: string;
  label: string;
  source: VacuumBackendSource;
  model?: string;
};

export type VacuumMapGrid = {
  width: number;
  height: number;
  resolution: number;
  originX: number;
  originY: number;
  originYaw: number;
  frameId: string | null;
  data: number[];
};

export type VacuumMapMetadata = {
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
};

export type VacuumMapAnnotationKind = "room" | "zone";

export type VacuumMapAnnotation = {
  id: string;
  kind: VacuumMapAnnotationKind;
  name: string;
  area:
    | {
        shape: "rectangle";
        minX: number;
        minY: number;
        maxX: number;
        maxY: number;
      }
    | {
        shape: "polygon";
        points: Array<{ x: number; y: number }>;
      };
  mapId: string | null;
  createdAt: number;
  updatedAt: number;
};

export type VacuumAvailability = {
  status: VacuumAvailabilityStatus;
  connected: boolean;
  detail?: string;
};

export type VacuumMapState = {
  readiness: VacuumReadinessState;
  /** @deprecated Compatibility mirror only. Backend topics belong in snapshot.diagnostics.map. */
  topic?: string;
  receiving: boolean;
  detail?: string;
  grid: VacuumMapGrid | null;
  metadata: VacuumMapMetadata;
  annotations: VacuumMapAnnotation[];
};

export type VacuumPoseState = {
  readiness: VacuumReadinessState;
  available: boolean;
  /** @deprecated Compatibility mirror only. Backend pose sources belong in snapshot.diagnostics.pose. */
  source?: string;
  coordinates: VacuumPoseCoordinates | null;
  detail?: string;
};

export type VacuumNavigationProgress = {
  distanceRemaining: number | null;
  initialDistance: number | null;
  recoveries: number | null;
  navigationTime: unknown;
  estimatedTimeRemaining: unknown;
};

export type VacuumNavigationStatus = {
  state: VacuumNavigationState;
  /** @deprecated Compatibility mirror only. Backend goal states belong in snapshot.diagnostics.navigation. */
  backendGoalState: string | null;
  active: boolean;
  isSending: boolean;
  isCanceling: boolean;
  currentTarget: VacuumGoalCoordinates | null;
  terminalState: VacuumNavigationTerminalState | null;
  planPath: VacuumPathPoint[] | null;
  progress: VacuumNavigationProgress;
  detail?: string;
};

export type VacuumLegacyMissionStatus = {
  state: VacuumMissionState;
  detail?: string;
  lastTerminalNavigation?: VacuumNavigationTerminalState | null;
};

export type VacuumMappingState =
  | "idle"
  | "manual_mapping"
  | "auto_mapping"
  | "paused"
  | "needs_assistance"
  | "review"
  | "accepted"
  | "discarded"
  | "error";

export type VacuumMappingMode = "auto" | "manual";

export type VacuumMappingStatus = {
  state: VacuumMappingState;
  mode: VacuumMappingMode | null;
  stateReason: string;
  knownRatio: number;
  unknownRatio: number;
  frontierCount: number;
  visitedGoalCount: number;
  failedGoalCount: number;
  activeGoal: VacuumGoalCoordinates | null;
  lastError: string | null;
  updatedAt: number | null;
  persistence: "session" | "persistent" | "unsupported";
  acceptedSessionLevel: boolean;
  savedMapPath: string | null;
  loadedMapPath: string | null;
  lastSavedAt: number | null;
  saveError: string | null;
  loadError: string | null;
  activeMapName: string | null;
  savedMaps: VacuumSavedMapSummary[];
};

export type VacuumSavedMapSummary = {
  id: string;
  name: string;
  /** @deprecated Compatibility mirror only. Saved map paths belong in snapshot.diagnostics.mapping. */
  yamlPath: string;
  /** @deprecated Compatibility mirror only. Saved map paths belong in snapshot.diagnostics.mapping. */
  imagePath: string | null;
  /** @deprecated Compatibility mirror only. Saved map paths belong in snapshot.diagnostics.mapping. */
  poseGraphPath: string | null;
  loadable: boolean;
  loadUnavailableReason: string | null;
  modifiedAt: number | null;
  sizeBytes: number;
  active: boolean;
};

export type VacuumReadinessSummary = {
  ready: boolean;
  blockingReasons: string[];
};

export type VacuumFaultState = {
  readiness: VacuumReadinessState;
  faults: string[];
  detail?: string;
};

export type VacuumBatteryState = {
  readiness: VacuumReadinessState;
  percentage: number | null;
  charging: boolean | null;
  detail?: string;
};

export type VacuumRuntimeHealthStatus = "online" | "degraded" | "offline" | "unknown";

export type VacuumSourceKind =
  | "turtlebot4_nav2"
  | "fixed_mock"
  | "valetudo_mock"
  | "valetudo_http"
  | "real_robot"
  | "unknown";

export type VacuumSourceStatus = "reachable" | "unreachable" | "stale" | "unknown";

export type VacuumDockState = "unknown" | "docked" | "undocked" | "returning" | "charging" | "error";

export type VacuumRuntimeHealth = {
  runtimeStatus: VacuumRuntimeHealthStatus;
  updatedAt?: number | string;
  detail?: string;
};

export type VacuumSourceState = {
  kind?: VacuumSourceKind;
  status?: VacuumSourceStatus;
  stale?: boolean;
  lastSeenAt?: number | string | null;
  reason?: string;
};

export type VacuumDockStatus = {
  supported?: boolean;
  state?: VacuumDockState;
  charging?: boolean;
  detail?: string;
};

export type VacuumCleaningSettingOption = {
  value: string;
  label: string;
};

export type VacuumCleaningSettingState = {
  current?: string;
  options: VacuumCleaningSettingOption[];
  readiness?: VacuumReadinessState;
  status?: string;
  detail?: string;
};

export type VacuumCleaningSettingsState = {
  fanSpeed?: VacuumCleaningSettingState;
  waterUsage?: VacuumCleaningSettingState;
};

export type VacuumConsumableStatus = "ok" | "warning" | "replace_soon" | "replace_now" | "unknown";

export type VacuumConsumableState = {
  id: string;
  label: string;
  remainingPercent?: number;
  remainingMinutes?: number;
  usedMinutes?: number;
  totalMinutes?: number;
  status?: VacuumConsumableStatus;
  detail?: string;
};

export type VacuumMaintenanceState = {
  consumables: VacuumConsumableState[];
};

export type VacuumCurrentStatisticsState = {
  durationSeconds?: number;
  areaSquareMeters?: number;
  startedAt?: number | string;
  updatedAt?: number | string;
  detail?: string;
};

export type VacuumStatisticsState = {
  current?: VacuumCurrentStatisticsState;
};

export type VacuumAdapterDiagnostics = {
  backend?: string;
  runtime?: unknown;
  source?: unknown;
  capabilities?: unknown;
  map?: unknown;
  pose?: unknown;
  navigation?: unknown;
  mapping?: unknown;
  warnings?: string[];
  raw?: unknown;
};

export type VacuumAdapterSnapshot = {
  identity: VacuumBackendIdentity;
  availability: VacuumAvailability;
  capabilities: VacuumCapabilities;
  health?: VacuumRuntimeHealth;
  source?: VacuumSourceState;
  dock?: VacuumDockStatus;
  cleaningSettings?: VacuumCleaningSettingsState;
  maintenance?: VacuumMaintenanceState;
  statistics?: VacuumStatisticsState;
  diagnostics?: VacuumAdapterDiagnostics;
  map: VacuumMapState;
  pose: VacuumPoseState;
  navigation: VacuumNavigationStatus;
  activity?: VacuumRobotActivity;
  mission: VacuumLegacyMissionStatus;
  activeMission: VacuumMissionSnapshot | null;
  missions: VacuumMissionCollection;
  mapping: VacuumMappingStatus;
  readiness: VacuumReadinessSummary;
  fault: VacuumFaultState;
  battery: VacuumBatteryState;
};
