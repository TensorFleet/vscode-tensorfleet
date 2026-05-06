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

export type VacuumAvailability = {
  status: VacuumAvailabilityStatus;
  connected: boolean;
  detail?: string;
};

export type VacuumMapState = {
  readiness: VacuumReadinessState;
  topic?: string;
  receiving: boolean;
  detail?: string;
  grid: VacuumMapGrid | null;
  metadata: VacuumMapMetadata;
};

export type VacuumPoseState = {
  readiness: VacuumReadinessState;
  available: boolean;
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

export type VacuumMissionStatus = {
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
  lastSavedAt: number | null;
  saveError: string | null;
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

export type VacuumAdapterSnapshot = {
  identity: VacuumBackendIdentity;
  availability: VacuumAvailability;
  capabilities: VacuumCapabilities;
  map: VacuumMapState;
  pose: VacuumPoseState;
  navigation: VacuumNavigationStatus;
  mission: VacuumMissionStatus;
  mapping: VacuumMappingStatus;
  readiness: VacuumReadinessSummary;
  fault: VacuumFaultState;
  battery: VacuumBatteryState;
};
