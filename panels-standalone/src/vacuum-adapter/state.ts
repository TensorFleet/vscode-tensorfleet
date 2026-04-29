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
  readiness: VacuumReadinessSummary;
  fault: VacuumFaultState;
  battery: VacuumBatteryState;
};
