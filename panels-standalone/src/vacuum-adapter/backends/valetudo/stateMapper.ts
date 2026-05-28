import type {
  VacuumAdapterSnapshot,
  VacuumMissionSnapshot,
  VacuumMissionState,
  VacuumNavigationStatus,
} from "../../state";
import { buildVacuumMapMetadata } from "../../mapGrid";
import type { ValetudoBackendCapability } from "./capabilityMapper";
import { mapValetudoCapabilities } from "./capabilityMapper";
import type { ValetudoRuntimeSnapshot } from "./runtimeContract";
import type { ValetudoRuntimeBoundary } from "./types";

const EMPTY_NAVIGATION: VacuumNavigationStatus = {
  state: "idle",
  backendGoalState: null,
  active: false,
  isSending: false,
  isCanceling: false,
  currentTarget: null,
  terminalState: null,
  planPath: null,
  progress: {
    distanceRemaining: null,
    initialDistance: null,
    recoveries: null,
    navigationTime: null,
    estimatedTimeRemaining: null,
  },
};

const RUNTIME_COMMAND_TO_BACKEND_CAPABILITY: Record<string, ValetudoBackendCapability> = {
  start_cleaning: "BasicControlCapability",
  pause: "BasicControlCapability",
  stop: "BasicControlCapability",
  return_to_dock: "BasicControlCapability",
};

const KNOWN_VALETUDO_BACKEND_CAPABILITIES = new Set<ValetudoBackendCapability>([
  "BasicControlCapability",
  "BatteryStateCapability",
  "ConsumableMonitoringCapability",
  "FanSpeedControlCapability",
  "GoToLocationCapability",
  "MapSegmentationCapability",
  "WaterUsageControlCapability",
  "ZoneCleaningCapability",
]);

function isKnownValetudoBackendCapability(value: string): value is ValetudoBackendCapability {
  return KNOWN_VALETUDO_BACKEND_CAPABILITIES.has(value as ValetudoBackendCapability);
}

function isDiagnosticOnlyCapability(value: ValetudoBackendCapability): boolean {
  return value !== "BasicControlCapability" && value !== "BatteryStateCapability";
}

export function isValetudoRuntimeSnapshot(value: unknown): value is ValetudoRuntimeSnapshot {
  if (value == null || typeof value !== "object") {
    return false;
  }
  const candidate = value as Record<string, unknown>;
  const robot = candidate.robot as Record<string, unknown> | undefined;
  const state = candidate.state as Record<string, unknown> | undefined;
  const connectivity = candidate.connectivity as Record<string, unknown> | undefined;
  const capabilities = candidate.capabilities as Record<string, unknown> | undefined;
  return (
    typeof candidate.backend === "string" &&
    robot != null &&
    typeof robot.id === "string" &&
    state != null &&
    typeof state.value === "string" &&
    connectivity != null &&
    typeof connectivity.online === "boolean" &&
    capabilities != null &&
    typeof capabilities.commands === "object" &&
    capabilities.commands != null
  );
}

function normalizeMissionState(snapshot: ValetudoRuntimeSnapshot): VacuumMissionState {
  const value = snapshot.state.value.toLowerCase();
  if (snapshot.state.paused || value.includes("pause")) {
    return "paused";
  }
  if (value.includes("return")) {
    return "returning";
  }
  if (snapshot.state.started || value.includes("clean")) {
    return "cleaning";
  }
  // Only treat the robot as charging when the source actually reports charging.
  // Being docked while idle is not a mission and must map to "idle".
  if (snapshot.battery?.charging) {
    return "charging";
  }
  return "idle";
}

export function mapValetudoRuntimeSnapshotToBoundary(
  snapshot: ValetudoRuntimeSnapshot,
): ValetudoRuntimeBoundary {
  const capabilities = new Set<ValetudoBackendCapability>();
  for (const [command, availability] of Object.entries(snapshot.capabilities.commands)) {
    const backendCapability = RUNTIME_COMMAND_TO_BACKEND_CAPABILITY[command];
    if (backendCapability && availability.available) {
      capabilities.add(backendCapability);
    }
  }
  if (snapshot.battery) {
    capabilities.add("BatteryStateCapability");
  }
  for (const diagnostic of snapshot.capabilities.diagnostics) {
    if (
      diagnostic.detected &&
      isKnownValetudoBackendCapability(diagnostic.name) &&
      isDiagnosticOnlyCapability(diagnostic.name)
    ) {
      capabilities.add(diagnostic.name);
    }
  }

  const faults = snapshot.source.stale ? ["Valetudo runtime source state is stale."] : [];

  return {
    connectionStatus: snapshot.connectivity.online ? "online" : "offline",
    capabilities: [...capabilities],
    state: {
      id: snapshot.robot.id,
      label: snapshot.robot.name,
      mapAvailable: false,
      pose: null,
      batteryPercentage: snapshot.battery?.level ?? null,
      charging: snapshot.battery?.charging ?? null,
      missionState: normalizeMissionState(snapshot),
      faults,
    },
    lastError: snapshot.connectivity.online ? undefined : "Valetudo runtime source is unreachable.",
  };
}

export function mapValetudoRuntimeUnavailable(message: string): ValetudoRuntimeBoundary {
  return {
    connectionStatus: "offline",
    capabilities: [],
    state: null,
    lastError: message,
  };
}

function buildValetudoActiveMission(runtime: ValetudoRuntimeBoundary): VacuumMissionSnapshot | null {
  const state = runtime.state;
  if (!state || state.missionState === "idle") {
    return null;
  }
  const status = state.missionState === "paused"
    ? "paused"
    : state.missionState === "returning"
      ? "returning"
      : state.missionState === "charging"
        ? "charging"
        : "running";
  return {
    id: "valetudo:active",
    type: state.missionState === "returning" ? "return_to_dock" : "hardware_cleaning",
    status,
    backendSource: "valetudo",
    startedAt: null,
    updatedAt: null,
    requestedCommand: "runtime_state",
    phase: state.missionState,
    progress: {
      percent: null,
      currentStep: null,
      totalSteps: null,
      distanceRemaining: null,
      areaCoveredSqM: null,
      areaRemainingSqM: null,
    },
    availableActions: status === "running" ? ["pause_mission", "cancel_mission", "return_to_dock"] : [],
    result: null,
    error: null,
    target: null,
  };
}

export function mapValetudoState(runtime: ValetudoRuntimeBoundary): VacuumAdapterSnapshot {
  const connected = runtime.connectionStatus === "online";
  const state = runtime.state;
  const activeMission = buildValetudoActiveMission(runtime);
  const faults = [...(state?.faults ?? [])];
  if (runtime.lastError) {
    faults.push(runtime.lastError);
  }

  return {
    identity: {
      id: state?.id ?? "valetudo",
      label: state?.label ?? "Valetudo Vacuum",
      source: "valetudo",
      model: state?.model,
    },
    availability: {
      status: runtime.connectionStatus,
      connected,
      detail: connected ? "Valetudo integration runtime is online." : "Valetudo integration runtime is not online.",
    },
    capabilities: mapValetudoCapabilities(runtime.capabilities),
    map: {
      readiness: "unavailable",
      receiving: false,
      detail: "Valetudo map rendering is unsupported in Layer 6A Milestone 2.",
      grid: null,
      metadata: buildVacuumMapMetadata(null, null),
      annotations: [],
    },
    pose: {
      readiness: "unavailable",
      available: false,
      source: "valetudo",
      coordinates: null,
      detail: "Valetudo pose is not exposed as a product navigation surface in Layer 6A Milestone 2.",
    },
    navigation: EMPTY_NAVIGATION,
    mission: {
      state: state?.missionState ?? "idle",
      detail: state ? `Valetudo mission state: ${state.missionState}.` : "Waiting for Valetudo mission state.",
      lastTerminalNavigation: null,
    },
    activeMission,
    missions: {
      active: activeMission,
      recent: [],
    },
    mapping: {
      state: "idle",
      mode: null,
      stateReason: "Auto mapping is not implemented for this backend.",
      knownRatio: 0,
      unknownRatio: 1,
      frontierCount: 0,
      visitedGoalCount: 0,
      failedGoalCount: 0,
      activeGoal: null,
      lastError: null,
      updatedAt: null,
      persistence: "unsupported",
      acceptedSessionLevel: false,
      savedMapPath: null,
      loadedMapPath: null,
      lastSavedAt: null,
      saveError: null,
      loadError: null,
      activeMapName: null,
      savedMaps: [],
    },
    readiness: {
      ready: connected && state != null,
      blockingReasons: connected && state != null ? [] : ["Valetudo runtime state is not available."],
    },
    fault: {
      readiness: faults.length > 0 ? "degraded" : connected ? "ready" : "unavailable",
      faults,
      detail: faults.length > 0 ? faults.join(" ") : "No Valetudo faults reported.",
    },
    battery: {
      readiness: state?.batteryPercentage == null ? (connected ? "waiting" : "unavailable") : "ready",
      percentage: state?.batteryPercentage ?? null,
      charging: state?.charging ?? null,
      detail:
        state?.batteryPercentage == null
          ? "Waiting for Valetudo battery state."
          : `Battery ${state.batteryPercentage}%.`,
    },
  };
}
