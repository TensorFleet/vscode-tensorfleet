import type {
  VacuumAdapterDiagnostics,
  VacuumAdapterSnapshot,
  VacuumDockState,
  VacuumMissionSnapshot,
  VacuumMissionState,
  VacuumNavigationStatus,
  VacuumRuntimeHealth,
  VacuumSourceState,
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

function isRecord(value: unknown): value is Record<string, unknown> {
  return value != null && typeof value === "object";
}

function isStringArray(value: unknown): value is string[] {
  return Array.isArray(value) && value.every((item) => typeof item === "string");
}

function normalizeSourceStatus(snapshot: ValetudoRuntimeSnapshot): VacuumSourceState["status"] {
  if (snapshot.source.stale) {
    return "stale";
  }
  if (snapshot.source.status === "reachable") {
    return "reachable";
  }
  if (snapshot.source.status === "unreachable") {
    return "unreachable";
  }
  return "unknown";
}

function sourceUnavailableReason(snapshot: ValetudoRuntimeSnapshot): string | undefined {
  if (snapshot.source.stale) {
    return "stale_source";
  }
  if (!snapshot.connectivity.online) {
    return "runtime_offline";
  }
  if (!snapshot.connectivity.reachable || snapshot.source.status === "unreachable") {
    return "source_unreachable";
  }
  if (snapshot.runtime.status === "degraded") {
    return "degraded_runtime";
  }
  return undefined;
}

function commandUnavailableReason(
  snapshot: ValetudoRuntimeSnapshot,
  commandReason: string | undefined,
): string | undefined {
  if (!snapshot.connectivity.online) {
    return sourceUnavailableReason(snapshot);
  }
  return commandReason ?? sourceUnavailableReason(snapshot);
}

function mapDockState(snapshot: ValetudoRuntimeSnapshot): VacuumDockState {
  if (snapshot.battery?.charging) {
    return "charging";
  }
  const state = snapshot.dock?.state.toLowerCase() ?? "";
  if (state.includes("return")) {
    return "returning";
  }
  if (state.includes("error") || state.includes("fault")) {
    return "error";
  }
  if (snapshot.dock?.docked) {
    return "docked";
  }
  if (snapshot.dock) {
    return "undocked";
  }
  return "unknown";
}

function mapHealth(snapshot: ValetudoRuntimeSnapshot): VacuumRuntimeHealth {
  return {
    runtimeStatus: snapshot.runtime.status,
    updatedAt: snapshot.updatedAt,
    detail:
      snapshot.runtime.status === "online"
        ? "Valetudo integration runtime is online."
        : snapshot.runtime.status === "degraded"
          ? "Valetudo integration runtime is degraded."
          : "Valetudo integration runtime is offline.",
  };
}

function mapDiagnostics(snapshot: ValetudoRuntimeSnapshot): VacuumAdapterDiagnostics {
  const warnings = [
    ...(snapshot.diagnostics.notes ?? []),
    ...(snapshot.source.stale ? ["Valetudo runtime source state is stale."] : []),
  ];
  return {
    backend: "valetudo",
    runtime: snapshot.runtime,
    source: snapshot.diagnostics.source ?? snapshot.source,
    capabilities: snapshot.capabilities.diagnostics,
    warnings,
    raw: {
      rawCapabilityNames: snapshot.diagnostics.rawCapabilityNames,
      capabilityTiers: snapshot.diagnostics.capabilityTiers,
      transports: snapshot.diagnostics.transports,
      lastCommand: snapshot.diagnostics.lastCommand,
      rawDiagnostics: snapshot.rawDiagnostics,
    },
  };
}

export function isValetudoRuntimeSnapshot(value: unknown): value is ValetudoRuntimeSnapshot {
  if (!isRecord(value)) {
    return false;
  }
  const candidate = value;
  const runtime = candidate.runtime;
  const robot = candidate.robot;
  const source = candidate.source;
  const state = candidate.state;
  const connectivity = candidate.connectivity;
  const capabilities = candidate.capabilities;
  const diagnostics = candidate.diagnostics;
  return (
    typeof candidate.backend === "string" &&
    isRecord(runtime) &&
    typeof runtime.id === "string" &&
    typeof runtime.version === "string" &&
    typeof runtime.status === "string" &&
    isRecord(robot) &&
    typeof robot.id === "string" &&
    typeof robot.name === "string" &&
    isRecord(source) &&
    typeof source.kind === "string" &&
    typeof source.status === "string" &&
    typeof source.stale === "boolean" &&
    (typeof source.lastSeenAt === "number" || source.lastSeenAt === null) &&
    isRecord(state) &&
    typeof state.value === "string" &&
    typeof state.label === "string" &&
    typeof state.started === "boolean" &&
    typeof state.paused === "boolean" &&
    isRecord(connectivity) &&
    typeof connectivity.reachable === "boolean" &&
    typeof connectivity.online === "boolean" &&
    isRecord(capabilities) &&
    isRecord(capabilities.commands) &&
    Array.isArray(capabilities.diagnostics) &&
    isRecord(diagnostics) &&
    typeof diagnostics.mode === "string" &&
    isStringArray(diagnostics.rawCapabilityNames) &&
    typeof candidate.updatedAt === "number"
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
  const commandAvailability: ValetudoRuntimeBoundary["commandAvailability"] = {};
  for (const [command, availability] of Object.entries(snapshot.capabilities.commands)) {
    const backendCapability = RUNTIME_COMMAND_TO_BACKEND_CAPABILITY[command];
    if (backendCapability) {
      capabilities.add(backendCapability);
    }
    commandAvailability[command] = {
      available: availability.available,
      reason: commandUnavailableReason(snapshot, availability.reason),
    };
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

  const sourceReason = sourceUnavailableReason(snapshot);
  const faults = [
    ...(snapshot.source.stale ? ["Valetudo runtime source state is stale."] : []),
    ...(sourceReason === "source_unreachable" ? ["Valetudo runtime source is unreachable."] : []),
    ...(snapshot.runtime.status === "degraded" ? ["Valetudo integration runtime is degraded."] : []),
  ];

  return {
    connectionStatus: snapshot.connectivity.online ? "online" : "offline",
    capabilities: [...capabilities],
    commandAvailability,
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
    health: mapHealth(snapshot),
    source: {
      kind: snapshot.source.kind,
      status: normalizeSourceStatus(snapshot),
      stale: snapshot.source.stale,
      lastSeenAt: snapshot.source.lastSeenAt,
      reason: sourceReason,
    },
    dock: {
      supported: snapshot.dock != null || snapshot.battery?.charging != null,
      state: mapDockState(snapshot),
      charging: snapshot.battery?.charging,
      detail: snapshot.dock?.state ?? (snapshot.battery?.charging ? "Charging." : "Dock state unknown."),
    },
    diagnostics: mapDiagnostics(snapshot),
    lastError: snapshot.connectivity.online ? undefined : "Valetudo integration runtime is offline.",
  };
}

export function mapValetudoRuntimeUnavailable(message: string): ValetudoRuntimeBoundary {
  return {
    connectionStatus: "offline",
    capabilities: [],
    state: null,
    health: {
      runtimeStatus: "offline",
      detail: message,
    },
    source: {
      kind: "unknown",
      status: "unknown",
      stale: false,
      lastSeenAt: null,
      reason: "runtime_offline",
    },
    dock: {
      supported: false,
      state: "unknown",
      charging: undefined,
      detail: "Dock state is unavailable while the runtime is offline.",
    },
    diagnostics: {
      backend: "valetudo",
      warnings: [message],
    },
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
    capabilities: mapValetudoCapabilities(runtime.capabilities, {
      commandAvailability: runtime.commandAvailability,
      unavailableReason:
        runtime.connectionStatus === "online" && runtime.source?.reason
          ? runtime.source.reason
          : runtime.connectionStatus === "online"
            ? undefined
            : "runtime_offline",
    }),
    health: runtime.health,
    source: runtime.source,
    dock: runtime.dock,
    diagnostics: runtime.diagnostics,
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
