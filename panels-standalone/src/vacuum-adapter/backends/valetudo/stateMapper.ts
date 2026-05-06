import type { VacuumAdapterSnapshot, VacuumNavigationStatus } from "../../state";
import { buildVacuumMapMetadata } from "../../mapGrid";
import { mapValetudoCapabilities } from "./capabilityMapper";
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

export function mapValetudoState(runtime: ValetudoRuntimeBoundary): VacuumAdapterSnapshot {
  const connected = runtime.connectionStatus === "online";
  const state = runtime.state;
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
      readiness: state?.mapAvailable ? "ready" : connected ? "waiting" : "unavailable",
      receiving: state?.mapAvailable ?? false,
      detail: state?.mapAvailable ? "Map is available." : "Waiting for Valetudo map state.",
      grid: null,
      metadata: buildVacuumMapMetadata(null, null),
    },
    pose: {
      readiness: state?.pose ? "ready" : connected ? "waiting" : "unavailable",
      available: state?.pose != null,
      source: "valetudo",
      coordinates: state?.pose ?? null,
      detail: state?.pose ? "Pose is available." : "Waiting for Valetudo pose state.",
    },
    navigation: EMPTY_NAVIGATION,
    mission: {
      state: state?.missionState ?? "idle",
      detail: state ? `Valetudo mission state: ${state.missionState}.` : "Waiting for Valetudo mission state.",
      lastTerminalNavigation: null,
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
      lastSavedAt: null,
      saveError: null,
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
