import {
  extractStampedPose,
  getRecordEntry,
} from "../../../components/Nav2/runtime/nav2RuntimeUtils";
import type { Nav2RuntimeState, GoalState, TopicHealth } from "../../../components/Nav2/runtime/nav2RuntimeTypes";
import type {
  VacuumAdapterSnapshot,
  VacuumAvailabilityStatus,
  VacuumGoalCoordinates,
  VacuumMapGrid,
  VacuumMapMetadata,
  VacuumMappingStatus,
  VacuumMissionState,
  VacuumNavigationState,
  VacuumNavigationTerminalState,
  VacuumPathPoint,
  VacuumReadinessState,
} from "../../state";
import { buildVacuumMapMetadata } from "../../mapGrid";
import { mapTurtleBot4Nav2Capabilities } from "./capabilityMapper";

const ACTIVE_GOAL_STATES = new Set<GoalState>(["sending", "accepted", "executing", "canceling"]);

export type TurtleBot4Nav2StateMapperInput = {
  runtime: Nav2RuntimeState;
  currentTarget?: VacuumGoalCoordinates | null;
  initialDistance?: number | null;
  mapGrid?: VacuumMapGrid | null;
  mapMetadata?: VacuumMapMetadata | null;
  mapping?: VacuumMappingStatus | null;
};

function getTopicStatus(runtime: Nav2RuntimeState, topic: string): TopicHealth["status"] | null {
  return runtime.topicHealth.find((entry) => entry.topic === topic)?.status ?? null;
}

function toFiniteNumber(value: unknown): number | null {
  const numeric = typeof value === "string" ? Number(value) : value;
  return typeof numeric === "number" && Number.isFinite(numeric) ? numeric : null;
}

function mapConnectionStatus(status: Nav2RuntimeState["connectionStatus"]): VacuumAvailabilityStatus {
  if (status === "connected") {
    return "online";
  }
  if (status === "connecting") {
    return "connecting";
  }
  return "offline";
}

function mapGoalState(goalState: GoalState): VacuumNavigationState {
  if (goalState === "ready") {
    return "idle";
  }
  if (goalState === "accepted" || goalState === "executing") {
    return "active";
  }
  if (goalState === "succeeded") {
    return "completed";
  }
  if (goalState === "aborted" || goalState === "rejected") {
    return "failed";
  }
  return goalState;
}

function mapReadinessFromTopic(status: TopicHealth["status"] | null): VacuumReadinessState {
  if (status === "receiving") {
    return "ready";
  }
  if (status === "stale" || status === "advertised") {
    return "degraded";
  }
  return "waiting";
}

function deriveMissionState(navigationActive: boolean, mappingState?: VacuumMappingStatus["state"]): VacuumMissionState {
  if (mappingState === "auto_mapping" || mappingState === "manual_mapping" || mappingState === "needs_assistance" || mappingState === "review") {
    return "mapping";
  }
  if (navigationActive) {
    return "navigating";
  }
  return "idle";
}

function defaultMappingStatus(mapMetadata: VacuumMapMetadata): VacuumMappingStatus {
  return {
    state: "idle",
    mode: null,
    stateReason: "No mapping session active.",
    knownRatio: mapMetadata.knownRatio,
    unknownRatio: mapMetadata.unknownRatio,
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
  };
}

function deriveTerminalState(goalState: GoalState): VacuumNavigationTerminalState | null {
  if (goalState === "succeeded") {
    return "completed";
  }
  if (goalState === "canceled") {
    return "canceled";
  }
  if (goalState === "aborted" || goalState === "rejected" || goalState === "unknown") {
    return "failed";
  }
  return null;
}

function extractPlanPath(message: Record<string, unknown> | null): VacuumPathPoint[] | null {
  if (!message) {
    return null;
  }
  const poses = getRecordEntry(message, "poses");
  if (!Array.isArray(poses) || poses.length === 0) {
    return null;
  }
  const points: VacuumPathPoint[] = [];
  for (const poseEntry of poses) {
    if (!poseEntry || typeof poseEntry !== "object") {
      continue;
    }
    const pose = extractStampedPose(poseEntry as Record<string, unknown>);
    if (!pose) {
      continue;
    }
    const position = getRecordEntry(pose, "position");
    if (!position || typeof position !== "object") {
      continue;
    }
    const x = toFiniteNumber(getRecordEntry(position as Record<string, unknown>, "x"));
    const y = toFiniteNumber(getRecordEntry(position as Record<string, unknown>, "y"));
    if (x == null || y == null) {
      continue;
    }
    points.push({ x, y });
  }
  return points.length > 0 ? points : null;
}

function getFaults(runtime: Nav2RuntimeState): string[] {
  const faults: string[] = [];
  if (runtime.connectionStatus === "disconnected") {
    faults.push("Robot bridge is disconnected.");
  }
  if (runtime.preflightStatus.state === "blocked") {
    faults.push(...runtime.preflightStatus.missingTopics.map((topic) => `Missing topic: ${topic}`));
    faults.push(...runtime.preflightStatus.missingServices.map((service) => `Missing service: ${service}`));
  }
  if (runtime.tfHealth.status === "error") {
    faults.push(runtime.tfHealth.detail);
  }
  for (const lifecycle of runtime.lifecycleHealth) {
    if (lifecycle.required && lifecycle.status === "error") {
      faults.push(`${lifecycle.node}: ${lifecycle.detail}`);
    }
  }
  return faults;
}

function getReadinessBlockers(runtime: Nav2RuntimeState, mapReady: boolean): string[] {
  const blockers: string[] = [];
  if (runtime.connectionStatus !== "connected") {
    blockers.push("Robot bridge is not connected.");
  }
  if (!mapReady) {
    blockers.push("Live map is not ready.");
  }
  if (runtime.preflightStatus.state !== "ready") {
    blockers.push("Navigation checks are not ready.");
  }
  return blockers;
}

export function mapTurtleBot4Nav2State(
  input: TurtleBot4Nav2StateMapperInput | Nav2RuntimeState,
  legacyCurrentTarget?: VacuumGoalCoordinates | null,
): VacuumAdapterSnapshot {
  const { runtime, currentTarget, initialDistance, mapGrid, mapMetadata, mapping } = isMapperInput(input)
    ? input
    : {
        runtime: input,
        currentTarget: legacyCurrentTarget ?? null,
        initialDistance: null,
        mapGrid: null,
        mapMetadata: null,
        mapping: null,
      };

  const mapStatus = getTopicStatus(runtime, "/map");
  const poseAvailable = runtime.currentMapCoordinates != null;
  const faults = getFaults(runtime);
  const availabilityStatus = mapConnectionStatus(runtime.connectionStatus);
  const normalizedMapMetadata = mapMetadata ?? buildVacuumMapMetadata(mapGrid ?? null, null);
  const normalizedMapping = mapping ?? defaultMappingStatus(normalizedMapMetadata);
  const mapReady = runtime.connectionStatus === "connected" && (mapStatus === "receiving" || normalizedMapMetadata.hasMap);
  const mapReadiness =
    runtime.connectionStatus === "connected"
      ? normalizedMapMetadata.hasMap
        ? "ready"
        : mapReadinessFromTopic(mapStatus)
      : "unavailable";
  const poseReadiness = poseAvailable
    ? "ready"
    : runtime.connectionStatus === "connected"
      ? "degraded"
      : "unavailable";

  const navigationState = mapGoalState(runtime.goalState);
  const active = ACTIVE_GOAL_STATES.has(runtime.goalState);
  const terminalState = deriveTerminalState(runtime.goalState);
  const missionState = deriveMissionState(active, normalizedMapping.state);
  const readinessBlockers = getReadinessBlockers(runtime, mapReady);
  const planPath = extractPlanPath(runtime.planMessage);

  return {
    identity: {
      id: "turtlebot4-nav2",
      label: "TurtleBot4 Nav2",
      source: "turtlebot4_nav2",
      model: "TurtleBot4 simulation",
    },
    availability: {
      status: availabilityStatus,
      connected: runtime.connectionStatus === "connected",
      detail: runtime.connectionStatus === "connected" ? "Foxglove bridge connected." : "Robot bridge is not online.",
    },
    capabilities: mapTurtleBot4Nav2Capabilities(runtime),
    map: {
      readiness: mapReadiness,
      topic: "/map",
      receiving: mapStatus === "receiving" || normalizedMapMetadata.hasMap,
      detail: mapStatus === "receiving" || normalizedMapMetadata.hasMap ? "Map is receiving." : "Waiting for live occupancy-grid data.",
      grid: mapGrid ?? null,
      metadata: normalizedMapMetadata,
    },
    pose: {
      readiness: poseReadiness,
      available: poseAvailable,
      source: runtime.helperPoseSource,
      coordinates: runtime.currentMapCoordinates,
      detail: poseAvailable ? "Pose is available." : "Waiting for localized pose or odometry fallback.",
    },
    navigation: {
      state: navigationState,
      backendGoalState: runtime.goalState,
      active,
      isSending: runtime.isSendingGoal,
      isCanceling: runtime.isCancelingGoal,
      currentTarget: currentTarget ?? null,
      terminalState,
      planPath,
      progress: {
        distanceRemaining: toFiniteNumber(runtime.feedbackDistanceRemaining),
        initialDistance: initialDistance ?? null,
        recoveries: toFiniteNumber(runtime.feedbackRecoveries),
        navigationTime: runtime.feedbackNavigationTime,
        estimatedTimeRemaining: runtime.feedbackEta,
      },
      detail: runtime.validationSummary.detail,
    },
    mission: {
      state: missionState,
      detail:
        missionState === "mapping"
          ? normalizedMapping.stateReason
          : missionState === "navigating"
            ? "Robot is navigating to a selected location."
            : terminalState
              ? `Last navigation ${terminalState}.`
              : "Robot is idle.",
      lastTerminalNavigation: terminalState,
    },
    mapping: normalizedMapping,
    readiness: {
      ready: readinessBlockers.length === 0,
      blockingReasons: readinessBlockers,
    },
    fault: {
      readiness: faults.length > 0 ? "degraded" : "ready",
      faults,
      detail: faults.length > 0 ? faults.join(" ") : "No adapter-level faults reported.",
    },
    battery: {
      readiness: "unavailable",
      percentage: null,
      charging: null,
      detail: "Battery state is not part of the TurtleBot4/Nav2 adapter slice.",
    },
  };
}

function isMapperInput(value: TurtleBot4Nav2StateMapperInput | Nav2RuntimeState): value is TurtleBot4Nav2StateMapperInput {
  return (value as TurtleBot4Nav2StateMapperInput).runtime !== undefined;
}
