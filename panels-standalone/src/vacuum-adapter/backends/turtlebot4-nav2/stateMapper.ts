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
  VacuumMapAnnotation,
  VacuumMappingStatus,
  VacuumMissionAction,
  VacuumMissionSnapshot,
  VacuumMissionState,
  VacuumMissionStatus,
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
  mission?: VacuumMissionSnapshot | null;
  recentMissions?: VacuumMissionSnapshot[];
  annotations?: VacuumMapAnnotation[];
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

function emptyMissionProgress(): VacuumMissionSnapshot["progress"] {
  return {
    percent: null,
    currentStep: null,
    totalSteps: null,
    distanceRemaining: null,
    areaCoveredSqM: null,
    areaRemainingSqM: null,
  };
}

function mapNavigationMissionStatus(goalState: GoalState): VacuumMissionStatus {
  if (goalState === "sending") {
    return "preparing";
  }
  if (goalState === "accepted" || goalState === "executing") {
    return "running";
  }
  if (goalState === "canceling") {
    return "canceling";
  }
  if (goalState === "succeeded") {
    return "completed";
  }
  if (goalState === "canceled") {
    return "canceled";
  }
  if (goalState === "aborted" || goalState === "rejected" || goalState === "unknown" || goalState === "blocked") {
    return "failed";
  }
  return "idle";
}

function mapMappingMissionStatus(mappingState: VacuumMappingStatus["state"]): VacuumMissionStatus {
  if (mappingState === "auto_mapping" || mappingState === "manual_mapping" || mappingState === "review") {
    return "running";
  }
  if (mappingState === "paused") {
    return "paused";
  }
  if (mappingState === "needs_assistance") {
    return "needs_assistance";
  }
  if (mappingState === "accepted") {
    return "completed";
  }
  if (mappingState === "discarded") {
    return "canceled";
  }
  if (mappingState === "error") {
    return "failed";
  }
  return "idle";
}

function terminalResultFromStatus(
  status: VacuumMissionStatus,
  updatedAt: number | null,
  summary: string,
): VacuumMissionSnapshot["result"] {
  if (status !== "completed" && status !== "failed" && status !== "canceled" && status !== "unsupported") {
    return null;
  }
  return { status, completedAt: updatedAt, summary };
}

function isRuntimeMissionActive(status: VacuumMissionStatus): boolean {
  return [
    "preparing",
    "running",
    "paused",
    "canceling",
    "returning",
    "charging",
    "resuming",
    "needs_assistance",
  ].includes(status);
}

function isMappingMissionActive(mappingState: VacuumMappingStatus["state"]): boolean {
  return mappingState === "auto_mapping" || mappingState === "manual_mapping" || mappingState === "paused" || mappingState === "needs_assistance" || mappingState === "review";
}

function normalizeRuntimeMissionActions(
  mission: VacuumMissionSnapshot,
  hasCancelMission: boolean,
): VacuumMissionSnapshot {
  return {
    ...mission,
    availableActions: mission.availableActions.filter((action) => {
      if (action === "cancel_mission") {
        return hasCancelMission;
      }
      if (mission.type === "navigation") {
        return action !== "pause_mission" && action !== "resume_mission" && action !== "retry_mission_step" && action !== "skip_mission_step";
      }
      return true;
    }),
  };
}

function missionSortTime(mission: VacuumMissionSnapshot): number {
  return mission.result?.completedAt ?? mission.updatedAt ?? mission.startedAt ?? 0;
}

function mergeRecentMissions(missions: VacuumMissionSnapshot[]): VacuumMissionSnapshot[] {
  const byId = new Map<string, VacuumMissionSnapshot>();
  for (const mission of missions) {
    if (mission.status === "idle") {
      continue;
    }
    const existing = byId.get(mission.id);
    if (!existing || missionSortTime(mission) >= missionSortTime(existing)) {
      byId.set(mission.id, mission);
    }
  }
  return [...byId.values()]
    .sort((a, b) => missionSortTime(b) - missionSortTime(a))
    .slice(0, 10);
}

function deriveAvailableMissionActions(
  status: VacuumMissionStatus,
  type: VacuumMissionSnapshot["type"],
  hasCancelMission: boolean,
  mappingState?: VacuumMappingStatus["state"],
): VacuumMissionAction[] {
  if (type === "navigation") {
    return status === "running" || status === "preparing" || status === "canceling"
      ? hasCancelMission
        ? ["cancel_mission"]
        : []
      : [];
  }
  if (type === "mapping") {
    if (mappingState === "auto_mapping") {
      return ["pause_mapping", "finish_mapping", "discard_mapping"];
    }
    if (mappingState === "manual_mapping" || mappingState === "paused" || mappingState === "needs_assistance") {
      return ["resume_mapping", "finish_mapping", "discard_mapping"];
    }
    if (mappingState === "review") {
      return ["accept_map", "discard_mapping"];
    }
  }
  return [];
}

function buildActiveMission(input: {
  runtime: Nav2RuntimeState;
  mapping: VacuumMappingStatus;
  navigationState: VacuumNavigationState;
  terminalState: VacuumNavigationTerminalState | null;
  currentTarget: VacuumGoalCoordinates | null | undefined;
  initialDistance: number | null | undefined;
  distanceRemaining: number | null;
  hasCancelMission: boolean;
  runtimeMission: VacuumMissionSnapshot | null | undefined;
}): VacuumMissionSnapshot | null {
  if (input.runtimeMission && input.runtimeMission.status !== "idle" && isRuntimeMissionActive(input.runtimeMission.status)) {
    return normalizeRuntimeMissionActions(input.runtimeMission, input.hasCancelMission);
  }

  const mappingStatus = mapMappingMissionStatus(input.mapping.state);
  if (mappingStatus !== "idle" && isMappingMissionActive(input.mapping.state)) {
    return {
      id: "turtlebot4-nav2:mapping",
      type: "mapping",
      status: mappingStatus,
      backendSource: "turtlebot4_nav2",
      startedAt: null,
      updatedAt: input.mapping.updatedAt,
      requestedCommand: input.mapping.mode === "auto" ? "start_mapping:auto" : input.mapping.mode === "manual" ? "start_mapping:manual" : "mapping",
      phase: input.mapping.state,
      progress: {
        ...emptyMissionProgress(),
        percent: input.mapping.knownRatio,
      },
      availableActions: deriveAvailableMissionActions(mappingStatus, "mapping", input.hasCancelMission, input.mapping.state),
      result: terminalResultFromStatus(mappingStatus, input.mapping.updatedAt, input.mapping.stateReason),
      error: input.mapping.lastError
        ? {
            code: "mapping_error",
            message: input.mapping.lastError,
            recoverable: mappingStatus === "needs_assistance",
          }
        : null,
      target: input.mapping.activeGoal,
    };
  }

  if (input.runtimeMission && input.runtimeMission.status !== "idle") {
    return normalizeRuntimeMissionActions(input.runtimeMission, input.hasCancelMission);
  }

  const navigationStatus = mapNavigationMissionStatus(input.runtime.goalState);
  if (navigationStatus === "idle") {
    return null;
  }
  return {
    id: "turtlebot4-nav2:navigation",
    type: "navigation",
    status: navigationStatus,
    backendSource: "turtlebot4_nav2",
    startedAt: null,
    updatedAt: null,
    requestedCommand: "start_navigation",
    phase: input.runtime.goalState,
    progress: {
      ...emptyMissionProgress(),
      percent:
        input.initialDistance != null && input.distanceRemaining != null && input.initialDistance > 0
          ? Math.max(0, Math.min(1, 1 - input.distanceRemaining / input.initialDistance))
          : null,
      distanceRemaining: input.distanceRemaining,
    },
    availableActions: deriveAvailableMissionActions(navigationStatus, "navigation", input.hasCancelMission),
    result: terminalResultFromStatus(
      navigationStatus,
      null,
      input.terminalState ? `Navigation ${input.terminalState}.` : input.navigationState,
    ),
    error:
      navigationStatus === "failed"
        ? {
            code: "navigation_failed",
            message: input.runtime.validationSummary.detail,
            recoverable: true,
          }
        : null,
    target: input.currentTarget ?? null,
  };
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
  const { runtime, currentTarget, initialDistance, mapGrid, mapMetadata, mapping, mission, recentMissions, annotations } = isMapperInput(input)
    ? input
    : {
        runtime: input,
        currentTarget: legacyCurrentTarget ?? null,
        initialDistance: null,
        mapGrid: null,
        mapMetadata: null,
        mapping: null,
        mission: null,
        recentMissions: [],
        annotations: [],
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
  const readinessBlockers = getReadinessBlockers(runtime, mapReady);
  const planPath = extractPlanPath(runtime.planMessage);
  const distanceRemaining = toFiniteNumber(runtime.feedbackDistanceRemaining);
  const capabilities = mapTurtleBot4Nav2Capabilities(runtime);
  const activeMission = buildActiveMission({
    runtime,
    mapping: normalizedMapping,
    navigationState,
    terminalState,
    currentTarget,
    initialDistance,
    distanceRemaining,
    hasCancelMission: capabilities.cancel_mission.supported,
    runtimeMission: mission,
  });
  const navigationMission = activeMission?.type === "navigation" ? activeMission : null;
  const navigationMissionTarget =
    navigationMission?.target && typeof navigationMission.target === "object"
      ? (navigationMission.target as VacuumGoalCoordinates)
      : null;
  const navigationFromMissionState: VacuumNavigationState | null = navigationMission
    ? navigationMission.status === "preparing"
      ? "sending"
      : navigationMission.status === "running"
        ? "active"
        : navigationMission.status === "canceling"
          ? "canceling"
          : navigationMission.status === "completed"
            ? "completed"
            : navigationMission.status === "canceled"
              ? "canceled"
              : navigationMission.status === "failed" || navigationMission.status === "needs_assistance"
                ? "failed"
                : null
    : null;
  const terminalFromMission: VacuumNavigationTerminalState | null = navigationMission
    ? navigationMission.status === "completed"
      ? "completed"
      : navigationMission.status === "canceled"
        ? "canceled"
        : navigationMission.status === "failed"
          ? "failed"
          : null
    : null;
  const navigationMissionActive = navigationMission ? ["preparing", "running", "canceling"].includes(navigationMission.status) : false;
  const activeMissionRunning = activeMission ? ["preparing", "running", "canceling", "resuming", "paused", "needs_assistance"].includes(activeMission.status) : false;
  const activeCleaningMission =
    activeMission?.type === "coverage" || activeMission?.type === "room_cleaning" || activeMission?.type === "zone_cleaning";
  const missionState =
    activeMission?.type === "navigation" && navigationMissionActive
      ? "navigating"
      : activeCleaningMission && activeMissionRunning
        ? activeMission.status === "paused" || activeMission.status === "needs_assistance"
          ? "paused"
          : "cleaning"
        : deriveMissionState(active, normalizedMapping.state);

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
    capabilities,
    health: {
      runtimeStatus: runtime.connectionStatus === "connected" ? "online" : runtime.connectionStatus === "disconnected" ? "offline" : "unknown",
      detail: runtime.connectionStatus === "connected" ? "Foxglove bridge connected." : "Robot bridge status is not online.",
    },
    source: {
      kind: "turtlebot4_nav2",
      status: runtime.connectionStatus === "connected" ? "reachable" : "unknown",
      stale: false,
      lastSeenAt: null,
      reason: runtime.connectionStatus === "connected" ? undefined : "bridge_not_connected",
    },
    dock: {
      supported: false,
      state: "unknown",
      charging: undefined,
      detail: "Dock state is not part of the TurtleBot4/Nav2 adapter slice.",
    },
    diagnostics: {
      backend: "turtlebot4_nav2",
      runtime: {
        connectionStatus: runtime.connectionStatus,
        availableTopicCount: runtime.availableTopics.length,
        availableServiceCount: runtime.availableServices.length,
      },
      warnings: faults,
    },
    map: {
      readiness: mapReadiness,
      topic: "/map",
      receiving: mapStatus === "receiving" || normalizedMapMetadata.hasMap,
      detail: mapStatus === "receiving" || normalizedMapMetadata.hasMap ? "Map is receiving." : "Waiting for live occupancy-grid data.",
      grid: mapGrid ?? null,
      metadata: normalizedMapMetadata,
      annotations: annotations ?? [],
    },
    pose: {
      readiness: poseReadiness,
      available: poseAvailable,
      source: runtime.helperPoseSource,
      coordinates: runtime.currentMapCoordinates,
      detail: poseAvailable ? "Pose is available." : "Waiting for localized pose or odometry fallback.",
    },
    navigation: {
      state: navigationFromMissionState ?? navigationState,
      backendGoalState: navigationMission?.phase ?? runtime.goalState,
      active: navigationMission ? ["preparing", "running", "canceling"].includes(navigationMission.status) : active,
      isSending: navigationMission ? navigationMission.status === "preparing" : runtime.isSendingGoal,
      isCanceling: navigationMission ? navigationMission.status === "canceling" : runtime.isCancelingGoal,
      currentTarget: navigationMissionTarget ?? currentTarget ?? null,
      terminalState: terminalFromMission ?? terminalState,
      planPath,
      progress: {
        distanceRemaining: navigationMission?.progress.distanceRemaining ?? distanceRemaining,
        initialDistance: initialDistance ?? null,
        recoveries: toFiniteNumber(runtime.feedbackRecoveries),
        navigationTime: runtime.feedbackNavigationTime,
        estimatedTimeRemaining: runtime.feedbackEta,
      },
      detail: navigationMission?.error?.message ?? navigationMission?.result?.summary ?? runtime.validationSummary.detail,
    },
    mission: {
      state: missionState,
      detail:
        missionState === "mapping"
          ? normalizedMapping.stateReason
          : missionState === "navigating"
            ? "Robot is navigating to a selected location."
            : missionState === "cleaning"
              ? activeMission?.type === "room_cleaning"
                ? "Robot is cleaning a selected room."
                : activeMission?.type === "zone_cleaning"
                  ? "Robot is cleaning a selected zone."
                  : "Robot is cleaning a selected area."
              : missionState === "paused" && activeCleaningMission
                ? activeMission?.error?.message ?? "Cleaning is paused."
            : (terminalFromMission ?? terminalState)
              ? `Last navigation ${terminalFromMission ?? terminalState}.`
              : "Robot is idle.",
      lastTerminalNavigation: terminalFromMission ?? terminalState,
    },
    activeMission,
    missions: {
      active: activeMission,
      recent: mergeRecentMissions(activeMission?.result ? [activeMission, ...(recentMissions ?? [])] : recentMissions ?? []),
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
