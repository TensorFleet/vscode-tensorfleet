import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { ros2Bridge } from "tensorfleet-ros";
import { normalizeRosMessage } from "../../../components/Nav2/runtime/nav2RuntimeUtils";
import { useNav2Runtime } from "../../../components/Nav2/runtime/useNav2Runtime";
import type { VacuumAdapter } from "../../adapter";
import type { VacuumCommand, VacuumCommandResult } from "../../commands";
import { buildVacuumMapMetadata, parseVacuumMapGrid } from "../../mapGrid";
import type { VacuumGoalCoordinates, VacuumMapGrid, VacuumMappingStatus, VacuumMissionSnapshot, VacuumSavedMapSummary } from "../../state";
import { MAPPING_STATUS_TOPIC, MISSION_SERVICE_NAMES, MISSION_STATUS_TOPIC } from "./capabilityMapper";
import { dispatchTurtleBot4Nav2Command } from "./commandDispatcher";
import { mapTurtleBot4Nav2State } from "./stateMapper";

function toFiniteNumber(value: unknown): number | null {
  const numeric = typeof value === "string" ? Number(value) : value;
  return typeof numeric === "number" && Number.isFinite(numeric) ? numeric : null;
}

function parseMappingStatus(message: Record<string, unknown> | null): VacuumMappingStatus | null {
  const rawData = message?.data;
  if (typeof rawData !== "string") {
    return null;
  }
  try {
    const parsed = JSON.parse(rawData) as Record<string, unknown>;
    const activeGoalRecord =
      parsed.activeGoal && typeof parsed.activeGoal === "object" ? (parsed.activeGoal as Record<string, unknown>) : null;
    const activeGoal =
      activeGoalRecord != null
        ? {
            x: toFiniteNumber(activeGoalRecord.x) ?? 0,
            y: toFiniteNumber(activeGoalRecord.y) ?? 0,
            yaw: toFiniteNumber(activeGoalRecord.yaw) ?? 0,
          }
        : null;
    const state = typeof parsed.state === "string" ? parsed.state : "idle";
    const mode = parsed.mode === "auto" || parsed.mode === "manual" ? parsed.mode : null;
    const savedMaps = Array.isArray(parsed.savedMaps)
      ? parsed.savedMaps.flatMap((entry): VacuumSavedMapSummary[] => {
          if (!entry || typeof entry !== "object") {
            return [];
          }
          const record = entry as Record<string, unknown>;
          const id = typeof record.id === "string" ? record.id : typeof record.name === "string" ? record.name : null;
          if (!id) {
            return [];
          }
          return [
            {
              id,
              name: typeof record.name === "string" ? record.name : id,
              yamlPath: typeof record.yamlPath === "string" ? record.yamlPath : "",
              imagePath: typeof record.imagePath === "string" ? record.imagePath : null,
              modifiedAt: toFiniteNumber(record.modifiedAt),
              sizeBytes: toFiniteNumber(record.sizeBytes) ?? 0,
              active: Boolean(record.active),
            },
          ];
        })
      : [];
    return {
      state: [
        "idle",
        "manual_mapping",
        "auto_mapping",
        "paused",
        "needs_assistance",
        "review",
        "accepted",
        "discarded",
        "error",
      ].includes(state)
        ? (state as VacuumMappingStatus["state"])
        : "error",
      mode,
      stateReason: typeof parsed.stateReason === "string" ? parsed.stateReason : "Mapping status received.",
      knownRatio: toFiniteNumber(parsed.knownRatio) ?? 0,
      unknownRatio: toFiniteNumber(parsed.unknownRatio) ?? 1,
      frontierCount: toFiniteNumber(parsed.frontierCount) ?? 0,
      visitedGoalCount: toFiniteNumber(parsed.visitedGoalCount) ?? 0,
      failedGoalCount: toFiniteNumber(parsed.failedGoalCount) ?? 0,
      activeGoal,
      lastError: typeof parsed.lastError === "string" ? parsed.lastError : null,
      updatedAt: toFiniteNumber(parsed.updatedAt),
      persistence:
        parsed.persistence === "session" || parsed.persistence === "persistent" || parsed.persistence === "unsupported"
          ? parsed.persistence
          : "session",
      acceptedSessionLevel: Boolean(parsed.acceptedSessionLevel),
      savedMapPath: typeof parsed.savedMapPath === "string" ? parsed.savedMapPath : null,
      loadedMapPath: typeof parsed.loadedMapPath === "string" ? parsed.loadedMapPath : null,
      lastSavedAt: toFiniteNumber(parsed.lastSavedAt),
      saveError: typeof parsed.saveError === "string" ? parsed.saveError : null,
      loadError: typeof parsed.loadError === "string" ? parsed.loadError : null,
      activeMapName: typeof parsed.activeMapName === "string" ? parsed.activeMapName : null,
      savedMaps,
    };
  } catch {
    return {
      state: "error",
      mode: null,
      stateReason: "Mapping status payload could not be parsed.",
      knownRatio: 0,
      unknownRatio: 1,
      frontierCount: 0,
      visitedGoalCount: 0,
      failedGoalCount: 0,
      activeGoal: null,
      lastError: "Mapping status payload could not be parsed.",
      updatedAt: Date.now(),
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
}

function isMissionStatus(value: unknown): value is VacuumMissionSnapshot["status"] {
  return (
    typeof value === "string" &&
    [
      "idle",
      "preparing",
      "running",
      "paused",
      "canceling",
      "returning",
      "charging",
      "resuming",
      "needs_assistance",
      "completed",
      "failed",
      "canceled",
      "unsupported",
    ].includes(value)
  );
}

function isMissionType(value: unknown): value is VacuumMissionSnapshot["type"] {
  return (
    typeof value === "string" &&
    [
      "mapping",
      "navigation",
      "coverage",
      "return_to_dock",
      "room_cleaning",
      "zone_cleaning",
      "hardware_cleaning",
    ].includes(value)
  );
}

function toMissionProgress(value: unknown): VacuumMissionSnapshot["progress"] {
  const record = value && typeof value === "object" ? (value as Record<string, unknown>) : {};
  return {
    percent: toFiniteNumber(record.percent),
    currentStep: toFiniteNumber(record.currentStep),
    totalSteps: toFiniteNumber(record.totalSteps),
    distanceRemaining: toFiniteNumber(record.distanceRemaining),
    areaCoveredSqM: toFiniteNumber(record.areaCoveredSqM),
    areaRemainingSqM: toFiniteNumber(record.areaRemainingSqM),
  };
}

function parseMissionStatusPayload(rawData: unknown): VacuumMissionSnapshot | null {
  if (typeof rawData !== "string") {
    return null;
  }
  try {
    const parsed = JSON.parse(rawData) as Record<string, unknown>;
    const mission = parsed.activeMission && typeof parsed.activeMission === "object"
      ? (parsed.activeMission as Record<string, unknown>)
      : null;
    if (!mission) {
      return null;
    }
    const status = isMissionStatus(mission.status) ? mission.status : "failed";
    const result = mission.result && typeof mission.result === "object"
      ? (mission.result as Record<string, unknown>)
      : null;
    const error = mission.error && typeof mission.error === "object"
      ? (mission.error as Record<string, unknown>)
      : null;
    const availableActions = Array.isArray(mission.availableActions)
      ? mission.availableActions.filter((action): action is VacuumMissionSnapshot["availableActions"][number] =>
          [
            "pause_mission",
            "resume_mission",
            "cancel_mission",
            "retry_mission_step",
            "skip_mission_step",
            "return_to_dock",
            "pause_mapping",
            "resume_mapping",
            "finish_mapping",
            "accept_map",
            "discard_mapping",
          ].includes(String(action)),
        )
      : [];
    return {
      id: typeof mission.id === "string" ? mission.id : "turtlebot4-nav2:navigation",
      type: isMissionType(mission.type) ? mission.type : "navigation",
      status,
      backendSource: "turtlebot4_nav2",
      startedAt: toFiniteNumber(mission.startedAt),
      updatedAt: toFiniteNumber(mission.updatedAt),
      requestedCommand: typeof mission.requestedCommand === "string" ? mission.requestedCommand : "start_navigation",
      phase: typeof mission.phase === "string" ? mission.phase : status,
      progress: toMissionProgress(mission.progress),
      availableActions,
      result:
        result && isMissionStatus(result.status) && ["completed", "failed", "canceled", "unsupported"].includes(result.status)
          ? {
              status: result.status as "completed" | "failed" | "canceled" | "unsupported",
              completedAt: toFiniteNumber(result.completedAt),
              summary: typeof result.summary === "string" ? result.summary : undefined,
            }
          : null,
      error:
        error && typeof error.message === "string"
          ? {
              code: typeof error.code === "string" ? error.code : "mission_error",
              message: error.message,
              recoverable: Boolean(error.recoverable),
            }
          : null,
      target: mission.target ?? null,
    };
  } catch {
    return null;
  }
}

function parseMissionStatus(message: Record<string, unknown> | null): VacuumMissionSnapshot | null {
  return parseMissionStatusPayload(message?.data);
}

function parseMissionServiceResponse(response: Record<string, unknown> | null): VacuumMissionSnapshot | null {
  return parseMissionStatusPayload(response?.message ?? response?.data);
}

function buildOptimisticNavigationMission(
  target: VacuumGoalCoordinates,
  canCancelMission: boolean,
): VacuumMissionSnapshot {
  const now = Date.now();
  return {
    id: `pending-navigation-${now}`,
    type: "navigation",
    status: "preparing",
    backendSource: "turtlebot4_nav2",
    startedAt: now,
    updatedAt: now,
    requestedCommand: "start_navigation",
    phase: "dispatching",
    progress: {
      percent: null,
      currentStep: null,
      totalSteps: null,
      distanceRemaining: null,
      areaCoveredSqM: null,
      areaRemainingSqM: null,
    },
    availableActions: canCancelMission ? ["cancel_mission"] : [],
    result: null,
    error: null,
    target,
  };
}

export function useTurtleBot4Nav2Adapter(): VacuumAdapter {
  const runtime = useNav2Runtime();
  const [currentTarget, setCurrentTarget] = useState<VacuumGoalCoordinates | null>(null);
  const [initialDistance, setInitialDistance] = useState<number | null>(null);
  const [mapGrid, setMapGrid] = useState<VacuumMapGrid | null>(null);
  const [mapLastUpdateAt, setMapLastUpdateAt] = useState<number | null>(null);
  const [mappingStatus, setMappingStatus] = useState<VacuumMappingStatus | null>(null);
  const [missionStatus, setMissionStatus] = useState<VacuumMissionSnapshot | null>(null);
  const runtimeRef = useRef(runtime);
  runtimeRef.current = runtime;

  const fetchMissionSnapshot = useCallback(async (): Promise<void> => {
    if (!ros2Bridge.isConnected()) {
      return;
    }
    try {
      const response = await ros2Bridge.callService<Record<string, unknown>>(
        MISSION_SERVICE_NAMES.getSnapshot,
        {},
        { timeoutMs: 5_000 },
      );
      const mission = parseMissionServiceResponse(response ?? null);
      setMissionStatus(mission);
    } catch {
      // Older runtimes may only publish /vacuum_mission/status.
    }
  }, []);

  useEffect(() => {
    const unsubscribe = ros2Bridge.subscribe({ topic: "/map", type: "nav_msgs/msg/OccupancyGrid" }, (message) => {
      const normalized = normalizeRosMessage(message);
      const grid = parseVacuumMapGrid(normalized);
      if (grid) {
        setMapGrid(grid);
        setMapLastUpdateAt(Date.now());
      }
    });
    return unsubscribe;
  }, []);

  useEffect(() => {
    const unsubscribe = ros2Bridge.subscribe({ topic: MAPPING_STATUS_TOPIC, type: "std_msgs/msg/String" }, (message) => {
      const normalized = normalizeRosMessage(message);
      setMappingStatus(parseMappingStatus(normalized));
    });
    return unsubscribe;
  }, []);

  useEffect(() => {
    const unsubscribe = ros2Bridge.subscribe({ topic: MISSION_STATUS_TOPIC, type: "std_msgs/msg/String" }, (message) => {
      const normalized = normalizeRosMessage(message);
      setMissionStatus(parseMissionStatus(normalized));
    });
    return unsubscribe;
  }, []);

  useEffect(() => {
    if (!runtime.availableServices.includes(MISSION_SERVICE_NAMES.getSnapshot)) {
      return;
    }
    void fetchMissionSnapshot();
  }, [fetchMissionSnapshot, runtime.availableServices]);

  const mapMetadata = useMemo(() => buildVacuumMapMetadata(mapGrid, mapLastUpdateAt), [mapGrid, mapLastUpdateAt]);

  const snapshot = useMemo(
    () =>
      mapTurtleBot4Nav2State({
        runtime,
        currentTarget,
        initialDistance,
        mapGrid,
        mapMetadata,
        mapping: mappingStatus,
        mission: missionStatus,
      }),
    [currentTarget, initialDistance, mapGrid, mapMetadata, mappingStatus, missionStatus, runtime],
  );

  const sendCommand = useCallback(
    async (command: VacuumCommand): Promise<VacuumCommandResult> => {
      const result = await dispatchTurtleBot4Nav2Command(command, {
        runtime: {
          ...runtimeRef.current,
          callService: async (name, request, opts) => await ros2Bridge.callService(name, request, opts),
        },
        snapshot,
        setCurrentTarget,
        setInitialDistance,
      });
      if (result.ok && command.command === "start_navigation") {
        setMissionStatus(buildOptimisticNavigationMission(command.target, snapshot.capabilities.cancel_mission.supported));
        void fetchMissionSnapshot();
      } else if (result.ok && command.command === "cancel_mission") {
        void fetchMissionSnapshot();
      }
      return result;
    },
    [fetchMissionSnapshot, snapshot.capabilities, snapshot.readiness],
  );

  return {
    snapshot,
    sendCommand,
  };
}
