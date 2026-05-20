import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { ros2Bridge } from "tensorfleet-ros";
import { normalizeRosMessage } from "../../../components/Nav2/runtime/nav2RuntimeUtils";
import { useNav2Runtime } from "../../../components/Nav2/runtime/useNav2Runtime";
import type { VacuumAdapter } from "../../adapter";
import type { VacuumCommand, VacuumCommandResult } from "../../commands";
import { buildVacuumMapMetadata, parseVacuumMapGrid } from "../../mapGrid";
import type { VacuumGoalCoordinates, VacuumMapAnnotation, VacuumMapGrid, VacuumMappingStatus, VacuumMissionSnapshot, VacuumSavedMapSummary } from "../../state";
import { MAPPING_STATUS_TOPIC, MISSION_SERVICE_NAMES, MISSION_STATUS_TOPIC } from "./capabilityMapper";
import { dispatchTurtleBot4Nav2Command } from "./commandDispatcher";
import { mapTurtleBot4Nav2State } from "./stateMapper";

function toFiniteNumber(value: unknown): number | null {
  const numeric = typeof value === "string" ? Number(value) : value;
  return typeof numeric === "number" && Number.isFinite(numeric) ? numeric : null;
}

function readStoredAnnotations(storageKey: string): VacuumMapAnnotation[] {
  if (typeof window === "undefined") {
    return [];
  }
  try {
    const raw = window.localStorage.getItem(storageKey);
    if (!raw) {
      return [];
    }
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) {
      return [];
    }
    return parsed.flatMap((entry): VacuumMapAnnotation[] => {
      if (!entry || typeof entry !== "object") {
        return [];
      }
      const record = entry as Partial<VacuumMapAnnotation>;
      if (
        typeof record.id !== "string" ||
        (record.kind !== "room" && record.kind !== "zone") ||
        typeof record.name !== "string" ||
        !record.area ||
        typeof record.area !== "object"
      ) {
        return [];
      }
      return [
        {
          id: record.id,
          kind: record.kind,
          name: record.name,
          area: record.area as VacuumMapAnnotation["area"],
          mapId: typeof record.mapId === "string" ? record.mapId : null,
          createdAt: toFiniteNumber(record.createdAt) ?? Date.now(),
          updatedAt: toFiniteNumber(record.updatedAt) ?? Date.now(),
        },
      ];
    });
  } catch {
    return [];
  }
}

function writeStoredAnnotations(storageKey: string, annotations: VacuumMapAnnotation[]): void {
  if (typeof window === "undefined") {
    return;
  }
  try {
    window.localStorage.setItem(storageKey, JSON.stringify(annotations));
  } catch {
    // Annotation durability is best-effort until VM-owned persistence lands.
  }
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

function buildOptimisticCoverageMission(
  command: Extract<VacuumCommand, { command: "start_coverage" }>,
  canCancelMission: boolean,
): VacuumMissionSnapshot {
  const now = Date.now();
  return {
    id: `pending-coverage-${now}`,
    type: "coverage",
    status: "preparing",
    backendSource: "turtlebot4_nav2",
    startedAt: now,
    updatedAt: now,
    requestedCommand: "start_coverage",
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
    target: {
      area: command.area,
      coverage: command.coverage ?? null,
      route: null,
    },
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
  const [annotations, setAnnotations] = useState<VacuumMapAnnotation[]>([]);
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

  useEffect(() => {
    if (!runtime.availableServices.includes(MISSION_SERVICE_NAMES.getSnapshot)) {
      return undefined;
    }
    let canceled = false;
    const poll = () => {
      if (!canceled) {
        void fetchMissionSnapshot();
      }
    };
    poll();
    const interval = window.setInterval(poll, 1_000);
    return () => {
      canceled = true;
      window.clearInterval(interval);
    };
  }, [fetchMissionSnapshot, runtime.availableServices]);

  const mapMetadata = useMemo(() => buildVacuumMapMetadata(mapGrid, mapLastUpdateAt), [mapGrid, mapLastUpdateAt]);
  const annotationStorageKey = useMemo(
    () => `tensorfleet:vacuums:turtlebot4-nav2:map-annotations:${mappingStatus?.activeMapName ?? mappingStatus?.loadedMapPath ?? mappingStatus?.savedMapPath ?? "live-map"}`,
    [mappingStatus?.activeMapName, mappingStatus?.loadedMapPath, mappingStatus?.savedMapPath],
  );

  useEffect(() => {
    setAnnotations(readStoredAnnotations(annotationStorageKey));
  }, [annotationStorageKey]);

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
        annotations,
      }),
    [annotations, currentTarget, initialDistance, mapGrid, mapMetadata, mappingStatus, missionStatus, runtime],
  );

  const sendCommand = useCallback(
    async (command: VacuumCommand): Promise<VacuumCommandResult> => {
      if (command.command === "save_map_annotation") {
        if (!snapshot.capabilities.map_annotations.supported) {
          return {
            ok: false,
            command: command.command,
            error: {
              code: "unsupported",
              command: command.command,
              message: "Map annotations are not supported by this backend.",
            },
          };
        }
        const now = Date.now();
        const annotation: VacuumMapAnnotation = {
          ...command.annotation,
          name: command.annotation.name.trim() || (command.annotation.kind === "room" ? "Room" : "Zone"),
          createdAt: command.annotation.createdAt ?? now,
          updatedAt: now,
        };
        setAnnotations((current) => {
          const existing = current.find((entry) => entry.id === annotation.id);
          const nextAnnotation = existing ? { ...annotation, createdAt: existing.createdAt } : annotation;
          const next = [...current.filter((entry) => entry.id !== annotation.id), nextAnnotation]
            .sort((a, b) => a.name.localeCompare(b.name));
          writeStoredAnnotations(annotationStorageKey, next);
          return next;
        });
        return { ok: true, command: command.command, message: "Saved map annotation." };
      }

      if (command.command === "delete_map_annotation") {
        setAnnotations((current) => {
          const next = current.filter((entry) => entry.id !== command.id);
          writeStoredAnnotations(annotationStorageKey, next);
          return next;
        });
        return { ok: true, command: command.command, message: "Deleted map annotation." };
      }

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
      } else if (result.ok && command.command === "start_coverage") {
        setMissionStatus(buildOptimisticCoverageMission(command, snapshot.capabilities.cancel_mission.supported));
        void fetchMissionSnapshot();
      } else if (
        result.ok &&
        (
          command.command === "cancel_mission" ||
          command.command === "pause_mission" ||
          command.command === "resume_mission" ||
          command.command === "retry_mission_step" ||
          command.command === "skip_mission_step"
        )
      ) {
        void fetchMissionSnapshot();
      }
      return result;
    },
    [annotationStorageKey, fetchMissionSnapshot, snapshot.capabilities, snapshot.readiness],
  );

  return {
    snapshot,
    sendCommand,
  };
}
