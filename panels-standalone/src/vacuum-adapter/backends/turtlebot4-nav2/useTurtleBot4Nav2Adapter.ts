import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { ros2Bridge } from "tensorfleet-ros";
import { normalizeRosMessage } from "../../../components/Nav2/runtime/nav2RuntimeUtils";
import { useNav2Runtime } from "../../../components/Nav2/runtime/useNav2Runtime";
import type { VacuumAdapter } from "../../adapter";
import type { VacuumCommand, VacuumCommandResult } from "../../commands";
import { buildVacuumMapMetadata, parseVacuumMapGrid } from "../../mapGrid";
import type { VacuumGoalCoordinates, VacuumMapAnnotation, VacuumMapGrid, VacuumMappingStatus, VacuumMissionSnapshot, VacuumSavedMapSummary } from "../../state";
import { MAP_ANNOTATION_SERVICE_NAMES, MAPPING_STATUS_TOPIC, MISSION_SERVICE_NAMES, MISSION_STATUS_TOPIC } from "./capabilityMapper";
import { dispatchTurtleBot4Nav2Command } from "./commandDispatcher";
import {
  hasMigratedLocalPrototypeMapAnnotations,
  markLocalPrototypeMapAnnotationsMigrated,
  readLocalPrototypeMapAnnotations,
} from "./localAnnotationMigration";
import { mapTurtleBot4Nav2State } from "./stateMapper";

const RECENT_MISSIONS_STORAGE_KEY = "tensorfleet:vacuums:turtlebot4-nav2:recent-missions";

function toFiniteNumber(value: unknown): number | null {
  const numeric = typeof value === "string" ? Number(value) : value;
  return typeof numeric === "number" && Number.isFinite(numeric) ? numeric : null;
}

function isTerminalMissionStatus(status: VacuumMissionSnapshot["status"]): boolean {
  return status === "completed" || status === "failed" || status === "canceled" || status === "unsupported";
}

function parseMapAnnotation(value: unknown): VacuumMapAnnotation | null {
  const record = value && typeof value === "object" ? (value as Partial<VacuumMapAnnotation>) : null;
  if (
    !record ||
    typeof record.id !== "string" ||
    (record.kind !== "room" && record.kind !== "zone") ||
    typeof record.name !== "string" ||
    !record.area ||
    typeof record.area !== "object"
  ) {
    return null;
  }
  return {
    id: record.id,
    kind: record.kind,
    name: record.name,
    area: record.area as VacuumMapAnnotation["area"],
    mapId: typeof record.mapId === "string" ? record.mapId : null,
    createdAt: toFiniteNumber(record.createdAt) ?? Date.now(),
    updatedAt: toFiniteNumber(record.updatedAt) ?? Date.now(),
  };
}

function parseMapAnnotationServiceResponse(response: Record<string, unknown> | null): VacuumMapAnnotation[] | null {
  const rawData = response?.message ?? response?.data;
  if (typeof rawData !== "string") {
    return null;
  }
  try {
    const parsed = JSON.parse(rawData) as Record<string, unknown>;
    if (!Array.isArray(parsed.annotations)) {
      return null;
    }
    return parsed.annotations.flatMap((entry) => {
      const annotation = parseMapAnnotation(entry);
      return annotation ? [annotation] : [];
    });
  } catch {
    return null;
  }
}

function filterMapAnnotationsForMap(annotations: VacuumMapAnnotation[], mapId: string): VacuumMapAnnotation[] {
  return annotations.flatMap((annotation) => {
    if (annotation.mapId != null && annotation.mapId !== mapId) {
      return [];
    }
    return [{ ...annotation, mapId: annotation.mapId ?? mapId }];
  });
}

function assertMapAnnotationServiceSuccess(response: Record<string, unknown> | null, serviceName: string): void {
  if (response?.success === false) {
    throw new Error(typeof response.message === "string" ? response.message : `${serviceName} returned failure.`);
  }
}

function missionSortTime(mission: VacuumMissionSnapshot): number {
  return mission.result?.completedAt ?? mission.updatedAt ?? mission.startedAt ?? 0;
}

function mergeRecentMissions(...missionLists: VacuumMissionSnapshot[][]): VacuumMissionSnapshot[] {
  const byId = new Map<string, VacuumMissionSnapshot>();
  for (const mission of missionLists.flat()) {
    if (!isTerminalMissionStatus(mission.status)) {
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

function readStoredRecentMissions(storageKey: string): VacuumMissionSnapshot[] {
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
    return mergeRecentMissions(parsed.flatMap((entry) => {
      const mission = parseMissionSnapshot(entry);
      return mission ? [mission] : [];
    }));
  } catch {
    return [];
  }
}

function writeStoredRecentMissions(storageKey: string, missions: VacuumMissionSnapshot[]): void {
  if (typeof window === "undefined") {
    return;
  }
  try {
    window.localStorage.setItem(storageKey, JSON.stringify(mergeRecentMissions(missions)) ?? "[]");
  } catch {
    // Recent mission summaries are best-effort until VM-owned persistence lands.
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

type ParsedMissionPayload = {
  activeMission: VacuumMissionSnapshot | null;
  recentMissions: VacuumMissionSnapshot[];
};

function parseMissionSnapshot(value: unknown): VacuumMissionSnapshot | null {
  const mission = value && typeof value === "object" ? (value as Record<string, unknown>) : null;
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
  const startedAt = toFiniteNumber(mission.startedAt);
  const updatedAt = toFiniteNumber(mission.updatedAt);
  return {
    id: typeof mission.id === "string" ? mission.id : `turtlebot4-nav2:mission:${updatedAt ?? startedAt ?? "unknown"}`,
    type: isMissionType(mission.type) ? mission.type : "navigation",
    status,
    backendSource: "turtlebot4_nav2",
    startedAt,
    updatedAt,
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
        : isTerminalMissionStatus(status)
          ? {
              status,
              completedAt: updatedAt,
              summary: typeof mission.phase === "string" ? mission.phase : undefined,
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
}

function parseMissionStatusPayload(rawData: unknown): ParsedMissionPayload {
  if (typeof rawData !== "string") {
    return { activeMission: null, recentMissions: [] };
  }
  try {
    const parsed = JSON.parse(rawData) as Record<string, unknown>;
    const activeMission = parseMissionSnapshot(parsed.activeMission);
    const missionsRecord = parsed.missions && typeof parsed.missions === "object" ? (parsed.missions as Record<string, unknown>) : null;
    const rawRecent = Array.isArray(missionsRecord?.recent)
      ? missionsRecord.recent
      : Array.isArray(parsed.recentMissions)
        ? parsed.recentMissions
        : [];
    const recentMissions = mergeRecentMissions(
      rawRecent.flatMap((entry) => {
        const mission = parseMissionSnapshot(entry);
        return mission ? [mission] : [];
      }),
      activeMission ? [activeMission] : [],
    );
    return { activeMission, recentMissions };
  } catch {
    return { activeMission: null, recentMissions: [] };
  }
}

function parseMissionStatus(message: Record<string, unknown> | null): ParsedMissionPayload {
  return parseMissionStatusPayload(message?.data);
}

function parseMissionServiceResponse(response: Record<string, unknown> | null): ParsedMissionPayload {
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

function buildOptimisticRoomZoneMission(
  command: Extract<VacuumCommand, { command: "start_room_cleaning" | "start_zone_cleaning" }>,
  canCancelMission: boolean,
): VacuumMissionSnapshot {
  const now = Date.now();
  const type = command.annotation.kind === "room" ? "room_cleaning" : "zone_cleaning";
  return {
    id: `pending-${command.annotation.kind}-cleaning-${now}`,
    type,
    status: "preparing",
    backendSource: "turtlebot4_nav2",
    startedAt: now,
    updatedAt: now,
    requestedCommand: command.command,
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
      area: command.annotation.area,
      annotation: {
        id: command.annotation.id,
        kind: command.annotation.kind,
        name: command.annotation.name,
        mapId: command.annotation.mapId,
      },
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
  const [recentMissions, setRecentMissions] = useState<VacuumMissionSnapshot[]>(() =>
    readStoredRecentMissions(RECENT_MISSIONS_STORAGE_KEY),
  );
  const [annotations, setAnnotations] = useState<VacuumMapAnnotation[]>([]);
  const runtimeRef = useRef(runtime);
  runtimeRef.current = runtime;

  const activeMapId = useMemo(
    () => mappingStatus?.activeMapName ?? mappingStatus?.loadedMapPath ?? mappingStatus?.savedMapPath ?? "live-map",
    [mappingStatus?.activeMapName, mappingStatus?.loadedMapPath, mappingStatus?.savedMapPath],
  );
  const activeMapIdRef = useRef(activeMapId);
  const annotationRequestIdRef = useRef(0);
  activeMapIdRef.current = activeMapId;

  const beginAnnotationRequest = useCallback((): number => {
    annotationRequestIdRef.current += 1;
    return annotationRequestIdRef.current;
  }, []);

  const commitAnnotationsForRequest = useCallback(
    (requestId: number, mapId: string, nextAnnotations: VacuumMapAnnotation[]): boolean => {
      if (annotationRequestIdRef.current !== requestId || activeMapIdRef.current !== mapId) {
        return false;
      }
      setAnnotations(filterMapAnnotationsForMap(nextAnnotations, mapId));
      return true;
    },
    [],
  );

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
      const missions = parseMissionServiceResponse(response ?? null);
      setMissionStatus(missions.activeMission);
      setRecentMissions((current) => mergeRecentMissions(missions.recentMissions, current));
    } catch {
      // Older runtimes may only publish /vacuum_mission/status.
    }
  }, []);

  const setRuntimeAnnotationRequest = useCallback(async (payload: Record<string, unknown>): Promise<void> => {
    const response = await ros2Bridge.callService<Record<string, unknown>>(
      MISSION_SERVICE_NAMES.setParameters,
      {
        parameters: [
          {
            name: "map_annotation_request",
            value: {
              type: 4,
              string_value: JSON.stringify(payload),
            },
          },
        ],
      },
      { timeoutMs: 5_000 },
    );
    const results = Array.isArray(response?.results) ? response.results : [];
    const failed = results.find((entry) => entry && typeof entry === "object" && (entry as { successful?: boolean }).successful === false);
    if (failed) {
      throw new Error(
        typeof (failed as { reason?: unknown }).reason === "string"
          ? (failed as { reason: string }).reason
          : "Map annotation request parameter update failed.",
      );
    }
  }, []);

  const fetchRuntimeAnnotations = useCallback(async (mapId: string): Promise<VacuumMapAnnotation[]> => {
    const requestId = beginAnnotationRequest();
    const commitAnnotations = (nextAnnotations: VacuumMapAnnotation[]): boolean =>
      commitAnnotationsForRequest(requestId, mapId, nextAnnotations);

    commitAnnotations([]);
    if (
      !ros2Bridge.isConnected() ||
      !runtimeRef.current.availableServices.includes(MISSION_SERVICE_NAMES.setParameters) ||
      !runtimeRef.current.availableServices.includes(MAP_ANNOTATION_SERVICE_NAMES.getSnapshot)
    ) {
      return [];
    }
    try {
      await setRuntimeAnnotationRequest({ mapId });
      const response = await ros2Bridge.callService<Record<string, unknown>>(
        MAP_ANNOTATION_SERVICE_NAMES.getSnapshot,
        {},
        { timeoutMs: 5_000 },
      );
      const runtimeAnnotations = filterMapAnnotationsForMap(parseMapAnnotationServiceResponse(response ?? null) ?? [], mapId);
      if (runtimeAnnotations.length > 0) {
        markLocalPrototypeMapAnnotationsMigrated(mapId);
        commitAnnotations(runtimeAnnotations);
        return runtimeAnnotations;
      }
      if (hasMigratedLocalPrototypeMapAnnotations(mapId)) {
        commitAnnotations(runtimeAnnotations);
        return runtimeAnnotations;
      }

      const localAnnotations = readLocalPrototypeMapAnnotations(mapId);
      if (localAnnotations.length === 0) {
        markLocalPrototypeMapAnnotationsMigrated(mapId);
        commitAnnotations(runtimeAnnotations);
        return runtimeAnnotations;
      }

      let nextAnnotations: VacuumMapAnnotation[] | null = null;
      for (const annotation of localAnnotations) {
        await setRuntimeAnnotationRequest({ mapId, annotation });
        const saveResponse = await ros2Bridge.callService<Record<string, unknown>>(
          MAP_ANNOTATION_SERVICE_NAMES.save,
          {},
          { timeoutMs: 5_000 },
        );
        assertMapAnnotationServiceSuccess(saveResponse ?? null, MAP_ANNOTATION_SERVICE_NAMES.save);
        const saveAnnotations = parseMapAnnotationServiceResponse(saveResponse ?? null);
        nextAnnotations = saveAnnotations ? filterMapAnnotationsForMap(saveAnnotations, mapId) : nextAnnotations;
      }
      markLocalPrototypeMapAnnotationsMigrated(mapId);
      const migratedAnnotations = nextAnnotations ?? localAnnotations;
      commitAnnotations(migratedAnnotations);
      return migratedAnnotations;
    } catch {
      commitAnnotations([]);
      return [];
    }
  }, [beginAnnotationRequest, commitAnnotationsForRequest, setRuntimeAnnotationRequest]);

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
      const missions = parseMissionStatus(normalized);
      setMissionStatus(missions.activeMission);
      setRecentMissions((current) => mergeRecentMissions(missions.recentMissions, current));
    });
    return unsubscribe;
  }, []);

  useEffect(() => {
    writeStoredRecentMissions(RECENT_MISSIONS_STORAGE_KEY, recentMissions);
  }, [recentMissions]);

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

  useEffect(() => {
    void fetchRuntimeAnnotations(activeMapId);
  }, [activeMapId, fetchRuntimeAnnotations, runtime.availableServices]);

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
        recentMissions,
        annotations,
      }),
    [annotations, currentTarget, initialDistance, mapGrid, mapMetadata, mappingStatus, missionStatus, recentMissions, runtime],
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
          mapId: activeMapId,
          name: command.annotation.name.trim() || (command.annotation.kind === "room" ? "Room" : "Zone"),
          createdAt: command.annotation.createdAt ?? now,
          updatedAt: now,
        };
        try {
          const requestId = beginAnnotationRequest();
          await setRuntimeAnnotationRequest({ mapId: activeMapId, annotation });
          const response = await ros2Bridge.callService<Record<string, unknown>>(
            MAP_ANNOTATION_SERVICE_NAMES.save,
            {},
            { timeoutMs: 5_000 },
          );
          assertMapAnnotationServiceSuccess(response ?? null, MAP_ANNOTATION_SERVICE_NAMES.save);
          const nextAnnotations = parseMapAnnotationServiceResponse(response ?? null);
          if (nextAnnotations) {
            commitAnnotationsForRequest(requestId, activeMapId, nextAnnotations);
          } else {
            await fetchRuntimeAnnotations(activeMapId);
          }
          return { ok: true, command: command.command, message: "Saved map annotation." };
        } catch (error) {
          return {
            ok: false,
            command: command.command,
            error: {
              code: "backend_error",
              command: command.command,
              message: error instanceof Error ? error.message : String(error),
            },
          };
        }
      }

      if (command.command === "delete_map_annotation") {
        try {
          const requestId = beginAnnotationRequest();
          await setRuntimeAnnotationRequest({ mapId: activeMapId, id: command.id });
          const response = await ros2Bridge.callService<Record<string, unknown>>(
            MAP_ANNOTATION_SERVICE_NAMES.delete,
            {},
            { timeoutMs: 5_000 },
          );
          assertMapAnnotationServiceSuccess(response ?? null, MAP_ANNOTATION_SERVICE_NAMES.delete);
          const nextAnnotations = parseMapAnnotationServiceResponse(response ?? null);
          if (nextAnnotations) {
            commitAnnotationsForRequest(requestId, activeMapId, nextAnnotations);
          } else {
            await fetchRuntimeAnnotations(activeMapId);
          }
          return { ok: true, command: command.command, message: "Deleted map annotation." };
        } catch (error) {
          return {
            ok: false,
            command: command.command,
            error: {
              code: "backend_error",
              command: command.command,
              message: error instanceof Error ? error.message : String(error),
            },
          };
        }
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
      } else if (result.ok && (command.command === "start_room_cleaning" || command.command === "start_zone_cleaning")) {
        setMissionStatus(buildOptimisticRoomZoneMission(command, snapshot.capabilities.cancel_mission.supported));
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
    [
      activeMapId,
      beginAnnotationRequest,
      commitAnnotationsForRequest,
      fetchMissionSnapshot,
      fetchRuntimeAnnotations,
      setRuntimeAnnotationRequest,
      snapshot,
      snapshot.capabilities,
      snapshot.readiness,
    ],
  );

  return {
    snapshot,
    sendCommand,
  };
}
