import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { ros2Bridge } from "../../../ros2-bridge";
import { normalizeRosMessage } from "../../../components/Nav2/runtime/nav2RuntimeUtils";
import { useNav2Runtime } from "../../../components/Nav2/runtime/useNav2Runtime";
import type { VacuumAdapter } from "../../adapter";
import type { VacuumCommand, VacuumCommandResult } from "../../commands";
import { buildVacuumMapMetadata, parseVacuumMapGrid } from "../../mapGrid";
import type { VacuumGoalCoordinates, VacuumMapGrid, VacuumMappingStatus } from "../../state";
import { MAPPING_STATUS_TOPIC } from "./capabilityMapper";
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
      lastSavedAt: toFiniteNumber(parsed.lastSavedAt),
      saveError: typeof parsed.saveError === "string" ? parsed.saveError : null,
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
      lastSavedAt: null,
      saveError: null,
    };
  }
}

export function useTurtleBot4Nav2Adapter(): VacuumAdapter {
  const runtime = useNav2Runtime();
  const [currentTarget, setCurrentTarget] = useState<VacuumGoalCoordinates | null>(null);
  const [initialDistance, setInitialDistance] = useState<number | null>(null);
  const [mapGrid, setMapGrid] = useState<VacuumMapGrid | null>(null);
  const [mapLastUpdateAt, setMapLastUpdateAt] = useState<number | null>(null);
  const [mappingStatus, setMappingStatus] = useState<VacuumMappingStatus | null>(null);
  const runtimeRef = useRef(runtime);
  runtimeRef.current = runtime;

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
      }),
    [currentTarget, initialDistance, mapGrid, mapMetadata, mappingStatus, runtime],
  );

  const sendCommand = useCallback(
    async (command: VacuumCommand): Promise<VacuumCommandResult> => {
      return await dispatchTurtleBot4Nav2Command(command, {
        runtime: {
          ...runtimeRef.current,
          callService: async (name, request, opts) => await ros2Bridge.callService(name, request, opts),
        },
        snapshot,
        setCurrentTarget,
        setInitialDistance,
      });
    },
    [snapshot.capabilities, snapshot.readiness],
  );

  return {
    snapshot,
    sendCommand,
  };
}
