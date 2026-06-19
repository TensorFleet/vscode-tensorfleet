import type { Tool } from "@modelcontextprotocol/sdk/types.js";
import { resolveMcpRuntimeConfig } from "./config";
import {
  createVacuumRuntimeContext,
  dispatchVacuumCommand,
  fetchVacuumHealth,
  fetchVacuumSnapshot,
  type VacuumBackendId,
} from "./vacuum-runtime";
import { mcpFailure, mcpSuccess, type TensorFleetMcpResult } from "./result";

type JsonRecord = Record<string, unknown>;

type VacuumToolDefinition = Tool & {
  execute: (args: JsonRecord) => Promise<TensorFleetMcpResult>;
};

type RuntimeSnapshot = JsonRecord & {
  identity?: JsonRecord;
  runtime?: JsonRecord;
  robot?: JsonRecord;
  availability?: JsonRecord;
  health?: JsonRecord;
  source?: JsonRecord;
  connectivity?: JsonRecord;
  state?: JsonRecord;
  activity?: JsonRecord;
  battery?: JsonRecord;
  dock?: JsonRecord;
  cleaningSettings?: {
    fanSpeed?: { current?: string; options?: Array<string | { value?: string; label?: string }> };
    waterUsage?: { current?: string; options?: Array<string | { value?: string; label?: string }> };
  };
  map?: JsonRecord & {
    readiness?: string;
    receiving?: boolean;
    metadata?: JsonRecord;
    layeredMetadata?: JsonRecord;
    preview?: unknown;
    layeredPreview?: unknown;
    targets?: {
      segments?: MapTarget[];
      rooms?: MapTarget[];
      zones?: MapTarget[];
    };
    annotations?: MapAnnotation[];
  };
  pose?: JsonRecord;
  navigation?: JsonRecord & {
    active?: boolean;
    isSending?: boolean;
    isCanceling?: boolean;
    currentTarget?: unknown;
    terminalState?: string | null;
    planPath?: unknown;
    progress?: JsonRecord;
    state?: string;
    detail?: string;
  };
  mission?: JsonRecord;
  activeMission?: JsonRecord | null;
  missions?: JsonRecord;
  mapping?: JsonRecord;
  readiness?: JsonRecord;
  fault?: JsonRecord;
  capabilities?: {
    commands?: Record<string, CommandAvailability>;
    diagnostics?: unknown[];
  } & Record<string, unknown>;
  diagnostics?: JsonRecord;
  rawDiagnostics?: unknown;
  updatedAt?: number;
};

type MapTarget = {
  id?: string;
  label?: string;
  kind?: string;
  available?: boolean;
  geometry?: unknown;
  detail?: string;
};

type MapAnnotation = {
  id?: string;
  name?: string;
  kind?: string;
  area?: unknown;
};

type CommandAvailability = {
  supported?: boolean;
  available?: boolean;
  reason?: string;
  availabilityReason?: string;
  status?: string;
  notes?: string;
};

const SIMULATION_CAPABILITY_NAMES = [
  "map",
  "pose",
  "navigation_status",
  "mission_state",
  "start_navigation",
  "go_to_location",
  "start_coverage",
  "pause_mission",
  "resume_mission",
  "cancel_mission",
  "retry_mission_step",
  "skip_mission_step",
  "mapping_session",
  "auto_mapping",
  "map_annotations",
  "room_semantics",
  "zone_semantics",
  "room_cleaning",
  "zone_cleaning",
  "fan_speed",
  "water_usage",
] as const;

const COMMAND_TOOL_TO_RUNTIME_COMMAND = {
  vacuum_start_cleaning: "start_cleaning",
  vacuum_pause: "pause",
  vacuum_resume: "resume",
  vacuum_stop: "stop",
  vacuum_return_to_dock: "return_to_dock",
  vacuum_set_fan_speed: "set_fan_speed",
  vacuum_set_water_usage: "set_water_usage",
  vacuum_pause_mission: "pause_mission",
  vacuum_resume_mission: "resume_mission",
  vacuum_cancel_mission: "cancel_mission",
  vacuum_retry_mission_step: "retry_mission_step",
  vacuum_skip_mission_step: "skip_mission_step",
} as const;

const SIMULATION_MISSION_ACTION_COMMANDS = [
  "pause_mission",
  "resume_mission",
  "cancel_mission",
  "retry_mission_step",
  "skip_mission_step",
] as const;

export const VACUUM_MCP_TOOL_NAMES = [
  "vacuum_get_health",
  "vacuum_get_snapshot",
  "vacuum_get_capabilities",
  "vacuum_get_map_targets",
  "vacuum_get_pose",
  "vacuum_get_map_summary",
  "vacuum_get_mission_state",
  "vacuum_get_navigation_state",
  "vacuum_check_navigation_readiness",
  "vacuum_check_clean_area_readiness",
  "vacuum_get_supported_actions",
  "vacuum_start_navigation",
  "vacuum_start_clean_area",
  ...Object.keys(COMMAND_TOOL_TO_RUNTIME_COMMAND),
] as const;

type CommandToolName = keyof typeof COMMAND_TOOL_TO_RUNTIME_COMMAND;
type RuntimeCommandName = (typeof COMMAND_TOOL_TO_RUNTIME_COMMAND)[CommandToolName];
type SimulationMissionActionCommand = (typeof SIMULATION_MISSION_ACTION_COMMANDS)[number];
type NavigationStartTarget = {
  x: number;
  y: number;
  theta: number;
  frameId: string;
  label?: string;
};
type CleanAreaStartArea = {
  type: "rectangle";
  x: number;
  y: number;
  width: number;
  height: number;
  frameId: string;
  label?: string;
};

export function createVacuumTools(): Map<string, VacuumToolDefinition> {
  const tools = new Map<string, VacuumToolDefinition>();

  tools.set("vacuum_get_health", {
    name: "vacuum_get_health",
    description: "Fetch TensorFleet vacuum runtime and source health for the selected backend.",
    inputSchema: emptyInputSchema(),
    execute: async () => withRuntime(async (context) => fetchVacuumHealth(context)),
  });

  tools.set("vacuum_get_snapshot", {
    name: "vacuum_get_snapshot",
    description: "Fetch a compact TensorFleet product-level vacuum snapshot.",
    inputSchema: {
      type: "object",
      properties: {
        include_diagnostics: { type: "boolean", default: false },
        include_raw_diagnostics: { type: "boolean", default: false },
        include_map_preview: { type: "boolean", default: false },
      },
      additionalProperties: false,
    },
    execute: async (args) =>
      withRuntime(async (context) => {
        const snapshot = await fetchSelectedVacuumSnapshot(context);
        if (!snapshot.ok || !snapshot.data) return snapshot;
        return mcpSuccess(
          "Vacuum snapshot fetched.",
          normalizeSnapshot(snapshot.data as RuntimeSnapshot, {
            backend: context.backend,
            includeDiagnostics: args.include_diagnostics === true,
            includeRawDiagnostics: args.include_raw_diagnostics === true,
            includeMapPreview: args.include_map_preview === true,
          }),
        );
      }),
  });

  tools.set("vacuum_get_capabilities", {
    name: "vacuum_get_capabilities",
    description: "Return normalized TensorFleet vacuum command support and current availability.",
    inputSchema: emptyInputSchema(),
    execute: async () =>
      withRuntime(async (context) => {
        const snapshot = await fetchSelectedVacuumSnapshot(context);
        if (!snapshot.ok || !snapshot.data) return snapshot;
        return mcpSuccess("Vacuum capabilities fetched.", normalizeCapabilities(snapshot.data as RuntimeSnapshot, context.backend));
      }),
  });

  tools.set("vacuum_get_map_targets", {
    name: "vacuum_get_map_targets",
    description: "Return read-only normalized vacuum map target inventory; this does not enable target cleaning commands.",
    inputSchema: {
      type: "object",
      properties: {
        include_geometry: { type: "boolean", default: false },
      },
      additionalProperties: false,
    },
    execute: async (args) =>
      withRuntime(async (context) => {
        const snapshot = await fetchSelectedVacuumSnapshot(context);
        if (!snapshot.ok || !snapshot.data) return snapshot;
        return mcpSuccess("Vacuum map targets fetched.", normalizeMapTargets(snapshot.data as RuntimeSnapshot, args.include_geometry === true));
      }),
  });

  tools.set("vacuum_get_pose", {
    name: "vacuum_get_pose",
    description: "Return the selected vacuum backend's normalized robot pose state when available.",
    inputSchema: emptyInputSchema(),
    execute: async () =>
      withRuntime(async (context) => {
        const snapshot = await fetchSelectedVacuumSnapshot(context);
        if (!snapshot.ok || !snapshot.data) return snapshot;
        return mcpSuccess("Vacuum pose fetched.", normalizePose(snapshot.data as RuntimeSnapshot));
      }),
  });

  tools.set("vacuum_get_map_summary", {
    name: "vacuum_get_map_summary",
    description: "Return a compact normalized vacuum map summary; grid and geometry payloads are omitted by default because they can be large.",
    inputSchema: {
      type: "object",
      properties: {
        include_grid: { type: "boolean", default: false },
        include_geometry: { type: "boolean", default: false },
      },
      additionalProperties: false,
    },
    execute: async (args) =>
      withRuntime(async (context) => {
        const snapshot = await fetchSelectedVacuumSnapshot(context);
        if (!snapshot.ok || !snapshot.data) return snapshot;
        return mcpSuccess("Vacuum map summary fetched.", normalizeMap(snapshot.data as RuntimeSnapshot, {
          includeGrid: args.include_grid === true,
          includeGeometry: args.include_geometry === true,
        }));
      }),
  });

  tools.set("vacuum_get_mission_state", {
    name: "vacuum_get_mission_state",
    description: "Return normalized active and recent vacuum mission state.",
    inputSchema: emptyInputSchema(),
    execute: async () =>
      withRuntime(async (context) => {
        const snapshot = await fetchSelectedVacuumSnapshot(context);
        if (!snapshot.ok || !snapshot.data) return snapshot;
        return mcpSuccess("Vacuum mission state fetched.", normalizeMissionState(snapshot.data as RuntimeSnapshot));
      }),
  });

  tools.set("vacuum_get_navigation_state", {
    name: "vacuum_get_navigation_state",
    description: "Return normalized vacuum navigation state and progress.",
    inputSchema: emptyInputSchema(),
    execute: async () =>
      withRuntime(async (context) => {
        const snapshot = await fetchSelectedVacuumSnapshot(context);
        if (!snapshot.ok || !snapshot.data) return snapshot;
        return mcpSuccess("Vacuum navigation state fetched.", normalizeNavigationState(snapshot.data as RuntimeSnapshot));
      }),
  });

  tools.set("vacuum_check_navigation_readiness", {
    name: "vacuum_check_navigation_readiness",
    description: "Read-only simulation preflight for a future navigation start; this does not dispatch movement.",
    inputSchema: {
      type: "object",
      properties: {
        target: {
          type: "object",
          properties: {
            x: { type: "number" },
            y: { type: "number" },
            theta: { type: "number" },
            frameId: { type: "string" },
          },
          required: ["x", "y"],
          additionalProperties: false,
        },
      },
      additionalProperties: false,
    },
    execute: async (args) => executeReadinessTool("navigation", args),
  });

  tools.set("vacuum_check_clean_area_readiness", {
    name: "vacuum_check_clean_area_readiness",
    description: "Read-only simulation preflight for a future Clean Area start; this does not dispatch coverage.",
    inputSchema: {
      type: "object",
      properties: {
        area: {
          type: "object",
          properties: {
            type: { type: "string", enum: ["rectangle"] },
            x: { type: "number" },
            y: { type: "number" },
            width: { type: "number" },
            height: { type: "number" },
            frameId: { type: "string" },
          },
          required: ["type", "x", "y", "width", "height"],
          additionalProperties: false,
        },
      },
      additionalProperties: false,
    },
    execute: async (args) => executeReadinessTool("clean_area", args),
  });

  tools.set("vacuum_get_supported_actions", {
    name: "vacuum_get_supported_actions",
    description: "Summarize selected-backend vacuum actions, including read tools, active mission actions, callable movement writes, and explicitly deferred actions.",
    inputSchema: emptyInputSchema(),
    execute: async () =>
      withRuntime(async (context) => {
        const snapshotResult = await fetchSelectedVacuumSnapshot(context);
        if (context.backend !== "turtlebot4_nav2") {
          if (!snapshotResult.ok || !snapshotResult.data) return snapshotResult;
          return mcpSuccess("Vacuum supported actions fetched.", supportedActionsSummary(context.backend, snapshotResult.data as RuntimeSnapshot));
        }
        if (!snapshotResult.ok || !snapshotResult.data) return snapshotResult;
        return mcpSuccess("Vacuum supported actions fetched.", supportedActionsSummary(context.backend, snapshotResult.data as RuntimeSnapshot));
      }),
  });

  tools.set("vacuum_start_navigation", {
    name: "vacuum_start_navigation",
    description: "Start a simulation backend navigation mission after product-level readiness gates pass.",
    inputSchema: {
      type: "object",
      properties: {
        target: {
          type: "object",
          properties: {
            x: { type: "number" },
            y: { type: "number" },
            theta: { type: "number" },
            frameId: { type: "string", default: "map" },
            label: { type: "string" },
          },
          required: ["x", "y", "theta"],
          additionalProperties: false,
        },
      },
      required: ["target"],
      additionalProperties: false,
    },
    execute: async (args) => executeStartNavigationTool(args),
  });

  tools.set("vacuum_start_clean_area", {
    name: "vacuum_start_clean_area",
    description: "Start a simulation backend rectangular Clean Area mission after product-level readiness gates pass.",
    inputSchema: {
      type: "object",
      properties: {
        area: {
          type: "object",
          properties: {
            type: { type: "string", enum: ["rectangle"] },
            x: { type: "number" },
            y: { type: "number" },
            width: { type: "number" },
            height: { type: "number" },
            frameId: { type: "string", default: "map" },
            label: { type: "string" },
          },
          required: ["type", "x", "y", "width", "height"],
          additionalProperties: false,
        },
      },
      required: ["area"],
      additionalProperties: false,
    },
    execute: async (args) => executeStartCleanAreaTool(args),
  });

  for (const [toolName, command] of Object.entries(COMMAND_TOOL_TO_RUNTIME_COMMAND) as Array<[CommandToolName, RuntimeCommandName]>) {
    tools.set(toolName, {
      name: toolName,
      description: commandDescription(toolName),
      inputSchema: commandInputSchema(command),
      execute: async (args) => executeCommandTool(command, args),
    });
  }

  return tools;
}

async function withRuntime(
  callback: (context: Extract<ReturnType<typeof createVacuumRuntimeContext>, { ok: true }>) => Promise<TensorFleetMcpResult>,
): Promise<TensorFleetMcpResult> {
  const runtimeConfig = await resolveMcpRuntimeConfig();
  const context = createVacuumRuntimeContext(runtimeConfig);
  if (!context.ok) {
    return context.result;
  }
  return callback(context);
}

type ReadinessAction = "navigation" | "clean_area";

async function executeReadinessTool(action: ReadinessAction, args: JsonRecord): Promise<TensorFleetMcpResult> {
  return withRuntime(async (context) => {
    if (context.backend !== "turtlebot4_nav2") {
      return mcpFailure("unsupported", "unsupported_backend", `Vacuum ${readinessActionLabel(action)} readiness is only supported by the simulation backend.`, {
        backend: context.backend,
        action,
        ready: false,
        status: "unsupported",
        blockingReasons: ["simulation_backend_required"],
        warnings: [],
        requiredInputs: [],
        capabilities: {},
        snapshotEvidence: {},
      });
    }

    const inputValidation = action === "navigation" ? validateNavigationTarget(args.target) : validateCleanArea(args.area);
    if (!inputValidation.ok) {
      return inputValidation;
    }

    const snapshotResult = await fetchSelectedVacuumSnapshot(context);
    if (!snapshotResult.ok || !snapshotResult.data) {
      return {
        ...snapshotResult,
        data: {
          ...asObjectData(snapshotResult.data),
          backend: context.backend,
          action,
          ready: false,
          status: "unavailable",
          blockingReasons: [snapshotResult.reason ?? "snapshot_unavailable"],
          warnings: [],
          requiredInputs: [],
          capabilities: {},
          snapshotEvidence: {},
        },
      };
    }

    const snapshot = snapshotResult.data as RuntimeSnapshot;
    return mcpSuccess(`Vacuum ${readinessActionLabel(action)} readiness checked.`, buildReadinessResult(context.backend, snapshot, action, inputValidation.data));
  });
}

async function executeCommandTool(command: RuntimeCommandName, args: JsonRecord): Promise<TensorFleetMcpResult> {
  return withRuntime(async (context) => {
    if (isSimulationMissionAction(command)) {
      return executeSimulationMissionActionTool(context, command);
    }

    if (context.backend === "turtlebot4_nav2" && (command === "set_fan_speed" || command === "set_water_usage")) {
      return mcpFailure("unsupported", "unsupported_command", `Vacuum command '${command}' is not supported by the simulation backend.`, {
        backend: context.backend,
        command,
      });
    }

    const snapshotResult = await fetchSelectedVacuumSnapshot(context);
    if (!snapshotResult.ok || !snapshotResult.data) return snapshotResult;

    const snapshot = snapshotResult.data as RuntimeSnapshot;
    const allowed = validateCommandRequest(snapshot, command, args);
    if (!allowed.ok) {
      return allowed;
    }

    const commandResult = await dispatchVacuumCommand(context, {
      command,
      params: commandParams(command, args),
    });
    if (!commandResult.ok) {
      return commandResult;
    }
    return mcpSuccess("Vacuum command dispatched.", {
      command,
      result: commandResult.data,
    });
  });
}

async function executeStartNavigationTool(args: JsonRecord): Promise<TensorFleetMcpResult> {
  return withRuntime(async (context) => {
    const requestedTarget = summarizeRequestedNavigationTarget(args.target);

    if (context.backend !== "turtlebot4_nav2") {
      return mcpFailure("unsupported", "unsupported_backend", "Vacuum start navigation is only supported by the simulation backend.", {
        backend: context.backend,
        action: "start_navigation",
        requestedTarget,
        blockingGate: "backend",
        blockingReasons: ["simulation_backend_required"],
        requiredInputs: [],
        capabilityEvidence: {},
        snapshotEvidence: {},
        activeMission: null,
      });
    }

    const targetValidation = validateNavigationStartTarget(args.target);
    if (!targetValidation.ok || !targetValidation.data) {
      return targetValidation;
    }
    const target = targetValidation.data as NavigationStartTarget;

    const snapshotResult = await fetchSelectedVacuumSnapshot(context);
    if (!snapshotResult.ok || !snapshotResult.data) {
      return withStartNavigationRefusalDetails(snapshotResult, context.backend, target, "snapshot");
    }

    const snapshot = snapshotResult.data as RuntimeSnapshot;
    const gate = buildStartNavigationGateResult(context.backend, snapshot, target);
    if (gate.ready !== true) {
      const reason = selectStartNavigationBlockingReason(gate);
      return mcpFailure(
        startNavigationFailureStatus(reason, gate.capabilities as JsonRecord | undefined),
        reason,
        "Vacuum start navigation was refused because readiness gates did not pass.",
        {
          ...gate,
          blockingGate: blockingGateForNavigationReason(reason),
        },
      );
    }

    const previousActiveMission = normalizeActiveMission(asRecord(snapshot.activeMission ?? snapshot.missions?.active));
    const commandResult = await dispatchVacuumCommand(context, {
      command: "start_navigation",
      target: {
        x: target.x,
        y: target.y,
        yaw: target.theta,
      },
    });
    if (!commandResult.ok) {
      return withStartNavigationRefusalDetails(commandResult, context.backend, target, "dispatch", previousActiveMission, gate);
    }

    const refreshedSnapshotResult = await fetchSelectedVacuumSnapshot(context);
    const refreshedActiveMission = refreshedSnapshotResult.ok && refreshedSnapshotResult.data
      ? normalizeActiveMission(asRecord((refreshedSnapshotResult.data as RuntimeSnapshot).activeMission ?? (refreshedSnapshotResult.data as RuntimeSnapshot).missions?.active))
      : null;

    return mcpSuccess("Vacuum start navigation dispatched.", {
      backend: context.backend,
      action: "start_navigation",
      requestedTarget: navigationTargetSummary(target),
      previousActiveMission,
      commandResult: summarizeCommandDispatchResult(commandResult.data),
      refreshedActiveMission,
      refresh: refreshedSnapshotResult.ok
        ? { ok: true }
        : { ok: false, status: refreshedSnapshotResult.status, reason: refreshedSnapshotResult.reason, message: refreshedSnapshotResult.message },
      warnings: Array.isArray(gate.warnings) ? gate.warnings : [],
    });
  });
}

async function executeStartCleanAreaTool(args: JsonRecord): Promise<TensorFleetMcpResult> {
  return withRuntime(async (context) => {
    const requestedArea = summarizeRequestedCleanArea(args.area);

    if (context.backend !== "turtlebot4_nav2") {
      return mcpFailure("unsupported", "unsupported_backend", "Vacuum start Clean Area is only supported by the simulation backend.", {
        backend: context.backend,
        action: "start_clean_area",
        requestedArea,
        blockingGate: "backend",
        blockingReasons: ["simulation_backend_required"],
        requiredInputs: [],
        capabilityEvidence: {},
        snapshotEvidence: {},
        activeMission: null,
      });
    }

    const areaValidation = validateCleanAreaStartArea(args.area);
    if (!areaValidation.ok || !areaValidation.data) {
      return areaValidation;
    }
    const area = areaValidation.data as CleanAreaStartArea;

    const snapshotResult = await fetchSelectedVacuumSnapshot(context);
    if (!snapshotResult.ok || !snapshotResult.data) {
      return withStartCleanAreaRefusalDetails(snapshotResult, context.backend, area, "snapshot");
    }

    const snapshot = snapshotResult.data as RuntimeSnapshot;
    const gate = buildStartCleanAreaGateResult(context.backend, snapshot, area);
    if (gate.ready !== true) {
      const reason = selectStartCleanAreaBlockingReason(gate);
      return mcpFailure(
        startCleanAreaFailureStatus(reason, gate.capabilities as JsonRecord | undefined),
        reason,
        "Vacuum start Clean Area was refused because readiness gates did not pass.",
        {
          ...gate,
          blockingGate: blockingGateForCleanAreaReason(reason),
        },
      );
    }

    const previousActiveMission = normalizeActiveMission(asRecord(snapshot.activeMission ?? snapshot.missions?.active));
    const commandResult = await dispatchVacuumCommand(context, {
      command: "start_coverage",
      area: cleanAreaToCoverageArea(area),
    });
    if (!commandResult.ok) {
      return withStartCleanAreaRefusalDetails(commandResult, context.backend, area, "dispatch", previousActiveMission, gate);
    }

    const refreshedSnapshotResult = await fetchSelectedVacuumSnapshot(context);
    const refreshedActiveMission = refreshedSnapshotResult.ok && refreshedSnapshotResult.data
      ? normalizeActiveMission(asRecord((refreshedSnapshotResult.data as RuntimeSnapshot).activeMission ?? (refreshedSnapshotResult.data as RuntimeSnapshot).missions?.active))
      : null;

    return mcpSuccess("Vacuum start Clean Area dispatched.", {
      backend: context.backend,
      action: "start_clean_area",
      dispatchedCommand: "start_coverage",
      requestedArea: cleanAreaSummary(area),
      previousActiveMission,
      commandResult: summarizeCommandDispatchResult(commandResult.data),
      refreshedActiveMission,
      refresh: refreshedSnapshotResult.ok
        ? { ok: true }
        : { ok: false, status: refreshedSnapshotResult.status, reason: refreshedSnapshotResult.reason, message: refreshedSnapshotResult.message },
      warnings: Array.isArray(gate.warnings) ? gate.warnings : [],
    });
  });
}

async function executeSimulationMissionActionTool(
  context: Extract<ReturnType<typeof createVacuumRuntimeContext>, { ok: true }>,
  action: SimulationMissionActionCommand,
): Promise<TensorFleetMcpResult> {
  if (context.backend !== "turtlebot4_nav2") {
    return mcpFailure("unsupported", "unsupported_backend", `Vacuum mission action '${action}' is only supported by the simulation backend.`, {
      actionRequested: action,
      backend: context.backend,
      blockingGate: "backend",
      normalizedReason: "simulation_backend_required",
    });
  }

  const snapshotResult = await fetchSelectedVacuumSnapshot(context);
  if (!snapshotResult.ok || !snapshotResult.data) {
    return withMissionActionRefusalDetails(snapshotResult, action, context.backend, "snapshot");
  }

  const snapshot = snapshotResult.data as RuntimeSnapshot;
  const activeMission = normalizeActiveMission(asRecord(snapshot.activeMission ?? snapshot.missions?.active));
  const refusalData = (blockingGate: string, normalizedReason: string, extra: JsonRecord = {}): JsonRecord => ({
    actionRequested: action,
    backend: context.backend,
    blockingGate,
    normalizedReason,
    activeMission,
    availableActions: Array.isArray(activeMission?.availableActions) ? activeMission.availableActions : undefined,
    ...extra,
  });

  const availabilityStatus = runtimeAvailabilityStatus(snapshot);
  if (availabilityStatus) {
    return withMissionActionRefusalDetails(availabilityStatus, action, context.backend, "runtime_availability", activeMission);
  }

  if (!activeMission?.id) {
    return mcpFailure(
      "invalid_state",
      "missing_active_mission",
      `Vacuum mission action '${action}' requires an active runtime-owned mission.`,
      refusalData("active_mission", "missing_active_mission"),
    );
  }

  const status = stringValue(activeMission.status);
  if (!isMissionStatusCompatible(action, status)) {
    return mcpFailure(
      "invalid_state",
      "mission_status_incompatible",
      `Vacuum mission action '${action}' is not compatible with the active mission status.`,
      refusalData("mission_status", "mission_status_incompatible", { missionStatus: status }),
    );
  }

  const availableActions = Array.isArray(activeMission.availableActions)
    ? activeMission.availableActions.filter((entry): entry is string => typeof entry === "string")
    : [];
  if (!availableActions.includes(action)) {
    return mcpFailure(
      "unavailable",
      "mission_action_unavailable",
      `Vacuum mission action '${action}' is not currently available on the active mission.`,
      refusalData("available_actions", "mission_action_not_listed"),
    );
  }

  const capabilityRefusal = validateMissionActionCapability(snapshot, action);
  if (capabilityRefusal) {
    return withMissionActionRefusalDetails(capabilityRefusal, action, context.backend, "capability", activeMission);
  }

  const commandResult = await dispatchVacuumCommand(context, { command: action });
  if (!commandResult.ok) {
    return withMissionActionRefusalDetails(commandResult, action, context.backend, "dispatch", activeMission);
  }

  const refreshedSnapshotResult = await fetchSelectedVacuumSnapshot(context);
  const refreshedActiveMission = refreshedSnapshotResult.ok && refreshedSnapshotResult.data
    ? normalizeActiveMission(asRecord((refreshedSnapshotResult.data as RuntimeSnapshot).activeMission ?? (refreshedSnapshotResult.data as RuntimeSnapshot).missions?.active))
    : null;

  return mcpSuccess("Vacuum mission action dispatched.", {
    actionRequested: action,
    backend: context.backend,
    status: "success",
    message: commandResult.message,
    previousActiveMission: activeMission,
    refreshedActiveMission,
    refresh: refreshedSnapshotResult.ok
      ? { ok: true }
      : { ok: false, status: refreshedSnapshotResult.status, reason: refreshedSnapshotResult.reason, message: refreshedSnapshotResult.message },
    result: commandResult.data,
  });
}

async function fetchSelectedVacuumSnapshot(
  context: Extract<ReturnType<typeof createVacuumRuntimeContext>, { ok: true }>,
): Promise<TensorFleetMcpResult> {
  const snapshot = await fetchVacuumSnapshot(context);
  if (!snapshot.ok || !snapshot.data) return snapshot;
  return {
    ...snapshot,
    data: extractSnapshotPayload(snapshot.data),
  };
}

function isSimulationMissionAction(command: RuntimeCommandName): command is SimulationMissionActionCommand {
  return (SIMULATION_MISSION_ACTION_COMMANDS as readonly string[]).includes(command);
}

function withMissionActionRefusalDetails(
  result: TensorFleetMcpResult,
  action: SimulationMissionActionCommand,
  backend: VacuumBackendId,
  blockingGate: string,
  activeMission: JsonRecord | null = null,
): TensorFleetMcpResult {
  if (result.ok) return result;
  const resultData = result.data && typeof result.data === "object" ? result.data as JsonRecord : {};
  return {
    ...result,
    data: {
      ...resultData,
      actionRequested: action,
      backend,
      blockingGate,
      normalizedReason: result.reason ?? result.status,
      activeMission,
      availableActions: Array.isArray(activeMission?.availableActions) ? activeMission.availableActions : undefined,
    },
  };
}

function withStartNavigationRefusalDetails(
  result: TensorFleetMcpResult,
  backend: VacuumBackendId,
  target: NavigationStartTarget,
  blockingGate: string,
  activeMission: JsonRecord | null = null,
  gate: JsonRecord | null = null,
): TensorFleetMcpResult {
  if (result.ok) return result;
  const resultData = result.data && typeof result.data === "object" ? result.data as JsonRecord : {};
  const gateReasons = Array.isArray(gate?.blockingReasons)
    ? gate.blockingReasons.filter((reason): reason is string => typeof reason === "string")
    : [];
  return {
    ...result,
    data: {
      ...resultData,
      backend,
      action: "start_navigation",
      requestedTarget: navigationTargetSummary(target),
      blockingGate,
      blockingReasons: gateReasons.length > 0 ? gateReasons : [result.reason ?? result.status],
      requiredInputs: Array.isArray(gate?.requiredInputs) ? gate.requiredInputs : [],
      capabilityEvidence: gate?.capabilityEvidence ?? gate?.capabilities ?? {},
      snapshotEvidence: gate?.snapshotEvidence ?? {},
      activeMission,
      commandResult: result.ok ? undefined : summarizeCommandDispatchResult(result.data),
    },
  };
}

function withStartCleanAreaRefusalDetails(
  result: TensorFleetMcpResult,
  backend: VacuumBackendId,
  area: CleanAreaStartArea,
  blockingGate: string,
  activeMission: JsonRecord | null = null,
  gate: JsonRecord | null = null,
): TensorFleetMcpResult {
  if (result.ok) return result;
  const resultData = result.data && typeof result.data === "object" ? result.data as JsonRecord : {};
  const gateReasons = Array.isArray(gate?.blockingReasons)
    ? gate.blockingReasons.filter((reason): reason is string => typeof reason === "string")
    : [];
  return {
    ...result,
    data: {
      ...resultData,
      backend,
      action: "start_clean_area",
      requestedArea: cleanAreaSummary(area),
      blockingGate,
      blockingReasons: gateReasons.length > 0 ? gateReasons : [result.reason ?? result.status],
      requiredInputs: Array.isArray(gate?.requiredInputs) ? gate.requiredInputs : [],
      capabilityEvidence: gate?.capabilityEvidence ?? gate?.capabilities ?? {},
      snapshotEvidence: gate?.snapshotEvidence ?? {},
      activeMission,
      commandResult: result.ok ? undefined : summarizeCommandDispatchResult(result.data),
    },
  };
}

function startNavigationFailureStatus(
  reason: string,
  capability: JsonRecord | undefined,
): "unsupported" | "unavailable" | "invalid_request" | "invalid_state" | "runtime_offline" | "source_unreachable" | "stale_source" {
  if (reason === "capability_unsupported" || capability?.supported === false) return "unsupported";
  if (reason === "target_frame_mismatch") return "invalid_request";
  if (reason === "active_mission_incompatible") return "invalid_state";
  if (reason === "runtime_offline") return "runtime_offline";
  if (reason === "source_unreachable") return "source_unreachable";
  if (reason === "stale_source") return "stale_source";
  return "unavailable";
}

function selectStartNavigationBlockingReason(gate: JsonRecord): string {
  const reasons = Array.isArray(gate.blockingReasons)
    ? gate.blockingReasons.filter((reason): reason is string => typeof reason === "string")
    : [];
  const priority = [
    "runtime_offline",
    "source_unreachable",
    "stale_source",
    "map_unavailable",
    "map_not_usable_for_navigation",
    "pose_unavailable",
    "target_frame_mismatch",
    "active_mission_incompatible",
    "capability_unsupported",
    "capability_unavailable",
  ];
  return priority.find((reason) => reasons.includes(reason))
    ?? reasons.find((reason) => reason.includes("capability"))
    ?? reasons.find((reason) => reason !== "availability_not_reported")
    ?? reasons[0]
    ?? "navigation_not_ready";
}

function blockingGateForNavigationReason(reason: string): string {
  if (reason.includes("runtime")) return "runtime_availability";
  if (reason.includes("source")) return "source_availability";
  if (reason.includes("map")) return "map";
  if (reason.includes("pose") || reason.includes("localization")) return "pose";
  if (reason.includes("mission")) return "active_mission";
  if (reason.includes("capability") || reason === "availability_not_reported" || reason.endsWith("_not_ready")) return "capability";
  if (reason.includes("frame") || reason.includes("target")) return "target";
  return "readiness";
}

function startCleanAreaFailureStatus(
  reason: string,
  capability: JsonRecord | undefined,
): "unsupported" | "unavailable" | "invalid_request" | "invalid_state" | "runtime_offline" | "source_unreachable" | "stale_source" {
  if (reason === "capability_unsupported" || capability?.supported === false) return "unsupported";
  if (reason === "area_frame_mismatch") return "invalid_request";
  if (reason === "active_mission_incompatible") return "invalid_state";
  if (reason === "runtime_offline") return "runtime_offline";
  if (reason === "source_unreachable") return "source_unreachable";
  if (reason === "stale_source") return "stale_source";
  return "unavailable";
}

function selectStartCleanAreaBlockingReason(gate: JsonRecord): string {
  const reasons = Array.isArray(gate.blockingReasons)
    ? gate.blockingReasons.filter((reason): reason is string => typeof reason === "string")
    : [];
  const priority = [
    "runtime_offline",
    "source_unreachable",
    "stale_source",
    "map_unavailable",
    "map_not_usable_for_coverage",
    "pose_unavailable",
    "area_frame_mismatch",
    "active_mission_incompatible",
    "capability_unsupported",
    "capability_unavailable",
  ];
  return priority.find((reason) => reasons.includes(reason))
    ?? reasons.find((reason) => reason.includes("capability"))
    ?? reasons.find((reason) => reason !== "availability_not_reported")
    ?? reasons[0]
    ?? "clean_area_not_ready";
}

function blockingGateForCleanAreaReason(reason: string): string {
  if (reason.includes("runtime")) return "runtime_availability";
  if (reason.includes("source")) return "source_availability";
  if (reason.includes("map")) return "map";
  if (reason.includes("pose") || reason.includes("localization")) return "pose";
  if (reason.includes("mission")) return "active_mission";
  if (reason.includes("capability") || reason === "availability_not_reported" || reason.endsWith("_not_ready")) return "capability";
  if (reason.includes("frame") || reason.includes("area")) return "area";
  return "readiness";
}

function isMissionStatusCompatible(action: SimulationMissionActionCommand, status: string | undefined): boolean {
  if (!status) return false;
  const compatibleStatuses: Record<SimulationMissionActionCommand, string[]> = {
    pause_mission: ["preparing", "running", "resuming", "returning", "charging"],
    resume_mission: ["paused", "needs_assistance"],
    cancel_mission: ["preparing", "running", "paused", "returning", "charging", "resuming", "needs_assistance"],
    retry_mission_step: ["failed", "needs_assistance"],
    skip_mission_step: ["running", "paused", "needs_assistance"],
  };
  return compatibleStatuses[action].includes(status);
}

function validateMissionActionCapability(snapshot: RuntimeSnapshot, action: SimulationMissionActionCommand): TensorFleetMcpResult | null {
  const normalizedCommands = normalizeCommandAvailability(snapshot);
  const commands = snapshot.capabilities?.commands ?? {};
  const commandSupport = normalizedCommands[action] ?? commands[action];
  const featureSupport = snapshot.capabilities?.[action] as CommandAvailability | undefined;
  const support = commandSupport ?? featureSupport;
  if (!support) {
    return null;
  }
  if (support.supported === false) {
    return mcpFailure("unsupported", support.reason ?? "unsupported_command", `Vacuum mission action '${action}' is not supported by the current runtime.`, {
      actionRequested: action,
      capability: support,
    });
  }
  if (support.available === false) {
    const reason = support.reason ?? support.availabilityReason ?? "capability_unavailable";
    return mcpFailure(
      normalizeAvailabilityReason(reason),
      reason,
      `Vacuum mission action '${action}' is currently unavailable in normalized capabilities.`,
      {
        actionRequested: action,
        capability: support,
      },
    );
  }
  return null;
}

export function validateCommandRequest(
  snapshot: RuntimeSnapshot,
  command: RuntimeCommandName,
  args: JsonRecord,
): TensorFleetMcpResult {
  const availabilityStatus = runtimeAvailabilityStatus(snapshot);
  if (availabilityStatus) {
    return availabilityStatus;
  }

  const commands = snapshot.capabilities?.commands ?? {};
  const normalizedCommands = normalizeCommandAvailability(snapshot);
  const support = normalizedCommands[command] ?? commands[command];
  if (!support) {
    return mcpFailure("unsupported", "unsupported_command", `Vacuum command '${command}' is not supported by the current runtime.`);
  }
  if (support.supported === false) {
    return mcpFailure("unsupported", support.reason ?? "unsupported_command", `Vacuum command '${command}' is not supported by the current runtime.`, {
      command,
      availability: support,
    });
  }
  if (support.available === false) {
    return mcpFailure(
      normalizeAvailabilityReason(support.reason),
      support.reason ?? "unavailable",
      `Vacuum command '${command}' is currently unavailable.`,
      { command, availability: support },
    );
  }

  if (command === "set_fan_speed") {
    return validateSettingValue(settingOptions(snapshot.cleaningSettings?.fanSpeed), "fan speed", command, args);
  }
  if (command === "set_water_usage") {
    return validateSettingValue(settingOptions(snapshot.cleaningSettings?.waterUsage), "water usage", command, args);
  }

  return mcpSuccess("Vacuum command is available.", { command });
}

export function validateSettingValue(
  options: string[] | undefined,
  label: string,
  command: RuntimeCommandName,
  args: JsonRecord,
): TensorFleetMcpResult<{ command: RuntimeCommandName; value: string }> {
  const value = typeof args.value === "string" ? args.value.trim() : "";
  if (!value) {
    return mcpFailure("invalid_request", "missing_value", `Missing ${label} value.`);
  }
  if (options && options.length > 0 && !options.includes(value)) {
    return mcpFailure("invalid_request", "invalid_value", `Selected ${label} value is not available.`, {
      command,
      value,
      options,
    });
  }
  return mcpSuccess("Vacuum setting value is valid.", { command, value });
}

function runtimeAvailabilityStatus(snapshot: RuntimeSnapshot): TensorFleetMcpResult | null {
  const runtimeStatus = stringValue(snapshot.runtime?.status);
  const sourceStatus = stringValue(snapshot.source?.status);
  const stale = snapshot.source?.stale === true;
  const online = snapshot.connectivity?.online;
  const reachable = snapshot.connectivity?.reachable;

  if (runtimeStatus === "offline" || online === false) {
    return mcpFailure("runtime_offline", "runtime_offline", "Vacuum runtime is offline.");
  }
  if (sourceStatus === "unreachable" || reachable === false) {
    return mcpFailure("source_unreachable", "source_unreachable", "Vacuum source is unreachable.");
  }
  if (stale || sourceStatus === "stale") {
    return mcpFailure("stale_source", "stale_source", "Vacuum source state is stale.");
  }
  if (runtimeStatus === "degraded") {
    return mcpFailure("unavailable", "degraded_runtime", "Vacuum runtime is degraded.");
  }
  return null;
}

function normalizeAvailabilityReason(reason: string | undefined): "unavailable" | "invalid_state" | "runtime_offline" | "source_unreachable" | "stale_source" {
  if (reason === "invalid_state" || reason === "command_invalid_state") return "invalid_state";
  if (reason === "runtime_offline") return "runtime_offline";
  if (reason === "source_unreachable") return "source_unreachable";
  if (reason === "stale_source") return "stale_source";
  return "unavailable";
}

function commandParams(command: RuntimeCommandName, args: JsonRecord): Record<string, unknown> | undefined {
  if (command !== "set_fan_speed" && command !== "set_water_usage") {
    return undefined;
  }
  return { value: String(args.value).trim() };
}

function buildReadinessResult(
  backend: VacuumBackendId,
  snapshot: RuntimeSnapshot,
  action: ReadinessAction,
  input: JsonRecord | null,
): JsonRecord {
  const map = normalizeMap(snapshot);
  const pose = normalizePose(snapshot);
  const mission = normalizeMissionState(snapshot);
  const navigation = normalizeNavigationState(snapshot);
  const capabilities = normalizeCapabilities(snapshot, backend);
  const features = capabilities.features && typeof capabilities.features === "object" ? capabilities.features as JsonRecord : {};
  const capabilityNames = action === "navigation" ? ["start_navigation", "go_to_location"] : ["start_coverage"];
  const actionCapabilities = capabilityAvailability(features, capabilityNames);
  const activeMission = mission.active && typeof mission.active === "object" ? mission.active as JsonRecord : null;
  const blockers = new Set<string>();
  const warnings: string[] = [];
  const requiredInputs: string[] = [];
  const availabilityStatus = runtimeAvailabilityStatus(snapshot);

  if (availabilityStatus) {
    blockers.add(availabilityStatus.reason ?? availabilityStatus.status);
  }
  if (map.available !== true) blockers.add("map_unavailable");
  if (action === "navigation" && (map.usableForNavigation as JsonRecord | undefined)?.available === false) {
    blockers.add(stringValue((map.usableForNavigation as JsonRecord).reason) ?? "map_not_usable_for_navigation");
  }
  if (action === "clean_area" && (map.usableForCoverage as JsonRecord | undefined)?.available === false) {
    blockers.add(stringValue((map.usableForCoverage as JsonRecord).reason) ?? "map_not_usable_for_coverage");
  }
  if (pose.available !== true) blockers.add("pose_unavailable");
  if (actionCapabilities.supported !== true) blockers.add("capability_unsupported");
  if (actionCapabilities.supported === true && actionCapabilities.available !== true) {
    blockers.add(stringValue(actionCapabilities.reason) ?? "capability_unavailable");
  }
  if (activeMission && isBlockingActiveMission(activeMission)) {
    blockers.add("active_mission_incompatible");
  }
  if (!input) {
    requiredInputs.push(action === "navigation" ? "target" : "area");
  } else {
    const frameWarning = frameCompatibilityWarning(input, map);
    if (frameWarning) warnings.push(frameWarning);
  }

  const blockingReasons = [...blockers];
  const ready = blockingReasons.length === 0 && requiredInputs.length === 0;
  const status = ready ? "ready" : requiredInputs.length > 0 && blockingReasons.length === 0 ? "needs_input" : "blocked";

  return {
    backend,
    action,
    ready,
    status,
    blockingReasons,
    warnings,
    requiredInputs,
    capabilities: actionCapabilities,
    snapshotEvidence: {
      runtime: {
        status: stringValue(snapshot.runtime?.status) ?? stringValue(snapshot.availability?.status) ?? "unknown",
        available: availabilityStatus == null,
      },
      source: {
        status: stringValue(snapshot.source?.status) ?? "unknown",
        stale: snapshot.source?.stale === true,
        lastSeenAt: numberOrString(snapshot.source?.lastSeenAt) ?? numberOrString(snapshot.updatedAt),
      },
      map: {
        available: map.available === true,
        readiness: map.readiness,
        usableForNavigation: map.usableForNavigation,
        usableForCoverage: map.usableForCoverage,
        dimensions: map.dimensions,
      },
      pose: {
        available: pose.available === true,
        readiness: pose.readiness,
      },
      localization: {
        acceptable: pose.available === true,
        evidence: pose.available === true ? "pose_available" : "pose_unavailable",
      },
      mission: {
        active: Boolean(activeMission?.id),
        activeMission,
      },
      navigation: {
        available: navigation.available === true,
        state: navigation.state ?? "unknown",
        active: navigation.active === true,
      },
    },
  };
}

function buildStartNavigationGateResult(
  backend: VacuumBackendId,
  snapshot: RuntimeSnapshot,
  target: NavigationStartTarget,
): JsonRecord {
  const readiness = buildReadinessResult(backend, snapshot, "navigation", target);
  const capabilities = normalizeCapabilities(snapshot, backend);
  const features = capabilities.features && typeof capabilities.features === "object" ? capabilities.features as JsonRecord : {};
  const startNavigationCapability = capabilityAvailability(features, ["start_navigation"]);
  const blockingReasons = new Set(
    Array.isArray(readiness.blockingReasons)
      ? readiness.blockingReasons.filter((reason): reason is string => typeof reason === "string")
      : [],
  );
  const map = normalizeMap(snapshot);
  const mapFrame = stringValue((map.dimensions as JsonRecord | null | undefined)?.frameId);

  if (mapFrame && target.frameId !== mapFrame) {
    blockingReasons.add("target_frame_mismatch");
  }
  if (startNavigationCapability.supported !== true) {
    blockingReasons.add("capability_unsupported");
  } else if (startNavigationCapability.available !== true) {
    blockingReasons.add(stringValue(startNavigationCapability.reason) ?? "capability_unavailable");
  }

  const reasons = [...blockingReasons];
  return {
    ...readiness,
    action: "start_navigation",
    ready: reasons.length === 0,
    status: reasons.length === 0 ? "ready" : "blocked",
    blockingReasons: reasons,
    requiredInputs: [],
    requestedTarget: navigationTargetSummary(target),
    capabilities: startNavigationCapability,
    capabilityEvidence: {
      startNavigation: startNavigationCapability,
    },
    activeMission: (readiness.snapshotEvidence as JsonRecord | undefined)?.mission
      && typeof (readiness.snapshotEvidence as JsonRecord).mission === "object"
      ? ((readiness.snapshotEvidence as JsonRecord).mission as JsonRecord).activeMission
      : null,
  };
}

function buildStartCleanAreaGateResult(
  backend: VacuumBackendId,
  snapshot: RuntimeSnapshot,
  area: CleanAreaStartArea,
): JsonRecord {
  const readiness = buildReadinessResult(backend, snapshot, "clean_area", area);
  const capabilities = normalizeCapabilities(snapshot, backend);
  const features = capabilities.features && typeof capabilities.features === "object" ? capabilities.features as JsonRecord : {};
  const startCoverageCapability = capabilityAvailability(features, ["start_coverage"]);
  const blockingReasons = new Set(
    Array.isArray(readiness.blockingReasons)
      ? readiness.blockingReasons.filter((reason): reason is string => typeof reason === "string")
      : [],
  );
  const map = normalizeMap(snapshot);
  const mapFrame = stringValue((map.dimensions as JsonRecord | null | undefined)?.frameId);

  if (mapFrame && area.frameId !== mapFrame) {
    blockingReasons.add("area_frame_mismatch");
  }
  if (startCoverageCapability.supported !== true) {
    blockingReasons.add("capability_unsupported");
  } else if (startCoverageCapability.available !== true) {
    blockingReasons.add(stringValue(startCoverageCapability.reason) ?? "capability_unavailable");
  }

  const reasons = [...blockingReasons];
  return {
    ...readiness,
    action: "start_clean_area",
    ready: reasons.length === 0,
    status: reasons.length === 0 ? "ready" : "blocked",
    blockingReasons: reasons,
    requiredInputs: [],
    requestedArea: cleanAreaSummary(area),
    capabilities: startCoverageCapability,
    capabilityEvidence: {
      startCoverage: startCoverageCapability,
    },
    activeMission: (readiness.snapshotEvidence as JsonRecord | undefined)?.mission
      && typeof (readiness.snapshotEvidence as JsonRecord).mission === "object"
      ? ((readiness.snapshotEvidence as JsonRecord).mission as JsonRecord).activeMission
      : null,
  };
}

function validateNavigationTarget(value: unknown): TensorFleetMcpResult<JsonRecord | null> {
  if (value == null) return mcpSuccess("Navigation target omitted.", null);
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    return invalidReadinessInput("navigation", "invalid_target", "Navigation target must be an object.");
  }
  const target = value as JsonRecord;
  if (numberValue(target.x) == null || numberValue(target.y) == null) {
    return invalidReadinessInput("navigation", "invalid_target", "Navigation target requires numeric x and y coordinates.");
  }
  if ("theta" in target && numberValue(target.theta) == null) {
    return invalidReadinessInput("navigation", "invalid_target", "Navigation target theta must be numeric when provided.");
  }
  if ("frameId" in target && !stringValue(target.frameId)) {
    return invalidReadinessInput("navigation", "invalid_target", "Navigation target frameId must be a non-empty string when provided.");
  }
  return mcpSuccess("Navigation target is valid.", {
    x: numberValue(target.x),
    y: numberValue(target.y),
    theta: numberValue(target.theta),
    frameId: stringValue(target.frameId),
  });
}

function validateNavigationStartTarget(value: unknown): TensorFleetMcpResult<NavigationStartTarget | JsonRecord | null> {
  if (value == null) {
    return mcpFailure("invalid_request", "missing_target", "Vacuum start navigation requires a target.", {
      backend: "turtlebot4_nav2",
      action: "start_navigation",
      requestedTarget: null,
      ready: false,
      status: "needs_input",
      blockingGate: "target",
      blockingReasons: ["missing_target"],
      requiredInputs: ["target"],
      capabilityEvidence: {},
      snapshotEvidence: {},
      activeMission: null,
    });
  }
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    return invalidNavigationStartTarget("invalid_target", "Navigation target must be an object.", value);
  }
  const target = value as JsonRecord;
  const x = numberValue(target.x);
  const y = numberValue(target.y);
  const theta = numberValue(target.theta);
  if (x == null || y == null) {
    return invalidNavigationStartTarget("invalid_target", "Navigation target requires numeric x and y coordinates.", value);
  }
  if (theta == null) {
    return invalidNavigationStartTarget(
      "missing_theta",
      "Navigation target requires numeric theta because the normalized start_navigation command requires yaw.",
      value,
    );
  }
  const frameId = stringValue(target.frameId) ?? "map";
  if (frameId !== "map") {
    return invalidNavigationStartTarget("unsupported_frame", "Navigation target frameId must be 'map'.", value);
  }
  if ("label" in target && target.label != null && !stringValue(target.label)) {
    return invalidNavigationStartTarget("invalid_target", "Navigation target label must be a non-empty string when provided.", value);
  }
  return mcpSuccess("Navigation start target is valid.", {
    x,
    y,
    theta,
    frameId,
    label: stringValue(target.label),
  });
}

function validateCleanAreaStartArea(value: unknown): TensorFleetMcpResult<CleanAreaStartArea | JsonRecord | null> {
  if (value == null) {
    return mcpFailure("invalid_request", "missing_area", "Vacuum start Clean Area requires an area.", {
      backend: "turtlebot4_nav2",
      action: "start_clean_area",
      requestedArea: null,
      ready: false,
      status: "needs_input",
      blockingGate: "area",
      blockingReasons: ["missing_area"],
      requiredInputs: ["area"],
      capabilityEvidence: {},
      snapshotEvidence: {},
      activeMission: null,
    });
  }
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    return invalidCleanAreaStartArea("invalid_area", "Clean Area payload must be an object.", value);
  }
  const area = value as JsonRecord;
  if (area.type !== "rectangle") {
    return invalidCleanAreaStartArea("unsupported_area_type", "Clean Area currently accepts only rectangle areas.", value);
  }
  const x = numberValue(area.x);
  const y = numberValue(area.y);
  const width = numberValue(area.width);
  const height = numberValue(area.height);
  if (x == null || y == null || width == null || height == null) {
    return invalidCleanAreaStartArea("invalid_area", "Clean Area requires numeric x, y, width, and height.", value);
  }
  if (width <= 0 || height <= 0) {
    return invalidCleanAreaStartArea("invalid_area_dimensions", "Clean Area width and height must be positive.", value);
  }
  const frameId = stringValue(area.frameId) ?? "map";
  if (frameId !== "map") {
    return invalidCleanAreaStartArea("unsupported_frame", "Clean Area frameId must be 'map'.", value);
  }
  if ("label" in area && area.label != null && !stringValue(area.label)) {
    return invalidCleanAreaStartArea("invalid_area", "Clean Area label must be a non-empty string when provided.", value);
  }
  return mcpSuccess("Clean Area start area is valid.", {
    type: "rectangle",
    x,
    y,
    width,
    height,
    frameId,
    label: stringValue(area.label),
  });
}

function invalidNavigationStartTarget(reason: string, message: string, value: unknown): TensorFleetMcpResult<NavigationStartTarget | JsonRecord | null> {
  return mcpFailure("invalid_request", reason, message, {
    backend: "turtlebot4_nav2",
    action: "start_navigation",
    requestedTarget: summarizeRequestedNavigationTarget(value),
    ready: false,
    status: "invalid_request",
    blockingGate: "target",
    blockingReasons: [reason],
    requiredInputs: reason === "missing_theta" ? ["target.theta"] : [],
    capabilityEvidence: {},
    snapshotEvidence: {},
    activeMission: null,
  });
}

function invalidCleanAreaStartArea(reason: string, message: string, value: unknown): TensorFleetMcpResult<CleanAreaStartArea | JsonRecord | null> {
  return mcpFailure("invalid_request", reason, message, {
    backend: "turtlebot4_nav2",
    action: "start_clean_area",
    requestedArea: summarizeRequestedCleanArea(value),
    ready: false,
    status: "invalid_request",
    blockingGate: "area",
    blockingReasons: [reason],
    requiredInputs: [],
    capabilityEvidence: {},
    snapshotEvidence: {},
    activeMission: null,
  });
}

function validateCleanArea(value: unknown): TensorFleetMcpResult<JsonRecord | null> {
  if (value == null) return mcpSuccess("Clean Area omitted.", null);
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    return invalidReadinessInput("clean_area", "invalid_area", "Clean Area payload must be an object.");
  }
  const area = value as JsonRecord;
  if (area.type !== "rectangle") {
    return invalidReadinessInput("clean_area", "invalid_area", "Clean Area currently accepts only rectangle areas.");
  }
  const x = numberValue(area.x);
  const y = numberValue(area.y);
  const width = numberValue(area.width);
  const height = numberValue(area.height);
  if (x == null || y == null || width == null || height == null) {
    return invalidReadinessInput("clean_area", "invalid_area", "Clean Area requires numeric x, y, width, and height.");
  }
  if (width <= 0 || height <= 0) {
    return invalidReadinessInput("clean_area", "invalid_area_dimensions", "Clean Area width and height must be positive.");
  }
  if ("frameId" in area && !stringValue(area.frameId)) {
    return invalidReadinessInput("clean_area", "invalid_area", "Clean Area frameId must be a non-empty string when provided.");
  }
  return mcpSuccess("Clean Area payload is valid.", {
    type: "rectangle",
    x,
    y,
    width,
    height,
    frameId: stringValue(area.frameId),
  });
}

function invalidReadinessInput(action: ReadinessAction, reason: string, message: string): TensorFleetMcpResult<JsonRecord | null> {
  return mcpFailure<JsonRecord>("invalid_request", reason, message, {
    backend: "turtlebot4_nav2",
    action,
    ready: false,
    status: "invalid_request",
    blockingReasons: [reason],
    warnings: [],
    requiredInputs: [],
    capabilities: {},
    snapshotEvidence: {},
  });
}

function isBlockingActiveMission(activeMission: JsonRecord): boolean {
  const status = stringValue(activeMission.status);
  if (!status || ["idle", "completed", "failed", "canceled", "unsupported"].includes(status)) return false;
  return Boolean(activeMission.id);
}

function frameCompatibilityWarning(input: JsonRecord, map: JsonRecord): string | null {
  const requestedFrame = stringValue(input.frameId);
  if (!requestedFrame) return null;
  const dimensions = map.dimensions && typeof map.dimensions === "object" ? map.dimensions as JsonRecord : null;
  const mapFrame = stringValue(dimensions?.frameId);
  if (!mapFrame) return "Input frame could not be compared because the map frame is not available in normalized metadata.";
  if (requestedFrame !== mapFrame) return "Input frame differs from the normalized map frame.";
  return null;
}

function supportedActionsSummary(backend: VacuumBackendId, snapshot: RuntimeSnapshot): JsonRecord {
  const capabilities = normalizeCapabilities(snapshot, backend);
  const features = capabilities.features && typeof capabilities.features === "object" ? capabilities.features as JsonRecord : {};
  const mission = normalizeMissionState(snapshot);
  const activeMission = mission.active && typeof mission.active === "object" ? mission.active as JsonRecord : null;
  const activeMissionActions = Array.isArray(activeMission?.availableActions) ? activeMission.availableActions : [];
  const startNavigation = capabilityAvailability(features, ["start_navigation"]);
  const startCoverage = capabilityAvailability(features, ["start_coverage"]);
  const callableMovementWriteTools = backend === "turtlebot4_nav2"
    ? [
        ...(startNavigation.supported === true ? ["vacuum_start_navigation"] : []),
        ...(startCoverage.supported === true ? ["vacuum_start_clean_area"] : []),
      ]
    : [];
  return {
    backend,
    readTools: [
      "vacuum_get_health",
      "vacuum_get_snapshot",
      "vacuum_get_capabilities",
      "vacuum_get_map_targets",
      "vacuum_get_pose",
      "vacuum_get_map_summary",
      "vacuum_get_mission_state",
      "vacuum_get_navigation_state",
      "vacuum_check_navigation_readiness",
      "vacuum_check_clean_area_readiness",
      "vacuum_get_supported_actions",
    ],
    activeMissionActions,
    callableMovementWriteTools,
    futureMovementActions: backend === "turtlebot4_nav2"
      ? {
          note: "Navigation start and rectangular Clean Area start are callable movement write tools when their simulation capabilities are supported; other movement starts remain deferred.",
          navigationStart: startNavigation,
          goToLocation: capabilityAvailability(features, ["go_to_location"]),
          cleanArea: startCoverage,
        }
      : {
          note: "Simulation movement preflight does not reinterpret Valetudo map targets.",
          navigation: { supported: false, available: false, reason: "unsupported_backend" },
          cleanArea: { supported: false, available: false, reason: "unsupported_backend" },
        },
    deferredActions: [
      "vacuum_go_to_location",
      "vacuum_start_room_cleaning",
      "vacuum_start_zone_cleaning",
      "arbitrary_waypoint_tools",
      "map_editing_tools",
    ],
    capabilities,
  };
}

function readinessActionLabel(action: ReadinessAction): string {
  return action === "navigation" ? "navigation" : "Clean Area";
}

function asObjectData(value: unknown): JsonRecord {
  return value && typeof value === "object" && !Array.isArray(value) ? value as JsonRecord : {};
}

export function normalizeSnapshot(
  snapshot: RuntimeSnapshot,
  options: { backend: VacuumBackendId; includeDiagnostics: boolean; includeRawDiagnostics: boolean; includeMapPreview: boolean },
): JsonRecord {
  const capabilities = normalizeCapabilities(snapshot, options.backend);
  const data: JsonRecord = {
    identity: {
      id: stringValue(snapshot.identity?.id) ?? stringValue(snapshot.robot?.id) ?? options.backend,
      label: stringValue(snapshot.identity?.label) ?? stringValue(snapshot.robot?.name) ?? backendLabel(options.backend),
      source: stringValue(snapshot.identity?.source) ?? options.backend,
      model: stringValue(snapshot.identity?.model),
    },
    availability: {
      status: snapshot.availability?.status,
      connected: snapshot.availability?.connected === true || snapshot.connectivity?.online === true,
      reachable: snapshot.connectivity?.reachable === true || snapshot.source?.status === "reachable",
      detail: snapshot.availability?.detail,
    },
    health: {
      runtime: snapshot.runtime ?? snapshot.health,
      source: snapshot.source,
      updatedAt: snapshot.updatedAt,
    },
    activity: snapshot.activity ?? snapshot.state,
    battery: snapshot.battery,
    dock: snapshot.dock,
    cleaningSettings: normalizeCleaningSettings(snapshot),
    map: normalizeMap(snapshot, {
      includeGrid: false,
      includeGeometry: false,
      includePreview: options.includeMapPreview,
    }),
    pose: normalizePose(snapshot),
    navigation: normalizeNavigationState(snapshot),
    mission: normalizeMissionState(snapshot),
    readiness: normalizeReadiness(snapshot, options.backend, capabilities),
    fault: snapshot.fault,
    capabilities,
    updatedAt: snapshot.updatedAt,
  };

  if (options.includeDiagnostics) {
    data.diagnostics = safeDiagnostics(snapshot, options.includeRawDiagnostics);
  }

  return data;
}

export function normalizeCapabilities(snapshot: RuntimeSnapshot, backend?: VacuumBackendId): JsonRecord {
  const normalizedCommands = normalizeCommandAvailability(snapshot);
  const normalizedFeatures = normalizeFeatureCapabilities(snapshot);
  const featureEntries = backend === "turtlebot4_nav2"
    ? Object.fromEntries(
        SIMULATION_CAPABILITY_NAMES.map((name) => [
          name,
          normalizedFeatures[name] ?? unavailableCapability("Capability is not present in the normalized snapshot."),
        ]),
      )
    : normalizedFeatures;

  return {
    commands: Object.fromEntries(
      Object.entries(normalizedCommands)
      .map(([command, availability]) => [
        command,
        {
          supported: availability.supported !== false,
          available: availability.available === true,
          reason: capabilityReason(availability),
          status: availability.status,
        },
      ]),
    ),
    features: featureEntries,
    settings: normalizeCleaningSettings(snapshot),
    readiness: normalizeReadiness(snapshot, backend ?? inferBackend(snapshot), { features: featureEntries }),
  };
}

function normalizeCommandAvailability(snapshot: RuntimeSnapshot): Record<string, CommandAvailability> {
  const rawCommands = snapshot.capabilities?.commands;
  if (rawCommands) {
    return Object.fromEntries(
      Object.entries(rawCommands)
        .filter(([command]) => Object.values(COMMAND_TOOL_TO_RUNTIME_COMMAND).includes(command as RuntimeCommandName))
        .map(([command, availability]) => [command, { supported: true, ...availability }]),
    );
  }

  const capabilities = snapshot.capabilities ?? {};
  const commands: Record<string, CommandAvailability> = {};
  for (const command of Object.values(COMMAND_TOOL_TO_RUNTIME_COMMAND)) {
    const feature = capabilities[command] as CommandAvailability | undefined;
    if (feature && typeof feature === "object") {
      commands[command] = featureToCommandAvailability(feature);
    }
  }

  const fanSpeed = capabilities.fan_speed as CommandAvailability | undefined;
  if (fanSpeed && typeof fanSpeed === "object") {
    commands.set_fan_speed = featureToCommandAvailability(fanSpeed);
  }
  const waterUsage = capabilities.water_usage as CommandAvailability | undefined;
  if (waterUsage && typeof waterUsage === "object") {
    commands.set_water_usage = featureToCommandAvailability(waterUsage);
  }

  return commands;
}

function normalizeFeatureCapabilities(snapshot: RuntimeSnapshot): JsonRecord {
  const capabilities = snapshot.capabilities ?? {};
  return Object.fromEntries(
    Object.entries(capabilities)
      .filter(([name, value]) => name !== "commands" && name !== "diagnostics" && value && typeof value === "object")
      .map(([name, value]) => {
        const capability = value as CommandAvailability & { commands?: string[]; attributes?: string[]; reasons?: unknown[] };
        const supported = capability.supported === true;
        const available = supported && capability.available === true;
        return [
          name,
          {
            supported,
            available,
            status: capability.status ?? (supported ? available ? "supported" : "unavailable" : "unsupported"),
            reason: capabilityReason(capability),
            commands: sanitizeCommandNames(capability.commands),
            reasons: normalizeReasons(capability.reasons),
          },
        ];
      }),
  );
}

function featureToCommandAvailability(feature: CommandAvailability): CommandAvailability {
  return {
    supported: feature.supported === true,
    available: feature.supported === true && feature.available === true,
    status: feature.status,
    reason: feature.supported === true
      ? capabilityReason(feature) ?? "availability_not_reported"
      : "unsupported_command",
  };
}

function normalizeCleaningSettings(snapshot: RuntimeSnapshot): JsonRecord {
  return {
    fanSpeed: normalizeSetting(snapshot.cleaningSettings?.fanSpeed),
    waterUsage: normalizeSetting(snapshot.cleaningSettings?.waterUsage),
  };
}

function normalizeSetting(setting: { current?: string; options?: Array<string | { value?: string; label?: string }> } | undefined): JsonRecord | null {
  if (!setting) return null;
  return {
    current: setting.current,
    options: (settingOptions(setting) ?? []).map((value) => ({ value, label: titleCase(value) })),
  };
}

function settingOptions(setting: { options?: Array<string | { value?: string; label?: string }> } | undefined): string[] | undefined {
  const options = setting?.options
    ?.map((entry) => typeof entry === "string" ? entry : entry.value)
    .filter((value): value is string => typeof value === "string" && value.trim().length > 0);
  return options && options.length > 0 ? options : undefined;
}

export function normalizeMap(
  snapshot: RuntimeSnapshot,
  options: { includeGrid?: boolean; includeGeometry?: boolean; includePreview?: boolean } = {},
): JsonRecord {
  const metadata = snapshot.map?.metadata;
  const layeredMetadata = snapshot.map?.layeredMetadata;
  const annotations = snapshot.map?.annotations ?? [];
  const available = snapshot.map?.available === true || snapshot.map?.readiness === "ready" || metadata?.hasMap === true;
  const annotationCounts = countAnnotations(annotations);
  const targets = normalizeMapTargets(snapshot, options.includeGeometry === true);
  const featureCapabilities = normalizeFeatureCapabilities(snapshot);
  const map: JsonRecord = {
    available,
    status: available ? "available" : "unavailable",
    reason: available ? undefined : snapshot.map?.detail ?? "Map is not available from the selected backend.",
    readiness: snapshot.map?.readiness ?? "unavailable",
    receiving: snapshot.map?.receiving === true,
    identity: {
      id: stringValue(layeredMetadata?.id) ?? stringValue(snapshot.mapping?.activeMapName),
      name: stringValue(snapshot.mapping?.activeMapName) ?? stringValue(layeredMetadata?.id),
    },
    dimensions: normalizeMapDimensions(metadata, layeredMetadata),
    cellSummary: normalizeCellSummary(metadata),
    annotations: {
      total: annotations.length,
      rooms: annotationCounts.room,
      zones: annotationCounts.zone,
    },
    targets: {
      segmentCount: arrayLength((targets.segments as unknown[] | undefined)),
      roomCount: arrayLength((targets.rooms as unknown[] | undefined)),
      zoneCount: arrayLength((targets.zones as unknown[] | undefined)),
      inventory: targets,
    },
    usableForNavigation: mapUsableFor("start_navigation", featureCapabilities, available),
    usableForCoverage: mapUsableFor("start_coverage", featureCapabilities, available),
    updatedAt: numberOrString(metadata?.lastUpdateAt) ?? numberOrString(layeredMetadata?.updatedAt) ?? numberOrString(snapshot.updatedAt),
    detail: snapshot.map?.detail,
  };
  if (options.includePreview) {
    map.preview = snapshot.map?.preview ?? snapshot.map?.layeredPreview;
  }
  if (options.includeGrid) {
    map.grid = snapshot.map?.grid ?? null;
  }
  return map;
}

function normalizeMapTargets(snapshot: RuntimeSnapshot, includeGeometry: boolean): JsonRecord {
  return {
    note: "Map targets are read-only in MCP; target cleaning commands remain unsupported.",
    segments: normalizeTargetList(snapshot.map?.targets?.segments, "segment", includeGeometry),
    rooms: [
      ...normalizeTargetList(snapshot.map?.targets?.rooms, "room", includeGeometry),
      ...normalizeAnnotationTargets(snapshot.map?.annotations, "room", includeGeometry),
    ],
    zones: normalizeTargetList(snapshot.map?.targets?.zones, "zone", includeGeometry),
  };
}

function normalizeTargetList(targets: MapTarget[] | undefined, kind: "segment" | "room" | "zone", includeGeometry: boolean): JsonRecord[] {
  return (targets ?? [])
    .filter((target) => typeof target.id === "string" && typeof target.label === "string")
    .map((target) => {
      const next: JsonRecord = {
        id: target.id,
        label: target.label,
        kind,
        available: target.available === true,
        detail: target.detail,
      };
      if (includeGeometry) {
        next.geometry = target.geometry;
      } else if (target.geometry && typeof target.geometry === "object") {
        const geometry = target.geometry as JsonRecord;
        next.geometrySummary = {
          type: geometry.type,
          pointCount: Array.isArray(geometry.points) ? geometry.points.length : undefined,
          hasBounds: Boolean(geometry.bounds),
        };
      }
      return next;
    });
}

function normalizeAnnotationTargets(annotations: MapAnnotation[] | undefined, kind: "room" | "zone", includeGeometry: boolean): JsonRecord[] {
  return (annotations ?? [])
    .filter((annotation) => annotation.kind === kind && typeof annotation.id === "string" && typeof annotation.name === "string")
    .map((annotation) => {
      const target: JsonRecord = {
        id: annotation.id,
        label: annotation.name,
        kind,
        available: true,
        source: "user",
      };
      if (includeGeometry) {
        target.geometry = annotation.area;
      }
      return target;
    });
}

export function normalizePose(snapshot: RuntimeSnapshot): JsonRecord {
  const available = snapshot.pose?.available === true && snapshot.pose?.coordinates != null;
  if (!available) {
    return {
      available: false,
      status: "unavailable",
      reason: snapshot.pose?.detail ?? "Pose is not available from the selected backend.",
      readiness: snapshot.pose?.readiness ?? "unavailable",
      updatedAt: numberOrString(snapshot.source?.lastSeenAt) ?? numberOrString(snapshot.updatedAt),
    };
  }
  return {
    available: true,
    status: "available",
    readiness: snapshot.pose?.readiness ?? "ready",
    coordinates: snapshot.pose?.coordinates ?? null,
    updatedAt: numberOrString(snapshot.source?.lastSeenAt) ?? numberOrString(snapshot.updatedAt),
    detail: snapshot.pose?.detail,
  };
}

export function normalizeNavigationState(snapshot: RuntimeSnapshot): JsonRecord {
  const navigation = snapshot.navigation;
  const mission = normalizeActiveMission(asRecord(snapshot.activeMission ?? snapshot.missions?.active));
  const missionNavigation = mission && mission.type === "navigation" ? mission : null;
  if (!navigation) {
    return {
      available: false,
      status: "unavailable",
      reason: mission ? "navigation_state_unavailable" : "No navigation state is available from the selected backend.",
      relatedMission: missionNavigation ?? mission,
    };
  }
  const progress = navigation.progress && typeof navigation.progress === "object" ? navigation.progress : undefined;
  const availableActions = Array.isArray(missionNavigation?.availableActions)
    ? missionNavigation.availableActions.filter((action): action is string => typeof action === "string")
    : [];
  return {
    available: true,
    state: navigation.state ?? "unknown",
    active: navigation.active === true,
    currentDestination: navigation.currentTarget ?? null,
    terminalState: navigation.terminalState ?? null,
    progress: {
      distanceRemaining: progress?.distanceRemaining ?? null,
      initialDistance: progress?.initialDistance ?? null,
      recoveries: progress?.recoveries ?? null,
      estimatedTimeRemaining: progress?.estimatedTimeRemaining ?? null,
    },
    path: summarizePath(navigation.planPath),
    availableActions,
    controls: {
      cancel: availableActions.includes("cancel_mission"),
      pause: availableActions.includes("pause_mission"),
      resume: availableActions.includes("resume_mission"),
    },
    relatedMission: missionNavigation,
    detail: navigation.detail,
  };
}

export function normalizeMissionState(snapshot: RuntimeSnapshot): JsonRecord {
  const active = normalizeActiveMission(asRecord(snapshot.activeMission ?? snapshot.missions?.active));
  const recentRaw = Array.isArray(snapshot.missions?.recent) ? snapshot.missions?.recent : [];
  const recent = recentRaw.map(normalizeMissionSummary).filter((mission): mission is JsonRecord => Boolean(mission));
  return {
    summary: snapshot.mission ? {
      state: snapshot.mission.state,
      detail: snapshot.mission.detail,
      lastTerminalNavigation: snapshot.mission.lastTerminalNavigation,
    } : null,
    active,
    recentCount: recent.length,
    recent,
    mapping: normalizeMappingState(snapshot.mapping),
    availableActions: active?.availableActions ?? (Array.isArray(snapshot.activity?.availableActions) ? snapshot.activity?.availableActions : []),
  };
}

function normalizeReadiness(snapshot: RuntimeSnapshot, backend: VacuumBackendId, capabilities: JsonRecord): JsonRecord {
  const features = capabilities.features && typeof capabilities.features === "object"
    ? capabilities.features as JsonRecord
    : normalizeFeatureCapabilities(snapshot);
  const sourceStatus = stringValue(snapshot.source?.status);
  const availabilityStatus = stringValue(snapshot.availability?.status) ?? stringValue(snapshot.runtime?.status) ?? stringValue(snapshot.health?.runtimeStatus);
  const connected = snapshot.availability?.connected === true || snapshot.connectivity?.online === true || availabilityStatus === "online";
  const reachable = snapshot.connectivity?.reachable === true || sourceStatus === "reachable";
  const stale = snapshot.source?.stale === true || sourceStatus === "stale";
  const map = normalizeMap(snapshot);
  const pose = normalizePose(snapshot);
  const mission = normalizeMissionState(snapshot);
  const navigation = normalizeNavigationState(snapshot);
  const movementCapabilities = capabilityAvailability(features, ["start_navigation", "go_to_location"]);
  const coverageCapabilities = capabilityAvailability(features, ["start_coverage"]);
  const blockers = normalizedBlockingReasons(snapshot, {
    connected,
    reachable,
    stale,
    mapAvailable: map.available === true,
    poseAvailable: pose.available === true,
    movementCapabilities,
  });

  return {
    selectedBackend: backend,
    movementReady: blockers.length === 0,
    runtime: {
      available: connected,
      status: availabilityStatus ?? (connected ? "online" : "unknown"),
      detail: snapshot.availability?.detail ?? snapshot.health?.detail,
    },
    source: {
      reachable,
      stale,
      status: sourceStatus ?? "unknown",
      lastSeenAt: numberOrString(snapshot.source?.lastSeenAt) ?? numberOrString(snapshot.updatedAt),
      freshness: stale ? "stale" : numberOrString(snapshot.source?.lastSeenAt) || numberOrString(snapshot.updatedAt) ? "timestamped" : "unknown",
    },
    map: {
      available: map.available === true,
      readiness: map.readiness,
      usableForNavigation: map.usableForNavigation,
      usableForCoverage: map.usableForCoverage,
    },
    pose: {
      available: pose.available === true,
      readiness: pose.readiness,
    },
    localization: {
      ready: pose.available === true,
      evidence: pose.available === true ? "pose_available" : "pose_unavailable",
    },
    mission: {
      active: Boolean((mission.active as JsonRecord | null | undefined)?.id),
      state: (mission.summary as JsonRecord | null | undefined)?.state,
      activeMission: mission.active,
    },
    navigation: {
      state: navigation.state ?? "unknown",
      active: navigation.active === true,
      available: navigation.available === true,
    },
    capabilities: {
      movement: movementCapabilities,
      coverage: coverageCapabilities,
      missionActions: capabilityAvailability(features, [
        "pause_mission",
        "resume_mission",
        "cancel_mission",
        "retry_mission_step",
        "skip_mission_step",
      ]),
    },
    blockingReasons: blockers,
  };
}

function normalizedBlockingReasons(
  snapshot: RuntimeSnapshot,
  evidence: {
    connected: boolean;
    reachable: boolean;
    stale: boolean;
    mapAvailable: boolean;
    poseAvailable: boolean;
    movementCapabilities: JsonRecord;
  },
): string[] {
  const blockers = new Set<string>();
  const existing = snapshot.readiness?.blockingReasons;
  if (Array.isArray(existing)) {
    for (const blocker of existing) {
      if (typeof blocker === "string" && blocker.trim()) {
        blockers.add(blocker.trim());
      }
    }
  }
  if (!evidence.connected) blockers.add("runtime_offline");
  if (!evidence.reachable) blockers.add("source_unreachable");
  if (evidence.stale) blockers.add("stale_source");
  if (!evidence.mapAvailable) blockers.add("map_unavailable");
  if (!evidence.poseAvailable) blockers.add("pose_unavailable");
  if (evidence.movementCapabilities.available !== true) {
    blockers.add(String(evidence.movementCapabilities.reason ?? "movement_capability_unavailable"));
  }
  return [...blockers];
}

function capabilityAvailability(features: JsonRecord, names: string[]): JsonRecord {
  const entries = names.map((name) => {
    const capability = features[name] && typeof features[name] === "object" ? features[name] as JsonRecord : null;
    return {
      name,
      supported: capability?.supported === true,
      available: capability?.available === true,
      reason: typeof capability?.reason === "string" ? capability.reason : undefined,
      status: typeof capability?.status === "string" ? capability.status : undefined,
    };
  });
  const supported = entries.some((entry) => entry.supported);
  const available = entries.some((entry) => entry.available);
  const firstReason = entries.find((entry) => entry.supported && !entry.available)?.reason
    ?? entries.find((entry) => !entry.supported)?.reason
    ?? (supported ? "availability_not_reported" : "unsupported");
  return {
    supported,
    available,
    reason: available ? undefined : firstReason,
    options: entries,
  };
}

function unavailableCapability(reason: string): JsonRecord {
  return {
    supported: false,
    available: false,
    status: "unsupported",
    reason,
    commands: [],
  };
}

function capabilityReason(capability: CommandAvailability): string | undefined {
  const explicitReason = stringValue(capability.availabilityReason) ?? stringValue(capability.reason);
  if (explicitReason) return explicitReason;
  if (capability.supported === false) return "unsupported";
  if (capability.supported === true && capability.available !== true) return "availability_not_reported";
  return undefined;
}

function sanitizeCommandNames(commands: string[] | undefined): string[] {
  return (commands ?? []).filter((command) => /^[a-z][a-z0-9_]*$/.test(command));
}

function normalizeReasons(reasons: unknown[] | undefined): JsonRecord[] | undefined {
  if (!Array.isArray(reasons) || reasons.length === 0) return undefined;
  return reasons
    .filter((reason): reason is JsonRecord => reason != null && typeof reason === "object")
    .map((reason) => {
      const code = stringValue(reason.code);
      const message = stringValue(reason.message);
      return {
        code,
        message: message && !containsInternalReference(message) ? message : undefined,
      };
    })
    .filter((reason) => reason.code || reason.message);
}

function containsInternalReference(value: string): boolean {
  return /\/|ros|nav2|foxglove|topic|service|action|vm ip|mqtt|http/i.test(value);
}

function normalizeMapDimensions(metadata: JsonRecord | undefined, layeredMetadata: JsonRecord | undefined): JsonRecord | null {
  const width = numberValue(metadata?.width) ?? numberValue(layeredMetadata?.width);
  const height = numberValue(metadata?.height) ?? numberValue(layeredMetadata?.height);
  const resolution = numberValue(metadata?.resolution) ?? numberValue(layeredMetadata?.pixelSize);
  if (width == null && height == null && resolution == null) return null;
  return {
    width,
    height,
    resolution,
    frameId: stringValue(metadata?.frameId),
  };
}

function normalizeCellSummary(metadata: JsonRecord | undefined): JsonRecord | null {
  if (!metadata) return null;
  return {
    knownCells: numberValue(metadata.knownCells),
    freeCells: numberValue(metadata.freeCells),
    occupiedCells: numberValue(metadata.occupiedCells),
    unknownCells: numberValue(metadata.unknownCells),
    totalCells: numberValue(metadata.totalCells),
    knownRatio: numberValue(metadata.knownRatio),
    freeRatio: numberValue(metadata.freeRatio),
    occupiedRatio: numberValue(metadata.occupiedRatio),
    unknownRatio: numberValue(metadata.unknownRatio),
    knownAreaSqM: numberValue(metadata.knownAreaSqM),
  };
}

function countAnnotations(annotations: MapAnnotation[]): { room: number; zone: number } {
  return {
    room: annotations.filter((annotation) => annotation.kind === "room").length,
    zone: annotations.filter((annotation) => annotation.kind === "zone").length,
  };
}

function mapUsableFor(capabilityName: string, features: JsonRecord, mapAvailable: boolean): JsonRecord {
  const capability = features[capabilityName] && typeof features[capabilityName] === "object" ? features[capabilityName] as JsonRecord : null;
  const available = mapAvailable && capability?.available === true;
  return {
    supported: capability?.supported === true,
    available,
    reason: available ? undefined : capability?.reason ?? (mapAvailable ? "capability_unavailable" : "map_unavailable"),
  };
}

function normalizeActiveMission(mission: JsonRecord | null | undefined): JsonRecord | null {
  if (!mission) return null;
  const activeMission: JsonRecord = {
    id: stringValue(mission.id),
    type: stringValue(mission.type),
    status: stringValue(mission.status),
    phase: stringValue(mission.phase),
    progress: normalizeMissionProgress(mission.progress),
    availableActions: Array.isArray(mission.availableActions)
      ? mission.availableActions.filter((action): action is string => typeof action === "string")
      : [],
    terminalResult: normalizeMissionResult(mission.result),
    error: normalizeMissionError(mission.error),
    target: summarizeTarget(mission.target),
    startedAt: numberOrString(mission.startedAt),
    updatedAt: numberOrString(mission.updatedAt),
  };
  return activeMission;
}

function normalizeMissionSummary(mission: unknown): JsonRecord | null {
  if (!mission || typeof mission !== "object") return null;
  const record = mission as JsonRecord;
  return {
    id: stringValue(record.id),
    type: stringValue(record.type),
    status: stringValue(record.status),
    phase: stringValue(record.phase),
    progress: normalizeMissionProgress(record.progress),
    terminalResult: normalizeMissionResult(record.result),
    updatedAt: numberOrString(record.updatedAt),
  };
}

function normalizeMissionProgress(progress: unknown): JsonRecord | null {
  if (!progress || typeof progress !== "object") return null;
  const record = progress as JsonRecord;
  return {
    percent: numberValue(record.percent),
    currentStep: numberValue(record.currentStep),
    totalSteps: numberValue(record.totalSteps),
    distanceRemaining: numberValue(record.distanceRemaining),
    areaCoveredSqM: numberValue(record.areaCoveredSqM),
    areaRemainingSqM: numberValue(record.areaRemainingSqM),
  };
}

function normalizeMissionResult(result: unknown): JsonRecord | null {
  if (!result || typeof result !== "object") return null;
  const record = result as JsonRecord;
  return {
    status: stringValue(record.status),
    completedAt: numberOrString(record.completedAt),
    summary: stringValue(record.summary),
  };
}

function normalizeMissionError(error: unknown): JsonRecord | null {
  if (!error || typeof error !== "object") return null;
  const record = error as JsonRecord;
  return {
    code: stringValue(record.code),
    message: stringValue(record.message),
    recoverable: typeof record.recoverable === "boolean" ? record.recoverable : undefined,
  };
}

function summarizeTarget(target: unknown): JsonRecord | null {
  if (!target || typeof target !== "object") return null;
  const record = target as JsonRecord;
  const x = numberValue(record.x);
  const y = numberValue(record.y);
  const yaw = numberValue(record.yaw);
  if (x != null || y != null || yaw != null) {
    return { x, y, yaw };
  }
  return { type: stringValue(record.type) ?? "target" };
}

function summarizeRequestedNavigationTarget(target: unknown): JsonRecord | null {
  if (!target || typeof target !== "object" || Array.isArray(target)) return null;
  const record = target as JsonRecord;
  const summary: JsonRecord = {};
  const x = numberValue(record.x);
  const y = numberValue(record.y);
  const theta = numberValue(record.theta);
  if (x != null) summary.x = x;
  if (y != null) summary.y = y;
  if (theta != null) summary.theta = theta;
  const frameId = stringValue(record.frameId);
  const label = stringValue(record.label);
  if (frameId) summary.frameId = frameId;
  if (label) summary.label = label;
  return Object.keys(summary).length > 0 ? summary : null;
}

function navigationTargetSummary(target: NavigationStartTarget): JsonRecord {
  return {
    x: target.x,
    y: target.y,
    theta: target.theta,
    frameId: target.frameId,
    label: target.label,
  };
}

function summarizeRequestedCleanArea(area: unknown): JsonRecord | null {
  if (!area || typeof area !== "object" || Array.isArray(area)) return null;
  const record = area as JsonRecord;
  const summary: JsonRecord = {};
  const type = stringValue(record.type);
  const x = numberValue(record.x);
  const y = numberValue(record.y);
  const width = numberValue(record.width);
  const height = numberValue(record.height);
  const frameId = stringValue(record.frameId);
  const label = stringValue(record.label);
  if (type) summary.type = type;
  if (x != null) summary.x = x;
  if (y != null) summary.y = y;
  if (width != null) summary.width = width;
  if (height != null) summary.height = height;
  if (frameId) summary.frameId = frameId;
  if (label) summary.label = label;
  return Object.keys(summary).length > 0 ? summary : null;
}

function cleanAreaSummary(area: CleanAreaStartArea): JsonRecord {
  return {
    type: area.type,
    x: area.x,
    y: area.y,
    width: area.width,
    height: area.height,
    frameId: area.frameId,
    label: area.label,
  };
}

function cleanAreaToCoverageArea(area: CleanAreaStartArea): JsonRecord {
  return {
    shape: "rectangle",
    minX: area.x,
    minY: area.y,
    maxX: area.x + area.width,
    maxY: area.y + area.height,
  };
}

function summarizeCommandDispatchResult(result: unknown): JsonRecord | null {
  if (!result || typeof result !== "object") return null;
  const record = result as JsonRecord;
  return {
    ok: typeof record.ok === "boolean" ? record.ok : typeof record.success === "boolean" ? record.success : undefined,
    status: stringValue(record.status),
    command: stringValue(record.command),
    message: stringValue(record.message),
    reason: stringValue(record.reason) ?? stringValue(record.code),
  };
}

function summarizePath(path: unknown): JsonRecord {
  if (!Array.isArray(path) || path.length === 0) {
    return { available: false, pointCount: 0 };
  }
  const points = path.filter((point): point is JsonRecord => point != null && typeof point === "object");
  return {
    available: points.length > 0,
    pointCount: points.length,
    start: points[0] ? { x: numberValue(points[0].x), y: numberValue(points[0].y) } : null,
    end: points[points.length - 1] ? { x: numberValue(points[points.length - 1].x), y: numberValue(points[points.length - 1].y) } : null,
  };
}

function normalizeMappingState(mapping: JsonRecord | undefined): JsonRecord | null {
  if (!mapping) return null;
  return {
    state: stringValue(mapping.state),
    mode: stringValue(mapping.mode),
    reason: stringValue(mapping.stateReason),
    knownRatio: numberValue(mapping.knownRatio),
    unknownRatio: numberValue(mapping.unknownRatio),
    frontierCount: numberValue(mapping.frontierCount),
    activeMapName: stringValue(mapping.activeMapName),
    updatedAt: numberOrString(mapping.updatedAt),
  };
}

function inferBackend(snapshot: RuntimeSnapshot): VacuumBackendId {
  const source = stringValue(snapshot.identity?.source) ?? stringValue(snapshot.source?.kind);
  return source === "valetudo" || source === "valetudo_http" || source === "valetudo_mock" ? "valetudo" : "turtlebot4_nav2";
}

function asRecord(value: unknown): JsonRecord | null {
  return value != null && typeof value === "object" ? value as JsonRecord : null;
}

function arrayLength(value: unknown[] | undefined): number {
  return Array.isArray(value) ? value.length : 0;
}

function numberOrString(value: unknown): number | string | undefined {
  if (typeof value === "number" && Number.isFinite(value)) return value;
  if (typeof value === "string" && value.trim()) return value.trim();
  return undefined;
}

function numberValue(value: unknown): number | null {
  const numeric = typeof value === "string" ? Number(value) : value;
  return typeof numeric === "number" && Number.isFinite(numeric) ? numeric : null;
}

function safeDiagnostics(snapshot: RuntimeSnapshot, includeRawDiagnostics: boolean): JsonRecord {
  const diagnostics: JsonRecord = {
    backend: snapshot.diagnostics?.backend,
    mode: snapshot.diagnostics?.mode,
    source: snapshot.diagnostics?.source,
    readiness: snapshot.readiness ?? snapshot.diagnostics?.readiness,
    capabilityTiers: snapshot.diagnostics?.capabilityTiers,
    notes: snapshot.diagnostics?.notes,
    lastCommand: snapshot.diagnostics?.lastCommand,
    warnings: snapshot.diagnostics?.warnings,
  };
  if (includeRawDiagnostics) {
    diagnostics.raw = snapshot.rawDiagnostics;
    diagnostics.rawCapabilityNames = snapshot.diagnostics?.rawCapabilityNames;
    diagnostics.runtime = snapshot.diagnostics?.runtime;
    diagnostics.map = snapshot.diagnostics?.map;
    diagnostics.pose = snapshot.diagnostics?.pose;
    diagnostics.navigation = snapshot.diagnostics?.navigation;
    diagnostics.mapping = snapshot.diagnostics?.mapping;
  }
  return diagnostics;
}

function extractSnapshotPayload(data: unknown): unknown {
  if (!data || typeof data !== "object") return data;
  const record = data as JsonRecord;
  return record.snapshot && typeof record.snapshot === "object" ? record.snapshot : data;
}

function backendLabel(backend: VacuumBackendId): string {
  return backend === "turtlebot4_nav2" ? "TurtleBot4 Nav2" : "Valetudo Vacuum";
}

function commandInputSchema(command: RuntimeCommandName): Tool["inputSchema"] {
  if (command === "set_fan_speed" || command === "set_water_usage") {
    return {
      type: "object",
      properties: {
        value: { type: "string" },
      },
      required: ["value"],
      additionalProperties: false,
    };
  }
  return emptyInputSchema();
}

function emptyInputSchema(): Tool["inputSchema"] {
  return {
    type: "object",
    properties: {},
    additionalProperties: false,
  };
}

function commandDescription(toolName: CommandToolName): string {
  const descriptions: Record<CommandToolName, string> = {
    vacuum_start_cleaning: "Start basic vacuum cleaning when currently supported and available.",
    vacuum_pause: "Pause active vacuum cleaning when currently supported and available.",
    vacuum_resume: "Resume paused vacuum cleaning when currently supported and available.",
    vacuum_stop: "Stop active vacuum cleaning when currently supported and available.",
    vacuum_return_to_dock: "Return the vacuum to dock when currently supported and available.",
    vacuum_set_fan_speed: "Set vacuum fan speed to a currently available preset.",
    vacuum_set_water_usage: "Set vacuum water usage to a currently available preset.",
    vacuum_pause_mission: "Pause an already active simulation runtime-owned mission when activeMission.availableActions permits it.",
    vacuum_resume_mission: "Resume an already active paused simulation runtime-owned mission when activeMission.availableActions permits it.",
    vacuum_cancel_mission: "Cancel an already active simulation runtime-owned mission when activeMission.availableActions permits it.",
    vacuum_retry_mission_step: "Retry the current simulation mission step when activeMission.availableActions permits it.",
    vacuum_skip_mission_step: "Skip the current simulation mission step when activeMission.availableActions permits it.",
  };
  return descriptions[toolName];
}

function stringValue(value: unknown): string | undefined {
  return typeof value === "string" && value.trim() ? value.trim() : undefined;
}

function titleCase(value: string): string {
  return value
    .split(/[_\s-]+/)
    .filter(Boolean)
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(" ");
}
