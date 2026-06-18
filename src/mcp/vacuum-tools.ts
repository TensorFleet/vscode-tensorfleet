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
} as const;

export const VACUUM_MCP_TOOL_NAMES = [
  "vacuum_get_health",
  "vacuum_get_snapshot",
  "vacuum_get_capabilities",
  "vacuum_get_map_targets",
  "vacuum_get_pose",
  "vacuum_get_map_summary",
  "vacuum_get_mission_state",
  "vacuum_get_navigation_state",
  ...Object.keys(COMMAND_TOOL_TO_RUNTIME_COMMAND),
] as const;

type CommandToolName = keyof typeof COMMAND_TOOL_TO_RUNTIME_COMMAND;
type RuntimeCommandName = (typeof COMMAND_TOOL_TO_RUNTIME_COMMAND)[CommandToolName];

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

async function executeCommandTool(command: RuntimeCommandName, args: JsonRecord): Promise<TensorFleetMcpResult> {
  return withRuntime(async (context) => {
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
  if (!evidence.connected) blockers.add("runtime_unavailable");
  if (!evidence.reachable) blockers.add("source_unreachable");
  if (evidence.stale) blockers.add("source_stale");
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
