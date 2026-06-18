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
  navigation?: JsonRecord;
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
        return mcpSuccess("Vacuum capabilities fetched.", normalizeCapabilities(snapshot.data as RuntimeSnapshot));
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
    description: "Return a compact normalized vacuum map summary without preview/grid payloads.",
    inputSchema: emptyInputSchema(),
    execute: async () =>
      withRuntime(async (context) => {
        const snapshot = await fetchSelectedVacuumSnapshot(context);
        if (!snapshot.ok || !snapshot.data) return snapshot;
        return mcpSuccess("Vacuum map summary fetched.", normalizeMap(snapshot.data as RuntimeSnapshot, false));
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

function normalizeSnapshot(
  snapshot: RuntimeSnapshot,
  options: { backend: VacuumBackendId; includeDiagnostics: boolean; includeRawDiagnostics: boolean; includeMapPreview: boolean },
): JsonRecord {
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
    map: normalizeMap(snapshot, options.includeMapPreview),
    pose: normalizePose(snapshot),
    navigation: normalizeNavigationState(snapshot),
    mission: normalizeMissionState(snapshot),
    readiness: snapshot.readiness ?? snapshot.diagnostics?.readiness,
    fault: snapshot.fault,
    capabilities: normalizeCapabilities(snapshot),
    updatedAt: snapshot.updatedAt,
  };

  if (options.includeDiagnostics) {
    data.diagnostics = safeDiagnostics(snapshot, options.includeRawDiagnostics);
  }

  return data;
}

function normalizeCapabilities(snapshot: RuntimeSnapshot): JsonRecord {
  const normalizedCommands = normalizeCommandAvailability(snapshot);
  const normalizedFeatures = normalizeFeatureCapabilities(snapshot);

  return {
    commands: Object.fromEntries(
      Object.entries(normalizedCommands)
      .map(([command, availability]) => [
        command,
        {
          supported: availability.supported !== false,
          available: availability.available === true,
          reason: availability.reason ?? availability.availabilityReason,
          status: availability.status,
          notes: availability.notes,
        },
      ]),
    ),
    features: normalizedFeatures,
    settings: normalizeCleaningSettings(snapshot),
    readiness: snapshot.readiness ?? snapshot.diagnostics?.readiness,
    capabilityTiers: snapshot.diagnostics?.capabilityTiers,
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
        return [
          name,
          {
            supported: capability.supported === true,
            available: capability.available,
            status: capability.status,
            reason: capability.availabilityReason ?? capability.reason,
            notes: capability.notes,
            commands: capability.commands,
            attributes: capability.attributes,
            reasons: capability.reasons,
          },
        ];
      }),
  );
}

function featureToCommandAvailability(feature: CommandAvailability): CommandAvailability {
  return {
    supported: feature.supported === true,
    available: feature.supported === true && feature.available !== false,
    status: feature.status,
    reason: feature.availabilityReason ?? feature.reason ?? (feature.supported === true ? undefined : "unsupported_command"),
    notes: feature.notes,
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

function normalizeMap(snapshot: RuntimeSnapshot, includePreview: boolean): JsonRecord {
  const map: JsonRecord = {
    available: snapshot.map?.available === true || snapshot.map?.readiness === "ready" || snapshot.map?.metadata?.hasMap === true,
    readiness: snapshot.map?.readiness,
    receiving: snapshot.map?.receiving,
    metadata: snapshot.map?.metadata ?? snapshot.map?.layeredMetadata,
    targets: normalizeMapTargets(snapshot, false),
    detail: snapshot.map?.detail,
  };
  if (includePreview) {
    map.preview = snapshot.map?.preview ?? snapshot.map?.layeredPreview;
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

function normalizePose(snapshot: RuntimeSnapshot): JsonRecord {
  return {
    available: snapshot.pose?.available === true,
    readiness: snapshot.pose?.readiness,
    coordinates: snapshot.pose?.coordinates ?? null,
    detail: snapshot.pose?.detail,
  };
}

function normalizeNavigationState(snapshot: RuntimeSnapshot): JsonRecord {
  return {
    state: snapshot.navigation?.state ?? "unknown",
    active: snapshot.navigation?.active === true,
    currentTarget: snapshot.navigation?.currentTarget ?? null,
    terminalState: snapshot.navigation?.terminalState ?? null,
    progress: snapshot.navigation?.progress,
    detail: snapshot.navigation?.detail,
  };
}

function normalizeMissionState(snapshot: RuntimeSnapshot): JsonRecord {
  return {
    summary: snapshot.mission,
    active: snapshot.activeMission ?? snapshot.missions?.active ?? null,
    recent: Array.isArray(snapshot.missions?.recent) ? snapshot.missions?.recent : [],
    mapping: snapshot.mapping,
    availableActions: Array.isArray(snapshot.activity?.availableActions) ? snapshot.activity?.availableActions : [],
  };
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
