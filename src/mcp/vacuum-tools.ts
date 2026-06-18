import type { Tool } from "@modelcontextprotocol/sdk/types.js";
import { resolveMcpRuntimeConfig } from "./config";
import { createVacuumRuntimeContext, dispatchVacuumCommand, fetchVacuumHealth, fetchVacuumSnapshot } from "./vacuum-runtime";
import { mcpFailure, mcpSuccess, type TensorFleetMcpResult } from "./result";

type JsonRecord = Record<string, unknown>;

type VacuumToolDefinition = Tool & {
  execute: (args: JsonRecord) => Promise<TensorFleetMcpResult>;
};

type RuntimeSnapshot = JsonRecord & {
  runtime?: JsonRecord;
  robot?: JsonRecord;
  source?: JsonRecord;
  connectivity?: JsonRecord;
  state?: JsonRecord;
  battery?: JsonRecord;
  dock?: JsonRecord;
  cleaningSettings?: {
    fanSpeed?: { current?: string; options?: string[] };
    waterUsage?: { current?: string; options?: string[] };
  };
  map?: JsonRecord & {
    metadata?: JsonRecord;
    preview?: unknown;
    targets?: {
      segments?: MapTarget[];
      zones?: MapTarget[];
    };
  };
  capabilities?: {
    commands?: Record<string, { available?: boolean; reason?: string }>;
    diagnostics?: unknown[];
  };
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
  ...Object.keys(COMMAND_TOOL_TO_RUNTIME_COMMAND),
] as const;

type CommandToolName = keyof typeof COMMAND_TOOL_TO_RUNTIME_COMMAND;
type RuntimeCommandName = (typeof COMMAND_TOOL_TO_RUNTIME_COMMAND)[CommandToolName];

export function createVacuumTools(): Map<string, VacuumToolDefinition> {
  const tools = new Map<string, VacuumToolDefinition>();

  tools.set("vacuum_get_health", {
    name: "vacuum_get_health",
    description: "Fetch TensorFleet vacuum runtime and source health through the VM Manager proxy.",
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
        const snapshot = await fetchVacuumSnapshot(context);
        if (!snapshot.ok || !snapshot.data) return snapshot;
        return mcpSuccess(
          "Vacuum snapshot fetched.",
          normalizeSnapshot(snapshot.data as RuntimeSnapshot, {
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
        const snapshot = await fetchVacuumSnapshot(context);
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
        const snapshot = await fetchVacuumSnapshot(context);
        if (!snapshot.ok || !snapshot.data) return snapshot;
        return mcpSuccess("Vacuum map targets fetched.", normalizeMapTargets(snapshot.data as RuntimeSnapshot, args.include_geometry === true));
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
    const snapshotResult = await fetchVacuumSnapshot(context);
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
  const support = commands[command];
  if (!support) {
    return mcpFailure("unsupported", "unsupported_command", `Vacuum command '${command}' is not supported by the current runtime.`);
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
    return validateSettingValue(snapshot.cleaningSettings?.fanSpeed?.options, "fan speed", command, args);
  }
  if (command === "set_water_usage") {
    return validateSettingValue(snapshot.cleaningSettings?.waterUsage?.options, "water usage", command, args);
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

function normalizeSnapshot(snapshot: RuntimeSnapshot, options: { includeDiagnostics: boolean; includeRawDiagnostics: boolean; includeMapPreview: boolean }): JsonRecord {
  const data: JsonRecord = {
    identity: {
      id: stringValue(snapshot.robot?.id) ?? "valetudo",
      label: stringValue(snapshot.robot?.name) ?? "Valetudo Vacuum",
      source: "valetudo",
    },
    availability: {
      connected: snapshot.connectivity?.online === true,
      reachable: snapshot.connectivity?.reachable === true,
    },
    health: {
      runtime: snapshot.runtime,
      source: snapshot.source,
      updatedAt: snapshot.updatedAt,
    },
    activity: snapshot.state,
    battery: snapshot.battery,
    dock: snapshot.dock,
    cleaningSettings: normalizeCleaningSettings(snapshot),
    map: normalizeMap(snapshot, options.includeMapPreview),
    capabilities: normalizeCapabilities(snapshot),
    updatedAt: snapshot.updatedAt,
  };

  if (options.includeDiagnostics) {
    data.diagnostics = safeDiagnostics(snapshot, options.includeRawDiagnostics);
  }

  return data;
}

function normalizeCapabilities(snapshot: RuntimeSnapshot): JsonRecord {
  const commands = snapshot.capabilities?.commands ?? {};
  const normalizedCommands = Object.fromEntries(
    Object.entries(commands)
      .filter(([command]) => Object.values(COMMAND_TOOL_TO_RUNTIME_COMMAND).includes(command as RuntimeCommandName))
      .map(([command, availability]) => [
        command,
        {
          supported: true,
          available: availability.available === true,
          reason: availability.reason,
        },
      ]),
  );

  return {
    commands: normalizedCommands,
    settings: normalizeCleaningSettings(snapshot),
    readiness: snapshot.diagnostics?.readiness,
    capabilityTiers: snapshot.diagnostics?.capabilityTiers,
  };
}

function normalizeCleaningSettings(snapshot: RuntimeSnapshot): JsonRecord {
  return {
    fanSpeed: normalizeSetting(snapshot.cleaningSettings?.fanSpeed),
    waterUsage: normalizeSetting(snapshot.cleaningSettings?.waterUsage),
  };
}

function normalizeSetting(setting: { current?: string; options?: string[] } | undefined): JsonRecord | null {
  if (!setting) return null;
  return {
    current: setting.current,
    options: (setting.options ?? []).map((value) => ({ value, label: titleCase(value) })),
  };
}

function normalizeMap(snapshot: RuntimeSnapshot, includePreview: boolean): JsonRecord {
  const map: JsonRecord = {
    available: snapshot.map?.available === true,
    metadata: snapshot.map?.metadata,
    targets: normalizeMapTargets(snapshot, false),
    detail: snapshot.map?.detail,
  };
  if (includePreview) {
    map.preview = snapshot.map?.preview;
  }
  return map;
}

function normalizeMapTargets(snapshot: RuntimeSnapshot, includeGeometry: boolean): JsonRecord {
  return {
    note: "Map targets are read-only in MCP; target cleaning commands remain unsupported.",
    segments: normalizeTargetList(snapshot.map?.targets?.segments, "segment", includeGeometry),
    zones: normalizeTargetList(snapshot.map?.targets?.zones, "zone", includeGeometry),
    rooms: [],
  };
}

function normalizeTargetList(targets: MapTarget[] | undefined, kind: "segment" | "zone", includeGeometry: boolean): JsonRecord[] {
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

function safeDiagnostics(snapshot: RuntimeSnapshot, includeRawDiagnostics: boolean): JsonRecord {
  const diagnostics: JsonRecord = {
    mode: snapshot.diagnostics?.mode,
    source: snapshot.diagnostics?.source,
    readiness: snapshot.diagnostics?.readiness,
    capabilityTiers: snapshot.diagnostics?.capabilityTiers,
    transports: snapshot.diagnostics?.transports,
    notes: snapshot.diagnostics?.notes,
    lastCommand: snapshot.diagnostics?.lastCommand,
  };
  if (includeRawDiagnostics) {
    diagnostics.raw = snapshot.rawDiagnostics;
    diagnostics.rawCapabilityNames = snapshot.diagnostics?.rawCapabilityNames;
  }
  return diagnostics;
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
