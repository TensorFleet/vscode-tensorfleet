import {
  formatError,
  getTurtleBot4Nav2VacuumRuntimeHealth,
  getTurtleBot4Nav2VacuumRuntimeSnapshot,
  getValetudoRuntimeHealth,
  getValetudoRuntimeSnapshot,
  isAuthError,
  sendTurtleBot4Nav2VacuumRuntimeCommand,
  sendValetudoRuntimeCommand,
  type TurtleBot4Nav2VacuumRuntimeHealth,
  type TurtleBot4Nav2VacuumRuntimeSnapshot,
  type HttpError,
  type ValetudoRuntimeCommandRequest,
  type ValetudoRuntimeCommandResult,
  type ValetudoRuntimeHealth,
  type ValetudoRuntimeSnapshot,
  type VmManagerClientOptions,
} from "tensorfleet-auth";
import type { McpRuntimeConfig } from "./config";
import { mcpFailure, type TensorFleetMcpResult, type TensorFleetMcpStatus } from "./result";

export type VacuumBackendId = "valetudo" | "turtlebot4_nav2";

export type VacuumRuntimeContext =
  | {
      ok: true;
      options: VmManagerClientOptions;
      config: McpRuntimeConfig;
      backend: VacuumBackendId;
    }
  | {
      ok: false;
      result: TensorFleetMcpResult;
    };

export function normalizeVacuumBackend(value: unknown): VacuumBackendId | null {
  if (typeof value !== "string") return null;
  const normalized = value.trim().toLowerCase().replace(/[-\s]+/g, "_");
  if (normalized === "valetudo") return "valetudo";
  if (normalized === "turtlebot4_nav2" || normalized === "turtlebot4" || normalized === "simulation") {
    return "turtlebot4_nav2";
  }
  return null;
}

export function createVacuumRuntimeContext(config: McpRuntimeConfig): VacuumRuntimeContext {
  if (!config.token) {
    return {
      ok: false,
      result: mcpFailure(
        "not_authenticated",
        "missing_token",
        "TensorFleet authentication is not configured for MCP.",
        {
          tokenAvailable: config.tokenAvailable,
          selectedRegion: config.selectedRegion,
          selectedBackend: config.selectedBackend,
          configSource: config.source,
        },
      ),
    };
  }

  if (!config.vmManagerUrl) {
    return {
      ok: false,
      result: mcpFailure(
        "unavailable",
        "missing_vm_manager_url",
        "TensorFleet VM Manager URL is not configured for MCP.",
        {
          selectedRegion: config.selectedRegion,
          selectedBackend: config.selectedBackend,
          configSource: config.source,
        },
      ),
    };
  }

  const backend = normalizeVacuumBackend(config.selectedBackend);
  if (!backend) {
    return {
      ok: false,
      result: mcpFailure(
        "invalid_state",
        config.selectedBackend ? "unsupported_vacuum_backend" : "missing_vacuum_backend",
        config.selectedBackend
          ? `Selected TensorFleet vacuum backend '${config.selectedBackend}' is not supported by MCP.`
          : "TensorFleet vacuum backend is not configured for MCP.",
        {
          selectedRegion: config.selectedRegion,
          selectedBackend: config.selectedBackend,
          supportedBackends: ["valetudo", "turtlebot4_nav2"],
          configSource: config.source,
        },
      ),
    };
  }

  return {
    ok: true,
    backend,
    config,
    options: {
      baseUrl: config.vmManagerUrl,
      token: config.token,
      timeoutMs: 5000,
    },
  };
}

export async function fetchVacuumHealth(
  context: Extract<VacuumRuntimeContext, { ok: true }>,
): Promise<TensorFleetMcpResult<ValetudoRuntimeHealth | TurtleBot4Nav2VacuumRuntimeHealth>> {
  try {
    if (context.backend === "turtlebot4_nav2") {
      return {
        ok: true,
        status: "success",
        message: "Vacuum health fetched.",
        data: await getTurtleBot4Nav2VacuumRuntimeHealth(context.options),
      };
    }
    return {
      ok: true,
      status: "success",
      message: "Vacuum health fetched.",
      data: await getValetudoRuntimeHealth(context.options),
    };
  } catch (error) {
    if (context.backend === "turtlebot4_nav2" && (error as HttpError)?.status === 404) {
      return mcpFailure("unavailable", "simulation_health_route_unavailable", "Simulation vacuum health is not exposed by the VM runtime yet.", errorData(error));
    }
    return mapRuntimeError(error, "Vacuum health could not be fetched.");
  }
}

export async function fetchVacuumSnapshot(
  context: Extract<VacuumRuntimeContext, { ok: true }>,
): Promise<TensorFleetMcpResult<ValetudoRuntimeSnapshot | TurtleBot4Nav2VacuumRuntimeSnapshot>> {
  try {
    if (context.backend === "turtlebot4_nav2") {
      return {
        ok: true,
        status: "success",
        message: "Vacuum snapshot fetched.",
        data: await getTurtleBot4Nav2VacuumRuntimeSnapshot(context.options),
      };
    }
    return {
      ok: true,
      status: "success",
      message: "Vacuum snapshot fetched.",
      data: await getValetudoRuntimeSnapshot(context.options),
    };
  } catch (error) {
    if (context.backend === "turtlebot4_nav2" && (error as HttpError)?.status === 404) {
      return mcpFailure(
        "unavailable",
        "simulation_snapshot_route_unavailable",
        "Simulation vacuum snapshot is not exposed by the VM runtime yet.",
        errorData(error),
      );
    }
    return mapRuntimeError(error, "Vacuum snapshot could not be fetched.");
  }
}

export async function dispatchVacuumCommand(
  context: Extract<VacuumRuntimeContext, { ok: true }>,
  request: ValetudoRuntimeCommandRequest,
): Promise<TensorFleetMcpResult<ValetudoRuntimeCommandResult | Record<string, unknown>>> {
  if (context.backend === "turtlebot4_nav2") {
    try {
      const result = await sendTurtleBot4Nav2VacuumRuntimeCommand(context.options, request);
      if (isRuntimeCommandSuccess(result)) {
        return {
          ok: true,
          status: "success",
          message: stringField(result, "message") ?? "Simulation vacuum command dispatched.",
          data: result,
        };
      }
      const status = stringField(result, "status");
      const reason = stringField(result, "code") ?? stringField(result, "reason") ?? status ?? "command_failed";
      return mcpFailure(
        normalizeRuntimeStatus(status, reason),
        reason,
        stringField(result, "message") ?? "Simulation vacuum command was refused by the runtime.",
        result,
      );
    } catch (error) {
      if ((error as HttpError)?.status === 404) {
        return mcpFailure(
          "unavailable",
          "simulation_command_route_unavailable",
          "Simulation vacuum command dispatch is not exposed by the VM runtime yet.",
          errorData(error),
        );
      }
      return mapRuntimeError(error, "Simulation vacuum command could not be dispatched.");
    }
  }

  try {
    const result = await sendValetudoRuntimeCommand(context.options, request);
    if (result.ok && result.status === "success") {
      return {
        ok: true,
        status: "success",
        message: result.message || "Vacuum command dispatched.",
        data: result,
      };
    }
    return mcpFailure(
      normalizeRuntimeStatus(result.status, result.code ?? result.reason),
      result.code ?? result.reason ?? result.status ?? "command_failed",
      result.message || "Vacuum command was refused by the runtime.",
      result,
    );
  } catch (error) {
    return mapRuntimeError(error, "Vacuum command could not be dispatched.");
  }
}

function isRuntimeCommandSuccess(result: Record<string, unknown>): boolean {
  if (result.ok === true) return true;
  if (result.success === true) return true;
  return stringField(result, "status") === "success";
}

function stringField(record: Record<string, unknown>, field: string): string | undefined {
  const value = record[field];
  return typeof value === "string" && value.trim() ? value.trim() : undefined;
}

function normalizeRuntimeStatus(status: string | undefined, reason: string | undefined): Exclude<TensorFleetMcpStatus, "success"> {
  const value = reason || status || "";
  if (value === "unsupported" || value === "unsupported_command" || status === "unsupported") return "unsupported";
  if (value === "invalid_request" || value === "invalid_json" || value === "missing_command" || value === "missing_value") {
    return "invalid_request";
  }
  if (value === "invalid_state" || value === "command_invalid_state") return "invalid_state";
  if (value === "runtime_offline") return "runtime_offline";
  if (value === "source_unreachable") return "source_unreachable";
  if (value === "stale_source") return "stale_source";
  if (status === "unavailable") return "unavailable";
  return "backend_error";
}

export function mapRuntimeError<T = null>(error: unknown, message: string): TensorFleetMcpResult<T> {
  if (isAuthError(error) || formatError(error) === "NOT_AUTHENTICATED") {
    return mcpFailure("not_authenticated", "invalid_or_missing_token", "TensorFleet authentication failed for MCP.");
  }

  const httpError = error as HttpError;
  if (typeof httpError.status === "number") {
    if (httpError.status === 400) {
      return mcpFailure("invalid_request", "bad_request", message, errorData(error));
    }
    if (httpError.status === 404) {
      return mcpFailure("runtime_offline", "runtime_endpoint_not_found", message, errorData(error));
    }
    if (httpError.status === 409) {
      return mcpFailure("invalid_state", "runtime_rejected_state", message, errorData(error));
    }
    if (httpError.status === 502 || httpError.status === 503 || httpError.status === 504) {
      return mcpFailure("source_unreachable", "vm_manager_proxy_unreachable", message, errorData(error));
    }
    if (httpError.status >= 500) {
      return mcpFailure("backend_error", "vm_manager_error", message, errorData(error));
    }
  }

  const text = formatError(error);
  if (/not responding|ECONNREFUSED|timed out|timeout/i.test(text)) {
    return mcpFailure("source_unreachable", "vm_manager_unreachable", message, { error: text } as T);
  }

  return mcpFailure("failed", "request_failed", message, { error: text } as T);
}

function errorData<T>(error: unknown): T {
  const httpError = error as HttpError;
  return {
    status: httpError.status,
    body: httpError.body,
    error: formatError(error),
  } as T;
}
