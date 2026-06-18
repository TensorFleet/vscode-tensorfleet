import {
  formatError,
  getValetudoRuntimeHealth,
  getValetudoRuntimeSnapshot,
  isAuthError,
  sendValetudoRuntimeCommand,
  type HttpError,
  type ValetudoRuntimeCommandRequest,
  type ValetudoRuntimeCommandResult,
  type ValetudoRuntimeHealth,
  type ValetudoRuntimeSnapshot,
  type VmManagerClientOptions,
} from "tensorfleet-auth";
import type { McpRuntimeConfig } from "./config";
import { mcpFailure, type TensorFleetMcpResult, type TensorFleetMcpStatus } from "./result";

export type VacuumRuntimeContext =
  | {
      ok: true;
      options: VmManagerClientOptions;
      config: McpRuntimeConfig;
    }
  | {
      ok: false;
      result: TensorFleetMcpResult;
    };

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

  return {
    ok: true,
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
): Promise<TensorFleetMcpResult<ValetudoRuntimeHealth>> {
  try {
    return {
      ok: true,
      status: "success",
      message: "Vacuum health fetched.",
      data: await getValetudoRuntimeHealth(context.options),
    };
  } catch (error) {
    return mapRuntimeError(error, "Vacuum health could not be fetched.");
  }
}

export async function fetchVacuumSnapshot(
  context: Extract<VacuumRuntimeContext, { ok: true }>,
): Promise<TensorFleetMcpResult<ValetudoRuntimeSnapshot>> {
  try {
    return {
      ok: true,
      status: "success",
      message: "Vacuum snapshot fetched.",
      data: await getValetudoRuntimeSnapshot(context.options),
    };
  } catch (error) {
    return mapRuntimeError(error, "Vacuum snapshot could not be fetched.");
  }
}

export async function dispatchVacuumCommand(
  context: Extract<VacuumRuntimeContext, { ok: true }>,
  request: ValetudoRuntimeCommandRequest,
): Promise<TensorFleetMcpResult<ValetudoRuntimeCommandResult>> {
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

