import type {
  ValetudoRuntimeCommandRequest,
  ValetudoRuntimeCommandResult,
  ValetudoRuntimeHealth,
  ValetudoRuntimeSnapshot,
} from "./runtimeContract";

export type ValetudoRuntimeClientConfig = {
  baseUrl?: string;
  token?: string;
  routeMode?: "vm-manager" | "direct";
};

export type ValetudoRuntimeClient = {
  getHealth: () => Promise<ValetudoRuntimeHealth>;
  getSnapshot: () => Promise<ValetudoRuntimeSnapshot>;
  sendCommand: (request: ValetudoRuntimeCommandRequest) => Promise<ValetudoRuntimeCommandResult>;
};

const VM_MANAGER_VALETUDO_PATH = "/vms/self/tensorfleet/api/v1/valetudo";
const DIRECT_VALETUDO_PATH = "/api/v1/valetudo";

function trimTrailingSlash(value: string): string {
  return value.replace(/\/+$/, "");
}

function readWindowValue(name: string): string {
  if (typeof window === "undefined") {
    return "";
  }
  const value = (window as unknown as Record<string, unknown>)[name];
  return typeof value === "string" ? value.trim() : "";
}

export function getDefaultValetudoRuntimeClientConfig(): ValetudoRuntimeClientConfig {
  const configuredBaseUrl =
    readWindowValue("TENSORFLEET_VALETUDO_RUNTIME_URL") ||
    readWindowValue("TENSORFLEET_VM_MANAGER_URL") ||
    "http://localhost:8080";
  const configuredRouteMode = readWindowValue("TENSORFLEET_VALETUDO_RUNTIME_ROUTE_MODE");
  return {
    baseUrl: configuredBaseUrl,
    token: readWindowValue("TENSORFLEET_JWT"),
    routeMode: configuredRouteMode === "direct" ? "direct" : "vm-manager",
  };
}

function endpoint(config: ValetudoRuntimeClientConfig, path: "health" | "snapshot" | "command"): string {
  const baseUrl = trimTrailingSlash(config.baseUrl || "http://localhost:8080");
  const routePrefix = config.routeMode === "direct" ? DIRECT_VALETUDO_PATH : VM_MANAGER_VALETUDO_PATH;
  return `${baseUrl}${routePrefix}/${path}`;
}

type FetchJsonOptions = {
  // Commands legitimately return structured non-2xx bodies (e.g. unavailable/failed),
  // so the caller can opt in to receiving the parsed error body instead of throwing.
  // Snapshot/health must NOT do this: a proxy error body (e.g. vm-manager 502
  // `{"error":"..."}`) would otherwise be mapped as a snapshot and crash the UI.
  allowErrorBody?: boolean;
};

async function fetchJson<T>(
  url: string,
  config: ValetudoRuntimeClientConfig,
  init: RequestInit = {},
  options: FetchJsonOptions = {},
): Promise<T> {
  const headers = new Headers(init.headers);
  headers.set("Accept", "application/json");
  if (init.body != null) {
    headers.set("Content-Type", "application/json");
  }
  if (config.token) {
    headers.set("Authorization", `Bearer ${config.token}`);
  }

  const response = await fetch(url, {
    ...init,
    headers,
  });
  const payload = (await response.json().catch(() => null)) as T | null;
  if (!response.ok) {
    if (options.allowErrorBody && payload != null) {
      return payload;
    }
    throw new Error(`Valetudo runtime request failed with HTTP ${response.status}.`);
  }
  if (payload == null) {
    throw new Error("Valetudo runtime returned an empty response.");
  }
  return payload;
}

export function createValetudoRuntimeClient(config: ValetudoRuntimeClientConfig = {}): ValetudoRuntimeClient {
  const resolvedConfig = {
    ...getDefaultValetudoRuntimeClientConfig(),
    ...config,
  };
  return {
    getHealth: () => fetchJson(endpoint(resolvedConfig, "health"), resolvedConfig),
    getSnapshot: () => fetchJson(endpoint(resolvedConfig, "snapshot"), resolvedConfig),
    sendCommand: (request) =>
      fetchJson(
        endpoint(resolvedConfig, "command"),
        resolvedConfig,
        {
          method: "POST",
          body: JSON.stringify(request),
        },
        { allowErrorBody: true },
      ),
  };
}
