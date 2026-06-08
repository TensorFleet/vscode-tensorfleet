export type ConnectionState = "connected" | "disconnected" | "not_authenticated";
export type VmState = "unknown" | "stopped" | "starting" | "running" | "stopping" | "failed" | "pending";

export interface VmStatusResponse {
  status: string;
  vm_id?: string;
  ip_address?: string;
  updated_at?: string;
  vmId?: string;
}

export interface VmInfoResponse extends VmStatusResponse {
  id?: string;
  created_at?: string;
  uptime_seconds?: number | null;
  provider?: string;
  region?: string;
}

export interface VmSnapshot {
  connection: ConnectionState;
  vmState: VmState;
  nodeId?: string;
  ipAddress?: string;
  provider?: string;
  region?: string;
  uptimeSeconds?: number | null;
  timestamp: number;
  error?: string;
}

export interface VmHttpError extends Error {
  status?: number;
  body?: string;
}

export function parseVmState(status?: string): VmState {
  const normalized = (status ?? "").toLowerCase().trim();
  if (normalized.includes("running")) return "running";
  if (normalized.includes("starting")) return "starting";
  if (normalized.includes("stopping")) return "stopping";
  if (normalized.includes("stopped")) return "stopped";
  if (normalized.includes("fail") || normalized.includes("error")) return "failed";
  return "unknown";
}

export function createVmSnapshot(params: Partial<VmSnapshot>): VmSnapshot {
  return {
    connection: params.connection ?? "connected",
    vmState: params.vmState ?? "unknown",
    nodeId: params.nodeId,
    ipAddress: params.ipAddress,
    provider: params.provider,
    region: params.region,
    uptimeSeconds: params.uptimeSeconds,
    timestamp: params.timestamp ?? Date.now(),
    error: params.error,
  };
}

export function buildVmSnapshot(params: {
  status?: VmStatusResponse;
  info?: VmInfoResponse;
  sawVmMissing?: boolean;
}): VmSnapshot {
  const vmState = params.status ? parseVmState(params.status.status) : "unknown";
  const resolvedState = vmState === "unknown" && params.sawVmMissing ? "pending" : vmState;

  return createVmSnapshot({
    connection: "connected",
    vmState: resolvedState,
    nodeId: params.info?.id ?? params.status?.vm_id ?? params.status?.vmId,
    ipAddress: params.info?.ip_address || params.status?.ip_address,
    provider: params.info?.provider,
    region: params.info?.region,
    uptimeSeconds: params.info?.uptime_seconds,
  });
}

export function isVmNotFoundError(error: unknown): boolean {
  return Boolean(error && typeof error === "object" && "status" in error && (error as VmHttpError).status === 404);
}

export function isVmAuthError(error: unknown): boolean {
  return Boolean(
    error &&
      typeof error === "object" &&
      "status" in error &&
      ((error as VmHttpError).status === 401 || (error as VmHttpError).status === 403),
  );
}

export async function fetchVmIdFromManager(params: {
  baseUrl: string;
  token?: string;
  signal?: AbortSignal;
  fetchImpl?: typeof fetch;
}): Promise<string | null> {
  const trimmedBaseUrl = params.baseUrl.trim().replace(/\/+$/, "");
  if (!trimmedBaseUrl) {
    return null;
  }

  const fetcher = params.fetchImpl ?? fetch;
  const response = await fetcher(`${trimmedBaseUrl}/vms/self/status`, {
    headers: params.token?.trim() ? { Authorization: `Bearer ${params.token.trim()}` } : undefined,
    signal: params.signal,
  });

  if (!response.ok) {
    throw new Error(`VM status returned ${response.status}`);
  }

  const payload = (await response.json()) as VmStatusResponse;
  const detectedVmId = payload.vm_id || payload.vmId;
  if (!detectedVmId) {
    throw new Error("VM ID missing from status response");
  }
  return detectedVmId;
}
