import * as http from 'http';
import * as https from 'https';

import type { VMConfig } from './vm-config';

export type ConnectionState = 'connected' | 'disconnected' | 'not_authenticated';
export type VmState =
  | 'unknown'
  | 'stopped'
  | 'starting'
  | 'running'
  | 'stopping'
  | 'failed'
  | 'pending';

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

export interface ApiHealthResponse {
  status: string;
  time: string;
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

export interface HttpError extends Error {
  status?: number;
  body?: string;
}

export interface GazeboPreset {
  name: string;
  description?: string;
  base_world: string;
  world_components: string[];
  model_components: string[];
}

export interface GazeboSelection {
  mode: 'default' | 'world' | 'preset';
  world?: string;
  preset?: string;
}

export interface VmManagerClientOptions {
  baseUrl: string;
  token?: string;
  timeoutMs?: number;
}

export function createVmSnapshot(params: Partial<VmSnapshot>): VmSnapshot {
  return {
    connection: params.connection ?? 'connected',
    vmState: params.vmState ?? 'unknown',
    nodeId: params.nodeId,
    ipAddress: params.ipAddress,
    provider: params.provider,
    region: params.region,
    uptimeSeconds: params.uptimeSeconds,
    timestamp: params.timestamp ?? Date.now(),
    error: params.error
  };
}

export function parseVmState(status?: string): VmState {
  const normalized = (status ?? '').toLowerCase().trim();
  if (normalized.includes('running')) return 'running';
  if (normalized.includes('starting')) return 'starting';
  if (normalized.includes('stopping')) return 'stopping';
  if (normalized.includes('stopped')) return 'stopped';
  if (normalized.includes('fail') || normalized.includes('error')) return 'failed';
  return 'unknown';
}

export function isNotFoundError(error: unknown): boolean {
  return !!(error && typeof error === 'object' && 'status' in error && (error as HttpError).status === 404);
}

export function isAuthError(error: unknown): boolean {
  return !!(
    error &&
    typeof error === 'object' &&
    'status' in error &&
    ((error as HttpError).status === 401 || (error as HttpError).status === 403)
  );
}

export function formatError(error: unknown): string {
  if (error instanceof Error) return error.message;
  if (typeof error === 'string') return error;
  try {
    return JSON.stringify(error);
  } catch {
    return 'Unknown error';
  }
}

export async function fetchVmSnapshot(options: VmManagerClientOptions): Promise<VmSnapshot> {
  if (!options.token) {
    return createVmSnapshot({
      connection: 'not_authenticated',
      vmState: 'unknown',
      error: 'Please login to access VM Manager'
    });
  }

  try {
    await apiRequest<ApiHealthResponse>(options, 'GET', '/vms/health', undefined, { includeAuth: false });

    let status: VmStatusResponse | undefined;
    let vmState: VmState = 'unknown';
    let sawVmMissing = false;
    let statusError: unknown;

    try {
      status = await apiRequest<VmStatusResponse>(options, 'GET', '/vms/self/status');
      if (status) {
        vmState = parseVmState(status.status);
      }
    } catch (error) {
      if (isAuthError(error)) {
        return authFailedSnapshot();
      }
      if (isNotFoundError(error)) {
        sawVmMissing = true;
      } else {
        statusError = error;
      }
    }

    let info: VmInfoResponse | undefined;
    try {
      info = await apiRequest<VmInfoResponse>(options, 'GET', '/vms/self/info');
    } catch (infoError) {
      if (isAuthError(infoError)) {
        return authFailedSnapshot();
      }
      if (isNotFoundError(infoError)) {
        sawVmMissing = true;
      } else if (!status) {
        throw infoError;
      }
    }

    if (!status && !info && statusError) {
      throw statusError;
    }

    const resolvedState = vmState === 'unknown' && sawVmMissing ? 'pending' : vmState;

    return createVmSnapshot({
      connection: 'connected',
      vmState: resolvedState,
      nodeId: info?.id ?? status?.vm_id,
      ipAddress: info?.ip_address || status?.ip_address,
      provider: info?.provider,
      region: info?.region,
      uptimeSeconds: info?.uptime_seconds
    });
  } catch (error) {
    if (isAuthError(error)) {
      return authFailedSnapshot();
    }
    throw error;
  }
}

export async function startVm(options: VmManagerClientOptions, config: VMConfig): Promise<{ status: string }> {
  return apiRequest<{ status: string }>(options, 'POST', '/vms/self/start', {
    sim_config: config.sim_config,
  });
}

export async function stopVm(options: VmManagerClientOptions): Promise<{ status: string }> {
  return apiRequest<{ status: string }>(options, 'POST', '/vms/self/stop');
}

export async function restartVm(options: VmManagerClientOptions, config: VMConfig): Promise<{ status: string; message?: string }> {
  return apiRequest<{ status: string; message?: string }>(options, 'POST', '/vms/self/restart', {
    sim_config: config.sim_config,
  });
}

export async function listGazeboPresets(options: VmManagerClientOptions): Promise<GazeboPreset[]> {
  const response = await apiRequest<{ presets?: GazeboPreset[] }>(
    options,
    'GET',
    '/vms/self/tensorfleet/api/v1/presets'
  );
  return response.presets ?? [];
}

export async function getGazeboSelection(options: VmManagerClientOptions): Promise<GazeboSelection> {
  return apiRequest<GazeboSelection>(
    options,
    'GET',
    '/vms/self/tensorfleet/api/v1/gazebo/world'
  );
}

export async function setGazeboPreset(options: VmManagerClientOptions, preset: string): Promise<string> {
  const trimmedPreset = preset.trim();
  const response = await apiRequest<{ message?: string }>(
    options,
    'POST',
    '/vms/self/tensorfleet/api/v1/gazebo/world',
    { preset: trimmedPreset }
  );
  return response.message ?? `Gazebo preset '${trimmedPreset}' switch requested`;
}

export async function resetGazeboSelection(options: VmManagerClientOptions): Promise<string> {
  const response = await apiRequest<{ message?: string }>(
    options,
    'POST',
    '/vms/self/tensorfleet/api/v1/gazebo/world',
    { reset: true }
  );
  return response.message ?? 'Gazebo selection reset requested';
}

function authFailedSnapshot(): VmSnapshot {
  return createVmSnapshot({
    connection: 'not_authenticated',
    vmState: 'unknown',
    error: 'VM Manager authentication failed - check settings or login again'
  });
}

async function apiRequest<T>(
  options: VmManagerClientOptions,
  method: string,
  endpoint: string,
  body?: any,
  requestOptions?: { includeAuth?: boolean }
): Promise<T> {
  const baseUrl = options.baseUrl.trim();
  const url = new URL(endpoint.replace(/^\//, ''), baseUrl.endsWith('/') ? baseUrl : `${baseUrl}/`);
  const isHttps = url.protocol === 'https:';
  const lib = isHttps ? https : http;
  const data = body ? JSON.stringify(body) : undefined;

  const headers: http.OutgoingHttpHeaders = {
    Accept: 'application/json',
    ...(data && { 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(data) })
  };

  const includeAuth = requestOptions?.includeAuth ?? true;
  if (includeAuth) {
    if (!options.token) {
      throw new Error('NOT_AUTHENTICATED');
    }
    headers.Authorization = `Bearer ${options.token}`;
  }

  return new Promise<T>((resolve, reject) => {
    const req = lib.request(
      {
        method,
        hostname: url.hostname,
        port: url.port || (isHttps ? 443 : 80),
        path: `${url.pathname}${url.search}`,
        headers
      },
      (res) => {
        const chunks: Buffer[] = [];
        res.on('data', (chunk) => chunks.push(chunk));
        res.on('end', () => {
          const bodyText = Buffer.concat(chunks).toString('utf8');

          if (res.statusCode && res.statusCode >= 200 && res.statusCode < 300) {
            if (!bodyText) {
              resolve(undefined as T);
              return;
            }
            try {
              resolve(JSON.parse(bodyText));
            } catch (error) {
              reject(error);
            }
            return;
          }

          if (res.statusCode === 401 || res.statusCode === 403) {
            const httpError: HttpError = new Error('VM Manager authentication failed');
            httpError.status = res.statusCode;
            httpError.body = bodyText;
            reject(httpError);
            return;
          }

          const httpError: HttpError = new Error(
            `Request failed (${res.statusCode}): ${bodyText || res.statusMessage || 'Unknown error'}`
          );
          httpError.status = res.statusCode;
          httpError.body = bodyText;
          reject(httpError);
        });
      }
    );

    req.on('error', (error) => {
      if (error && typeof error === 'object' && 'code' in error) {
        const code = String((error as NodeJS.ErrnoException).code);
        if (code === 'ECONNREFUSED') {
          reject(new Error(`VM Manager API not responding`));
          return;
        }
        if (code === 'ETIMEDOUT') {
          reject(new Error(`Request to ${url.origin} timed out`));
          return;
        }
      }
      reject(error);
    });

    req.setTimeout(options.timeoutMs ?? 5000, () => req.destroy(new Error(`Request timed out`)));
    if (data) req.write(data);
    req.end();
  });
}
