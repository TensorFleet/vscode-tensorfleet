import * as http from 'http';
import * as https from 'https';

export type DroneMode = 'REAL' | 'SITL';
export type DroneModeLowercase = 'real' | 'sitl';
export type DroneFlightStack = 'px4' | 'ardupilot' | 'unknown';
export type DroneModeApiErrorCode = 'COMMAND_FAILED' | 'COMMAND_START_FAILED' | 'COMMAND_TIMEOUT';

export interface DroneModeRequest {
  mode: DroneModeLowercase;
}

export interface DroneModeBaseResponse {
  requestedMode?: string;
  appliedMode?: string;
  stack?: string;
  mavrosStatus?: string;
  px4Status?: string;
  ardupilotStatus?: string;
  mavrosActive?: boolean;
  px4Active?: boolean;
  ardupilotActive?: boolean;
  droneService?: string;
  droneActive?: boolean;
  warnings?: string[];
}
export type DroneModeResponse = DroneModeBaseResponse;

export interface GetDroneModeResponse extends DroneModeBaseResponse {}
export interface SetDroneModeResponse extends DroneModeBaseResponse {}

export interface DroneModeErrorBody {
  code?: DroneModeApiErrorCode | string;
  message?: string;
  details?: string[] | string;
}

export class DroneModeApiError extends Error {
  public readonly status?: number;
  public readonly code?: string;
  public readonly details: string[];
  public readonly isTimeout: boolean;
  public readonly responseBody?: string;

  constructor(params: {
    message: string;
    status?: number;
    code?: string;
    details?: string[];
    isTimeout?: boolean;
    responseBody?: string;
  }) {
    super(params.message);
    this.name = 'DroneModeApiError';
    this.status = params.status;
    this.code = params.code;
    this.details = params.details ?? [];
    this.isTimeout = params.isTimeout ?? false;
    this.responseBody = params.responseBody;
  }
}

export function normalizeDroneMode(value: string | undefined | null): DroneMode | 'UNKNOWN' {
  const normalized = (value ?? '').trim().toLowerCase();
  if (normalized === 'real') return 'REAL';
  if (normalized === 'sitl') return 'SITL';
  return 'UNKNOWN';
}

export function normalizeDroneFlightStack(value: string | undefined | null): DroneFlightStack {
  const normalized = (value ?? '').trim().toLowerCase();
  if (normalized === 'px4') return 'px4';
  if (normalized === 'ardupilot') return 'ardupilot';
  return 'unknown';
}

export function toDroneModeRequest(mode: DroneMode): DroneModeLowercase {
  return mode === 'REAL' ? 'real' : 'sitl';
}

export interface DroneModeApiClientOptions {
  baseUrl: string;
  token: string;
  timeoutMs?: number;
  log?: (line: string) => void;
}

interface HttpError extends Error {
  status?: number;
  body?: string;
}

export class DroneModeApiClient {
  private readonly baseUrl: string;
  private readonly token: string;
  private readonly timeoutMs: number;
  private readonly log: (line: string) => void;

  constructor(options: DroneModeApiClientOptions) {
    this.baseUrl = options.baseUrl.trim().replace(/\/+$/, '');
    this.token = options.token;
    this.timeoutMs = options.timeoutMs ?? 30_000;
    this.log = options.log ?? (() => {});
  }

  async getDroneMode(vmId: string): Promise<DroneModeResponse> {
    return await this.request<GetDroneModeResponse>('GET', `/vms/${encodeURIComponent(vmId)}/drone-mode`);
  }

  async setDroneMode(vmId: string, request: DroneModeRequest): Promise<SetDroneModeResponse> {
    return await this.request<SetDroneModeResponse>('POST', `/vms/${encodeURIComponent(vmId)}/drone-mode`, request);
  }

  private async request<T>(method: string, endpoint: string, body?: unknown): Promise<T> {
    const url = new URL(endpoint.replace(/^\//, ''), `${this.baseUrl}/`);
    const isHttps = url.protocol === 'https:';
    const lib = isHttps ? https : http;
    const payload = body ? JSON.stringify(body) : undefined;

    const headers: http.OutgoingHttpHeaders = {
      Accept: 'application/json',
      Authorization: `Bearer ${this.token}`,
      ...(payload && { 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(payload) })
    };

    this.log(`[DroneMode] Request ${method} ${url.pathname}`);

    try {
      return await new Promise<T>((resolve, reject) => {
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
              const status = res.statusCode;
              this.log(`[DroneMode] Response ${method} ${url.pathname} -> ${String(status)}`);

              if (status && status >= 200 && status < 300) {
                if (!bodyText) {
                  resolve({} as T);
                  return;
                }
                try {
                  resolve(JSON.parse(bodyText) as T);
                } catch {
                  reject(new DroneModeApiError({ message: 'Invalid JSON response from VM Manager', status, responseBody: bodyText }));
                }
                return;
              }

              const parsed = parseDroneModeErrorBody(bodyText);
              reject(
                new DroneModeApiError({
                  message: parsed.message || `Request failed with status ${String(status)}`,
                  status,
                  code: parsed.code,
                  details: parsed.details,
                  responseBody: bodyText
                })
              );
            });
          }
        );

        req.on('error', (error) => {
          const httpError = error as HttpError;
          if (httpError?.message?.toLowerCase().includes('timed out')) {
            reject(new DroneModeApiError({ message: `Request timed out after ${this.timeoutMs}ms`, isTimeout: true }));
            return;
          }
          reject(new DroneModeApiError({ message: error instanceof Error ? error.message : String(error) }));
        });

        req.setTimeout(this.timeoutMs, () => {
          req.destroy(new Error(`Request timed out after ${this.timeoutMs}ms`));
        });

        if (payload) {
          req.write(payload);
        }
        req.end();
      });
    } catch (error) {
      if (error instanceof DroneModeApiError) {
        throw error;
      }
      throw new DroneModeApiError({
        message: error instanceof Error ? error.message : String(error)
      });
    }
  }
}

function parseDroneModeErrorBody(bodyText: string): { code?: string; message?: string; details: string[] } {
  if (!bodyText) return { details: [] };

  try {
    const parsed = JSON.parse(bodyText) as {
      error?: DroneModeErrorBody;
      code?: string;
      message?: string;
      details?: string[] | string;
    };

    const scoped = parsed.error ?? parsed;
    const details = normalizeDetails(scoped.details);
    return {
      code: scoped.code,
      message: scoped.message,
      details
    };
  } catch {
    return { message: bodyText, details: [] };
  }
}

function normalizeDetails(details: string[] | string | undefined): string[] {
  if (Array.isArray(details)) {
    return details.filter((item) => typeof item === 'string' && item.trim().length > 0);
  }
  if (typeof details === 'string' && details.trim().length > 0) {
    return [details];
  }
  return [];
}
