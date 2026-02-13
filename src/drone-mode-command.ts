import {
  DroneMode,
  DroneModeApiError,
  DroneModeApiErrorCode,
  DroneModeResponse,
  normalizeDroneMode
} from './drone-mode-api';

export interface DroneModeCommandDependencies {
  resolveVmId: () => string | undefined;
  pickMode: () => Promise<DroneMode | undefined>;
  runWithProgress: <T>(title: string, task: () => Promise<T>) => Promise<T>;
  setDroneMode: (vmId: string, mode: DroneMode) => Promise<DroneModeResponse>;
  isTelemetryTunnelActive: () => boolean;
  onModeApplied?: (details: {
    appliedMode: DroneMode | 'UNKNOWN';
    mavrosStatus: string;
    px4Status: string;
    warnings: string[];
  }) => void;
  onSitlModeWithTunnel?: () => Promise<void> | void;
  onRealModeWithoutTunnel?: () => Promise<void> | void;
  showInfo: (message: string) => void;
  showError: (message: string) => void;
  log: (message: string) => void;
}

export async function executeDroneModeCommand(
  deps: DroneModeCommandDependencies,
  forcedMode?: DroneMode
): Promise<void> {
  const vmId = deps.resolveVmId();
  if (!vmId) {
    deps.showError('No VM is selected. Start/select a VM first, then retry.');
    return;
  }

  const targetMode = forcedMode ?? (await deps.pickMode());
  if (!targetMode) {
    return;
  }

  const title = `TensorFleet: Switching Drone Mode to ${targetMode}`;
  deps.log(`[DroneMode] vmId=${vmId} targetMode=${targetMode}`);

  try {
    const response = await deps.runWithProgress(title, async () => await deps.setDroneMode(vmId, targetMode));
    const appliedMode = normalizeAppliedMode(response, targetMode);
    const mavrosStatus = normalizeStatusText(response.mavrosStatus, response.mavrosActive, 'mavros');
    const px4Status = normalizeStatusText(response.px4Status, response.px4Active, 'px4');
    const warnings = normalizeWarnings(response.warnings);

    const lines = [
      `Drone mode applied: ${appliedMode}`,
      `MAVROS: ${mavrosStatus}`,
      `PX4: ${px4Status}`
    ];

    if (warnings.length > 0) {
      lines.push(`Warnings: ${warnings.join('; ')}`);
    } else {
      lines.push('Warnings: none');
    }

    if (appliedMode === 'REAL' && !deps.isTelemetryTunnelActive()) {
      lines.push('If needed, run TensorFleet: Connect Drone Telemetry.');
    }

    deps.log(
      `[DroneMode] success vmId=${vmId} applied=${appliedMode} mavros=${mavrosStatus} px4=${px4Status} warnings=${warnings.length}`
    );
    deps.onModeApplied?.({
      appliedMode,
      mavrosStatus,
      px4Status,
      warnings
    });
    deps.showInfo(lines.join('\n'));

    if (appliedMode === 'SITL' && deps.isTelemetryTunnelActive() && deps.onSitlModeWithTunnel) {
      try {
        await deps.onSitlModeWithTunnel();
      } catch (hookError) {
        const hookMessage = hookError instanceof Error ? hookError.message : String(hookError);
        deps.log(`[DroneMode] sitl tunnel-disconnect hook failed: ${hookMessage}`);
      }
    }

    if (appliedMode === 'REAL' && !deps.isTelemetryTunnelActive() && deps.onRealModeWithoutTunnel) {
      try {
        await deps.onRealModeWithoutTunnel();
      } catch (hookError) {
        const hookMessage = hookError instanceof Error ? hookError.message : String(hookError);
        deps.log(`[DroneMode] post-switch tunnel hook failed: ${hookMessage}`);
      }
    }
  } catch (error) {
    const message = formatDroneModeError(error);
    deps.log(`[DroneMode] failure vmId=${vmId} targetMode=${targetMode} error=${message}`);
    deps.showError(message);
  }
}

function normalizeAppliedMode(response: DroneModeResponse, fallback: DroneMode): DroneMode | 'UNKNOWN' {
  const preferred = normalizeDroneMode(response.appliedMode);
  if (preferred !== 'UNKNOWN') return preferred;
  const requested = normalizeDroneMode(response.requestedMode);
  if (requested !== 'UNKNOWN') return requested;
  return fallback;
}

function normalizeWarnings(warnings?: string[]): string[] {
  if (!Array.isArray(warnings)) return [];
  return warnings.filter((warning) => typeof warning === 'string' && warning.trim().length > 0);
}

function normalizeStatusText(
  value: string | undefined,
  active: boolean | undefined,
  runtime: 'mavros' | 'px4'
): string {
  const normalized = value?.trim();
  if (normalized && normalized.length > 0) {
    return normalized;
  }
  if (typeof active === 'boolean') {
    if (runtime === 'mavros') {
      return active ? 'active' : 'inactive';
    }
    return active ? 'running' : 'stopped';
  }
  return 'unknown';
}

function formatDroneModeError(error: unknown): string {
  if (!(error instanceof DroneModeApiError)) {
    const fallback = error instanceof Error ? error.message : String(error);
    return `Failed to switch drone mode: ${fallback}. Suggestion: verify VM Manager connectivity and retry.`;
  }

  if (error.isTimeout || error.status === 504 || error.code === 'COMMAND_TIMEOUT') {
    return 'Drone mode switch timed out after 30s. Suggestion: verify VM is running and retry.';
  }

  if (error.status === 401 || error.status === 403) {
    return 'Not authorized to switch drone mode (401/403). Suggestion: run TensorFleet: Login and retry.';
  }

  if (error.status === 404) {
    return 'VM not found or not owned by current user. Suggestion: refresh VM selection and retry.';
  }

  if (isKnownCommandCode(error.code)) {
    return `Drone mode switch failed (${error.code}): ${error.message}. Suggestion: inspect VM logs and retry.`;
  }

  return `Failed to switch drone mode: ${error.message}. Suggestion: check VM Manager/API availability and retry.`;
}

function isKnownCommandCode(code: string | undefined): code is DroneModeApiErrorCode {
  return code === 'COMMAND_FAILED' || code === 'COMMAND_START_FAILED' || code === 'COMMAND_TIMEOUT';
}
