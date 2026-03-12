import { describe, expect, test } from 'bun:test';
import { executeDroneModeCommand } from '../src/drone-mode-command';
import { DroneModeApiError, DroneModeResponse } from '../src/drone-mode-api';

type RunOptions = {
  vmId?: string;
  forcedMode?: 'REAL' | 'SITL';
  tunnelActive?: boolean;
  response?: DroneModeResponse;
  error?: unknown;
};

function createHarness(options: RunOptions) {
  const infoMessages: string[] = [];
  const errorMessages: string[] = [];
  const logs: string[] = [];
  const pickedModes: ('REAL' | 'SITL')[] = [];
  const setCalls: Array<{ vmId: string; mode: 'REAL' | 'SITL' }> = [];
  let realModeHookCalls = 0;
  let sitlModeHookCalls = 0;

  return {
    infoMessages,
    errorMessages,
    logs,
    pickedModes,
    setCalls,
    get realModeHookCalls() {
      return realModeHookCalls;
    },
    get sitlModeHookCalls() {
      return sitlModeHookCalls;
    },
    run: async () => {
      await executeDroneModeCommand(
        {
          resolveVmId: () => options.vmId,
          pickMode: async () => {
            const mode = options.forcedMode ?? 'SITL';
            pickedModes.push(mode);
            return mode;
          },
          runWithProgress: async (_title, task) => await task(),
          setDroneMode: async (vmId, mode) => {
            setCalls.push({ vmId, mode });
            if (options.error) {
              throw options.error;
            }
            return options.response ?? { appliedMode: mode };
          },
          isTelemetryTunnelActive: () => Boolean(options.tunnelActive),
          onSitlModeWithTunnel: async () => {
            sitlModeHookCalls += 1;
          },
          onRealModeWithoutTunnel: async () => {
            realModeHookCalls += 1;
          },
          showInfo: (message) => infoMessages.push(message),
          showError: (message) => errorMessages.push(message),
          log: (message) => logs.push(message)
        },
        options.forcedMode
      );
    }
  };
}

describe('executeDroneModeCommand', () => {
  test('success: SITL', async () => {
    const harness = createHarness({
      vmId: 'vm-1',
      forcedMode: 'SITL',
      response: {
        appliedMode: 'sitl',
        mavrosStatus: 'active',
        px4Status: 'running',
        warnings: []
      }
    });

    await harness.run();

    expect(harness.setCalls).toEqual([{ vmId: 'vm-1', mode: 'SITL' }]);
    expect(harness.infoMessages[0]).toContain('Drone mode applied: SITL');
    expect(harness.infoMessages[0]).toContain('MAVROS: active');
    expect(harness.infoMessages[0]).toContain('PX4: running');
    expect(harness.infoMessages[0]).not.toContain('Connect Drone Telemetry');
    expect(harness.realModeHookCalls).toBe(0);
    expect(harness.sitlModeHookCalls).toBe(0);
  });

  test('success: REAL shows telemetry hint when tunnel inactive', async () => {
    const harness = createHarness({
      vmId: 'vm-1',
      forcedMode: 'REAL',
      tunnelActive: false,
      response: {
        appliedMode: 'real',
        mavrosStatus: 'active',
        px4Status: 'running',
        warnings: ['MAVROS restart observed']
      }
    });

    await harness.run();

    expect(harness.setCalls).toEqual([{ vmId: 'vm-1', mode: 'REAL' }]);
    expect(harness.infoMessages[0]).toContain('Drone mode applied: REAL');
    expect(harness.infoMessages[0]).toContain('Warnings: MAVROS restart observed');
    expect(harness.infoMessages[0]).toContain('If needed, run TensorFleet: Connect Drone Telemetry.');
    expect(harness.realModeHookCalls).toBe(1);
    expect(harness.sitlModeHookCalls).toBe(0);
  });

  test('success: SITL disconnect hook runs when tunnel is active', async () => {
    const harness = createHarness({
      vmId: 'vm-1',
      forcedMode: 'SITL',
      tunnelActive: true,
      response: {
        appliedMode: 'sitl',
        mavrosStatus: 'active',
        px4Status: 'running',
        warnings: []
      }
    });

    await harness.run();

    expect(harness.sitlModeHookCalls).toBe(1);
    expect(harness.realModeHookCalls).toBe(0);
  });

  test('success: runtime booleans map to display statuses', async () => {
    const harness = createHarness({
      vmId: 'vm-1',
      forcedMode: 'REAL',
      response: {
        appliedMode: 'real',
        mavrosActive: true,
        px4Active: false
      }
    });

    await harness.run();

    expect(harness.infoMessages[0]).toContain('MAVROS: active');
    expect(harness.infoMessages[0]).toContain('PX4: stopped');
  });

  test('no selected VM shows fail-fast error', async () => {
    const harness = createHarness({
      forcedMode: 'SITL'
    });

    await harness.run();

    expect(harness.setCalls.length).toBe(0);
    expect(harness.errorMessages[0]).toContain('No VM is selected');
  });

  test('404 maps to ownership/not found message', async () => {
    const harness = createHarness({
      vmId: 'vm-1',
      forcedMode: 'REAL',
      error: new DroneModeApiError({
        status: 404,
        message: 'vm missing'
      })
    });

    await harness.run();

    expect(harness.errorMessages[0]).toContain('VM not found or not owned by current user.');
  });

  test('502 command start failure maps to actionable message', async () => {
    const harness = createHarness({
      vmId: 'vm-1',
      forcedMode: 'REAL',
      error: new DroneModeApiError({
        status: 502,
        code: 'COMMAND_START_FAILED',
        message: 'failed to launch script'
      })
    });

    await harness.run();

    expect(harness.errorMessages[0]).toContain('COMMAND_START_FAILED');
    expect(harness.errorMessages[0]).toContain('inspect VM logs and retry');
  });

  test('504 timeout maps to timeout-specific message', async () => {
    const harness = createHarness({
      vmId: 'vm-1',
      forcedMode: 'REAL',
      error: new DroneModeApiError({
        status: 504,
        code: 'COMMAND_TIMEOUT',
        message: 'timed out'
      })
    });

    await harness.run();

    expect(harness.errorMessages[0]).toContain('timed out after 30s');
  });
});
