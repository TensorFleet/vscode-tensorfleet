import * as vscode from 'vscode';
import * as http from 'http';
import * as https from 'https';
import * as auth from './auth';
import { getVmManagerUrl } from './regions';
import { UnifiedStatusCoordinator } from './unified-status';
import type { TelemetryService } from './telemetry';

// Core state types (imported from unified-status, keeping local types for backward compatibility)
// Note: These types are now imported from unified-status.ts, but keeping local definitions
// for internal use to avoid breaking changes
type ConnectionState = 'connected' | 'disconnected' | 'not_authenticated';
type VmState =
  | 'unknown'
  | 'stopped'
  | 'starting'
  | 'running'
  | 'stopping'
  | 'failed'
  | 'pending';

// API response types
interface VmStatusResponse {
  status: string;
  vm_id?: string;
  ip_address?: string;
  updated_at?: string;
  vmId?: string;
}

interface VmInfoResponse extends VmStatusResponse {
  id?: string;
  created_at?: string;
  uptime_seconds?: number | null;
  provider?: string;
  region?: string;
}

interface ApiHealthResponse {
  status: string;
  time: string;
}

// Internal state representation
interface VmSnapshot {
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

interface VmQuickPickItem extends vscode.QuickPickItem {
  action?: () => Promise<void> | void | Thenable<void>;
}

interface HttpError extends Error {
  status?: number;
  body?: string;
}

// VM Configuration types
export interface VMConfig {
  id: string;
  name: string;
  description: string;
  sim_config: {
    config_version: string;
    world_components?: string;
    [key: string]: any;
  };
}

export interface VMConfigOption extends vscode.QuickPickItem {
  config: VMConfig;
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

export class VMManagerIntegration implements vscode.Disposable {
  private readonly context: vscode.ExtensionContext;
  private readonly statusBarItem: vscode.StatusBarItem;
  private readonly outputChannel: vscode.OutputChannel;
  private readonly unifiedCoordinator: UnifiedStatusCoordinator | null;
  private pollTimer: NodeJS.Timeout | null = null;
  private currentSnapshot: VmSnapshot;
  private lastNotifiedState: VmState | null = null;
  private pollInterval = 30_000;
  private userInitiatedAction: 'start' | 'stop' | null = null;

  public get snapshot(): VmSnapshot {
    return this.currentSnapshot;
  }

  private static readonly NORMAL_POLL_MS = 30_000;
  private static readonly FAST_POLL_MS = 5_000;

  // Static VM configuration constants
  public static readonly VM_CONFIGS: Record<string, VMConfig> = {
    'px4': {
      id: 'px4',
      name: 'PX4 Autopilot',
      description: 'PX4 flight stack with Gazebo simulation',
      sim_config: {
        config_version: "0.0.1",
        world_components: "static_obstacles_01;static_ground",
        gazebo_px4_enabled: "true"
      }
    },
    'ardupilot': {
      id: 'ardupilot',
      name: 'ArduPilot',
      description: 'ArduPilot flight stack with Gazebo simulation',
      sim_config: {
        config_version: "0.0.1",
        world_components: "static_obstacles_01;static_ground",
        gazebo_ardupilot_enabled: "true"
      }
    },
    'simple_robot': {
      id: 'simple_robot',
      name: 'Simple Robot',
      description: 'Basic ground robot with Gazebo simulation',
      sim_config: {
        config_version: "0.0.1",
        world_components: "static_obstacles_01;static_ground",
        simple_robot_enabled: "true"
      }
    },
    'turtlebot4': {
      id: 'turtlebot4',
      name: 'TurtleBot4',
      description: 'TurtleBot4 mobile robot with Gazebo simulation',
      sim_config: {
        config_version: "0.0.1",
        world_components: "static_bodies_01;turtlebot4_include",
        gazebo_turtlebot4_enabled: "true"
      }
    },
    'lerobot': {
      id: 'lerobot',
      name: 'Lerobot arm',
      description: 'Basic robotics arm simulation',
      sim_config: {
        config_version: "0.0.1",
        world_components: "lerobot/lerobot_world_01;static_ground",
        gazebo_lerobot_enabled: "true"
      }
    },
  };

  // Default config ID
  private static readonly DEFAULT_CONFIG_ID = 'simple_robot';
  private static readonly TEMPLATE_TO_CONFIG_ID: Record<string, string> = {
    'drone-js': 'px4',
    'robotic-js': 'simple_robot',
    'robotic': 'simple_robot',
    'lerobot-arm': 'lerobot'
  };

  constructor(context: vscode.ExtensionContext, unifiedCoordinator?: UnifiedStatusCoordinator | null, private readonly telemetry?: TelemetryService | null) {
    this.context = context;
    this.unifiedCoordinator = unifiedCoordinator || null;
    this.outputChannel = vscode.window.createOutputChannel('TensorFleet VM Manager');


    // Keep status bar item for backward compatibility, but hide it (unified coordinator shows it)
    this.statusBarItem = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Right, 98);
    this.statusBarItem.name = 'TensorFleet VM';
    this.statusBarItem.command = 'tensorfleet.showVMManagerMenu';
    this.statusBarItem.hide(); // Hide - unified coordinator handles display

    // Initialize with unknown state
    this.currentSnapshot = this.createSnapshot({ connection: 'disconnected', vmState: 'unknown' });

    context.subscriptions.push(
      this,
      this.statusBarItem,
      this.outputChannel,
      vscode.workspace.onDidChangeConfiguration((event) => {
        if (event.affectsConfiguration('tensorfleet.vmManager.apiBaseUrl')) {
          void this.refresh(true);
        }
      })
    );
  }

  private trackVmEvent(eventName: string, properties?: Record<string, string>) {
    this.telemetry?.trackEvent(eventName, properties);
  }

  private captureVmError(error: unknown, properties?: Record<string, string>) {
    this.telemetry?.captureError(error, properties);
  }

  initialize() {
    void this.ensureConfigFromWorkspace();
    void this.refresh(true);
    this.startPolling();
  }

  dispose() {
    this.stopPolling();
  }

  async showVmActions() {
    const items = this.buildMenuItems();
    const selection = await vscode.window.showQuickPick(items, {
      placeHolder: this.getMenuPlaceholder(),
      ignoreFocusOut: true
    });

    try {
      await selection?.action?.();
    } catch (error) {
      const message = this.formatError(error);
      this.outputChannel.appendLine(`[VM Manager] Action failed: ${message}`);
      this.captureVmError(error, { source: 'vm.action' });
      void vscode.window.showErrorMessage(`Action failed: ${message}`);
    }
  }

  /**
   * Public method to trigger a refresh of VM status
   * Useful for external triggers like auth state changes
   */
  refreshStatus(silent = true) {
    void this.refresh(silent);
  }

  // ========== Polling ==========

  private startPolling() {
    this.stopPolling();
    this.pollTimer = setInterval(() => void this.refresh(true), this.pollInterval);
  }

  private stopPolling() {
    if (this.pollTimer) {
      clearInterval(this.pollTimer);
      this.pollTimer = null;
    }
  }

  private updatePollingSpeed(vmState: VmState) {
    const newInterval = (vmState === 'starting' || vmState === 'stopping')
      ? VMManagerIntegration.FAST_POLL_MS
      : VMManagerIntegration.NORMAL_POLL_MS;

    if (newInterval !== this.pollInterval) {
      this.pollInterval = newInterval;
      this.startPolling();
    }
  }

  // ========== State Management ==========

  private async refresh(silent: boolean) {
    try {
      const snapshot = await this.fetchSnapshot();
      this.applySnapshot(snapshot);
    } catch (error) {
      const message = this.formatError(error);
      this.outputChannel.appendLine(`[VM Manager] Refresh failed: ${message}`);

      // Check if it's an auth error scoped to VM Manager
      if (this.isAuthError(error)) {
        this.applySnapshot(
          this.createSnapshot({
            connection: 'not_authenticated',
            vmState: 'unknown',
            error: 'VM Manager authentication failed - check settings or login again'
          })
        );
        return;
      }
      this.captureVmError(error, { source: 'vm.refresh' });

      // Mark as disconnected but preserve last known VM state
      this.applySnapshot(
        this.createSnapshot({
          connection: 'disconnected',
          vmState: this.currentSnapshot.vmState,
          error: message
        })
      );

      if (!silent) {
        void vscode.window.showWarningMessage(`Cannot reach VM Manager: ${message}`);
      }
    }
  }

  private async fetchSnapshot(): Promise<VmSnapshot> {
    // Check auth first - no token means not authenticated
    const token = await this.getAuthToken();
    if (!token) {
      return this.createSnapshot({
        connection: 'not_authenticated',
        vmState: 'unknown',
        error: 'Please login to access VM Manager'
      });
    }

    try {
      await this.ensureApiHealthy();

      let status: VmStatusResponse | undefined;
      let vmState: VmState = 'unknown';
      let sawVmMissing = false;

      try {
        status = await this.apiRequest<VmStatusResponse>('GET', '/vms/self/status');
        if (status) {
          vmState = this.parseVmState(status.status);
        }
      } catch (statusError) {
        if (this.isAuthError(statusError)) {
          return this.createSnapshot({
            connection: 'not_authenticated',
            vmState: 'unknown',
            error: 'VM Manager authentication failed - check settings or login again'
          });
        }
        if (this.isNotFoundError(statusError)) {
          sawVmMissing = true;
        } else {
          this.outputChannel.appendLine(`[VM Manager] Status fetch failed: ${this.formatError(statusError)}`);
          this.captureVmError(statusError, { source: 'vm.status_fetch' });
        }
      }

      let info: VmInfoResponse | undefined;
      try {
        info = await this.apiRequest<VmInfoResponse>('GET', '/vms/self/info');
      } catch (infoError) {
        if (this.isAuthError(infoError)) {
          return this.createSnapshot({
            connection: 'not_authenticated',
            vmState: 'unknown',
            error: 'VM Manager authentication failed - check settings or login again'
          });
        }
        if (this.isNotFoundError(infoError)) {
          sawVmMissing = true;
        } else {
          this.outputChannel.appendLine(`[VM Manager] Info fetch failed: ${this.formatError(infoError)}`);
          this.captureVmError(infoError, { source: 'vm.info_fetch' });
        }
      }

      const resolvedState = vmState === 'unknown' && sawVmMissing ? 'pending' : vmState;

      return this.createSnapshot({
        connection: 'connected',
        vmState: resolvedState,
        nodeId: info?.id ?? status?.vm_id,
        ipAddress: info?.ip_address || status?.ip_address,
        provider: info?.provider,
        region: info?.region,
        uptimeSeconds: info?.uptime_seconds
      });
    } catch (error) {
      // Check if it's auth error
      if (this.isAuthError(error)) {
        return this.createSnapshot({
          connection: 'not_authenticated',
          vmState: 'unknown',
          error: 'VM Manager authentication failed - check settings or login again'
        });
      }
      // Re-throw other errors to be handled by refresh()
      throw error;
    }
  }

  private async ensureApiHealthy(): Promise<ApiHealthResponse> {
    try {
      return await this.apiRequest<ApiHealthResponse>('GET', '/vms/health', undefined, { includeAuth: false });
    } catch (error) {
      this.captureVmError(error, { source: 'vm.health_check' });
      throw error;
    }
  }

  private applySnapshot(snapshot: VmSnapshot) {
    const previousState = this.currentSnapshot.vmState;
    const previousConnection = this.currentSnapshot.connection;

    this.currentSnapshot = snapshot;
    this.updateStatusBar();
    this.handleStateChange(previousConnection, previousState);
    this.updatePollingSpeed(snapshot.vmState);

    // Update unified coordinator
    if (this.unifiedCoordinator) {
      this.unifiedCoordinator.updateVmState({
        connection: snapshot.connection,
        vmState: snapshot.vmState,
        nodeId: snapshot.nodeId,
        ipAddress: snapshot.ipAddress,
        provider: snapshot.provider,
        region: snapshot.region,
        uptimeSeconds: snapshot.uptimeSeconds,
        error: snapshot.error
      });
    }
  }

  private handleStateChange(previousConnection: ConnectionState, previousState: VmState) {
    const { connection, vmState } = this.currentSnapshot;

    // Connection state changed
    if (previousConnection !== connection) {
      this.outputChannel.appendLine(`[VM Manager] Connection: ${previousConnection} → ${connection}`);
      this.lastNotifiedState = null;
      this.userInitiatedAction = null;
      return;
    }

    // No VM state change
    if (previousState === vmState) return;

    this.outputChannel.appendLine(`[VM Manager] State change: ${previousState} → ${vmState} (userAction: ${this.userInitiatedAction})`);

    // Don't notify for disconnected, not_authenticated, unknown, or pending states
    if (connection === 'disconnected' || connection === 'not_authenticated' || vmState === 'unknown' || vmState === 'pending') return;

    // Don't notify during transitions
    if (vmState === 'starting' || vmState === 'stopping') {
      this.lastNotifiedState = null;
      return;
    }

    // Handle user-initiated start: starting → running
    if (this.userInitiatedAction === 'start' && vmState === 'running') {
      this.userInitiatedAction = null;
      this.lastNotifiedState = vmState;
      this.outputChannel.appendLine('[VM Manager] VM started successfully (user-initiated)');
      return;
    }

    // Handle user-initiated stop: stopping → stopped
    if (this.userInitiatedAction === 'stop' && vmState === 'stopped') {
      this.userInitiatedAction = null;
      this.lastNotifiedState = vmState;
      this.outputChannel.appendLine('[VM Manager] VM stopped successfully (user-initiated)');
      return;
    }

    // Clear user action if we reach failed state
    if (vmState === 'failed') {
      this.userInitiatedAction = null;
    }

    // Don't re-notify for the same stable state
    if (this.lastNotifiedState === vmState) {
      this.outputChannel.appendLine(`[VM Manager] Skipping notification - already notified for ${vmState}`);
      return;
    }

    // Only show notifications for unexpected state changes
    this.outputChannel.appendLine(`[VM Manager] Showing notification for unexpected state: ${vmState}`);
    this.notifyStableState();
    this.lastNotifiedState = vmState;
  }

  private notifyStableState() {
    const { vmState } = this.currentSnapshot;

    // Only notify for states that weren't user-initiated
    switch (vmState) {
      case 'running': {
        // Only show success notification if this was an unexpected transition
        // (user will see the status bar update for their own actions)
        const actions = ['Menu'];
        void vscode.window
          .showInformationMessage('✅ VM is running and ready.', ...actions)
          .then((choice) => {
            if (choice === 'Menu') void this.showVmActions();
          });
        break;
      }
      case 'stopped': {
        // Only notify if the stop was unexpected (not user-initiated)
        void vscode.window
          .showWarningMessage('⚠️ VM has stopped unexpectedly.', 'Start VM', 'Menu')
          .then((choice) => {
            if (choice === 'Start VM') void this.startVm();
            if (choice === 'Menu') void this.showVmActions();
          });
        break;
      }
      case 'failed': {
        void vscode.window
          .showErrorMessage('❌ VM failed to start.', 'Retry', 'Menu', 'Logs')
          .then((choice) => {
            if (choice === 'Retry') void this.startVm();
            if (choice === 'Menu') void this.showVmActions();
            if (choice === 'Logs') this.outputChannel.show();
          });
        break;
      }
    }
  }

  // ========== UI ==========

  private updateStatusBar() {
    // Status bar is now handled by unified coordinator
    // Keep this method for backward compatibility but don't show the status bar
    // The unified coordinator will handle all status bar updates
  }

  private buildMenuItems(): VmQuickPickItem[] {
    const { connection, vmState, error } = this.currentSnapshot;
    const items: VmQuickPickItem[] = [];

    if (connection === 'not_authenticated') {
      items.push({
        label: '$(key) Login',
        action: () => vscode.commands.executeCommand('tensorfleet.login').then(() => this.refresh(false))
      });
      return items;
    } else if (connection === 'disconnected') {
      // Primary action
      items.push(
        {
          label: '$(refresh) Retry Connection',
          detail: error || 'Check network and API configuration',
          action: () => this.refresh(false)
        }
      );

      // Separator
      items.push({
        label: '',
        kind: vscode.QuickPickItemKind.Separator
      });

      // Secondary actions (only actionable items)
      items.push({
        label: '$(refresh) Refresh Status',
        action: () => this.refresh(false)
      });
    } else {
      const primaryActions: VmQuickPickItem[] = [];

      // Add informational status items for certain states (before primary actions)
      if (vmState === 'pending') {
        items.push({ label: '$(vm-outline) VM not started' });
      } else if (vmState === 'failed') {
        items.push({ label: '$(error) VM failed', detail: error || 'Check logs for details' });
      } else if (vmState === 'unknown') {
        items.push({ label: '$(question) VM status unclear', detail: 'VM may not exist yet' });
      } else if (vmState === 'starting') {
        items.push({ label: '$(vm-pending) VM is starting...', detail: 'Usually takes 30-60 seconds' });
      } else if (vmState === 'stopping') {
        items.push({ label: '$(debug-stop) VM is stopping...', detail: 'Usually takes 10-20 seconds' });
      }

      // Build primary actions based on VM state
      switch (vmState) {
        case 'running':
          primaryActions.push(
            { label: '$(debug-stop) Stop VM', action: () => this.stopVm() }
          );
          break;

        case 'stopped':
          primaryActions.push(
            { label: '$(debug-start) Start VM', action: () => this.startVm() }
          );
          break;

        case 'pending':
        case 'unknown':
          primaryActions.push(
            { label: '$(debug-start) Start VM', detail: 'May create VM if it doesn\'t exist', action: () => this.startVm() }
          );
          break;

        case 'failed':
          primaryActions.push(
            { label: '$(debug-restart) Retry Start', action: () => this.startVm() }
          );
          break;
      }

      // Add primary actions if any exist
      if (primaryActions.length > 0) {
        items.push(...primaryActions);

        // Add separator after primary actions
        items.push({
          label: '',
          kind: vscode.QuickPickItemKind.Separator
        });
      }

      // Add secondary actions (always shown)
      items.push({
        label: '$(refresh) Refresh Status',
        action: () => this.refresh(false)
      });
    }

    return items;
  }

  private getMenuPlaceholder(): string {
    const { connection, vmState } = this.currentSnapshot;
    if (connection === 'not_authenticated') return 'Not Logged In';
    if (connection === 'disconnected') return 'API Disconnected';

    switch (vmState) {
      case 'running': return 'Running';
      case 'starting': return 'Starting';
      case 'stopping': return 'Stopping';
      case 'failed': return 'Failed';
      case 'stopped': return 'Stopped';
      case 'pending': return 'Pending';
      default: return 'Checking...';
    }
  }

  // ========== VM Actions ==========

  /**
   * Public method to start VM with optional configuration
   */
  async startVm(configOrActionData?: VMConfig | any) {
    try {
      await this.ensureConfigFromWorkspace();
      this.userInitiatedAction = 'start';
      this.trackVmEvent('vm.start', { phase: 'start' });
      this.setOptimisticState('starting');

      // Determine config to use
      let configToUse: VMConfig;
      if (configOrActionData && typeof configOrActionData === 'object' && 'id' in configOrActionData && 'sim_config' in configOrActionData) {
        // It's a VMConfig object
        configToUse = configOrActionData as VMConfig;
        this.setLastUsedConfig(configToUse.id);
      } else if (configOrActionData) {
        // It's actionData (legacy support) - use last config but merge actionData
        configToUse = this.getLastUsedConfig();
        configToUse = {
          ...configToUse,
          sim_config: { ...configToUse.sim_config, ...configOrActionData }
        };
      } else {
        // No specific config provided - use last used config
        configToUse = this.getLastUsedConfig();
      }

      this.trackVmEvent('vm.start', { config: configToUse.id });

      const requestBody = {
        sim_config: configToUse.sim_config,
      };

      await this.apiRequest<{ status: string }>('POST', '/vms/self/start', requestBody);
      await this.refresh(true);
      this.outputChannel.appendLine(`[VM Manager] VM start initiated with config: ${configToUse.name}`);
      this.trackVmEvent('vm.start', { phase: 'success', config: configToUse.id });
    } catch (error) {
      this.userInitiatedAction = null;
      await this.refresh(true);
      this.captureVmError(error, { source: 'vm.start' });
      this.trackVmEvent('vm.start', { phase: 'error' });
      this.handleCommandError('start', error);
    }
  }

  /**
   * Public method to stop VM
   */
  async stopVm() {
    try {
      this.userInitiatedAction = 'stop';
      this.trackVmEvent('vm.stop', { phase: 'start' });
      this.setOptimisticState('stopping');
      await this.apiRequest<{ status: string }>('POST', '/vms/self/stop');
      await this.refresh(true);
      this.outputChannel.appendLine('[VM Manager] VM stop initiated');
      this.trackVmEvent('vm.stop', { phase: 'success' });
    } catch (error) {
      this.userInitiatedAction = null;
      await this.refresh(true);
      this.captureVmError(error, { source: 'vm.stop' });
      this.trackVmEvent('vm.stop', { phase: 'error' });
      this.handleCommandError('stop', error);
    }
  }

  async listGazeboPresets(): Promise<GazeboPreset[]> {
    const response = await this.apiRequest<{ presets?: GazeboPreset[] }>(
      'GET',
      '/vms/self/tensorfleet/api/v1/presets'
    );
    return response.presets ?? [];
  }

  async getGazeboSelection(): Promise<GazeboSelection> {
    return this.apiRequest<GazeboSelection>(
      'GET',
      '/vms/self/tensorfleet/api/v1/gazebo/world'
    );
  }

  async setGazeboPreset(preset: string): Promise<string> {
    const trimmedPreset = preset.trim();
    const response = await this.apiRequest<{ message?: string }>(
      'POST',
      '/vms/self/tensorfleet/api/v1/gazebo/world',
      { preset: trimmedPreset }
    );
    this.outputChannel.appendLine(`[VM Manager] Gazebo preset switch requested: ${trimmedPreset}`);
    return response.message ?? `Gazebo preset '${trimmedPreset}' switch requested`;
  }

  async resetGazeboSelection(): Promise<string> {
    const response = await this.apiRequest<{ message?: string }>(
      'POST',
      '/vms/self/tensorfleet/api/v1/gazebo/world',
      { reset: true }
    );
    this.outputChannel.appendLine('[VM Manager] Gazebo selection reset requested');
    return response.message ?? 'Gazebo selection reset requested';
  }

  private async restartVm() {
    try {
      await this.ensureConfigFromWorkspace();
      const { vmState } = this.currentSnapshot;

      // Set user action based on current state
      // If running/starting, we're stopping (will need to start again)
      // If stopped/failed, we're starting
      if (vmState === 'running' || vmState === 'starting') {
        this.userInitiatedAction = 'stop';
        this.setOptimisticState('stopping');
      } else if (vmState === 'stopped' || vmState === 'failed') {
        this.userInitiatedAction = 'start';
        this.setOptimisticState('starting');
      }

      // Use the same config logic as startVm
      const configToUse = this.getLastUsedConfig();
      const requestBody = {
        sim_config: configToUse.sim_config,
      };

      await this.apiRequest<{ status: string; message?: string }>('POST', '/vms/self/restart', requestBody);
      await this.refresh(true);
      this.outputChannel.appendLine('[VM Manager] VM restart initiated');
    } catch (error) {
      this.userInitiatedAction = null;
      await this.refresh(true);
      this.handleCommandError('restart', error);
    }
  }

  private handleCommandError(action: string, error: unknown) {
    const message = this.formatError(error);
    this.outputChannel.appendLine(`[VM Manager] ${action} failed: ${message}`);

    // Check for authentication errors first
    if (message === 'NOT_AUTHENTICATED' || message.includes('Authentication expired') || this.isAuthError(error)) {
      void vscode.window
        .showWarningMessage('Please login to use VM Manager', 'Login', 'Cancel')
        .then((choice) => {
          if (choice === 'Login') {
            void vscode.commands.executeCommand('tensorfleet.login').then(() => this.refresh(false));
          }
        });
      return;
    }

    const isConnectionIssue = /not responding|timed out|econnrefused/i.test(message);

    if (isConnectionIssue) {
      void vscode.window
        .showErrorMessage(`Cannot reach VM Manager: ${message}`, 'Logs')
        .then((choice) => {
          if (choice === 'Logs') {
            this.outputChannel.show();
          }
        });
    } else {
      void vscode.window
        .showErrorMessage(`VM ${action} failed: ${message}`, 'Retry', 'Logs')
        .then((choice) => {
          if (choice === 'Retry') {
            if (action === 'start') {
              void this.startVm(); // Use last config
            } else if (action === 'stop') {
              void this.stopVm();
            } else if (action === 'restart') {
              void this.restartVm();
            }
          } else if (choice === 'Logs') {
            this.outputChannel.show();
          }
        });
    }
  }

  // ========== HTTP Client ==========

  private async apiRequest<T>(
    method: string,
    endpoint: string,
    body?: any,
    options?: { includeAuth?: boolean; baseUrlOverride?: string; tokenOverride?: string }
  ): Promise<T> {
    const baseUrl = (options?.baseUrlOverride ?? this.getApiBaseUrl()).trim() || this.getApiBaseUrl();
    const url = new URL(endpoint.replace(/^\//, ''), baseUrl.endsWith('/') ? baseUrl : `${baseUrl}/`);
    const isHttps = url.protocol === 'https:';
    const lib = isHttps ? https : http;
    const data = body ? JSON.stringify(body) : undefined;

    const headers: http.OutgoingHttpHeaders = {
      Accept: 'application/json',
      ...(data && { 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(data) })
    };

    const includeAuth = options?.includeAuth ?? true;
    if (includeAuth) {
      const token = await this.getAuthToken();
      if (!token) {
        throw new Error('NOT_AUTHENTICATED');
      }
      headers.Authorization = `Bearer ${token}`;
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

            // Handle auth errors (401/403) scoped to VM Manager
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

      req.setTimeout(5000, () => req.destroy(new Error(`Request timed out`)));
      if (data) req.write(data);
      req.end();
    });
  }

  // ========== Helpers ==========

  private createSnapshot(params: Partial<VmSnapshot>): VmSnapshot {
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

  private setOptimisticState(vmState: VmState) {
    this.applySnapshot({ ...this.currentSnapshot, vmState, error: undefined, timestamp: Date.now() });
  }

  private parseVmState(status?: string): VmState {
    const normalized = (status ?? '').toLowerCase().trim();
    if (normalized.includes('running')) return 'running';
    if (normalized.includes('starting')) return 'starting';
    if (normalized.includes('stopping')) return 'stopping';
    if (normalized.includes('stopped')) return 'stopped';
    if (normalized.includes('fail') || normalized.includes('error')) return 'failed';
    return 'unknown';
  }


  private formatError(error: unknown): string {
    if (error instanceof Error) return error.message;
    if (typeof error === 'string') return error;
    try {
      return JSON.stringify(error);
    } catch {
      return 'Unknown error';
    }
  }

  private isValidConfigId(configId?: string): configId is string {
    return Boolean(configId && VMManagerIntegration.VM_CONFIGS[configId]);
  }

  private async ensureConfigFromWorkspace(): Promise<void> {
    const lastUsedConfigId = this.getLastUsedConfigId();
    if (this.isValidConfigId(lastUsedConfigId)) {
      return;
    }

    const detected = await this.detectConfigFromWorkspace();
    if (detected && this.isValidConfigId(detected.configId)) {
      this.setLastUsedConfigId(detected.configId);
      this.outputChannel.appendLine(
        `[VM Manager] Auto-selected VM config '${detected.configId}' from project template '${detected.template}'`
      );
    }
  }

  private async detectConfigFromWorkspace(): Promise<{ configId: string; template: string } | null> {
    const folders = vscode.workspace.workspaceFolders;
    if (!folders || folders.length === 0) {
      return null;
    }

    for (const folder of folders) {
      const markerUri = vscode.Uri.joinPath(folder.uri, '.tensorfleet');
      try {
        const buf = await vscode.workspace.fs.readFile(markerUri);
        const text = Buffer.from(buf).toString('utf8').trim();
        if (!text) {
          continue;
        }

        const metadata = JSON.parse(text) as { template?: string };
        const template = typeof metadata.template === 'string' ? metadata.template : '';
        const configId = VMManagerIntegration.TEMPLATE_TO_CONFIG_ID[template];
        if (configId) {
          return { configId, template };
        }
      } catch (error) {
        const code = (error as Partial<vscode.FileSystemError>)?.code;
        if (code === 'FileNotFound' || code === 'ENOENT') {
          continue;
        }
        this.outputChannel.appendLine(`[VM Manager] Failed to read .tensorfleet metadata: ${this.formatError(error)}`);
      }
    }

    return null;
  }

  private getApiBaseUrl(): string {
    const regionDefault = getVmManagerUrl().trim().replace(/\/+$/, '');
    const configuredUrl = vscode.workspace
      .getConfiguration('tensorfleet.vmManager')
      .get<string>('apiBaseUrl')
      ?.trim();

    if (configuredUrl && configuredUrl.length > 0) {
      return configuredUrl.replace(/\/+$/, '');
    }

    return regionDefault;
  }

  private async getAuthToken(): Promise<string | undefined> {
    // Get token from auth module (single source of truth)
    try {
      const authToken = await auth.getToken(this.context);
      return authToken;
    } catch (error) {
      this.outputChannel.appendLine(`[VM Manager] Failed to get auth token: ${this.formatError(error)}`);
      return undefined;
    }
  }

  private isNotFoundError(error: unknown): boolean {
    return !!(error && typeof error === 'object' && 'status' in error && (error as HttpError).status === 404);
  }

  private isAuthError(error: unknown): boolean {
    return !!(
      error &&
      typeof error === 'object' &&
      'status' in error &&
      ((error as HttpError).status === 401 || (error as HttpError).status === 403)
    );
  }

  // ========== Configuration Management ==========

  /**
   * Get the last used VM configuration ID from workspace state
   */
  private getLastUsedConfigId(): string | undefined {
    return this.context.workspaceState.get<string>('tensorfleet.vm.lastConfigId');
  }

  /**
   * Set the last used VM configuration ID in workspace state
   */
  private setLastUsedConfigId(configId: string): void {
    void this.context.workspaceState.update('tensorfleet.vm.lastConfigId', configId);
  }

  /**
   * Get the last used VM configuration
   */
  public getLastUsedConfig(): VMConfig {
    const configId = this.getLastUsedConfigId() || VMManagerIntegration.DEFAULT_CONFIG_ID;
    const config = VMManagerIntegration.VM_CONFIGS[configId] || VMManagerIntegration.VM_CONFIGS[VMManagerIntegration.DEFAULT_CONFIG_ID];

    if (!config) {
      // Fallback to first available config if default is missing
      const firstConfigId = Object.keys(VMManagerIntegration.VM_CONFIGS)[0];
      const fallbackConfig = VMManagerIntegration.VM_CONFIGS[firstConfigId];
      if (fallbackConfig) {
        this.outputChannel.appendLine(`[VM Manager] Warning: Default config '${VMManagerIntegration.DEFAULT_CONFIG_ID}' not found, using '${firstConfigId}'`);
        return fallbackConfig;
      }
      throw new Error('No VM configurations available');
    }

    return config;
  }

  /**
   * Set the last used VM configuration
   */
  public setLastUsedConfig(configId: string): void {
    if (VMManagerIntegration.VM_CONFIGS[configId]) {
      this.setLastUsedConfigId(configId);
    }
  }
}
