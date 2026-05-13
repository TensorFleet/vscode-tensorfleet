import * as vscode from 'vscode';
import * as auth from './auth';
import { getVmManagerUrl } from './regions';
import { UnifiedStatusCoordinator } from './unified-status';
import type { TelemetryService } from './telemetry';
import {
  DEFAULT_CONFIG_ID,
  VM_CONFIGS,
  getDefaultConfig,
  getConfigById,
  detectConfigFromWorkspace,
  isValidConfigId,
  type VMConfig,
} from 'tensorfleet-auth/vm-config';
import {
  createVmSnapshot,
  fetchVmSnapshot,
  formatError,
  getGazeboSelection,
  isAuthError,
  listGazeboPresets,
  resetGazeboSelection,
  restartVm as requestRestartVm,
  setGazeboPreset,
  startVm as requestStartVm,
  stopVm as requestStopVm,
  type ConnectionState,
  type GazeboPreset,
  type GazeboSelection,
  type VmManagerClientOptions,
  type VmSnapshot,
  type VmState,
} from 'tensorfleet-auth/vm-manager-client';

interface VmQuickPickItem extends vscode.QuickPickItem {
  action?: () => Promise<void> | void | Thenable<void>;
}

export type { VMConfig };

export interface VMConfigOption extends vscode.QuickPickItem {
  config: VMConfig;
}

export type { GazeboPreset, GazeboSelection };

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
    this.currentSnapshot = createVmSnapshot({ connection: 'disconnected', vmState: 'unknown' });

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
          createVmSnapshot({
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
        createVmSnapshot({
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
    const token = await this.getAuthToken();
    return fetchVmSnapshot(this.getClientOptions(token));
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

      await requestStartVm(await this.getClientOptionsWithToken(), configToUse);
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
      await requestStopVm(await this.getClientOptionsWithToken());
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
    return listGazeboPresets(await this.getClientOptionsWithToken());
  }

  async getGazeboSelection(): Promise<GazeboSelection> {
    return getGazeboSelection(await this.getClientOptionsWithToken());
  }

  async setGazeboPreset(preset: string): Promise<string> {
    const trimmedPreset = preset.trim();
    const message = await setGazeboPreset(await this.getClientOptionsWithToken(), trimmedPreset);
    this.outputChannel.appendLine(`[VM Manager] Gazebo preset switch requested: ${trimmedPreset}`);
    return message;
  }

  async resetGazeboSelection(): Promise<string> {
    const message = await resetGazeboSelection(await this.getClientOptionsWithToken());
    this.outputChannel.appendLine('[VM Manager] Gazebo selection reset requested');
    return message;
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
      await requestRestartVm(await this.getClientOptionsWithToken(), configToUse);
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

  // ========== Helpers ==========

  private setOptimisticState(vmState: VmState) {
    this.applySnapshot({ ...this.currentSnapshot, vmState, error: undefined, timestamp: Date.now() });
  }

  private formatError(error: unknown): string {
    return formatError(error);
  }

  private isValidConfigId(configId?: string): configId is string {
    return isValidConfigId(configId);
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

    return detectConfigFromWorkspace(folders, async (filePath) => {
      try {
        const markerUri = vscode.Uri.file(filePath);
        const buf = await vscode.workspace.fs.readFile(markerUri);
        return Buffer.from(buf).toString('utf8').trim();
      } catch (error) {
        const code = (error as Partial<vscode.FileSystemError>)?.code;
        if (code !== 'FileNotFound' && code !== 'ENOENT') {
          this.outputChannel.appendLine(`[VM Manager] Failed to read .tensorfleet metadata: ${this.formatError(error)}`);
        }
        return null;
      }
    });
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

  private getClientOptions(token?: string): VmManagerClientOptions {
    return {
      baseUrl: this.getApiBaseUrl(),
      token
    };
  }

  private async getClientOptionsWithToken(): Promise<VmManagerClientOptions> {
    const token = await this.getAuthToken();
    return this.getClientOptions(token);
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

  private isAuthError(error: unknown): boolean {
    return isAuthError(error);
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
    const configId = this.getLastUsedConfigId() || DEFAULT_CONFIG_ID;
    const config = getConfigById(configId) || getDefaultConfig();

    if (!config) {
      // Fallback to first available config if default is missing
      const firstConfigId = Object.keys(VM_CONFIGS)[0];
      const fallbackConfig = VM_CONFIGS[firstConfigId];
      if (fallbackConfig) {
        this.outputChannel.appendLine(`[VM Manager] Warning: Default config '${DEFAULT_CONFIG_ID}' not found, using '${firstConfigId}'`);
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
    if (isValidConfigId(configId)) {
      this.setLastUsedConfigId(configId);
    }
  }
}
