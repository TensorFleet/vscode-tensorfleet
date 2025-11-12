import * as vscode from 'vscode';
import * as http from 'http';
import * as https from 'https';

// Core state types
type ConnectionState = 'connected' | 'disconnected';
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
  ip_address?: string;
  updated_at?: string;
}

interface VmInfoResponse extends VmStatusResponse {
  created_at?: string;
  uptime_seconds?: number | null;
  provider?: string;
  region?: string;
}

interface ApiHealthResponse {
  active_vms: number;
  status: string;
  time: string;
}

// Internal state representation
interface VmSnapshot {
  connection: ConnectionState;
  vmState: VmState;
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

export class VMManagerIntegration implements vscode.Disposable {
  private readonly statusBarItem: vscode.StatusBarItem;
  private readonly outputChannel: vscode.OutputChannel;
  private pollTimer: NodeJS.Timeout | null = null;
  private currentSnapshot: VmSnapshot;
  private lastNotifiedState: VmState | null = null;
  private pollInterval = 30_000;
  private userInitiatedAction: 'start' | 'stop' | null = null;

  private static readonly NORMAL_POLL_MS = 30_000;
  private static readonly FAST_POLL_MS = 5_000;

  constructor(context: vscode.ExtensionContext) {
    this.outputChannel = vscode.window.createOutputChannel('TensorFleet VM Manager');
    this.statusBarItem = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Right, 98);
    this.statusBarItem.name = 'TensorFleet VM';
    this.statusBarItem.command = 'tensorfleet.showVMManagerMenu';
    this.statusBarItem.show();

    // Initialize with unknown state
    this.currentSnapshot = this.createSnapshot({ connection: 'disconnected', vmState: 'unknown' });

    context.subscriptions.push(
      this,
      this.statusBarItem,
      this.outputChannel,
      vscode.workspace.onDidChangeConfiguration((event) => {
        if (
          event.affectsConfiguration('tensorfleet.vmManager.apiBaseUrl') ||
          event.affectsConfiguration('tensorfleet.vmManager.authToken')
        ) {
          void this.refresh(true);
        }
      })
    );
  }

  initialize() {
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
      void vscode.window.showErrorMessage(`Action failed: ${message}`);
    }
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
      if (this.isNotFoundError(statusError)) {
        sawVmMissing = true;
      } else {
        this.outputChannel.appendLine(`[VM Manager] Status fetch failed: ${this.formatError(statusError)}`);
      }
    }

    let info: VmInfoResponse | undefined;
    try {
      info = await this.apiRequest<VmInfoResponse>('GET', '/vms/self/info');
    } catch (infoError) {
      if (this.isNotFoundError(infoError)) {
        sawVmMissing = true;
      } else {
        this.outputChannel.appendLine(`[VM Manager] Info fetch failed: ${this.formatError(infoError)}`);
      }
    }

    const resolvedState = vmState === 'unknown' && sawVmMissing ? 'pending' : vmState;

    return this.createSnapshot({
      connection: 'connected',
      vmState: resolvedState,
      ipAddress: info?.ip_address || status?.ip_address,
      provider: info?.provider,
      region: info?.region,
      uptimeSeconds: info?.uptime_seconds
    });
  }

  private async ensureApiHealthy(): Promise<ApiHealthResponse> {
    return this.apiRequest<ApiHealthResponse>('GET', '/health', undefined, { includeAuth: false });
  }

  private applySnapshot(snapshot: VmSnapshot) {
    const previousState = this.currentSnapshot.vmState;
    const previousConnection = this.currentSnapshot.connection;
    
    this.currentSnapshot = snapshot;
    this.updateStatusBar();
    this.handleStateChange(previousConnection, previousState);
    this.updatePollingSpeed(snapshot.vmState);
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

    // Don't notify for disconnected, unknown, or pending states
    if (connection === 'disconnected' || vmState === 'unknown' || vmState === 'pending') return;

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
    const { connection, vmState, ipAddress } = this.currentSnapshot;
    const ip = ipAddress ? ` (${ipAddress})` : '';

    if (connection === 'disconnected') {
      const lastState = this.getVmStateLabel(vmState);
      this.statusBarItem.text = lastState 
        ? `⚠️ API Disconnected · Last: ${lastState}${ip}` 
        : '⚠️ API Disconnected';
    } else {
      this.statusBarItem.text = this.getVmStateIcon(vmState) + ' ' + this.getVmStateLabel(vmState) + ip;
    }

    this.statusBarItem.tooltip = this.buildTooltip();
  }

  private buildTooltip(): string {
    const { connection, vmState, ipAddress, provider, region, uptimeSeconds, error, timestamp } = this.currentSnapshot;
    const lines: string[] = [];

    if (connection === 'disconnected') {
      lines.push('⚠️ Cannot reach VM Manager API');
      lines.push(`Last known state: ${vmState}`);
      if (ipAddress) lines.push(`Last known IP: ${ipAddress}`);
    } else {
      lines.push('✓ Connected to VM Manager API');
      lines.push(`VM State: ${vmState}`);
      if (vmState === 'pending') {
        lines.push('⚠️ VM exists but has not started yet');
      }
    }

    if (ipAddress) lines.push(`IP: ${ipAddress}`);
    if (provider) lines.push(`Provider: ${provider}`);
    if (region) lines.push(`Region: ${region}`);
    
    const uptime = this.formatUptime(uptimeSeconds);
    if (uptime) lines.push(`Uptime: ${uptime}`);
    
    if (error) lines.push(`Error: ${error}`);
    lines.push(`API: ${this.getApiBaseUrl()}`);
    lines.push(`Updated: ${new Date(timestamp).toLocaleTimeString()}`);

    return lines.join('\n');
  }

  private buildMenuItems(): VmQuickPickItem[] {
    const { connection, vmState, error } = this.currentSnapshot;
    const items: VmQuickPickItem[] = [];

    if (connection === 'disconnected') {
      items.push(
        { label: '⚠️ Cannot reach VM Manager API', detail: error || 'Check network and API configuration' },
        { label: '🔄 Retry Connection', detail: 'Attempt to reconnect', action: () => this.refresh(false) },
        { 
          label: '⚙️ Configure API', 
          detail: 'Open settings', 
          action: () => vscode.commands.executeCommand('workbench.action.openSettings', 'tensorfleet.vmManager')
        }
      );
    } else {
      switch (vmState) {
        case 'running':
          items.push(
            { label: '⏹ Stop VM', detail: 'Shut down the VM', action: () => this.stopVm() }
          );
          break;

        case 'starting':
          items.push({ label: '$(sync~spin) VM is starting...', detail: 'Usually takes 30-60 seconds' });
          break;

        case 'stopping':
          items.push({ label: '$(sync~spin) VM is stopping...', detail: 'Usually takes 10-20 seconds' });
          break;

        case 'pending':
          items.push(
            { label: '🔵 VM not started', detail: 'VM has not booted yet' },
            { label: '▶ Start VM', detail: 'Attempt to create/start VM', action: () => this.startVm() }
          );
          break;

        case 'failed':
          items.push(
            { label: '❌ VM failed', detail: error || 'Check logs for details' },
            { label: '🔄 Retry Start', detail: 'Try starting again', action: () => this.startVm() }
          );
          break;

        case 'unknown':
          items.push(
            { label: '❓ VM status unclear', detail: 'VM may not exist yet' },
            { label: '▶ Start VM', detail: 'Attempt to create/start VM', action: () => this.startVm() }
          );
          break;

        case 'stopped':
          items.push({ label: '▶ Start VM', detail: 'Boot up your VM', action: () => this.startVm() });
          break;
      }
    }

    items.push({ label: '🔄 Refresh Status', detail: 'Check current state', action: () => this.refresh(false) });
    return items;
  }

  private getMenuPlaceholder(): string {
    const { connection, vmState } = this.currentSnapshot;
    if (connection === 'disconnected') return 'API Disconnected';
    
    switch (vmState) {
      case 'running': return 'VM is Running';
      case 'starting': return 'VM is Starting';
      case 'stopping': return 'VM is Stopping';
      case 'failed': return 'VM Failed';
      case 'stopped': return 'VM is Stopped';
      case 'pending': return 'VM Pending Start';
      default: return 'VM Status Unknown';
    }
  }

  // ========== VM Actions ==========

  private async startVm() {
    try {
      this.userInitiatedAction = 'start';
      this.setOptimisticState('starting');
      await this.apiRequest<{ status: string }>('POST', '/vms/self/start');
      await this.refresh(true);
      this.outputChannel.appendLine('[VM Manager] VM start initiated');
    } catch (error) {
      this.userInitiatedAction = null;
      await this.refresh(true);
      this.handleCommandError('start', error);
    }
  }

  private async stopVm() {
    try {
      this.userInitiatedAction = 'stop';
      this.setOptimisticState('stopping');
      await this.apiRequest<{ status: string }>('POST', '/vms/self/stop');
      await this.refresh(true);
      this.outputChannel.appendLine('[VM Manager] VM stop initiated');
    } catch (error) {
      this.userInitiatedAction = null;
      await this.refresh(true);
      this.handleCommandError('stop', error);
    }
  }

  private handleCommandError(action: string, error: unknown) {
    const message = this.formatError(error);
    this.outputChannel.appendLine(`[VM Manager] ${action} failed: ${message}`);

    const isConnectionIssue = /not responding|timed out|econnrefused/i.test(message);

    if (isConnectionIssue) {
      void vscode.window
        .showErrorMessage(`Cannot reach VM Manager: ${message}`, 'Configure', 'Logs')
        .then((choice) => {
          if (choice === 'Configure') {
            void vscode.commands.executeCommand('workbench.action.openSettings', 'tensorfleet.vmManager');
          } else if (choice === 'Logs') {
            this.outputChannel.show();
          }
        });
    } else {
      void vscode.window
        .showErrorMessage(`VM ${action} failed: ${message}`, 'Retry', 'Logs')
        .then((choice) => {
          if (choice === 'Retry') {
            void (action === 'start' ? this.startVm() : this.stopVm());
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
    options?: { includeAuth?: boolean }
  ): Promise<T> {
    const baseUrl = this.getApiBaseUrl();
    const url = new URL(endpoint.replace(/^\//, ''), baseUrl.endsWith('/') ? baseUrl : `${baseUrl}/`);
    const isHttps = url.protocol === 'https:';
    const lib = isHttps ? https : http;
    const data = body ? JSON.stringify(body) : undefined;

    const headers: http.OutgoingHttpHeaders = {
      Accept: 'application/json',
      ...(data && { 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(data) })
    };

    const includeAuth = options?.includeAuth ?? true;
    const token = this.getAuthToken();
    if (includeAuth && token) {
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
            reject(new Error(`VM Manager API not responding at ${url.origin}`));
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

  private getVmStateIcon(state: VmState): string {
    switch (state) {
      case 'running': return '🟢';
      case 'starting': return '🟡';
      case 'stopping': return '🟡';
      case 'failed': return '🔴';
      case 'stopped': return '⚫';
      case 'pending': return '🔵';
      default: return '❓';
    }
  }

  private getVmStateLabel(state: VmState): string {
    switch (state) {
      case 'running': return 'Running';
      case 'starting': return 'Starting...';
      case 'stopping': return 'Stopping...';
      case 'failed': return 'Failed';
      case 'stopped': return 'Stopped';
      case 'pending': return 'Pending...';
      default: return 'Unknown';
    }
  }

  private formatUptime(seconds?: number | null): string | undefined {
    if (seconds == null) return undefined;
    const total = Math.max(0, Math.floor(seconds));
    const h = Math.floor(total / 3600);
    const m = Math.floor((total % 3600) / 60);
    const s = total % 60;
    const parts: string[] = [];
    if (h > 0) parts.push(`${h}h`);
    if (m > 0) parts.push(`${m}m`);
    if (parts.length === 0 || s > 0) parts.push(`${s}s`);
    return parts.join(' ');
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

  private getApiBaseUrl(): string {
    const config = vscode.workspace.getConfiguration('tensorfleet');
    return config.get<string>('vmManager.apiBaseUrl', 'http://localhost:8080').trim() || 'http://localhost:8080';
  }

  private getAuthToken(): string | undefined {
    const config = vscode.workspace.getConfiguration('tensorfleet');
    return config.get<string>('vmManager.authToken')?.trim() || undefined;
  }

  private isNotFoundError(error: unknown): boolean {
    return !!(error && typeof error === 'object' && 'status' in error && (error as HttpError).status === 404);
  }
}
