import * as vscode from 'vscode';
import * as http from 'http';
import * as https from 'https';
import * as auth from './auth';
import { UnifiedStatusCoordinator } from './unified-status';

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
  private readonly context: vscode.ExtensionContext;
  private readonly statusBarItem: vscode.StatusBarItem;
  private readonly outputChannel: vscode.OutputChannel;
  private readonly unifiedCoordinator: UnifiedStatusCoordinator | null;
  private pollTimer: NodeJS.Timeout | null = null;
  private currentSnapshot: VmSnapshot;
  private lastNotifiedState: VmState | null = null;
  private pollInterval = 30_000;
  private userInitiatedAction: 'start' | 'stop' | null = null;

  private static readonly NORMAL_POLL_MS = 30_000;
  private static readonly FAST_POLL_MS = 5_000;

  constructor(context: vscode.ExtensionContext, unifiedCoordinator?: UnifiedStatusCoordinator | null) {
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
      
      // Check if it's an auth error
      if (this.isAuthError(error)) {
        // Token is invalid - clear it
        await auth.clearToken(this.context);
        this.applySnapshot(
          this.createSnapshot({
            connection: 'not_authenticated',
            vmState: 'unknown',
            error: 'Authentication expired - please login again'
          })
        );
        return;
      }
      
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
          // Token is invalid - clear it and return not_authenticated
          await auth.clearToken(this.context);
          return this.createSnapshot({
            connection: 'not_authenticated',
            vmState: 'unknown',
            error: 'Authentication expired - please login again'
          });
        }
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
        if (this.isAuthError(infoError)) {
          // Token is invalid - clear it and return not_authenticated
          await auth.clearToken(this.context);
          return this.createSnapshot({
            connection: 'not_authenticated',
            vmState: 'unknown',
            error: 'Authentication expired - please login again'
          });
        }
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
    } catch (error) {
      // Check if it's auth error
      if (this.isAuthError(error)) {
        // Token is invalid - clear it
        await auth.clearToken(this.context);
        return this.createSnapshot({
          connection: 'not_authenticated',
          vmState: 'unknown',
          error: 'Authentication expired - please login again'
        });
      }
      // Re-throw other errors to be handled by refresh()
      throw error;
    }
  }

  private async ensureApiHealthy(): Promise<ApiHealthResponse> {
    return this.apiRequest<ApiHealthResponse>('GET', '/vms/health', undefined, { includeAuth: false });
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
      // Primary action
      items.push(
        { 
          label: '🔑 Login', 
          action: () => vscode.commands.executeCommand('tensorfleet.login').then(() => this.refresh(false))
        }
      );
      
      // Separator
      items.push({
        label: '',
        kind: vscode.QuickPickItemKind.Separator
      });
      
      // Secondary actions (only actionable items)
      items.push(
        { 
          label: '⚙️ Configure API', 
          action: () => vscode.commands.executeCommand('workbench.action.openSettings', 'tensorfleet.vmManager')
        }
      );
      return items;
    } else if (connection === 'disconnected') {
      // Primary action
      items.push(
        { 
          label: '🔄 Retry Connection', 
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
      items.push(
        { 
          label: '⚙️ Configure API', 
          action: () => vscode.commands.executeCommand('workbench.action.openSettings', 'tensorfleet.vmManager')
        },
        { label: '🔄 Refresh Status', action: () => this.refresh(false) }
      );
    } else {
      const primaryActions: VmQuickPickItem[] = [];
      
      // Add informational status items for certain states (before primary actions)
      if (vmState === 'pending') {
        items.push({ label: '🔵 VM not started' });
      } else if (vmState === 'failed') {
        items.push({ label: '❌ VM failed', detail: error || 'Check logs for details' });
      } else if (vmState === 'unknown') {
        items.push({ label: '❓ VM status unclear', detail: 'VM may not exist yet' });
      } else if (vmState === 'starting') {
        items.push({ label: '$(sync~spin) VM is starting...', detail: 'Usually takes 30-60 seconds' });
      } else if (vmState === 'stopping') {
        items.push({ label: '$(sync~spin) VM is stopping...', detail: 'Usually takes 10-20 seconds' });
      }
      
      // Build primary actions based on VM state
      switch (vmState) {
        case 'running':
          primaryActions.push(
            { label: '⏹ Stop VM', action: () => this.stopVm() }
          );
          break;

        case 'stopped':
          primaryActions.push(
            { label: '▶ Start VM', action: () => this.startVm() }
          );
          break;

        case 'pending':
        case 'unknown':
          primaryActions.push(
            { label: '▶ Start VM', detail: 'May create VM if it doesn\'t exist', action: () => this.startVm() }
          );
          break;

        case 'failed':
          primaryActions.push(
            { label: '🔄 Retry Start', action: () => this.startVm() }
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
      items.push(
        { 
          label: '⚙️ Configure API', 
          action: () => vscode.commands.executeCommand('workbench.action.openSettings', 'tensorfleet.vmManager')
        },
        { label: '🔄 Refresh Status', action: () => this.refresh(false) }
      );
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
   * Public method to start VM
   */
  async startVm() {
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

  /**
   * Public method to stop VM
   */
  async stopVm() {
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

            // Handle auth errors (401/403) - clear token and throw special error
            if (res.statusCode === 401 || res.statusCode === 403) {
              auth.clearToken(this.context).catch((err) => {
                this.outputChannel.appendLine(`[VM Manager] Failed to clear expired token: ${this.formatError(err)}`);
              });
              const httpError: HttpError = new Error('Authentication expired - please login again');
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
}
