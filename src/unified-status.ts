/**
 * Unified Status Coordinator
 * 
 * Coordinates auth and VM Manager states into a single unified status bar.
 * Provides event system for state coordination and adaptive menu structure.
 */

import * as vscode from 'vscode';

// Auth state
export type AuthState = 'authenticated' | 'not_authenticated' | 'checking';

// VM connection state (from vm-manager)
export type ConnectionState = 'connected' | 'disconnected' | 'not_authenticated';

// VM state (from vm-manager)
export type VmState =
  | 'unknown'
  | 'stopped'
  | 'starting'
  | 'running'
  | 'stopping'
  | 'failed'
  | 'pending';

// Unified state snapshot
export interface UnifiedState {
  auth: AuthState;
  connection: ConnectionState;
  vmState: VmState;
  ipAddress?: string;
  provider?: string;
  region?: string;
  uptimeSeconds?: number | null;
  error?: string;
  timestamp: number;
}

// Event listeners
type StateChangeListener = (state: UnifiedState) => void;

export class UnifiedStatusCoordinator implements vscode.Disposable {
  private readonly statusBarItem: vscode.StatusBarItem;
  private currentState: UnifiedState;
  private listeners: Set<StateChangeListener> = new Set();

  constructor(_context: vscode.ExtensionContext) {
    this.statusBarItem = vscode.window.createStatusBarItem(
      vscode.StatusBarAlignment.Right,
      98 // High priority, right side
    );
    this.statusBarItem.name = 'TensorFleet';
    this.statusBarItem.command = 'tensorfleet.showUnifiedMenu';
    this.statusBarItem.show();

    // Initialize with default state
    this.currentState = {
      auth: 'checking',
      connection: 'disconnected',
      vmState: 'unknown',
      timestamp: Date.now()
    };

    _context.subscriptions.push(this, this.statusBarItem);
  }

  /**
   * Update auth state
   */
  updateAuth(auth: AuthState) {
    const previousState = { ...this.currentState };
    this.currentState = {
      ...this.currentState,
      auth,
      timestamp: Date.now()
    };
    this.notifyStateChange(previousState);
    this.updateStatusBar();
  }

  /**
   * Update VM state
   */
  updateVmState(params: {
    connection: ConnectionState;
    vmState: VmState;
    ipAddress?: string;
    provider?: string;
    region?: string;
    uptimeSeconds?: number | null;
    error?: string;
  }) {
    const previousState = { ...this.currentState };
    this.currentState = {
      ...this.currentState,
      connection: params.connection,
      vmState: params.vmState,
      ipAddress: params.ipAddress,
      provider: params.provider,
      region: params.region,
      uptimeSeconds: params.uptimeSeconds,
      error: params.error,
      timestamp: Date.now()
    };
    this.notifyStateChange(previousState);
    this.updateStatusBar();
  }

  /**
   * Get current unified state
   */
  getState(): UnifiedState {
    return { ...this.currentState };
  }

  /**
   * Subscribe to state changes
   */
  onStateChange(listener: StateChangeListener): vscode.Disposable {
    this.listeners.add(listener);
    return new vscode.Disposable(() => {
      this.listeners.delete(listener);
    });
  }

  /**
   * Update status bar display
   */
  private updateStatusBar() {
    const { auth, connection, vmState, ipAddress } = this.currentState;

    // Determine icon and text based on unified state
    let icon: string;
    let text: string;
    let backgroundColor: vscode.ThemeColor | undefined;
    let color: vscode.ThemeColor | undefined;

    // Priority 1: Not authenticated
    if (auth === 'not_authenticated' || connection === 'not_authenticated') {
      icon = '$(lock)';
      text = 'TensorFleet · Not Logged In';
      backgroundColor = new vscode.ThemeColor('statusBarItem.warningBackground');
    }
    // Priority 2: API disconnected
    else if (connection === 'disconnected') {
      icon = '$(warning)';
      text = 'TensorFleet · Disconnected';
      backgroundColor = undefined;
    }
    // Priority 3: Checking/Authenticating
    else if (auth === 'checking') {
      icon = '$(sync~spin)';
      text = 'TensorFleet · Checking...';
      backgroundColor = undefined;
    }
    // Priority 4: VM states (when authenticated and connected)
    else {
      switch (vmState) {
        case 'starting':
          icon = '$(loading~spin)';
          text = 'TensorFleet · Starting...';
          break;
        case 'stopping':
          icon = '$(loading~spin)';
          text = 'TensorFleet · Stopping...';
          break;
        case 'running':
          icon = '$(circle-filled)';
          text = ipAddress ? `TensorFleet · ${ipAddress}` : 'TensorFleet · Running';
          // Subtle green tint for running state
          color = new vscode.ThemeColor('terminal.ansiGreen');
          break;
        case 'failed':
          icon = '$(error)';
          text = 'TensorFleet · Failed';
          backgroundColor = new vscode.ThemeColor('statusBarItem.errorBackground');
          break;
        case 'stopped':
          icon = '$(circle-outline)';
          text = 'TensorFleet · Stopped';
          break;
        case 'pending':
          icon = '$(sync~spin)';
          text = 'TensorFleet · Pending...';
          break;
        case 'unknown':
        default:
          icon = '$(sync~spin)';
          text = 'TensorFleet · Checking VM...';
          break;
      }
    }

    this.statusBarItem.text = `${icon} ${text}`;
    this.statusBarItem.backgroundColor = backgroundColor;
    this.statusBarItem.color = color;
    this.statusBarItem.tooltip = this.buildTooltip();
  }

  /**
   * Build tooltip text
   */
  private buildTooltip(): string {
    const { auth, connection, vmState, ipAddress, provider, region, uptimeSeconds, error, timestamp } = this.currentState;
    const lines: string[] = [];

    if (auth === 'not_authenticated' || connection === 'not_authenticated') {
      lines.push('$(lock) Not authenticated');
      lines.push('Click to login to TensorFleet');
      if (error) lines.push(`Error: ${error}`);
    } else if (connection === 'disconnected') {
      lines.push('$(warning) Cannot reach VM Manager API');
      lines.push(`Last known state: ${vmState}`);
      if (ipAddress) lines.push(`Last known IP: ${ipAddress}`);
    } else {
      lines.push('$(check) Connected to VM Manager API');
      lines.push(`VM State: ${vmState}`);
      if (vmState === 'pending') {
        lines.push('$(warning) VM exists but has not started yet');
      }
    }

    if (ipAddress) lines.push(`IP: ${ipAddress}`);
    if (provider) lines.push(`Provider: ${provider}`);
    if (region) lines.push(`Region: ${region}`);
    
    const uptime = this.formatUptime(uptimeSeconds);
    if (uptime) lines.push(`Uptime: ${uptime}`);
    
    // Only show error if not already shown in not_authenticated section
    if (error && auth !== 'not_authenticated' && connection !== 'not_authenticated') {
      lines.push(`Error: ${error}`);
    }
    lines.push(`Updated: ${new Date(timestamp).toLocaleTimeString()}`);

    return lines.join('\n');
  }

  /**
   * Format uptime
   */
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

  /**
   * Notify listeners of state change
   */
  private notifyStateChange(previousState: UnifiedState) {
    for (const listener of this.listeners) {
      try {
        listener(this.currentState);
      } catch (error) {
        console.error('[UnifiedStatus] Listener error:', error);
      }
    }
  }

  dispose() {
    this.statusBarItem.dispose();
    this.listeners.clear();
  }
}
