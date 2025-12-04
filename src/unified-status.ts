/**
 * Unified Status Coordinator
 * 
 * Coordinates auth and VM Manager states into a single unified status bar.
 * Provides event system for state coordination and adaptive menu structure.
 */

import * as vscode from 'vscode';
import { getSelectedRegion } from './regions';

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
    const { auth, connection, vmState } = this.currentState;

    // Determine icon and text based on unified state
    let icon: string;
    let text: string;
    let backgroundColor: vscode.ThemeColor | undefined;
    let color: vscode.ThemeColor | undefined;

    // Priority 1: Explicit auth check in progress
    if (auth === 'checking') {
      icon = '$(sync~spin)';
      text = 'TensorFleet · Checking login...';
      backgroundColor = undefined;
    }
    // Priority 2: User not logged in
    else if (auth === 'not_authenticated') {
      icon = '$(sign-in)';
      text = 'TensorFleet · Login required';
      backgroundColor = new vscode.ThemeColor('statusBarItem.warningBackground');
    }
    // Priority 3: VM Manager auth problem (user is logged in but VM Manager rejected token)
    else if (connection === 'not_authenticated') {
      icon = '$(key)';
      text = 'TensorFleet · VM auth error';
      backgroundColor = new vscode.ThemeColor('statusBarItem.warningBackground');
    }
    // Priority 4: API disconnected (after successful auth)
    else if (connection === 'disconnected') {
      icon = '$(vm-connect)';
      text = 'TensorFleet · VM disconnected';
      backgroundColor = undefined;
    }
    // Priority 5: VM states (when authenticated and connected)
    else {
      switch (vmState) {
        case 'starting':
          icon = '$(vm-pending)';
          text = 'TensorFleet · Starting...';
          break;
        case 'stopping':
          icon = '$(debug-stop)';
          text = 'TensorFleet · Stopping...';
          break;
        case 'running':
          icon = '$(vm-active)';
          text = 'TensorFleet · Running';
          // Subtle green tint for running state
          color = new vscode.ThemeColor('terminal.ansiGreen');
          break;
        case 'failed':
          icon = '$(error)';
          text = 'TensorFleet · Failed';
          backgroundColor = new vscode.ThemeColor('statusBarItem.errorBackground');
          break;
        case 'stopped':
          icon = '$(vm-outline)';
          text = 'TensorFleet · Stopped';
          break;
        case 'pending':
          icon = '$(vm-pending)';
          text = 'TensorFleet · Pending...';
          break;
        case 'unknown':
        default:
          icon = '$(sync~spin)';
          text = 'TensorFleet · Checking VM...';
          break;
      }
    }

    const finalText = `${icon} ${text}`;

    this.statusBarItem.text = finalText;
    this.statusBarItem.backgroundColor = backgroundColor;
    this.statusBarItem.color = color;
    this.statusBarItem.tooltip = this.buildTooltip();
  }

  /**
   * Build tooltip text
   */
  private buildTooltip(): string {
    const { auth, connection, vmState, ipAddress, provider, uptimeSeconds, error, timestamp } =
      this.currentState;
    const lines: string[] = [];

    // Always show high-level auth and connection summary first
    const authLabel =
      auth === 'authenticated'
        ? 'Logged in'
        : auth === 'checking'
          ? 'Checking...'
          : 'Not logged in';
    lines.push(`Auth: ${authLabel}`);

    let connectionLabel: string;
    switch (connection) {
      case 'connected':
        connectionLabel = 'Connected';
        break;
      case 'disconnected':
        connectionLabel = 'Disconnected';
        break;
      case 'not_authenticated':
      default:
        connectionLabel = 'Auth error';
        break;
    }
    lines.push(`VM API: ${connectionLabel}`);

    // Detailed guidance based on state combination
    if (auth === 'not_authenticated') {
      lines.push('');
      lines.push('You are not logged in to TensorFleet.');
      lines.push('Click to login and unlock VM controls.');
      if (error) lines.push(`Error: ${error}`);
    } else if (connection === 'not_authenticated') {
      lines.push('');
      lines.push('VM Manager rejected the current token.');
      lines.push('Click to retry VM status or logout.');
      if (error) lines.push(`Error: ${error}`);
    } else if (connection === 'disconnected') {
      lines.push('');
      lines.push('Cannot reach VM Manager API.');
      lines.push(`Last known state: ${vmState}`);
      if (ipAddress) lines.push(`Last known IP: ${ipAddress}`);
      if (error) lines.push(`Error: ${error}`);
    } else {
      lines.push('');
      lines.push('Connected to VM Manager API.');
      lines.push(`VM State: ${vmState}`);
      if (vmState === 'pending') {
        lines.push('VM exists but has not started yet.');
      }
    }

    if (ipAddress) lines.push(`IP: ${ipAddress}`);
    if (provider) lines.push(`Provider: ${provider}`);
    const selectedRegion = getSelectedRegion();
    lines.push(`Region: ${selectedRegion.icon} ${selectedRegion.name}`);

    const uptime = this.formatUptime(uptimeSeconds);
    if (uptime) lines.push(`Uptime: ${uptime}`);

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
