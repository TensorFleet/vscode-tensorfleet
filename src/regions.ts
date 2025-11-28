/**
 * TensorFleet Region Configuration
 * 
 * Maps regions to their respective API domains.
 * Users select a region, and all URLs are derived automatically.
 */

import * as vscode from 'vscode';

export interface RegionConfig {
  /** Display name for the region */
  name: string;
  /** Short code/id for the region */
  id: string;
  /** Backend API URL (auth, data) */
  backendUrl: string;
  /** VM Manager API URL */
  vmManagerUrl: string;
  /** Foxglove WebSocket URL (derived from VM IP at runtime) */
  foxglovePort: number;
  /** ROS2 WebSocket URL (derived from VM IP at runtime) */
  ros2Port: number;
  /** Geographic description */
  description: string;
  /** Icon for display */
  icon: string;
}

/**
 * Available regions and their configurations
 */
export const REGIONS: Record<string, RegionConfig> = {
  'us-west': {
    id: 'us-west',
    name: 'US West',
    backendUrl: 'https://app.tensorfleet.net',
    vmManagerUrl: 'https://vm.tensorfleet.net',
    foxglovePort: 8765,
    ros2Port: 9091,
    description: 'Production environment',
    icon: '🇺🇸'
  },
  'local': {
    id: 'local',
    name: 'Local',
    backendUrl: 'http://localhost:3000',
    vmManagerUrl: 'http://localhost:8080',
    foxglovePort: 8765,
    ros2Port: 9091,
    description: 'Local development',
    icon: '💻'
  }
};

/**
 * Default region if none is configured
 */
export const DEFAULT_REGION = 'us-west';

/**
 * Get the currently selected region ID from configuration
 */
export function getSelectedRegionId(): string {
  const config = vscode.workspace.getConfiguration('tensorfleet');
  return config.get<string>('region') || DEFAULT_REGION;
}

/**
 * Get the configuration for the currently selected region
 */
export function getSelectedRegion(): RegionConfig {
  const regionId = getSelectedRegionId();
  return REGIONS[regionId] || REGIONS[DEFAULT_REGION];
}

/**
 * Get the backend URL for the current region
 */
export function getBackendUrl(): string {
  return getSelectedRegion().backendUrl;
}

/**
 * Get the VM Manager URL for the current region
 */
export function getVmManagerUrl(): string {
  return getSelectedRegion().vmManagerUrl;
}

/**
 * Get the Foxglove WebSocket URL for a given IP
 */
export function getFoxgloveUrl(ipAddress: string): string {
  const region = getSelectedRegion();
  return `ws://${ipAddress}:${region.foxglovePort}`;
}

/**
 * Get the ROS2 WebSocket URL for a given IP
 */
export function getRos2WebsocketUrl(ipAddress: string): string {
  const region = getSelectedRegion();
  return `ws://${ipAddress}:${region.ros2Port}`;
}

/**
 * Set the selected region
 */
export async function setSelectedRegion(regionId: string): Promise<void> {
  if (!REGIONS[regionId]) {
    throw new Error(`Unknown region: ${regionId}`);
  }
  const config = vscode.workspace.getConfiguration('tensorfleet');
  await config.update('region', regionId, vscode.ConfigurationTarget.Global);
}

/**
 * Get all available regions as quick pick items
 */
export function getRegionQuickPickItems(): vscode.QuickPickItem[] {
  const currentRegionId = getSelectedRegionId();
  
  return Object.values(REGIONS).map(region => ({
    label: `${region.icon} ${region.name}`,
    description: region.id === currentRegionId ? '$(check) Current' : '',
    detail: region.description,
    // Store region ID for selection
    picked: region.id === currentRegionId
  }));
}

/**
 * Extract region ID from a quick pick selection
 */
export function getRegionIdFromQuickPick(label: string): string | undefined {
  for (const [id, region] of Object.entries(REGIONS)) {
    if (label.includes(region.name)) {
      return id;
    }
  }
  return undefined;
}

/**
 * Listen for region configuration changes
 */
export function onRegionChange(callback: (newRegion: RegionConfig) => void): vscode.Disposable {
  return vscode.workspace.onDidChangeConfiguration(event => {
    if (event.affectsConfiguration('tensorfleet.region')) {
      callback(getSelectedRegion());
    }
  });
}

