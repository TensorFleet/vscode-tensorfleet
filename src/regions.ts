/**
 * TensorFleet Region Configuration
 * 
 * Maps regions to their respective API domains.
 * Users select a region, and all URLs are derived automatically.
 * 
 * Note: The "local" region is hidden at RUNTIME in production mode (marketplace install).
 * The code still exists in the build - it's just not shown to users unless debugging (F5).
 */

import * as vscode from 'vscode';
import { isFeatureEnabled } from './env';
import {
  getAvailableRegions as getConfiguredAvailableRegions,
  getRegionOrDefault,
  type RegionConfig,
} from '../panels-standalone/packages/tensorfleet-util/src/config/server-config';
import { DEFAULT_REGION } from '../panels-standalone/packages/tensorfleet-util/src/config/server-config';

export type { RegionConfig };
export { DEFAULT_REGION, REGIONS } from '../panels-standalone/packages/tensorfleet-util/src/config/server-config';

/**
 * Get regions available for the current runtime mode
 * Filters out dev-only regions when running in production mode
 */
export function getAvailableRegions(): Record<string, RegionConfig> {
  return getConfiguredAvailableRegions(isFeatureEnabled('localRegion'));
}

/**
 * Get the currently selected region ID from configuration
 * Falls back to default if the configured region is not available (e.g., local in production)
 */
export function getSelectedRegionId(): string {
  const config = vscode.workspace.getConfiguration('tensorfleet');
  const configuredRegion = config.get<string>('region') || DEFAULT_REGION;

  // If the configured region is available, use it
  const availableRegions = getAvailableRegions();
  if (availableRegions[configuredRegion]) {
    return configuredRegion;
  }

  // Fall back to default if configured region is not available
  // (e.g., "local" was configured but we're in production mode)
  return DEFAULT_REGION;
}

/**
 * Get the configuration for the currently selected region
 */
export function getSelectedRegion(): RegionConfig {
  const regionId = getSelectedRegionId();
  return getRegionOrDefault(regionId, isFeatureEnabled('localRegion'));
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
 * Only allows setting regions that are available in the current build mode
 */
export async function setSelectedRegion(regionId: string): Promise<void> {
  const availableRegions = getAvailableRegions();
  if (!availableRegions[regionId]) {
    throw new Error(`Region not available: ${regionId}`);
  }
  const config = vscode.workspace.getConfiguration('tensorfleet');
  await config.update('region', regionId, vscode.ConfigurationTarget.Global);
}

/**
 * Get available regions as quick pick items
 * Only shows regions available in the current build mode
 */
export function getRegionQuickPickItems(): vscode.QuickPickItem[] {
  const currentRegionId = getSelectedRegionId();
  const availableRegions = getAvailableRegions();

  return Object.values(availableRegions).map(region => ({
    label: `${region.icon} ${region.name}`,
    description: region.id === currentRegionId ? '$(check) Current' : '',
    detail: region.description,
    // Store region ID for selection
    picked: region.id === currentRegionId
  }));
}

/**
 * Extract region ID from a quick pick selection
 * Only matches regions available in the current build mode
 */
export function getRegionIdFromQuickPick(label: string): string | undefined {
  const availableRegions = getAvailableRegions();
  for (const [id, region] of Object.entries(availableRegions)) {
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
