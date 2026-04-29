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
  getRegionBackendUrl,
  getRegionFoxgloveUrl,
  getRegionIdFromPickLabel,
  getRegionOrDefault,
  getRegionPickItems,
  getRegionRos2WebsocketUrl,
  getRegionVmManagerUrl,
  resolveRegionId,
  assertRegionAvailable,
  DEFAULT_REGION,
  REGIONS,
  type RegionPickItem,
  type RegionConfig,
} from 'tensorfleet-auth';

export type { RegionConfig, RegionPickItem };
export { DEFAULT_REGION, REGIONS };

function includeDevRegions(): boolean {
  return isFeatureEnabled('localRegion');
}

function getConfiguredRegionId(): string | undefined {
  return vscode.workspace.getConfiguration('tensorfleet').get<string>('region');
}

/**
 * Get regions available for the current runtime mode
 * Filters out dev-only regions when running in production mode
 */
export function getAvailableRegions(): Record<string, RegionConfig> {
  return getConfiguredAvailableRegions(includeDevRegions());
}

/**
 * Get the currently selected region ID from configuration
 * Falls back to default if the configured region is not available (e.g., local in production)
 */
export function getSelectedRegionId(): string {
  return resolveRegionId(getConfiguredRegionId(), includeDevRegions());
}

/**
 * Get the configuration for the currently selected region
 */
export function getSelectedRegion(): RegionConfig {
  return getRegionOrDefault(getConfiguredRegionId(), includeDevRegions());
}

/**
 * Get the backend URL for the current region
 */
export function getBackendUrl(): string {
  return getRegionBackendUrl(getConfiguredRegionId(), includeDevRegions());
}

/**
 * Get the VM Manager URL for the current region
 */
export function getVmManagerUrl(): string {
  return getRegionVmManagerUrl(getConfiguredRegionId(), includeDevRegions());
}

/**
 * Get the Foxglove WebSocket URL for a given IP
 */
export function getFoxgloveUrl(ipAddress: string): string {
  return getRegionFoxgloveUrl(ipAddress, getConfiguredRegionId(), includeDevRegions());
}

/**
 * Get the ROS2 WebSocket URL for a given IP
 */
export function getRos2WebsocketUrl(ipAddress: string): string {
  return getRegionRos2WebsocketUrl(ipAddress, getConfiguredRegionId(), includeDevRegions());
}

/**
 * Set the selected region
 * Only allows setting regions that are available in the current build mode
 */
export async function setSelectedRegion(regionId: string): Promise<void> {
  assertRegionAvailable(regionId, includeDevRegions());
  const config = vscode.workspace.getConfiguration('tensorfleet');
  await config.update('region', regionId, vscode.ConfigurationTarget.Global);
}

/**
 * Get available regions as quick pick items
 * Only shows regions available in the current build mode
 */
export function getRegionQuickPickItems(): vscode.QuickPickItem[] {
  return getRegionPickItems(getConfiguredRegionId(), includeDevRegions());
}

/**
 * Extract region ID from a quick pick selection
 * Only matches regions available in the current build mode
 */
export function getRegionIdFromQuickPick(label: string): string | undefined {
  return getRegionIdFromPickLabel(label, includeDevRegions());
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
