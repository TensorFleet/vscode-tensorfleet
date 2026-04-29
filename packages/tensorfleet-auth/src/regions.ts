/**
 * TensorFleet Region Configuration
 *
 * Maps regions to their respective API domains. Callers provide their runtime
 * policy for whether development-only regions should be available.
 */

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
  /** If true, this region is only shown when running in dev mode */
  devOnly?: boolean;
}

export interface RegionPickItem {
  label: string;
  description: string;
  detail: string;
  picked: boolean;
}

export const REGIONS: Record<string, RegionConfig> = {
  eu: {
    id: 'eu',
    name: 'EU Central',
    backendUrl: 'https://app.tensorfleet.net',
    vmManagerUrl: 'https://eu.vm.tensorfleet.net',
    foxglovePort: 8765,
    ros2Port: 9091,
    description: 'Europe - Central',
    icon: '🇪🇺',
  },
  asia: {
    id: 'asia',
    name: 'Asia',
    backendUrl: 'https://app.tensorfleet.net',
    vmManagerUrl: 'http://vm-manager-asia-1.tail4f6a7.ts.net',
    foxglovePort: 8765,
    ros2Port: 9091,
    description: 'Asia - Southeast (beta/staging)',
    icon: '🇹🇭',
    devOnly: true,
  },
  local: {
    id: 'local',
    name: 'Local Development',
    backendUrl: 'https://app.tensorfleet.net',
    vmManagerUrl: 'http://localhost:8080',
    foxglovePort: 8765,
    ros2Port: 9091,
    description: 'Local development server',
    icon: '💻',
    devOnly: true,
  },
};

export const DEFAULT_REGION = 'eu';

export function getAvailableRegions(includeDevOnly: boolean): Record<string, RegionConfig> {
  if (includeDevOnly) {
    return REGIONS;
  }

  return Object.fromEntries(
    Object.entries(REGIONS).filter(([, config]) => !config.devOnly),
  );
}

export function getRegionById(regionId: string, includeDevOnly: boolean): RegionConfig | undefined {
  return getAvailableRegions(includeDevOnly)[regionId];
}

export function getRegionOrDefault(regionId: string | undefined, includeDevOnly: boolean): RegionConfig {
  const availableRegions = getAvailableRegions(includeDevOnly);
  if (regionId && availableRegions[regionId]) {
    return availableRegions[regionId];
  }

  return availableRegions[DEFAULT_REGION] ?? REGIONS[DEFAULT_REGION];
}

export function resolveRegionId(regionId: string | undefined, includeDevOnly: boolean): string {
  const availableRegions = getAvailableRegions(includeDevOnly);
  if (regionId && availableRegions[regionId]) {
    return regionId;
  }

  return DEFAULT_REGION;
}

export function assertRegionAvailable(regionId: string, includeDevOnly: boolean): void {
  if (!getRegionById(regionId, includeDevOnly)) {
    throw new Error(`Region not available: ${regionId}`);
  }
}

export function getRegionBackendUrl(regionId: string | undefined, includeDevOnly: boolean): string {
  return getRegionOrDefault(regionId, includeDevOnly).backendUrl;
}

export function getRegionVmManagerUrl(regionId: string | undefined, includeDevOnly: boolean): string {
  return getRegionOrDefault(regionId, includeDevOnly).vmManagerUrl;
}

export function getRegionFoxgloveUrl(
  ipAddress: string,
  regionId: string | undefined,
  includeDevOnly: boolean,
): string {
  const region = getRegionOrDefault(regionId, includeDevOnly);
  return `ws://${ipAddress}:${region.foxglovePort}`;
}

export function getRegionRos2WebsocketUrl(
  ipAddress: string,
  regionId: string | undefined,
  includeDevOnly: boolean,
): string {
  const region = getRegionOrDefault(regionId, includeDevOnly);
  return `ws://${ipAddress}:${region.ros2Port}`;
}

export function getRegionPickItems(
  currentRegionId: string | undefined,
  includeDevOnly: boolean,
): RegionPickItem[] {
  const selectedRegionId = resolveRegionId(currentRegionId, includeDevOnly);

  return Object.values(getAvailableRegions(includeDevOnly)).map(region => ({
    label: `${region.icon} ${region.name}`,
    description: region.id === selectedRegionId ? '$(check) Current' : '',
    detail: region.description,
    picked: region.id === selectedRegionId,
  }));
}

export function getRegionIdFromPickLabel(
  label: string,
  includeDevOnly: boolean,
): string | undefined {
  for (const [id, region] of Object.entries(getAvailableRegions(includeDevOnly))) {
    if (label.includes(region.name)) {
      return id;
    }
  }

  return undefined;
}
