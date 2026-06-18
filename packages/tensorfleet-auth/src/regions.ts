export interface RegionConfig {
  name: string;
  id: string;
  backendUrl: string;
  vmManagerUrl: string;
  foxglovePort: number;
  ros2Port: number;
  description: string;
  icon: string;
  devOnly?: boolean;
}

export interface RegionPickItem {
  label: string;
  description: string;
  detail: string;
  picked: boolean;
}

export const DEFAULT_REGION = "eu";

export const REGIONS: Record<string, RegionConfig> = {
  eu: {
    id: "eu",
    name: "EU Central",
    backendUrl: "https://app.tensorfleet.net",
    vmManagerUrl: "https://eu.vm.tensorfleet.net",
    foxglovePort: 8765,
    ros2Port: 9091,
    description: "Europe - Central",
    icon: "🇪🇺",
  },
  asia: {
    id: "asia",
    name: "Asia",
    backendUrl: "https://app.tensorfleet.net",
    vmManagerUrl: "http://vm-manager-asia-1.tail4f6a7.ts.net",
    foxglovePort: 8765,
    ros2Port: 9091,
    description: "Asia - Southeast (beta/staging)",
    icon: "🇹🇭",
    devOnly: true,
  },
  local: {
    id: "local",
    name: "Local Development",
    backendUrl: "https://app.tensorfleet.net",
    vmManagerUrl: "http://localhost:8080",
    foxglovePort: 8765,
    ros2Port: 9091,
    description: "Local development server",
    icon: "💻",
    devOnly: true,
  },
};

export function getAvailableRegions(includeDevOnly = false): Record<string, RegionConfig> {
  if (includeDevOnly) {
    return REGIONS;
  }
  return Object.fromEntries(Object.entries(REGIONS).filter(([, config]) => !config.devOnly));
}

export function getRegionById(regionId: string, includeDevOnly = false): RegionConfig | undefined {
  return getAvailableRegions(includeDevOnly)[regionId];
}

export function getRegionOrDefault(regionId: string | undefined, includeDevOnly = false): RegionConfig {
  const availableRegions = getAvailableRegions(includeDevOnly);
  if (regionId && availableRegions[regionId]) {
    return availableRegions[regionId];
  }
  return availableRegions[DEFAULT_REGION] ?? REGIONS[DEFAULT_REGION];
}

export function resolveRegionId(regionId: string | undefined, includeDevOnly = false): string {
  const availableRegions = getAvailableRegions(includeDevOnly);
  if (regionId && availableRegions[regionId]) {
    return regionId;
  }
  return DEFAULT_REGION;
}

export function assertRegionAvailable(regionId: string, includeDevOnly = false): void {
  if (!getRegionById(regionId, includeDevOnly)) {
    throw new Error(`Region not available: ${regionId}`);
  }
}

export function getRegionBackendUrl(regionId: string | undefined, includeDevOnly = false): string {
  return getRegionOrDefault(regionId, includeDevOnly).backendUrl;
}

export function getRegionVmManagerUrl(regionId: string | undefined, includeDevOnly = false): string {
  return getRegionOrDefault(regionId, includeDevOnly).vmManagerUrl;
}

export function getRegionFoxgloveUrl(
  ipAddress: string,
  regionId: string | undefined,
  includeDevOnly = false,
): string {
  return getFoxgloveUrl(ipAddress, getRegionOrDefault(regionId, includeDevOnly));
}

export function getRegionRos2WebsocketUrl(
  ipAddress: string,
  regionId: string | undefined,
  includeDevOnly = false,
): string {
  return getRos2WebsocketUrl(ipAddress, getRegionOrDefault(regionId, includeDevOnly));
}

export function getRegionPickItems(
  currentRegionId: string | undefined,
  includeDevOnly = false,
): RegionPickItem[] {
  const selectedRegionId = resolveRegionId(currentRegionId, includeDevOnly);

  return Object.values(getAvailableRegions(includeDevOnly)).map((region) => ({
    label: `${region.icon} ${region.name}`,
    description: region.id === selectedRegionId ? "$(check) Current" : "",
    detail: region.description,
    picked: region.id === selectedRegionId,
  }));
}

export function getRegionIdFromPickLabel(
  label: string,
  includeDevOnly = false,
): string | undefined {
  for (const [id, region] of Object.entries(getAvailableRegions(includeDevOnly))) {
    if (label.includes(region.name)) {
      return id;
    }
  }

  return undefined;
}

export function getFoxgloveUrl(ipAddress: string, region: RegionConfig): string {
  return `ws://${ipAddress}:${region.foxglovePort}`;
}

export function getRos2WebsocketUrl(ipAddress: string, region: RegionConfig): string {
  return `ws://${ipAddress}:${region.ros2Port}`;
}
