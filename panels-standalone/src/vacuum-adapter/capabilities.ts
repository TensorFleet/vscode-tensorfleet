export type VacuumBackendSource = "turtlebot4_nav2" | "valetudo" | "mock";

export const VACUUM_CAPABILITY_NAMES = [
  "go_to_location",
  "cancel_navigation",
  "manual_control",
  "map",
  "pose",
  "navigation_status",
  "start_cleaning",
  "pause",
  "resume",
  "stop",
  "return_to_dock",
  "dock_state",
  "segment_cleaning",
  "zone_cleaning",
  "fan_speed",
  "water_usage",
  "battery",
  "consumables",
  "events",
  "fault_state",
] as const;

export type VacuumCapabilityName = (typeof VACUUM_CAPABILITY_NAMES)[number];

export type CapabilitySupport = {
  supported: boolean;
  source?: VacuumBackendSource;
  backendCapability?: string;
  commands?: string[];
  attributes?: string[];
  notes?: string;
};

export type VacuumCapabilities = Record<VacuumCapabilityName, CapabilitySupport>;

const UNSUPPORTED_CAPABILITY: CapabilitySupport = {
  supported: false,
  notes: "Not supported by this backend.",
};

export function createUnsupportedCapabilities(): VacuumCapabilities {
  return Object.fromEntries(
    VACUUM_CAPABILITY_NAMES.map((name) => [name, { ...UNSUPPORTED_CAPABILITY }]),
  ) as VacuumCapabilities;
}

