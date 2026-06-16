export type VacuumBackendSource = "turtlebot4_nav2" | "valetudo" | "mock";

export type CapabilityStatus = "supported" | "unsupported" | "unavailable" | "detected_not_ready";

export type CapabilityReason = {
  code: string;
  message: string;
};

export const VACUUM_CAPABILITY_NAMES = [
  "mission_state",
  "start_navigation",
  "go_to_location",
  "cancel_navigation",
  "manual_control",
  "map",
  "mapping_session",
  "auto_mapping",
  "coverage_mission",
  "map_annotations",
  "room_semantics",
  "zone_semantics",
  "room_cleaning",
  "zone_cleaning",
  "pose",
  "navigation_status",
  "start_coverage",
  "pause_mission",
  "resume_mission",
  "cancel_mission",
  "retry_mission_step",
  "skip_mission_step",
  "start_cleaning",
  "pause",
  "resume",
  "stop",
  "return_to_dock",
  "dock_state",
  "segment_cleaning",
  "fan_speed",
  "water_usage",
  "battery",
  "consumables",
  "statistics",
  "attachments",
  "dock_components",
  "events",
  "fault_state",
] as const;

export type VacuumCapabilityName = (typeof VACUUM_CAPABILITY_NAMES)[number];

export type CapabilitySupport = {
  supported: boolean;
  source?: VacuumBackendSource;
  /** @deprecated Compatibility mirror only. Backend identifiers belong in snapshot.diagnostics.capabilities. */
  backendCapability?: string;
  commands?: string[];
  attributes?: string[];
  notes?: string;
  status?: CapabilityStatus;
  available?: boolean;
  availabilityReason?: string;
  reasons?: CapabilityReason[];
};

export type VacuumCapabilities = Record<VacuumCapabilityName, CapabilitySupport>;

const UNSUPPORTED_CAPABILITY: CapabilitySupport = {
  supported: false,
  status: "unsupported",
  available: false,
  notes: "Not supported by this backend.",
};

export function createUnsupportedCapabilities(): VacuumCapabilities {
  return Object.fromEntries(
    VACUUM_CAPABILITY_NAMES.map((name) => [name, { ...UNSUPPORTED_CAPABILITY }]),
  ) as VacuumCapabilities;
}
