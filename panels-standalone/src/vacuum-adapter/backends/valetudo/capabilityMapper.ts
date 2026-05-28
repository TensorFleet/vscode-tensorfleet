import type { CapabilitySupport, VacuumCapabilities, VacuumCapabilityName } from "../../capabilities";
import { createUnsupportedCapabilities } from "../../capabilities";

export type ValetudoBackendCapability =
  | "BasicControlCapability"
  | "GoToLocationCapability"
  | "MapSegmentationCapability"
  | "ZoneCleaningCapability"
  | "FanSpeedControlCapability"
  | "WaterUsageControlCapability";

export const VALETUDO_CAPABILITY_MAP: Record<ValetudoBackendCapability, VacuumCapabilityName[]> = {
  BasicControlCapability: ["start_cleaning", "pause", "stop", "return_to_dock"],
  GoToLocationCapability: [],
  MapSegmentationCapability: [],
  ZoneCleaningCapability: [],
  FanSpeedControlCapability: [],
  WaterUsageControlCapability: [],
};

const SOURCE = "valetudo" as const;

const VALETUDO_BASE_CAPABILITIES: VacuumCapabilityName[] = [
  "battery",
  "dock_state",
  "events",
  "fault_state",
  "mission_state",
];

const VALETUDO_BACKEND_COMMANDS: Partial<Record<VacuumCapabilityName, string[]>> = {
  start_cleaning: ["start_cleaning"],
  pause: ["pause"],
  stop: ["stop"],
  return_to_dock: ["return_to_dock"],
  go_to_location: ["go_to_location"],
  start_navigation: ["start_navigation"],
  segment_cleaning: ["segment_cleaning"],
  zone_cleaning: ["start_zone_cleaning"],
  fan_speed: ["set_fan_speed"],
  water_usage: ["set_water_usage"],
};

function supportedCapability(
  backendCapability: string,
  overrides: Omit<CapabilitySupport, "supported" | "source" | "backendCapability"> = {},
): CapabilitySupport {
  return {
    supported: true,
    source: SOURCE,
    backendCapability,
    ...overrides,
  };
}

function unsupportedCapability(notes: string): CapabilitySupport {
  return {
    supported: false,
    source: SOURCE,
    notes,
  };
}

export function mapValetudoCapabilities(
  backendCapabilities: Iterable<ValetudoBackendCapability> = [],
): VacuumCapabilities {
  const capabilities = createUnsupportedCapabilities();
  const advertised = new Set(backendCapabilities);

  for (const name of VALETUDO_BASE_CAPABILITIES) {
    capabilities[name] = supportedCapability("runtime_state", {
      attributes: ["normalized_state"],
      notes: "Expected from the Valetudo integration runtime state stream.",
    });
  }

  for (const backendCapability of advertised) {
    const publicCapabilities = VALETUDO_CAPABILITY_MAP[backendCapability];
    for (const name of publicCapabilities) {
      capabilities[name] = supportedCapability(`runtime_command:${name}`, {
        commands: VALETUDO_BACKEND_COMMANDS[name],
        notes: "Mapped from normalized Valetudo runtime command availability.",
      });
    }
  }

  capabilities.map = unsupportedCapability(
    "Valetudo map rendering is not implemented in Layer 6A Milestone 2.",
  );
  capabilities.pose = unsupportedCapability(
    "Valetudo pose is not exposed as a product navigation surface in Layer 6A Milestone 2.",
  );
  capabilities.start_navigation = unsupportedCapability(
    "Valetudo go-to execution is detected only as diagnostics until the product workflow is implemented.",
  );
  capabilities.go_to_location = unsupportedCapability(
    "Valetudo go-to execution is detected only as diagnostics until the product workflow is implemented.",
  );
  capabilities.segment_cleaning = unsupportedCapability(
    "Valetudo segment cleaning is diagnostics-only until segment mapping is implemented.",
  );
  capabilities.fan_speed = unsupportedCapability(
    "Valetudo fan speed controls are diagnostics-only in Layer 6A Milestone 2.",
  );
  capabilities.water_usage = unsupportedCapability(
    "Valetudo water controls are diagnostics-only in Layer 6A Milestone 2.",
  );

  capabilities.resume = unsupportedCapability(
    "Valetudo resume support must be mapped explicitly once the selected robot capability surface is known.",
  );
  capabilities.cancel_navigation = unsupportedCapability(
    "Valetudo go-to cancellation must be mapped explicitly by the Layer 6 backend integration.",
  );
  capabilities.manual_control = unsupportedCapability(
    "Streaming manual teleop is not part of the Valetudo backend interface.",
  );
  capabilities.mapping_session = unsupportedCapability(
    "Backend mapping sessions must be implemented explicitly before use.",
  );
  capabilities.auto_mapping = unsupportedCapability(
    "Auto mapping is not implemented for the Valetudo backend stub.",
  );
  capabilities.coverage_mission = unsupportedCapability(
    "Coverage missions must be mapped through zone, segment, or explicit unsupported behavior in the Layer 6 runtime.",
  );
  capabilities.map_annotations = unsupportedCapability(
    "Map annotation persistence must be implemented by the Valetudo integration runtime before use.",
  );
  capabilities.room_semantics = unsupportedCapability(
    "Valetudo room semantics must be mapped from vendor segments or adapter-owned annotations in Layer 6.",
  );
  capabilities.zone_semantics = unsupportedCapability(
    "Valetudo zone semantics must be mapped from vendor zones or adapter-owned annotations in Layer 6.",
  );
  capabilities.room_cleaning = unsupportedCapability(
    "Valetudo room cleaning must be mapped from vendor segments or adapter-owned annotations in Layer 6.",
  );
  capabilities.zone_cleaning = unsupportedCapability(
    "Valetudo zone cleaning requires explicit Layer 6 geometry mapping before product room/zone commands are enabled.",
  );
  capabilities.start_coverage = unsupportedCapability(
    "Coverage missions must be mapped through zone, segment, or explicit unsupported behavior in the Layer 6 runtime.",
  );
  capabilities.pause_mission = capabilities.pause.supported
    ? supportedCapability("runtime_command:pause", {
        commands: ["pause_mission"],
        notes: "Mapped to backend pause by the Valetudo integration runtime when a mission is active.",
      })
    : unsupportedCapability("Valetudo pause support is not available.");
  capabilities.resume_mission = unsupportedCapability(
    "Valetudo resume support must be mapped explicitly once the selected robot capability surface is known.",
  );
  capabilities.cancel_mission = capabilities.stop.supported
    ? supportedCapability("runtime_command:stop", {
        commands: ["cancel_mission"],
        notes: "Mapped to backend stop by the Valetudo integration runtime when a mission is active.",
      })
    : unsupportedCapability("Valetudo stop support is not available.");
  capabilities.retry_mission_step = unsupportedCapability(
    "Mission step retry requires runtime-owned recovery semantics.",
  );
  capabilities.skip_mission_step = unsupportedCapability(
    "Mission step skip requires runtime-owned recovery semantics.",
  );
  capabilities.navigation_status = capabilities.go_to_location.supported
    ? supportedCapability("runtime_navigation_status", {
        attributes: ["normalized_mission_state"],
        notes: "Navigation status is derived from normalized Valetudo state.",
      })
    : unsupportedCapability("Valetudo go-to status is not exposed as a product navigation surface.");

  return capabilities;
}
