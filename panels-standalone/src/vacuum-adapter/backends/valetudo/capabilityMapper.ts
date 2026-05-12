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
  GoToLocationCapability: ["go_to_location"],
  MapSegmentationCapability: ["segment_cleaning"],
  ZoneCleaningCapability: ["zone_cleaning"],
  FanSpeedControlCapability: ["fan_speed"],
  WaterUsageControlCapability: ["water_usage"],
};

const SOURCE = "valetudo" as const;

const VALETUDO_BASE_CAPABILITIES: VacuumCapabilityName[] = [
  "map",
  "pose",
  "battery",
  "dock_state",
  "events",
  "fault_state",
];

const VALETUDO_BACKEND_COMMANDS: Partial<Record<VacuumCapabilityName, string[]>> = {
  start_cleaning: ["start_cleaning"],
  pause: ["pause"],
  stop: ["stop"],
  return_to_dock: ["return_to_dock"],
  go_to_location: ["go_to_location"],
  segment_cleaning: ["segment_cleaning"],
  zone_cleaning: ["zone_cleaning"],
  fan_speed: ["set_fan_speed"],
  water_usage: ["set_water_usage"],
};

function supportedCapability(
  backendCapability: ValetudoBackendCapability | "ValetudoRobotState",
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
    capabilities[name] = supportedCapability("ValetudoRobotState", {
      attributes: ["normalized_state"],
      notes: "Expected from the Valetudo integration runtime state stream.",
    });
  }

  for (const backendCapability of advertised) {
    const publicCapabilities = VALETUDO_CAPABILITY_MAP[backendCapability];
    for (const name of publicCapabilities) {
      capabilities[name] = supportedCapability(backendCapability, {
        commands: VALETUDO_BACKEND_COMMANDS[name],
        notes: "Mapped privately from the Valetudo backend capability set.",
      });
    }
  }

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
  capabilities.navigation_status = capabilities.go_to_location.supported
    ? supportedCapability("GoToLocationCapability", {
        attributes: ["normalized_mission_state"],
        notes: "Navigation status is derived from normalized Valetudo state.",
      })
    : unsupportedCapability("GoToLocationCapability is not advertised by the Valetudo backend.");

  return capabilities;
}
