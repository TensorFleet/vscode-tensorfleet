import type { VacuumCapabilityName } from "../../capabilities";

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
