import type { VacuumCommandError } from "./errors";
import type { VacuumGoalCoordinates } from "./state";

export type VacuumCoverageArea =
  | {
      shape: "rectangle";
      minX: number;
      minY: number;
      maxX: number;
      maxY: number;
    }
  | {
      shape: "polygon";
      points: Array<{ x: number; y: number }>;
    };

export type VacuumCommandName =
  | "start_navigation"
  | "go_to_location"
  | "cancel_navigation"
  | "manual_control"
  | "start_mapping"
  | "pause_mapping"
  | "resume_mapping"
  | "finish_mapping"
  | "discard_mapping"
  | "accept_map"
  | "load_map"
  | "start_coverage"
  | "pause_mission"
  | "resume_mission"
  | "cancel_mission"
  | "retry_mission_step"
  | "skip_mission_step"
  | "start_cleaning"
  | "pause"
  | "resume"
  | "stop"
  | "return_to_dock"
  | "segment_cleaning"
  | "zone_cleaning"
  | "set_fan_speed"
  | "set_water_usage";

export type VacuumGoToLocationCommand = {
  command: "go_to_location";
  target: VacuumGoalCoordinates;
};

export type VacuumStartNavigationCommand = {
  command: "start_navigation";
  target: VacuumGoalCoordinates;
};

export type VacuumCancelNavigationCommand = {
  command: "cancel_navigation";
};

export type VacuumStartMappingCommand = {
  command: "start_mapping";
  mode: "auto" | "manual";
  name?: string;
};

export type VacuumAcceptMapCommand = {
  command: "accept_map";
  name?: string;
};

export type VacuumLoadMapCommand = {
  command: "load_map";
  name: string;
};

export type VacuumStartCoverageCommand = {
  command: "start_coverage";
  area: VacuumCoverageArea;
  route?: VacuumGoalCoordinates[];
  coverage?: {
    swathWidth?: number;
    laneSpacing?: number;
    completionThreshold?: number;
    boundaryExtension?: number;
  };
};

export type VacuumSetFanSpeedCommand = {
  command: "set_fan_speed";
  value: string;
};

export type VacuumSetWaterUsageCommand = {
  command: "set_water_usage";
  value: string;
};

export type VacuumSimpleCommand = {
  command: Exclude<
    VacuumCommandName,
    | "start_navigation"
    | "go_to_location"
    | "cancel_navigation"
    | "start_mapping"
    | "accept_map"
    | "load_map"
    | "start_coverage"
    | "set_fan_speed"
    | "set_water_usage"
  >;
};

export type VacuumCommand =
  | VacuumStartNavigationCommand
  | VacuumGoToLocationCommand
  | VacuumCancelNavigationCommand
  | VacuumStartMappingCommand
  | VacuumAcceptMapCommand
  | VacuumLoadMapCommand
  | VacuumStartCoverageCommand
  | VacuumSetFanSpeedCommand
  | VacuumSetWaterUsageCommand
  | VacuumSimpleCommand;

export type VacuumCommandResult =
  | {
      ok: true;
      command: VacuumCommandName;
      message?: string;
    }
  | {
      ok: false;
      command: VacuumCommandName;
      error: VacuumCommandError;
    };
