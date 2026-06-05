import type { VacuumCommandName } from "../../commands";
import type { VacuumCapabilityName } from "../../capabilities";
import type { VacuumCommandErrorCode } from "../../errors";
import type {
  VacuumAdapterDiagnostics,
  VacuumDockStatus,
  VacuumGoalCoordinates,
  VacuumMissionState,
  VacuumPoseCoordinates,
  VacuumRuntimeHealth,
  VacuumSourceState,
} from "../../state";
import type { ValetudoCapabilityAvailability } from "./capabilityMapper";
import type { ValetudoBackendCapability } from "./capabilityMapper";

export type ValetudoConnectionStatus = "offline" | "connecting" | "online";

export type ValetudoRuntimeBoundary = {
  connectionStatus: ValetudoConnectionStatus;
  endpoint?: string;
  capabilities: ValetudoBackendCapability[];
  commandAvailability?: Partial<Record<string, ValetudoCapabilityAvailability>>;
  unsupportedCommands?: Partial<Record<VacuumCapabilityName, string>>;
  state: ValetudoRobotState | null;
  health?: VacuumRuntimeHealth;
  source?: VacuumSourceState;
  dock?: VacuumDockStatus;
  diagnostics?: VacuumAdapterDiagnostics;
  lastError?: string;
};

export type ValetudoRobotState = {
  id: string;
  label: string;
  model?: string;
  mapAvailable: boolean;
  pose: VacuumPoseCoordinates | null;
  batteryPercentage: number | null;
  charging: boolean | null;
  missionState: VacuumMissionState;
  faults: string[];
};

export type ValetudoCommandRequest =
  | {
      type: "basic_control";
      action: "start" | "pause" | "stop" | "home";
    }
  | {
      type: "go_to_location";
      target: VacuumGoalCoordinates;
    }
  | {
      type: "segment_cleaning";
      segmentIds: string[];
    }
  | {
      type: "zone_cleaning";
      zones: string[];
    }
  | {
      type: "set_fan_speed" | "set_water_usage";
      value: string;
    };

export type ValetudoCommandMappingResult =
  | {
      ok: true;
      command: VacuumCommandName;
      request: ValetudoCommandRequest;
    }
  | {
      ok: false;
      command: VacuumCommandName;
      reason: VacuumCommandErrorCode;
      message: string;
    };
