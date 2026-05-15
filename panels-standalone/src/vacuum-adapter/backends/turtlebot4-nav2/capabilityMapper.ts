import {
  CANCEL_GOAL_SERVICE,
  SEND_GOAL_SERVICE,
} from "../../../components/Nav2/runtime/nav2RuntimeConstants";
import type { Nav2RuntimeState } from "../../../components/Nav2/runtime/nav2RuntimeTypes";
import type {
  CapabilitySupport,
  VacuumCapabilities,
  VacuumCapabilityName,
} from "../../capabilities";
import { createUnsupportedCapabilities } from "../../capabilities";
import type { VacuumCommandName, VacuumCommandResult } from "../../commands";
import { unsupportedCommand } from "../../errors";

const SOURCE = "turtlebot4_nav2" as const;

export const MAPPING_STATUS_TOPIC = "/vacuum_mapping/status";
export const MAPPING_SERVICE_NAMES = {
  startAuto: "/vacuum_mapping/start_auto",
  startManual: "/vacuum_mapping/start_manual",
  pause: "/vacuum_mapping/pause",
  resume: "/vacuum_mapping/resume",
  finish: "/vacuum_mapping/finish",
  discard: "/vacuum_mapping/discard",
  accept: "/vacuum_mapping/accept",
  saveMap: "/vacuum_mapping/save_map",
  loadMap: "/vacuum_mapping/load_map",
  listMaps: "/vacuum_mapping/list_maps",
} as const;
export const MISSION_STATUS_TOPIC = "/vacuum_mission/status";
export const MISSION_SERVICE_NAMES = {
  startNavigation: "/vacuum_mission/start_navigation",
  startCoverage: "/vacuum_mission/start_coverage",
  cancel: "/vacuum_mission/cancel",
  pause: "/vacuum_mission/pause",
  resume: "/vacuum_mission/resume",
  retryStep: "/vacuum_mission/retry_step",
  skipStep: "/vacuum_mission/skip_step",
  getSnapshot: "/vacuum_mission/get_snapshot",
  setParameters: "/vacuum_mission_runtime/set_parameters",
} as const;

const UNSUPPORTED_VACUUM_FEATURES: VacuumCapabilityName[] = [
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
];

export const TURTLEBOT4_NAV2_UNSUPPORTED_COMMANDS: VacuumCommandName[] = [
  "start_cleaning",
  "pause",
  "resume",
  "stop",
  "return_to_dock",
  "segment_cleaning",
  "zone_cleaning",
  "set_fan_speed",
  "set_water_usage",
];

function supportedCapability(overrides: Omit<CapabilitySupport, "supported" | "source">): CapabilitySupport {
  return {
    supported: true,
    source: SOURCE,
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

export function mapTurtleBot4Nav2Capabilities(runtime: Nav2RuntimeState): VacuumCapabilities {
  const capabilities = createUnsupportedCapabilities();
  const hasNavigateToPose = runtime.availableServices.includes(SEND_GOAL_SERVICE);
  const hasCancelNavigation = runtime.availableServices.includes(CANCEL_GOAL_SERVICE);
  const hasMissionStartNavigation = runtime.availableServices.includes(MISSION_SERVICE_NAMES.startNavigation);
  const hasMissionStartCoverage = runtime.availableServices.includes(MISSION_SERVICE_NAMES.startCoverage);
  const hasMissionSetParameters = runtime.availableServices.includes(MISSION_SERVICE_NAMES.setParameters);
  const hasMissionCancel = runtime.availableServices.includes(MISSION_SERVICE_NAMES.cancel);
  const hasMissionPause = runtime.availableServices.includes(MISSION_SERVICE_NAMES.pause);
  const hasMissionResume = runtime.availableServices.includes(MISSION_SERVICE_NAMES.resume);
  const hasMissionRetryStep = runtime.availableServices.includes(MISSION_SERVICE_NAMES.retryStep);
  const hasMissionSkipStep = runtime.availableServices.includes(MISSION_SERVICE_NAMES.skipStep);
  const hasMissionSnapshot = runtime.availableServices.includes(MISSION_SERVICE_NAMES.getSnapshot);
  const hasMissionStatus = runtime.availableTopics.some((topic) => topic.topic === MISSION_STATUS_TOPIC);

  capabilities.go_to_location = hasNavigateToPose
    ? supportedCapability({
        backendCapability: "nav2_msgs/action/NavigateToPose",
        commands: ["go_to_location"],
        attributes: ["map_frame_target", "yaw"],
        notes: "Backed by the Nav2 NavigateToPose action.",
      })
    : unsupportedCapability("NavigateToPose send_goal service is not advertised.");

  capabilities.start_navigation = hasMissionStartNavigation && hasMissionSetParameters
    ? supportedCapability({
        backendCapability: "vacuum_mission_runtime",
        commands: ["start_navigation"],
        attributes: ["map_frame_target", "yaw", "active_mission_snapshot"],
        notes: "Product command for a point-navigation mission owned by the VM runtime.",
      })
    : unsupportedCapability(
        hasMissionStartNavigation
          ? "VM navigation mission runtime parameter service is not advertised."
          : "VM navigation mission runtime is not advertised.",
      );

  capabilities.cancel_navigation = hasCancelNavigation
    ? supportedCapability({
        backendCapability: "nav2_msgs/action/NavigateToPose cancel_goal",
        commands: ["cancel_navigation"],
        notes: "Backed by the Nav2 action cancel service.",
      })
    : unsupportedCapability("NavigateToPose cancel_goal service is not advertised.");

  capabilities.mission_state = supportedCapability({
    backendCapability: hasMissionStatus || hasMissionSnapshot ? "vacuum_mission_runtime" : "vacuum_adapter mission normalization",
    attributes: ["activeMission", "missions", "availableActions", "terminalResult"],
    notes: hasMissionStatus || hasMissionSnapshot
      ? "The VM runtime publishes or serves backend-neutral mission snapshots for UI hydration."
      : "The adapter exposes a backend-neutral mission snapshot bridge for UI hydration.",
  });
  capabilities.cancel_mission = hasMissionCancel
    ? supportedCapability({
        backendCapability: "vacuum_mission_runtime",
        commands: ["cancel_mission"],
        notes: "Cancels the active VM-owned mission when supported by that mission.",
      })
    : unsupportedCapability("VM mission cancel service is not advertised.");
  capabilities.coverage_mission = hasMissionStartCoverage && hasMissionSetParameters
    ? supportedCapability({
        backendCapability: "vacuum_mission_runtime",
        commands: ["start_coverage", "cancel_mission", "pause_mission", "resume_mission", "retry_mission_step", "skip_mission_step"],
        attributes: ["coverage_area", "runtime_route", "coverage_progress", "active_mission_snapshot"],
        notes: "Product command for a coverage mission owned by the VM runtime.",
      })
    : unsupportedCapability(
        hasMissionStartCoverage
          ? "VM coverage mission runtime parameter service is not advertised."
          : "VM coverage mission runtime is not advertised.",
      );
  capabilities.start_coverage = hasMissionStartCoverage && hasMissionSetParameters
    ? supportedCapability({
        backendCapability: "vacuum_mission_runtime",
        commands: ["start_coverage"],
        attributes: ["coverage_area", "optional_route", "active_mission_snapshot"],
        notes: "Starts a backend-neutral coverage mission owned by the VM runtime.",
      })
    : unsupportedCapability(
        hasMissionStartCoverage
          ? "VM coverage mission runtime parameter service is not advertised."
          : "VM coverage mission runtime is not advertised.",
      );
  capabilities.pause_mission = hasMissionPause
    ? supportedCapability({
        backendCapability: "vacuum_mission_runtime",
        commands: ["pause_mission"],
        notes: "Pauses the active VM-owned mission when supported by that mission.",
      })
    : unsupportedCapability("VM mission pause service is not advertised.");
  capabilities.resume_mission = hasMissionResume
    ? supportedCapability({
        backendCapability: "vacuum_mission_runtime",
        commands: ["resume_mission"],
        notes: "Resumes the active VM-owned mission when supported by that mission.",
      })
    : unsupportedCapability("VM mission resume service is not advertised.");
  capabilities.retry_mission_step = hasMissionRetryStep
    ? supportedCapability({
        backendCapability: "vacuum_mission_runtime",
        commands: ["retry_mission_step"],
        notes: "Retries the current runtime-owned mission step when supported.",
      })
    : unsupportedCapability("VM mission retry-step service is not advertised.");
  capabilities.skip_mission_step = hasMissionSkipStep
    ? supportedCapability({
        backendCapability: "vacuum_mission_runtime",
        commands: ["skip_mission_step"],
        notes: "Skips the current runtime-owned mission step when supported.",
      })
    : unsupportedCapability("VM mission skip-step service is not advertised.");

  capabilities.manual_control = supportedCapability({
    backendCapability: "/cmd_vel_raw",
    commands: ["manual_control"],
    attributes: ["geometry_msgs/msg/Twist"],
    notes: "Manual teleop publishes Twist commands to the VM deadman input.",
  });
  capabilities.map = supportedCapability({
    backendCapability: "/map",
    attributes: ["nav_msgs/msg/OccupancyGrid"],
  });
  const mappingServices = Object.values(MAPPING_SERVICE_NAMES);
  const hasMappingServices = mappingServices.every((serviceName) => runtime.availableServices.includes(serviceName));
  const hasMappingStatus = runtime.availableTopics.some((topic) => topic.topic === MAPPING_STATUS_TOPIC);
  capabilities.mapping_session = hasMappingServices
    ? supportedCapability({
        backendCapability: "vacuum_frontier_explorer services",
        commands: ["start_mapping", "pause_mapping", "resume_mapping", "finish_mapping", "discard_mapping", "accept_map", "load_map"],
        attributes: ["mapping_state", "session_acceptance", "persistent_map_save", "saved_map_inventory"],
        notes: "Backed by the VM-owned mapping runtime.",
      })
    : unsupportedCapability("VM mapping services are not advertised.");
  capabilities.auto_mapping = hasMappingServices && hasMappingStatus
    ? supportedCapability({
        backendCapability: "vacuum_frontier_explorer",
        commands: ["start_mapping", "pause_mapping", "resume_mapping", "finish_mapping", "discard_mapping"],
        attributes: ["frontier_count", "visited_goal_count", "failed_goal_count"],
        notes: "The autonomous frontier loop runs inside the VM runtime.",
      })
    : unsupportedCapability("VM frontier exploration runtime is not advertised.");
  capabilities.pose = supportedCapability({
    backendCapability: "/pose",
    attributes: ["geometry_msgs/msg/PoseWithCovarianceStamped", "nav_msgs/msg/Odometry fallback"],
  });
  capabilities.navigation_status = supportedCapability({
    backendCapability: "/navigate_to_pose/_action/status",
    attributes: ["GoalStatusArray", "NavigateToPose feedback"],
  });
  capabilities.fault_state = supportedCapability({
    backendCapability: "Nav2 runtime health",
    attributes: ["preflight", "TF health", "lifecycle health"],
  });

  for (const name of UNSUPPORTED_VACUUM_FEATURES) {
    capabilities[name] = unsupportedCapability("Not supported by the TurtleBot4/Nav2 adapter slice.");
  }

  return capabilities;
}

export function unsupportedTurtleBot4Nav2Command(command: VacuumCommandName): VacuumCommandResult {
  return {
    ok: false,
    command,
    error: unsupportedCommand(command, `Command ${command} is not supported by the TurtleBot4/Nav2 adapter.`),
  };
}
