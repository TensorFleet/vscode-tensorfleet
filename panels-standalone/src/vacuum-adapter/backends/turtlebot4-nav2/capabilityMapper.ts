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

  capabilities.go_to_location = hasNavigateToPose
    ? supportedCapability({
        backendCapability: "nav2_msgs/action/NavigateToPose",
        commands: ["go_to_location"],
        attributes: ["map_frame_target", "yaw"],
        notes: "Backed by the Nav2 NavigateToPose action.",
      })
    : unsupportedCapability("NavigateToPose send_goal service is not advertised.");

  capabilities.cancel_navigation = hasCancelNavigation
    ? supportedCapability({
        backendCapability: "nav2_msgs/action/NavigateToPose cancel_goal",
        commands: ["cancel_navigation"],
        notes: "Backed by the Nav2 action cancel service.",
      })
    : unsupportedCapability("NavigateToPose cancel_goal service is not advertised.");

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
