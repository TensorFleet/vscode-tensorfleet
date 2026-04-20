import { TeleopConfig } from "./types";

export type TeleopLayout = "twist-pad" | "custom";

export type TeleopTopicConfig = {
  topic: string;
  type: string;
  label?: string;
};

export type TeleopProfile = {
  id: string;
  title: string;
  description: string;
  layout: TeleopLayout;
  defaultConfig: TeleopConfig;
  preferredTopics: TeleopTopicConfig[];
  compatibleMessageTypes: string[];
  topicSelectionMode: "strict" | "compatible";
  showAdvancedMapping: boolean;
};

const DEFAULT_TWIST_CONFIG: TeleopConfig = {
  topic: "/cmd_vel",
  publishRate: 10,
  upButton: { field: "linear-x", value: 1 },
  downButton: { field: "linear-x", value: -1 },
  leftButton: { field: "angular-z", value: 1 },
  rightButton: { field: "angular-z", value: -1 },
};

const TWIST_TYPES = [
  "geometry_msgs/msg/Twist",
  "geometry_msgs/Twist",
];

const TWIST_STAMPED_TYPES = [
  "geometry_msgs/msg/TwistStamped",
  "geometry_msgs/TwistStamped",
];

const GENERIC_TWIST_TYPES = [...TWIST_TYPES, ...TWIST_STAMPED_TYPES];

const GENERIC_PROFILE: TeleopProfile = {
  id: "generic",
  title: "Teleop Control",
  description: "Keyboard teleop using compatible velocity command topics.",
  layout: "twist-pad",
  defaultConfig: DEFAULT_TWIST_CONFIG,
  preferredTopics: [
    { topic: "/cmd_vel", type: "geometry_msgs/msg/Twist" },
    { topic: "/cmd_vel_raw", type: "geometry_msgs/msg/Twist" },
  ],
  compatibleMessageTypes: GENERIC_TWIST_TYPES,
  topicSelectionMode: "compatible",
  showAdvancedMapping: true,
};

const PROFILES: Record<string, TeleopProfile> = {
  simple_robot: {
    ...GENERIC_PROFILE,
    id: "simple_robot",
    preferredTopics: [
      { topic: "/cmd_vel_raw", type: "geometry_msgs/msg/Twist", label: "Primary velocity command" },
      { topic: "/cmd_vel", type: "geometry_msgs/msg/Twist" },
    ],
    defaultConfig: {
      ...DEFAULT_TWIST_CONFIG,
      topic: "/cmd_vel_raw",
    },
  },
  turtlebot4: {
    ...GENERIC_PROFILE,
    id: "turtlebot4",
    title: "TurtleBot4 Teleop",
    description: "Uses the diff-drive controller command topic for operator teleop.",
    preferredTopics: [
      {
        topic: "/turtlebot4/diffdrive_controller/cmd_vel",
        type: "geometry_msgs/msg/TwistStamped",
        label: "Diff-drive controller",
      },
    ],
    defaultConfig: {
      ...DEFAULT_TWIST_CONFIG,
      topic: "/turtlebot4/diffdrive_controller/cmd_vel",
    },
    compatibleMessageTypes: TWIST_STAMPED_TYPES,
    topicSelectionMode: "strict",
    showAdvancedMapping: false,
  },
  px4: {
    ...GENERIC_PROFILE,
    id: "px4",
    title: "Drone Velocity Teleop",
    description: "Publishes MAVROS velocity setpoints for manual motion testing.",
    preferredTopics: [
      {
        topic: "/mavros/setpoint_velocity/cmd_vel",
        type: "geometry_msgs/msg/TwistStamped",
        label: "MAVROS velocity setpoint",
      },
    ],
    defaultConfig: {
      ...DEFAULT_TWIST_CONFIG,
      topic: "/mavros/setpoint_velocity/cmd_vel",
    },
    compatibleMessageTypes: TWIST_STAMPED_TYPES,
    topicSelectionMode: "compatible",
    showAdvancedMapping: true,
  },
  ardupilot: {
    ...GENERIC_PROFILE,
    id: "ardupilot",
    title: "Drone Velocity Teleop",
    description: "Publishes MAVROS velocity setpoints for manual motion testing.",
    preferredTopics: [
      {
        topic: "/mavros/setpoint_velocity/cmd_vel",
        type: "geometry_msgs/msg/TwistStamped",
        label: "MAVROS velocity setpoint",
      },
    ],
    defaultConfig: {
      ...DEFAULT_TWIST_CONFIG,
      topic: "/mavros/setpoint_velocity/cmd_vel",
    },
    compatibleMessageTypes: TWIST_STAMPED_TYPES,
    topicSelectionMode: "compatible",
    showAdvancedMapping: true,
  },
  lerobot: {
    ...GENERIC_PROFILE,
    id: "lerobot",
    title: "LeRobot Teleop",
    description: "This robot uses a dedicated teleop workflow rather than a velocity topic.",
    layout: "custom",
    preferredTopics: [],
    defaultConfig: {
      ...DEFAULT_TWIST_CONFIG,
      topic: undefined,
    },
    topicSelectionMode: "strict",
    showAdvancedMapping: false,
  },
};

export function getTeleopProfile(vmConfigId: string): TeleopProfile {
  return PROFILES[vmConfigId] ?? GENERIC_PROFILE;
}

export function getTeleopStorageKey(vmConfigId: string): string {
  return `teleopConfig:${vmConfigId || "generic"}`;
}

