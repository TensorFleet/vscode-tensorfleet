import { DirectionalPadAction, TeleopButtonKey, TeleopConfig } from "./types";

export type TeleopLayout = "ground" | "drone" | "custom";
export type TeleopPadId = "primary" | "secondary";

export type TeleopTopicConfig = {
  topic: string;
  type: string;
  label?: string;
};

export type TeleopButtonDefinition = {
  key: TeleopButtonKey;
  label: string;
  description: string;
};

export type TeleopPadDefinition = {
  id: TeleopPadId;
  title: string;
  description: string;
  actions: Record<DirectionalPadAction, TeleopButtonKey>;
};

export type KeyboardHint = {
  label: string;
  keys: string[];
};

export type TeleopServiceAction = {
  id: string;
  label: string;
  service: string;
  tone?: "primary" | "secondary" | "danger";
  description?: string;
};

export type TeleopManualControlActions = {
  enable: TeleopServiceAction;
  disable: TeleopServiceAction;
};

export type TensorFleetVmConfig = {
  id?: string;
  name?: string;
  description?: string;
  sim_config?: Record<string, unknown>;
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
  buttonDefinitions: TeleopButtonDefinition[];
  pads: TeleopPadDefinition[];
  keyboardHints: KeyboardHint[];
  serviceActions: TeleopServiceAction[];
  manualControlActions?: TeleopManualControlActions;
};

const DEFAULT_GROUND_CONFIG: TeleopConfig = {
  topic: "/cmd_vel",
  publishRate: 10,
  upButton: { field: "linear-x", value: 1 },
  downButton: { field: "linear-x", value: -1 },
  leftButton: { field: "angular-z", value: 1 },
  rightButton: { field: "angular-z", value: -1 },
  secondaryUpButton: { field: "linear-z", value: 1 },
  secondaryDownButton: { field: "linear-z", value: -1 },
  secondaryLeftButton: { field: "angular-z", value: 1 },
  secondaryRightButton: { field: "angular-z", value: -1 },
};

const DEFAULT_DRONE_CONFIG: TeleopConfig = {
  topic: "/drone/cmd_vel",
  publishRate: 10,
  upButton: { field: "linear-x", value: 1 },
  downButton: { field: "linear-x", value: -1 },
  leftButton: { field: "linear-y", value: 1 },
  rightButton: { field: "linear-y", value: -1 },
  secondaryUpButton: { field: "linear-z", value: 1 },
  secondaryDownButton: { field: "linear-z", value: -1 },
  secondaryLeftButton: { field: "angular-z", value: 1 },
  secondaryRightButton: { field: "angular-z", value: -1 },
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

const GROUND_BUTTONS: TeleopButtonDefinition[] = [
  { key: "upButton", label: "Forward", description: "Linear X+" },
  { key: "downButton", label: "Reverse", description: "Linear X-" },
  { key: "leftButton", label: "Turn Left", description: "Yaw +" },
  { key: "rightButton", label: "Turn Right", description: "Yaw -" },
];

const DRONE_BUTTONS: TeleopButtonDefinition[] = [
  { key: "upButton", label: "Forward", description: "Linear X+" },
  { key: "downButton", label: "Backward", description: "Linear X-" },
  { key: "leftButton", label: "Strafe Left", description: "Linear Y+" },
  { key: "rightButton", label: "Strafe Right", description: "Linear Y-" },
  { key: "secondaryUpButton", label: "Climb", description: "Linear Z+" },
  { key: "secondaryDownButton", label: "Descend", description: "Linear Z-" },
  { key: "secondaryLeftButton", label: "Yaw Left", description: "Angular Z+" },
  { key: "secondaryRightButton", label: "Yaw Right", description: "Angular Z-" },
];

const GROUND_PADS: TeleopPadDefinition[] = [
  {
    id: "primary",
    title: "Drive",
    description: "Forward, reverse, and turning.",
    actions: {
      [DirectionalPadAction.UP]: "upButton",
      [DirectionalPadAction.DOWN]: "downButton",
      [DirectionalPadAction.LEFT]: "leftButton",
      [DirectionalPadAction.RIGHT]: "rightButton",
    },
  },
];

const DRONE_PADS: TeleopPadDefinition[] = [
  {
    id: "primary",
    title: "Planar Velocity",
    description: "Forward/back and left/right strafe velocity.",
    actions: {
      [DirectionalPadAction.UP]: "upButton",
      [DirectionalPadAction.DOWN]: "downButton",
      [DirectionalPadAction.LEFT]: "leftButton",
      [DirectionalPadAction.RIGHT]: "rightButton",
    },
  },
  {
    id: "secondary",
    title: "Altitude + Yaw",
    description: "Climb/descend and yaw rate control.",
    actions: {
      [DirectionalPadAction.UP]: "secondaryUpButton",
      [DirectionalPadAction.DOWN]: "secondaryDownButton",
      [DirectionalPadAction.LEFT]: "secondaryLeftButton",
      [DirectionalPadAction.RIGHT]: "secondaryRightButton",
    },
  },
];

const GROUND_KEYBOARD_HINTS: KeyboardHint[] = [
  { label: "Move", keys: ["W", "S", "↑", "↓"] },
  { label: "Turn", keys: ["A", "D", "←", "→"] },
];

const DRONE_KEYBOARD_HINTS: KeyboardHint[] = [
  { label: "Planar", keys: ["W", "A", "S", "D"] },
  { label: "Yaw", keys: ["Q", "E", "←", "→"] },
  { label: "Altitude", keys: ["R", "F", "PgUp", "PgDn"] },
];

const DRONE_SERVICE_ACTIONS: TeleopServiceAction[] = [
  { id: "arm", label: "Arm", service: "/drone/arm", description: "Prepare motors for flight." },
  { id: "takeoff", label: "Take Off", service: "/drone/takeoff", description: "Lift into controlled flight." },
  { id: "land", label: "Land", service: "/drone/land", tone: "secondary", description: "Exit flight and descend." },
  { id: "stop", label: "Stop Motion", service: "/drone/stop", tone: "secondary", description: "Send zero motion and clear inputs." },
  { id: "disarm", label: "Disarm", service: "/drone/disarm", tone: "danger", description: "Cut motors when safe." },
];

const DRONE_MANUAL_CONTROL_ACTIONS: TeleopManualControlActions = {
  enable: {
    id: "enable_external",
    label: "Enable Manual Control",
    service: "/drone/enable_external_control",
    tone: "primary",
    description: "Switch to OFFBOARD or GUIDED for pad input.",
  },
  disable: {
    id: "disable_external",
    label: "Exit Manual Control",
    service: "/drone/disable_external_control",
    tone: "secondary",
    description: "Hand control back to hold mode.",
  },
};

const GENERIC_PROFILE: TeleopProfile = {
  id: "generic",
  title: "Teleop Control",
  description: "Keyboard teleop using compatible velocity command topics.",
  layout: "ground",
  defaultConfig: DEFAULT_GROUND_CONFIG,
  preferredTopics: [
    { topic: "/cmd_vel", type: "geometry_msgs/msg/Twist" },
    { topic: "/cmd_vel_raw", type: "geometry_msgs/msg/Twist" },
  ],
  compatibleMessageTypes: GENERIC_TWIST_TYPES,
  topicSelectionMode: "compatible",
  showAdvancedMapping: true,
  buttonDefinitions: GROUND_BUTTONS,
  pads: GROUND_PADS,
  keyboardHints: GROUND_KEYBOARD_HINTS,
  serviceActions: [],
};

function createGroundProfile(overrides: Partial<TeleopProfile>): TeleopProfile {
  return {
    ...GENERIC_PROFILE,
    ...overrides,
    defaultConfig: {
      ...DEFAULT_GROUND_CONFIG,
      ...overrides.defaultConfig,
    },
    preferredTopics: overrides.preferredTopics ?? GENERIC_PROFILE.preferredTopics,
    compatibleMessageTypes: overrides.compatibleMessageTypes ?? GENERIC_PROFILE.compatibleMessageTypes,
    buttonDefinitions: overrides.buttonDefinitions ?? GROUND_BUTTONS,
    pads: overrides.pads ?? GROUND_PADS,
    keyboardHints: overrides.keyboardHints ?? GROUND_KEYBOARD_HINTS,
    serviceActions: overrides.serviceActions ?? [],
  };
}

function createDroneProfile(id: string, title: string, description: string): TeleopProfile {
  return {
    id,
    title,
    description,
    layout: "drone",
    defaultConfig: DEFAULT_DRONE_CONFIG,
    preferredTopics: [
      { topic: "/drone/cmd_vel", type: "geometry_msgs/msg/Twist", label: "Drone command velocity" },
    ],
    compatibleMessageTypes: GENERIC_TWIST_TYPES,
    topicSelectionMode: "compatible",
    showAdvancedMapping: true,
    buttonDefinitions: DRONE_BUTTONS,
    pads: DRONE_PADS,
    keyboardHints: DRONE_KEYBOARD_HINTS,
    serviceActions: DRONE_SERVICE_ACTIONS,
    manualControlActions: DRONE_MANUAL_CONTROL_ACTIONS,
  };
}

const PROFILES: Record<string, TeleopProfile> = {
  simple_robot: createGroundProfile({
    id: "simple_robot",
    title: "Simple Robot Teleop",
    description: "Ground robot velocity control using `/cmd_vel_raw`.",
    preferredTopics: [
      { topic: "/cmd_vel_raw", type: "geometry_msgs/msg/Twist", label: "Primary velocity command" },
      { topic: "/cmd_vel", type: "geometry_msgs/msg/Twist" },
    ],
    defaultConfig: {
      ...DEFAULT_GROUND_CONFIG,
      topic: "/cmd_vel_raw",
    },
  }),
  turtlebot4: createGroundProfile({
    id: "turtlebot4",
    title: "TurtleBot4 Teleop",
    description: "Operator teleop publishes to `/cmd_vel_raw` before VM-side deadman filtering.",
    preferredTopics: [
      {
        topic: "/cmd_vel_raw",
        type: "geometry_msgs/msg/Twist",
        label: "Primary velocity command",
      },
      { topic: "/cmd_vel", type: "geometry_msgs/msg/Twist" },
    ],
    compatibleMessageTypes: GENERIC_TWIST_TYPES,
    topicSelectionMode: "strict",
    showAdvancedMapping: false,
    defaultConfig: {
      ...DEFAULT_GROUND_CONFIG,
      topic: "/cmd_vel_raw",
    },
  }),
  px4: createDroneProfile(
    "px4",
    "PX4 Drone Teleops",
    "Manual flight teleop using `/drone/cmd_vel` plus VM-side trigger services.",
  ),
  ardupilot: createDroneProfile(
    "ardupilot",
    "ArduPilot Drone Teleops",
    "Manual flight teleop using `/drone/cmd_vel` plus VM-side trigger services.",
  ),
  lerobot: {
    id: "lerobot",
    title: "LeRobot Teleop",
    description: "This robot uses a dedicated teleop workflow rather than a velocity topic.",
    layout: "custom",
    preferredTopics: [],
    compatibleMessageTypes: [],
    topicSelectionMode: "strict",
    showAdvancedMapping: false,
    defaultConfig: {
      ...DEFAULT_GROUND_CONFIG,
      topic: undefined,
    },
    buttonDefinitions: [],
    pads: [],
    keyboardHints: [],
    serviceActions: [],
  },
};

function isDroneVmConfig(vmConfigId: string, vmConfig?: TensorFleetVmConfig | null): boolean {
  if (vmConfigId === "px4" || vmConfigId === "ardupilot") {
    return true;
  }

  const simConfig = vmConfig?.sim_config ?? {};
  return simConfig.gazebo_px4_enabled === "true" || simConfig.gazebo_ardupilot_enabled === "true";
}

export function getTeleopProfile(vmConfigId: string, vmConfig?: TensorFleetVmConfig | null): TeleopProfile {
  if (vmConfigId && PROFILES[vmConfigId]) {
    return PROFILES[vmConfigId]!;
  }

  if (isDroneVmConfig(vmConfigId, vmConfig)) {
    const vehicleLabel = vmConfig?.name?.trim() || "Drone";
    return createDroneProfile(
      vmConfigId || "drone",
      `${vehicleLabel} Teleops`,
      `${vehicleLabel} control via \`/drone/cmd_vel\` and VM-side trigger services.`,
    );
  }

  return GENERIC_PROFILE;
}

export function getTeleopStorageKey(vmConfigId: string): string {
  return `teleopConfig:${vmConfigId || "generic"}`;
}

export function mergeTeleopConfig(defaultConfig: TeleopConfig, savedConfig: unknown): TeleopConfig {
  if (!savedConfig || typeof savedConfig !== "object") {
    return defaultConfig;
  }

  const parsed = savedConfig as Partial<TeleopConfig>;

  return {
    ...defaultConfig,
    ...parsed,
    upButton: { ...defaultConfig.upButton, ...parsed.upButton },
    downButton: { ...defaultConfig.downButton, ...parsed.downButton },
    leftButton: { ...defaultConfig.leftButton, ...parsed.leftButton },
    rightButton: { ...defaultConfig.rightButton, ...parsed.rightButton },
    secondaryUpButton: { ...defaultConfig.secondaryUpButton, ...parsed.secondaryUpButton },
    secondaryDownButton: { ...defaultConfig.secondaryDownButton, ...parsed.secondaryDownButton },
    secondaryLeftButton: { ...defaultConfig.secondaryLeftButton, ...parsed.secondaryLeftButton },
    secondaryRightButton: { ...defaultConfig.secondaryRightButton, ...parsed.secondaryRightButton },
  };
}
