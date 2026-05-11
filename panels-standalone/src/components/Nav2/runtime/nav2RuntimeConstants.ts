import type { LifecycleSpec, TopicHealthConfig } from "./nav2RuntimeTypes";

export const VM_CONFIG_ID =
  (typeof window !== "undefined" ? (window as any).TENSORFLEET_VM_CONFIG_ID : "") ?? "";
export const ACTION_PREFIX = "/navigate_to_pose/_action";
export const SEND_GOAL_SERVICE = `${ACTION_PREFIX}/send_goal`;
export const GET_RESULT_SERVICE = `${ACTION_PREFIX}/get_result`;
export const CANCEL_GOAL_SERVICE = `${ACTION_PREFIX}/cancel_goal`;
export const ACTION_STATUS_TOPIC = `${ACTION_PREFIX}/status`;
export const ACTION_FEEDBACK_TOPIC = `${ACTION_PREFIX}/feedback`;
export const TF_TOPIC = "/tf";
export const TF_STATIC_TOPIC = "/tf_static";
export const PLAN_TOPIC = "/plan";
export const TRANSFORMED_GLOBAL_PLAN_TOPIC = "/transformed_global_plan";
export const CMD_VEL_NAV_TOPIC = "/cmd_vel_nav";
export const STOP_STATUS_TOPIC = "/stop_status";
export const TF_EDGE_STALE_AFTER_MS = 5_000;
export const PRE_FLIGHT_DISCOVERY_GRACE_MS = 1_500;
export const LIFECYCLE_POLL_MS = 5_000;

export const REQUIRED_ACTION_SERVICES = [
  SEND_GOAL_SERVICE,
  GET_RESULT_SERVICE,
  CANCEL_GOAL_SERVICE,
];
export const REQUIRED_ACTION_TOPICS = [
  ACTION_STATUS_TOPIC,
  ACTION_FEEDBACK_TOPIC,
  TF_TOPIC,
  TF_STATIC_TOPIC,
];

export const TOPIC_HEALTH_CONFIGS: TopicHealthConfig[] = [
  { topic: "/map", label: "SLAM map", type: "nav_msgs/msg/OccupancyGrid", staleAfterMs: 15_000 },
  { topic: "/scan", label: "Lidar scan", type: "sensor_msgs/msg/LaserScan", staleAfterMs: 5_000 },
  { topic: "/odom", label: "Odometry", type: "nav_msgs/msg/Odometry", staleAfterMs: 5_000 },
  {
    topic: "/pose",
    label: "Localized pose",
    type: "geometry_msgs/msg/PoseWithCovarianceStamped",
    staleAfterMs: 5_000,
  },
  { topic: TF_TOPIC, label: "TF", type: "tf2_msgs/msg/TFMessage", staleAfterMs: 5_000 },
  {
    topic: TF_STATIC_TOPIC,
    label: "TF static",
    type: "tf2_msgs/msg/TFMessage",
    staleAfterMs: 60_000,
    isStatic: true,
  },
  {
    topic: "/local_costmap/costmap",
    label: "Local costmap",
    type: "nav_msgs/msg/OccupancyGrid",
    staleAfterMs: 15_000,
  },
  {
    topic: "/global_costmap/costmap",
    label: "Global costmap",
    type: "nav_msgs/msg/OccupancyGrid",
    staleAfterMs: 15_000,
  },
  { topic: PLAN_TOPIC, label: "Global plan", type: "nav_msgs/msg/Path", staleAfterMs: 5_000 },
  {
    topic: TRANSFORMED_GLOBAL_PLAN_TOPIC,
    label: "Transformed global plan",
    type: "nav_msgs/msg/Path",
    staleAfterMs: 5_000,
  },
  {
    topic: CMD_VEL_NAV_TOPIC,
    label: "Nav cmd_vel",
    type: "geometry_msgs/msg/TwistStamped",
    staleAfterMs: 5_000,
  },
  {
    topic: STOP_STATUS_TOPIC,
    label: "Stop status",
    type: "irobot_create_msgs/msg/StopStatus",
    staleAfterMs: 15_000,
  },
  {
    topic: ACTION_STATUS_TOPIC,
    label: "Action status",
    type: "action_msgs/msg/GoalStatusArray",
    staleAfterMs: 5_000,
  },
  {
    topic: ACTION_FEEDBACK_TOPIC,
    label: "Action feedback",
    type: "nav2_msgs/action/NavigateToPose_FeedbackMessage",
    staleAfterMs: 5_000,
  },
];

export const LIFECYCLE_SPECS: LifecycleSpec[] = [
  {
    node: "controller_server",
    required: true,
    evidenceTopics: [CMD_VEL_NAV_TOPIC, "/local_costmap/costmap"],
  },
  {
    node: "planner_server",
    required: true,
    evidenceTopics: [PLAN_TOPIC, "/global_costmap/costmap"],
  },
  {
    node: "bt_navigator",
    required: true,
    evidenceTopics: [
      ACTION_STATUS_TOPIC,
      ACTION_FEEDBACK_TOPIC,
      TRANSFORMED_GLOBAL_PLAN_TOPIC,
    ],
  },
  {
    node: "recoveries_server",
    required: false,
    evidenceTopics: [STOP_STATUS_TOPIC],
  },
  {
    node: "amcl",
    required: false,
    evidenceTopics: ["/pose"],
  },
];
