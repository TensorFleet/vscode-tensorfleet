import React, { useEffect, useMemo, useRef, useState } from "react";
import clipboard from "@lichtblick/suite-base/util/clipboard";
import { ros2Bridge, type Subscription } from "../../ros2-bridge";
import {
  type TfEdgeSnapshot,
  type TfGraphSnapshot,
} from "tensorfleet-util/ros/ros-bridge-api";
import { ConnectionSettingsProvider, ConnectionSettingsTrigger } from "../ConnectionSettingsProvider";
import "./Nav2Panel.css";

type GoalState =
  | "blocked"
  | "ready"
  | "sending"
  | "accepted"
  | "executing"
  | "canceling"
  | "succeeded"
  | "canceled"
  | "aborted"
  | "rejected"
  | "unknown";

type ValidationSummaryState = "blocked" | "pass" | "fail" | "in-progress";
type HealthSeverity = "healthy" | "warning" | "error" | "unknown" | "pending";

type TopicHealthConfig = {
  topic: string;
  label: string;
  type: string;
  staleAfterMs: number;
  isStatic?: boolean;
};

type TopicHealth = TopicHealthConfig & {
  advertised: boolean;
  lastMessageAt: number | null;
  status: "missing" | "advertised" | "receiving" | "stale";
};

type LifecycleSpec = {
  node: string;
  required: boolean;
  evidenceTopics: string[];
};

type LifecycleCheck = {
  pending: boolean;
  lastCheckedAt: number | null;
  response: Record<string, unknown> | null;
  error: string | null;
};

type LifecycleHealth = {
  node: string;
  required: boolean;
  serviceName: string;
  status: HealthSeverity;
  detail: string;
  response: Record<string, unknown> | null;
};

type TfHealth = {
  status: "healthy" | "error" | "pending";
  detail: string;
  baseFrame: string | null;
  missingFrames: string[];
  missingEdges: string[];
  staleEdges: string[];
};

type ValidationSummary = {
  state: ValidationSummaryState;
  title: string;
  detail: string;
};

type ActiveGoal = {
  uuid: number[];
  key: string;
};

type PreflightStatus = {
  state: "pending" | "ready" | "blocked";
  missingTopics: string[];
  missingServices: string[];
};

type DebugSection = {
  id: string;
  title: string;
  payload: unknown;
};

type CopyFeedback = {
  type: "idle" | "info" | "error";
  message: string;
};

const VM_CONFIG_ID = (typeof window !== "undefined" ? (window as any).TENSORFLEET_VM_CONFIG_ID : "") ?? "";
const ACTION_PREFIX = "/navigate_to_pose/_action";
const SEND_GOAL_SERVICE = `${ACTION_PREFIX}/send_goal`;
const GET_RESULT_SERVICE = `${ACTION_PREFIX}/get_result`;
const CANCEL_GOAL_SERVICE = `${ACTION_PREFIX}/cancel_goal`;
const ACTION_STATUS_TOPIC = `${ACTION_PREFIX}/status`;
const ACTION_FEEDBACK_TOPIC = `${ACTION_PREFIX}/feedback`;
const TF_TOPIC = "/tf";
const TF_STATIC_TOPIC = "/tf_static";
const PLAN_TOPIC = "/plan";
const TRANSFORMED_GLOBAL_PLAN_TOPIC = "/transformed_global_plan";
const CMD_VEL_NAV_TOPIC = "/cmd_vel_nav";
const STOP_STATUS_TOPIC = "/stop_status";
const TF_EDGE_STALE_AFTER_MS = 5_000;
const PRE_FLIGHT_DISCOVERY_GRACE_MS = 1_500;
const LIFECYCLE_POLL_MS = 5_000;

const REQUIRED_ACTION_SERVICES = [SEND_GOAL_SERVICE, GET_RESULT_SERVICE, CANCEL_GOAL_SERVICE];
const REQUIRED_ACTION_TOPICS = [ACTION_STATUS_TOPIC, ACTION_FEEDBACK_TOPIC, TF_TOPIC, TF_STATIC_TOPIC];

const TOPIC_HEALTH_CONFIGS: TopicHealthConfig[] = [
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
  { topic: TF_STATIC_TOPIC, label: "TF static", type: "tf2_msgs/msg/TFMessage", staleAfterMs: 60_000, isStatic: true },
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
  { topic: CMD_VEL_NAV_TOPIC, label: "Nav cmd_vel", type: "geometry_msgs/msg/TwistStamped", staleAfterMs: 5_000 },
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

const LIFECYCLE_SPECS: LifecycleSpec[] = [
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
    evidenceTopics: [ACTION_STATUS_TOPIC, ACTION_FEEDBACK_TOPIC, TRANSFORMED_GLOBAL_PLAN_TOPIC],
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

function nowMs(): number {
  return Date.now();
}

function snakeToCamel(value: string): string {
  return value.replace(/_([a-z])/g, (_, letter: string) => letter.toUpperCase());
}

function getRecordEntry(record: Record<string, unknown>, key: string): unknown {
  if (key in record) {
    return record[key];
  }
  const camelKey = snakeToCamel(key);
  if (camelKey in record) {
    return record[camelKey];
  }
  return null;
}

function getNestedRecordEntry(value: unknown, keys: string[]): unknown {
  let current: unknown = value;
  for (const key of keys) {
    if (!current || typeof current !== "object") {
      return null;
    }
    current = getRecordEntry(current as Record<string, unknown>, key);
  }
  return current;
}

function normalizeRosMessage(message: unknown): Record<string, unknown> | null {
  if (!message || typeof message !== "object") {
    return null;
  }
  const record = message as Record<string, unknown>;
  if (record.msg && typeof record.msg === "object") {
    return record.msg as Record<string, unknown>;
  }
  return record;
}

function formatTimeAgo(timestamp: number | null): string {
  if (timestamp == null) {
    return "never";
  }
  const deltaMs = Math.max(0, nowMs() - timestamp);
  if (deltaMs < 1_000) {
    return "just now";
  }
  if (deltaMs < 60_000) {
    return `${Math.floor(deltaMs / 1_000)}s ago`;
  }
  return `${Math.floor(deltaMs / 60_000)}m ago`;
}

function formatRosDuration(value: unknown): string {
  if (typeof value === "number" && Number.isFinite(value)) {
    if (value < 60) {
      return `${value.toFixed(1)}s`;
    }
    const minutes = Math.floor(value / 60);
    const seconds = value % 60;
    return `${minutes}m ${seconds.toFixed(0)}s`;
  }
  if (!value || typeof value !== "object") {
    return "n/a";
  }
  const record = value as Record<string, unknown>;
  const sec = Number(getRecordEntry(record, "sec") ?? getRecordEntry(record, "secs") ?? 0);
  const nanosec = Number(getRecordEntry(record, "nanosec") ?? getRecordEntry(record, "nsecs") ?? 0);
  if (!Number.isFinite(sec) || !Number.isFinite(nanosec)) {
    return "n/a";
  }
  const totalSeconds = sec + nanosec / 1_000_000_000;
  if (totalSeconds < 60) {
    return `${totalSeconds.toFixed(1)}s`;
  }
  const minutes = Math.floor(totalSeconds / 60);
  const seconds = totalSeconds % 60;
  return `${minutes}m ${seconds.toFixed(0)}s`;
}

function formatNumber(value: unknown, digits = 2): string {
  const numeric = typeof value === "string" ? Number(value) : value;
  if (typeof numeric !== "number" || Number.isNaN(numeric)) {
    return "n/a";
  }
  return numeric.toFixed(digits);
}

function degreesToQuaternion(yawDegrees: number) {
  const radians = (yawDegrees * Math.PI) / 180;
  const halfYaw = radians / 2;
  return {
    x: 0,
    y: 0,
    z: Math.sin(halfYaw),
    w: Math.cos(halfYaw),
  };
}

function quaternionToYawDegrees(orientation: unknown): number | null {
  if (!orientation || typeof orientation !== "object") {
    return null;
  }
  const record = orientation as Record<string, unknown>;
  const x = Number(record.x ?? 0);
  const y = Number(record.y ?? 0);
  const z = Number(record.z ?? 0);
  const w = Number(record.w ?? 1);
  const sinyCosp = 2 * (w * z + x * y);
  const cosyCosp = 1 - 2 * (y * y + z * z);
  return (Math.atan2(sinyCosp, cosyCosp) * 180) / Math.PI;
}

function formatPose(pose: unknown): string {
  if (!pose || typeof pose !== "object") {
    return "n/a";
  }
  const record = pose as Record<string, unknown>;
  const position = (record.position ?? {}) as Record<string, unknown>;
  const orientation = (record.orientation ?? {}) as Record<string, unknown>;
  const yaw = quaternionToYawDegrees(orientation);
  return `x ${formatNumber(position.x)}  y ${formatNumber(position.y)}  yaw ${yaw == null ? "n/a" : yaw.toFixed(1)}deg`;
}

function extractStampedPose(message: Record<string, unknown> | null): Record<string, unknown> | null {
  if (!message) {
    return null;
  }
  if (message.pose && typeof message.pose === "object") {
    const nestedPose = message.pose as Record<string, unknown>;
    if (nestedPose.pose && typeof nestedPose.pose === "object") {
      return nestedPose.pose as Record<string, unknown>;
    }
    return nestedPose;
  }
  return null;
}

function getPoseCoordinates(pose: Record<string, unknown> | null): { x: number; y: number; yaw: number | null } | null {
  if (!pose) {
    return null;
  }
  const position = (pose.position ?? {}) as Record<string, unknown>;
  const orientation = (pose.orientation ?? {}) as Record<string, unknown>;
  const x = Number(position.x);
  const y = Number(position.y);
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    return null;
  }
  return { x, y, yaw: quaternionToYawDegrees(orientation) };
}

function computeTopicHealth(topics: Subscription[], messageTimestamps: Record<string, number | null>): TopicHealth[] {
  const advertisedTopics = new Set(topics.map((topic) => topic.topic));
  const currentTime = nowMs();

  return TOPIC_HEALTH_CONFIGS.map((config) => {
    const lastMessageAt = messageTimestamps[config.topic] ?? null;
    let status: TopicHealth["status"] = "missing";

    if (advertisedTopics.has(config.topic)) {
      status = "advertised";
    }
    if (lastMessageAt != null) {
      status = config.isStatic || currentTime - lastMessageAt <= config.staleAfterMs ? "receiving" : "stale";
    }

    return {
      ...config,
      advertised: advertisedTopics.has(config.topic),
      lastMessageAt,
      status,
    };
  });
}

function getRecentTimestamp(messageTimestamps: Record<string, number | null>, topic: string): number | null {
  return messageTimestamps[topic] ?? null;
}

function isFresh(timestamp: number | null, staleAfterMs: number): boolean {
  return timestamp != null && nowMs() - timestamp <= staleAfterMs;
}

function getPathPointCount(message: Record<string, unknown> | null): number {
  const poses = message?.poses;
  return Array.isArray(poses) ? poses.length : 0;
}

function getTwistSummary(message: Record<string, unknown> | null): string {
  if (!message) {
    return "n/a";
  }
  const twistCandidate = (message.twist && typeof message.twist === "object" ? message.twist : message) as Record<
    string,
    unknown
  >;
  const linear = (twistCandidate.linear ?? {}) as Record<string, unknown>;
  const angular = (twistCandidate.angular ?? {}) as Record<string, unknown>;
  return `vx ${formatNumber(linear.x)}  wz ${formatNumber(angular.z)}`;
}

function createGoalUuid(): number[] {
  const bytes = new Uint8Array(16);
  if (typeof crypto !== "undefined" && typeof crypto.getRandomValues === "function") {
    crypto.getRandomValues(bytes);
  } else {
    for (let index = 0; index < bytes.length; index += 1) {
      bytes[index] = Math.floor(Math.random() * 256);
    }
  }
  return Array.from(bytes.values());
}

function uuidToKey(value: unknown): string | null {
  const uuidArray = extractUuidArray(value);
  if (!uuidArray) {
    return null;
  }
  return uuidArray.map((byte) => ((byte % 256) + 256) % 256).map((byte) => byte.toString(16).padStart(2, "0")).join("");
}

function extractUuidArray(value: unknown): number[] | null {
  if (Array.isArray(value)) {
    const numbers = value.map((item) => Number(item));
    return numbers.every(Number.isFinite) ? numbers : null;
  }
  if (!value || typeof value !== "object") {
    return null;
  }
  const record = value as Record<string, unknown>;
  const directUuid = getRecordEntry(record, "uuid");
  if (Array.isArray(directUuid)) {
    return extractUuidArray(directUuid);
  }
  const goalId = getRecordEntry(record, "goal_id");
  if (goalId && typeof goalId === "object") {
    return extractUuidArray(goalId);
  }
  const goalInfo = getRecordEntry(record, "goal_info");
  if (goalInfo && typeof goalInfo === "object") {
    return extractUuidArray(goalInfo);
  }
  return null;
}

function getGoalStatusEntry(
  statusMessage: Record<string, unknown> | null,
  activeGoalKey: string | null,
): Record<string, unknown> | null {
  if (!statusMessage || !activeGoalKey) {
    return null;
  }
  const statusList = getRecordEntry(statusMessage, "status_list");
  if (!Array.isArray(statusList)) {
    return null;
  }
  for (const entry of statusList) {
    if (!entry || typeof entry !== "object") {
      continue;
    }
    const record = entry as Record<string, unknown>;
    const goalInfo = getRecordEntry(record, "goal_info");
    if (goalInfo && uuidToKey(goalInfo) === activeGoalKey) {
      return record;
    }
  }
  return null;
}

function getFeedbackForGoal(
  feedbackMessage: Record<string, unknown> | null,
  activeGoalKey: string | null,
): Record<string, unknown> | null {
  if (!feedbackMessage || !activeGoalKey) {
    return null;
  }
  const goalId = getRecordEntry(feedbackMessage, "goal_id");
  return uuidToKey(goalId) === activeGoalKey ? feedbackMessage : null;
}

function mapActionStatusCodeToGoalState(statusCode: unknown): GoalState | null {
  const numeric = Number(statusCode);
  if (!Number.isFinite(numeric)) {
    return null;
  }
  switch (numeric) {
    case 1:
      return "accepted";
    case 2:
      return "executing";
    case 3:
      return "canceling";
    case 4:
      return "succeeded";
    case 5:
      return "canceled";
    case 6:
      return "aborted";
    default:
      return "unknown";
  }
}

function mapResultStatusToGoalState(resultMessage: Record<string, unknown> | null): GoalState | null {
  if (!resultMessage) {
    return null;
  }
  return mapActionStatusCodeToGoalState(resultMessage.status);
}

function getGoalStateClassName(state: GoalState): string {
  return `nav2-goal-state nav2-goal-state--${state}`;
}

function getValidationSummaryClassName(state: ValidationSummaryState): string {
  return `nav2-banner nav2-banner--summary nav2-banner--${state}`;
}

function getTopicChipClassName(state: TopicHealth["status"]): string {
  return `nav2-chip nav2-chip--${state}`;
}

function getHealthChipClassName(state: HealthSeverity | TfHealth["status"]): string {
  return `nav2-chip nav2-chip--${state}`;
}

function getLifecycleServiceName(node: string): string {
  return `/${node}/get_state`;
}

function parseLifecycleLabel(response: Record<string, unknown> | null): string | null {
  if (!response) {
    return null;
  }
  const currentState = response.current_state;
  if (!currentState || typeof currentState !== "object") {
    return null;
  }
  const label = (currentState as Record<string, unknown>).label;
  return typeof label === "string" && label.trim() ? label.trim() : null;
}

function computeLifecycleHealth(
  availableServices: string[],
  lifecycleChecks: Record<string, LifecycleCheck>,
  messageTimestamps: Record<string, number | null>,
): LifecycleHealth[] {
  return LIFECYCLE_SPECS.map((spec) => {
    const serviceName = getLifecycleServiceName(spec.node);
    const serviceAdvertised = availableServices.includes(serviceName);
    const evidenceFresh = spec.evidenceTopics.some((topic) => isFresh(messageTimestamps[topic] ?? null, 10_000));
    const check = lifecycleChecks[spec.node];

    if (!serviceAdvertised) {
      if (evidenceFresh) {
        return {
          node: spec.node,
          required: spec.required,
          serviceName,
          status: "unknown",
          detail: "Lifecycle service missing, but related traffic is active.",
          response: null,
        };
      }
      return {
        node: spec.node,
        required: spec.required,
        serviceName,
        status: spec.required ? "error" : "unknown",
        detail: spec.required ? "Lifecycle service not advertised." : "Lifecycle service not advertised.",
        response: null,
      };
    }

    if (!check || check.pending) {
      return {
        node: spec.node,
        required: spec.required,
        serviceName,
        status: "pending",
        detail: "Waiting for lifecycle state.",
        response: check?.response ?? null,
      };
    }

    if (check.error) {
      return {
        node: spec.node,
        required: spec.required,
        serviceName,
        status: "error",
        detail: check.error,
        response: check.response,
      };
    }

    const label = parseLifecycleLabel(check.response);
    if (!label) {
      return {
        node: spec.node,
        required: spec.required,
        serviceName,
        status: "unknown",
        detail: "Lifecycle response missing current_state label.",
        response: check.response,
      };
    }

    const normalizedLabel = label.toLowerCase();
    if (normalizedLabel === "active") {
      return {
        node: spec.node,
        required: spec.required,
        serviceName,
        status: "healthy",
        detail: "Lifecycle state is active.",
        response: check.response,
      };
    }
    if (["inactive", "configuring", "activating", "deactivating", "cleaningup", "shuttingdown", "unconfigured"].includes(normalizedLabel)) {
      return {
        node: spec.node,
        required: spec.required,
        serviceName,
        status: "warning",
        detail: `Lifecycle state is ${normalizedLabel}.`,
        response: check.response,
      };
    }
    return {
      node: spec.node,
      required: spec.required,
      serviceName,
      status: "error",
      detail: `Lifecycle state is ${normalizedLabel}.`,
      response: check.response,
    };
  });
}

function findTfEdge(
  edges: TfEdgeSnapshot[],
  parentFrame: string,
  childFrame: string,
): TfEdgeSnapshot | null {
  return edges.find((edge) => edge.parentFrame === parentFrame && edge.childFrame === childFrame) ?? null;
}

function edgeIsStale(edge: TfEdgeSnapshot | null): boolean {
  return !!edge && !edge.isStatic && nowMs() - edge.lastMessageAt > TF_EDGE_STALE_AFTER_MS;
}

function computeTfHealth(snapshot: TfGraphSnapshot, connectionStatus: "connected" | "connecting" | "disconnected"): TfHealth {
  if (connectionStatus !== "connected") {
    return {
      status: "pending",
      detail: "Waiting for Foxglove bridge connection.",
      baseFrame: null,
      missingFrames: [],
      missingEdges: [],
      staleEdges: [],
    };
  }

  const frameSet = new Set(ros2Bridge.getKnownTfFrames());
  const baseFrame = frameSet.has("base_link") ? "base_link" : frameSet.has("base_footprint") ? "base_footprint" : null;
  const missingFrames: string[] = [];

  if (!frameSet.has("map")) {
    missingFrames.push("map");
  }
  if (!frameSet.has("odom")) {
    missingFrames.push("odom");
  }
  if (!baseFrame) {
    missingFrames.push("base_link/base_footprint");
  }

  const allEdges = [...snapshot.dynamicEdges, ...snapshot.staticEdges];
  const missingEdges: string[] = [];
  const staleEdges: string[] = [];
  const mapToOdom = findTfEdge(allEdges, "map", "odom");
  const odomToBase = baseFrame ? findTfEdge(allEdges, "odom", baseFrame) : null;

  if (!mapToOdom) {
    missingEdges.push("map -> odom");
  } else if (edgeIsStale(mapToOdom)) {
    staleEdges.push("map -> odom");
  }
  if (baseFrame) {
    if (!odomToBase) {
      missingEdges.push(`odom -> ${baseFrame}`);
    } else if (edgeIsStale(odomToBase)) {
      staleEdges.push(`odom -> ${baseFrame}`);
    }
  }

  if (missingFrames.length > 0 || missingEdges.length > 0 || staleEdges.length > 0) {
    const parts: string[] = [];
    if (missingFrames.length > 0) {
      parts.push(`missing frames: ${missingFrames.join(", ")}`);
    }
    if (missingEdges.length > 0) {
      parts.push(`missing edges: ${missingEdges.join(", ")}`);
    }
    if (staleEdges.length > 0) {
      parts.push(`stale dynamic edges: ${staleEdges.join(", ")}`);
    }
    return {
      status: "error",
      detail: parts.join("; "),
      baseFrame,
      missingFrames,
      missingEdges,
      staleEdges,
    };
  }

  return {
    status: "healthy",
    detail: `Observed usable TF chain map -> odom -> ${baseFrame}.`,
    baseFrame,
    missingFrames,
    missingEdges,
    staleEdges,
  };
}

function computePreflightStatus(
  connectionStatus: "connected" | "connecting" | "disconnected",
  availableTopics: Subscription[],
  availableServices: string[],
  connectedAt: number | null,
): PreflightStatus {
  if (connectionStatus !== "connected") {
    return { state: "pending", missingTopics: [], missingServices: [] };
  }

  const discoverySettled =
    connectedAt != null &&
    (nowMs() - connectedAt >= PRE_FLIGHT_DISCOVERY_GRACE_MS || availableTopics.length > 0 || availableServices.length > 0);

  if (!discoverySettled) {
    return { state: "pending", missingTopics: [], missingServices: [] };
  }

  const topicSet = new Set(availableTopics.map((topic) => topic.topic));
  const missingTopics = REQUIRED_ACTION_TOPICS.filter((topic) => !topicSet.has(topic));
  const missingServices = REQUIRED_ACTION_SERVICES.filter((service) => !availableServices.includes(service));
  return missingTopics.length > 0 || missingServices.length > 0
    ? { state: "blocked", missingTopics, missingServices }
    : { state: "ready", missingTopics: [], missingServices: [] };
}

function computeValidationSummary(
  connectionStatus: "connected" | "connecting" | "disconnected",
  preflightStatus: PreflightStatus,
  goalState: GoalState,
  tfHealth: TfHealth,
  lifecycleHealth: LifecycleHealth[],
): ValidationSummary {
  const lifecycleErrors = lifecycleHealth.filter((entry) => entry.required && entry.status === "error");
  const hasHardInfrastructureFailure = tfHealth.status === "error" || lifecycleErrors.length > 0;

  if (connectionStatus !== "connected") {
    return {
      state: "blocked",
      title: "Validation blocked",
      detail: "Foxglove bridge is disconnected.",
    };
  }

  if (preflightStatus.state === "pending") {
    return {
      state: "in-progress",
      title: "Checking Nav2 prerequisites",
      detail: "Discovering action endpoints and TF topics.",
    };
  }

  if (preflightStatus.state === "blocked") {
    return {
      state: "blocked",
      title: "Validation blocked",
      detail: "Hidden NavigateToPose action endpoints are not fully advertised by Foxglove.",
    };
  }

  if (goalState === "succeeded") {
    if (!hasHardInfrastructureFailure) {
      return {
        state: "pass",
        title: "PASS",
        detail: "NavigateToPose reached terminal success with healthy required TF and lifecycle state.",
      };
    }
    return {
      state: "fail",
      title: "FAIL",
      detail: "Goal succeeded, but required TF or lifecycle validation is failing.",
    };
  }

  if (goalState === "aborted" || goalState === "rejected" || goalState === "canceled" || goalState === "unknown") {
    return {
      state: "fail",
      title: "FAIL",
      detail: "NavigateToPose did not complete successfully.",
    };
  }

  if (goalState === "sending" || goalState === "accepted" || goalState === "executing" || goalState === "canceling") {
    if (hasHardInfrastructureFailure) {
      return {
        state: "fail",
        title: "FAIL",
        detail: "A goal is in progress, but TF or required lifecycle health is failing.",
      };
    }
    return {
      state: "in-progress",
      title: "IN PROGRESS",
      detail: "A NavigateToPose validation goal is active.",
    };
  }

  if (hasHardInfrastructureFailure) {
    return {
      state: "blocked",
      title: "Validation blocked",
      detail: "Required TF or lifecycle health is failing before a validation goal has completed.",
    };
  }

  return {
    state: "in-progress",
    title: "IN PROGRESS",
    detail: "Prerequisites are ready. Send a NavigateToPose goal to validate the stack.",
  };
}

function getFeedbackMetric(feedbackMessage: Record<string, unknown> | null, key: string): unknown {
  if (!feedbackMessage) {
    return null;
  }
  const feedback = getRecordEntry(feedbackMessage, "feedback");
  if (!feedback || typeof feedback !== "object") {
    return null;
  }
  return getRecordEntry(feedback as Record<string, unknown>, key);
}

function createDebugSections(args: {
  sendGoalResponse: Record<string, unknown> | null;
  actionStatusMessage: Record<string, unknown> | null;
  actionFeedbackMessage: Record<string, unknown> | null;
  resultResponse: Record<string, unknown> | null;
  lifecycleChecks: Record<string, LifecycleCheck>;
  tfGraphSnapshot: TfGraphSnapshot;
  planMessage: Record<string, unknown> | null;
  transformedPlanMessage: Record<string, unknown> | null;
  stopStatusMessage: Record<string, unknown> | null;
}) {
  return [
    { id: "send-goal", title: "Latest send_goal response", payload: args.sendGoalResponse },
    { id: "status", title: "Latest action status", payload: args.actionStatusMessage },
    { id: "feedback", title: "Latest action feedback", payload: args.actionFeedbackMessage },
    { id: "result", title: "Latest get_result response", payload: args.resultResponse },
    { id: "lifecycle", title: "Latest lifecycle responses", payload: args.lifecycleChecks },
    { id: "tf-graph", title: "Latest TF graph snapshot", payload: args.tfGraphSnapshot },
    { id: "plan", title: "Latest plan", payload: args.planMessage },
    { id: "transformed-plan", title: "Latest transformed plan", payload: args.transformedPlanMessage },
    { id: "stop-status", title: "Latest stop status", payload: args.stopStatusMessage },
  ] satisfies DebugSection[];
}

function createRawDebugBundle(debugSections: DebugSection[]) {
  return Object.fromEntries(
    debugSections.map((section) => [
      section.id,
      {
        title: section.title,
        payload: section.payload,
      },
    ]),
  );
}

function createRawDebugText(debugSections: DebugSection[]): string {
  return debugSections
    .map((section) => {
      const body = section.payload ? JSON.stringify(section.payload, null, 2) : "No payload received yet.";
      return `## ${section.title}\n${body}`;
    })
    .join("\n\n");
}

function compactJson(value: unknown, maxChars = 320): string {
  if (value == null) {
    return "n/a";
  }
  try {
    const serialized = JSON.stringify(value);
    if (!serialized) {
      return "n/a";
    }
    return serialized.length <= maxChars ? serialized : `${serialized.slice(0, maxChars - 3)}...`;
  } catch {
    return String(value);
  }
}

function createFeedbackSnippet(feedbackMessage: Record<string, unknown> | null): Record<string, unknown> | null {
  if (!feedbackMessage) {
    return null;
  }
  const feedback = getRecordEntry(feedbackMessage, "feedback");
  if (!feedback || typeof feedback !== "object") {
    return null;
  }
  const feedbackRecord = feedback as Record<string, unknown>;
  const currentPose = getRecordEntry(feedbackRecord, "current_pose");
  const currentPoseRecord = currentPose && typeof currentPose === "object" ? (currentPose as Record<string, unknown>) : null;
  const currentPoseValue = getNestedRecordEntry(currentPoseRecord, ["pose"]);
  return {
    distance_remaining: getRecordEntry(feedbackRecord, "distance_remaining"),
    number_of_recoveries: getRecordEntry(feedbackRecord, "number_of_recoveries"),
    navigation_time: getRecordEntry(feedbackRecord, "navigation_time"),
    estimated_time_remaining: getRecordEntry(feedbackRecord, "estimated_time_remaining"),
    current_pose: currentPoseValue ? formatPose(currentPoseValue) : null,
  };
}

function createKeyTfEdges(snapshot: TfGraphSnapshot, baseFrame: string | null) {
  const allEdges = [...snapshot.dynamicEdges, ...snapshot.staticEdges];
  return {
    map_to_odom: findTfEdge(allEdges, "map", "odom"),
    odom_to_base: baseFrame ? findTfEdge(allEdges, "odom", baseFrame) : null,
  };
}

function createLlmHandoffBundle(args: {
  generatedAt: string;
  connectionStatus: "connected" | "connecting" | "disconnected";
  validationSummary: ValidationSummary;
  preflightStatus: PreflightStatus;
  goalState: GoalState;
  activeGoal: ActiveGoal | null;
  goalInput: { x: string; y: string; yaw: string };
  sendGoalResponse: Record<string, unknown> | null;
  activeGoalStatusEntry: Record<string, unknown> | null;
  activeGoalFeedback: Record<string, unknown> | null;
  resultResponse: Record<string, unknown> | null;
  requestState: { type: "idle" | "info" | "error"; message: string };
  lifecycleHealth: LifecycleHealth[];
  tfHealth: TfHealth;
  tfGraphSnapshot: TfGraphSnapshot;
  knownTfFrames: string[];
  topicHealth: TopicHealth[];
  planPointCount: number;
  transformedPlanPointCount: number;
  cmdVelSummary: string;
  messageTimestamps: Record<string, number | null>;
}) {
  const requiredLifecycleFailures = args.lifecycleHealth
    .filter((entry) => entry.required && entry.status === "error")
    .map((entry) => ({
      node: entry.node,
      status: entry.status,
      detail: entry.detail,
    }));
  const lifecycleWarnings = args.lifecycleHealth
    .filter((entry) => entry.status === "warning" || entry.status === "unknown")
    .map((entry) => ({
      node: entry.node,
      status: entry.status,
      detail: entry.detail,
      required: entry.required,
    }));
  const topicAnomalies = args.topicHealth
    .filter((topic) => topic.status === "missing" || topic.status === "stale")
    .map((topic) => ({
      topic: topic.topic,
      label: topic.label,
      status: topic.status,
      lastMessageAgo: formatTimeAgo(topic.lastMessageAt),
    }));
  const feedbackSnippet = createFeedbackSnippet(args.activeGoalFeedback);
  const keyTfEdges = createKeyTfEdges(args.tfGraphSnapshot, args.tfHealth.baseFrame);

  return {
    generatedAt: args.generatedAt,
    panel: "nav2-validation",
    validation: {
      summaryState: args.validationSummary.state,
      title: args.validationSummary.title,
      detail: args.validationSummary.detail,
      connectionStatus: args.connectionStatus,
      preflightState: args.preflightStatus.state,
      missingServices: args.preflightStatus.missingServices,
      missingTopics: args.preflightStatus.missingTopics,
    },
    goal: {
      state: args.goalState,
      activeGoalId: args.activeGoal?.key ?? null,
      requestedPose: args.goalInput,
      accepted: args.sendGoalResponse?.accepted ?? null,
      latestOperatorMessage: args.requestState.type === "idle" ? null : args.requestState.message,
    },
    findings: {
      requiredLifecycleFailures,
      lifecycleWarnings,
      tfHealth: {
        status: args.tfHealth.status,
        detail: args.tfHealth.detail,
        baseFrame: args.tfHealth.baseFrame,
        missingFrames: args.tfHealth.missingFrames,
        missingEdges: args.tfHealth.missingEdges,
        staleEdges: args.tfHealth.staleEdges,
      },
      topicAnomalies,
    },
    evidence: {
      actionStatus: args.activeGoalStatusEntry,
      actionFeedback: feedbackSnippet,
      result: args.resultResponse,
      sendGoalResponse: args.sendGoalResponse,
      supportingTraffic: {
        globalPlanPoints: args.planPointCount,
        transformedPlanPoints: args.transformedPlanPointCount,
        cmdVelSummary: args.cmdVelSummary,
        freshness: {
          actionStatus: formatTimeAgo(getRecentTimestamp(args.messageTimestamps, ACTION_STATUS_TOPIC)),
          actionFeedback: formatTimeAgo(getRecentTimestamp(args.messageTimestamps, ACTION_FEEDBACK_TOPIC)),
          navCmdVel: formatTimeAgo(getRecentTimestamp(args.messageTimestamps, CMD_VEL_NAV_TOPIC)),
          globalPlan: formatTimeAgo(getRecentTimestamp(args.messageTimestamps, PLAN_TOPIC)),
          transformedPlan: formatTimeAgo(getRecentTimestamp(args.messageTimestamps, TRANSFORMED_GLOBAL_PLAN_TOPIC)),
        },
      },
      tf: {
        knownFrames: args.knownTfFrames,
        graphUpdated: formatTimeAgo(args.tfGraphSnapshot.lastUpdatedAt),
        keyEdges: keyTfEdges,
      },
    },
    suggestedScreenshot:
      "Capture the validation summary banner, Lifecycle health, TF frame state, and LLM handoff cards together.",
    note: "This bundle is curated on purpose. Use the Raw debug payloads section only if the model asks for exact wire payloads.",
  };
}

function createLlmSummary(args: {
  generatedAt: string;
  validationSummary: ValidationSummary;
  connectionStatus: "connected" | "connecting" | "disconnected";
  preflightStatus: PreflightStatus;
  goalState: GoalState;
  activeGoal: ActiveGoal | null;
  goalInput: { x: string; y: string; yaw: string };
  sendGoalResponse: Record<string, unknown> | null;
  requestState: { type: "idle" | "info" | "error"; message: string };
  lifecycleHealth: LifecycleHealth[];
  tfHealth: TfHealth;
  activeGoalStatusEntry: Record<string, unknown> | null;
  activeGoalFeedback: Record<string, unknown> | null;
  resultResponse: Record<string, unknown> | null;
  topicHealth: TopicHealth[];
  planPointCount: number;
  transformedPlanPointCount: number;
  cmdVelSummary: string;
  messageTimestamps: Record<string, number | null>;
}) {
  const lines = [
    "# Nav2 Validation Handoff",
    `Generated: ${args.generatedAt}`,
    `Validation: ${args.validationSummary.title} (${args.validationSummary.state})`,
    `Validation detail: ${args.validationSummary.detail}`,
    `Connection: ${args.connectionStatus}`,
    `Preflight: ${args.preflightStatus.state}`,
    `Goal state: ${args.goalState}`,
    `Active goal id: ${args.activeGoal?.key ?? "none"}`,
    `Goal request: x ${args.goalInput.x}  y ${args.goalInput.y}  yaw ${args.goalInput.yaw}deg`,
    `Goal accepted: ${args.sendGoalResponse?.accepted === true ? "yes" : args.sendGoalResponse ? "no" : "n/a"}`,
  ];

  if (args.requestState.type !== "idle") {
    lines.push(`Panel message: ${args.requestState.message}`);
  }

  lines.push("", "## Critical Findings");
  if (args.preflightStatus.state === "blocked") {
    lines.push(`- Missing services: ${args.preflightStatus.missingServices.join(", ") || "none"}`);
    lines.push(`- Missing topics: ${args.preflightStatus.missingTopics.join(", ") || "none"}`);
    lines.push("- Hidden action endpoints must be exposed by Foxglove (`INCLUDE_HIDDEN=true`).");
  }

  if (args.tfHealth.status !== "healthy") {
    lines.push(`- TF: ${args.tfHealth.status} - ${args.tfHealth.detail}`);
  } else {
    lines.push(`- TF: healthy - ${args.tfHealth.detail}`);
  }

  const lifecycleIssues = args.lifecycleHealth.filter((entry) => entry.status !== "healthy");
  if (lifecycleIssues.length === 0) {
    lines.push("- Lifecycle: all required nodes report active.");
  } else {
    lifecycleIssues.forEach((entry) => {
      lines.push(`- Lifecycle ${entry.node}: ${entry.status} - ${entry.detail}`);
    });
  }

  const topicAnomalies = args.topicHealth.filter((topic) => topic.status === "missing" || topic.status === "stale");
  if (topicAnomalies.length > 0) {
    lines.push(
      `- Topic anomalies: ${topicAnomalies
        .slice(0, 4)
        .map((topic) => `${topic.label}=${topic.status}`)
        .join(", ")}`,
    );
  }

  const feedbackSnippet = createFeedbackSnippet(args.activeGoalFeedback);

  lines.push("", "## Action Evidence");
  lines.push(`- Latest status entry: ${compactJson(args.activeGoalStatusEntry)}`);
  lines.push(`- Latest feedback: ${compactJson(feedbackSnippet)}`);
  lines.push(`- Latest result: ${compactJson(args.resultResponse)}`);
  lines.push(`- send_goal response: ${compactJson(args.sendGoalResponse)}`);

  lines.push("", "## Supporting Signals");
  lines.push(`- Global plan points: ${args.planPointCount || "n/a"}`);
  lines.push(`- Transformed plan points: ${args.transformedPlanPointCount || "n/a"}`);
  lines.push(`- cmd_vel_nav: ${args.cmdVelSummary}`);
  lines.push(`- Action status freshness: ${formatTimeAgo(getRecentTimestamp(args.messageTimestamps, ACTION_STATUS_TOPIC))}`);
  lines.push(`- Action feedback freshness: ${formatTimeAgo(getRecentTimestamp(args.messageTimestamps, ACTION_FEEDBACK_TOPIC))}`);
  lines.push(`- Nav cmd_vel freshness: ${formatTimeAgo(getRecentTimestamp(args.messageTimestamps, CMD_VEL_NAV_TOPIC))}`);

  lines.push("", "## Suggested Screenshot");
  lines.push("- Capture the validation summary banner, Lifecycle health, TF frame state, and this LLM handoff card.");
  lines.push("- Paste this summary first. If the model asks for exact payloads, send the LLM JSON bundle or specific raw payload sections.");

  return lines.join("\n");
}

export function Nav2Panel(): React.JSX.Element {
  const [connectionStatus, setConnectionStatus] = useState<"connected" | "connecting" | "disconnected">("connecting");
  const [connectedAt, setConnectedAt] = useState<number | null>(null);
  const [availableTopics, setAvailableTopics] = useState<Subscription[]>([]);
  const [availableServices, setAvailableServices] = useState<string[]>([]);
  const [messageTimestamps, setMessageTimestamps] = useState<Record<string, number | null>>({});
  const [odomMessage, setOdomMessage] = useState<Record<string, unknown> | null>(null);
  const [poseMessage, setPoseMessage] = useState<Record<string, unknown> | null>(null);
  const [planMessage, setPlanMessage] = useState<Record<string, unknown> | null>(null);
  const [transformedPlanMessage, setTransformedPlanMessage] = useState<Record<string, unknown> | null>(null);
  const [cmdVelNavMessage, setCmdVelNavMessage] = useState<Record<string, unknown> | null>(null);
  const [stopStatusMessage, setStopStatusMessage] = useState<Record<string, unknown> | null>(null);
  const [actionStatusMessage, setActionStatusMessage] = useState<Record<string, unknown> | null>(null);
  const [actionFeedbackMessage, setActionFeedbackMessage] = useState<Record<string, unknown> | null>(null);
  const [tfGraphSnapshot, setTfGraphSnapshot] = useState<TfGraphSnapshot>(ros2Bridge.getTfGraphSnapshot());
  const [goalX, setGoalX] = useState("0.0");
  const [goalY, setGoalY] = useState("0.0");
  const [goalYaw, setGoalYaw] = useState("0.0");
  const [goalState, setGoalState] = useState<GoalState>("blocked");
  const [activeGoal, setActiveGoal] = useState<ActiveGoal | null>(null);
  const [sendGoalResponse, setSendGoalResponse] = useState<Record<string, unknown> | null>(null);
  const [resultResponse, setResultResponse] = useState<Record<string, unknown> | null>(null);
  const [lifecycleChecks, setLifecycleChecks] = useState<Record<string, LifecycleCheck>>({});
  const [requestState, setRequestState] = useState<{ type: "idle" | "info" | "error"; message: string }>({
    type: "idle",
    message: "",
  });
  const [copyFeedback, setCopyFeedback] = useState<CopyFeedback>({ type: "idle", message: "" });
  const [isSendingGoal, setIsSendingGoal] = useState(false);
  const [isCancelingGoal, setIsCancelingGoal] = useState(false);
  const activeGoalTokenRef = useRef(0);

  useEffect(() => {
    const updateConnectionStatus = () => {
      setConnectionStatus(ros2Bridge.isConnected() ? "connected" : "disconnected");
    };

    updateConnectionStatus();
    const connectionTimer = window.setInterval(updateConnectionStatus, 1_000);
    const tfSnapshotTimer = window.setInterval(() => {
      setTfGraphSnapshot(ros2Bridge.getTfGraphSnapshot());
    }, 1_000);

    const unsubscribeTopics = ros2Bridge.onAvailableTopicsChanged((topics) => {
      setAvailableTopics(topics);
    });
    const unsubscribeServices = ros2Bridge.onAvailableServicesChanged((services) => {
      setAvailableServices(services);
    });

    const markTopicMessage = (topic: string) => {
      setMessageTimestamps((current) => ({ ...current, [topic]: nowMs() }));
      setTfGraphSnapshot(ros2Bridge.getTfGraphSnapshot());
    };

    const subscriptions = TOPIC_HEALTH_CONFIGS.map((config) =>
      ros2Bridge.subscribe({ topic: config.topic, type: config.type }, (message) => {
        const rosMessage = normalizeRosMessage(message);
        markTopicMessage(config.topic);
        if (config.topic === "/odom") {
          setOdomMessage(rosMessage);
        } else if (config.topic === "/pose") {
          setPoseMessage(rosMessage);
        } else if (config.topic === PLAN_TOPIC) {
          setPlanMessage(rosMessage);
        } else if (config.topic === TRANSFORMED_GLOBAL_PLAN_TOPIC) {
          setTransformedPlanMessage(rosMessage);
        } else if (config.topic === CMD_VEL_NAV_TOPIC) {
          setCmdVelNavMessage(rosMessage);
        } else if (config.topic === STOP_STATUS_TOPIC) {
          setStopStatusMessage(rosMessage);
        } else if (config.topic === ACTION_STATUS_TOPIC) {
          setActionStatusMessage(rosMessage);
        } else if (config.topic === ACTION_FEEDBACK_TOPIC) {
          setActionFeedbackMessage(rosMessage);
        }
      }),
    );

    return () => {
      unsubscribeTopics();
      unsubscribeServices();
      subscriptions.forEach((unsubscribe) => unsubscribe());
      clearInterval(connectionTimer);
      clearInterval(tfSnapshotTimer);
    };
  }, []);

  useEffect(() => {
    if (connectionStatus === "connected") {
      setConnectedAt((current) => current ?? nowMs());
      return;
    }
    setConnectedAt(null);
    setGoalState("blocked");
  }, [connectionStatus]);

  useEffect(() => {
    if (connectionStatus !== "connected") {
      return;
    }

    let cancelled = false;
    const pollLifecycle = async () => {
      await Promise.all(
        LIFECYCLE_SPECS.map(async (spec) => {
          const serviceName = getLifecycleServiceName(spec.node);
          const serviceAdvertised = availableServices.includes(serviceName);

          if (!serviceAdvertised) {
            setLifecycleChecks((current) => ({
              ...current,
              [spec.node]: {
                pending: false,
                lastCheckedAt: current[spec.node]?.lastCheckedAt ?? null,
                response: current[spec.node]?.response ?? null,
                error: null,
              },
            }));
            return;
          }

          setLifecycleChecks((current) => ({
            ...current,
            [spec.node]: {
              pending: true,
              lastCheckedAt: current[spec.node]?.lastCheckedAt ?? null,
              response: current[spec.node]?.response ?? null,
              error: null,
            },
          }));

          try {
            const response = (await ros2Bridge.callService<Record<string, unknown>>(serviceName, {}, { timeoutMs: 5_000 })) ?? null;
            if (cancelled) {
              return;
            }
            setLifecycleChecks((current) => ({
              ...current,
              [spec.node]: {
                pending: false,
                lastCheckedAt: nowMs(),
                response,
                error: null,
              },
            }));
          } catch (error) {
            if (cancelled) {
              return;
            }
            setLifecycleChecks((current) => ({
              ...current,
              [spec.node]: {
                pending: false,
                lastCheckedAt: nowMs(),
                response: current[spec.node]?.response ?? null,
                error: error instanceof Error ? error.message : String(error),
              },
            }));
          }
        }),
      );
    };

    void pollLifecycle();
    const timer = window.setInterval(() => {
      void pollLifecycle();
    }, LIFECYCLE_POLL_MS);

    return () => {
      cancelled = true;
      clearInterval(timer);
    };
  }, [availableServices, connectionStatus]);

  const preflightStatus = useMemo(
    () => computePreflightStatus(connectionStatus, availableTopics, availableServices, connectedAt),
    [availableServices, availableTopics, connectedAt, connectionStatus],
  );

  useEffect(() => {
    if (preflightStatus.state === "blocked") {
      setGoalState("blocked");
    } else if (preflightStatus.state === "ready" && goalState === "blocked") {
      setGoalState("ready");
    }
  }, [goalState, preflightStatus.state]);

  const activeGoalStatusEntry = useMemo(
    () => getGoalStatusEntry(actionStatusMessage, activeGoal?.key ?? null),
    [actionStatusMessage, activeGoal],
  );
  const activeGoalFeedback = useMemo(
    () => getFeedbackForGoal(actionFeedbackMessage, activeGoal?.key ?? null),
    [actionFeedbackMessage, activeGoal],
  );

  useEffect(() => {
    if (!activeGoalStatusEntry) {
      return;
    }
    const nextGoalState = mapActionStatusCodeToGoalState(activeGoalStatusEntry.status);
    if (!nextGoalState) {
      return;
    }
    setGoalState((current) => {
      if (current === "sending" && nextGoalState === "unknown") {
        return current;
      }
      return nextGoalState;
    });
  }, [activeGoalStatusEntry]);

  const robotPose = useMemo(() => extractStampedPose(odomMessage), [odomMessage]);
  const currentMapPose = useMemo(() => extractStampedPose(poseMessage), [poseMessage]);
  const helperPose = currentMapPose ?? robotPose;
  const helperPoseSource = currentMapPose ? "localized pose" : robotPose ? "odometry fallback" : "unavailable";
  const currentMapCoordinates = useMemo(() => getPoseCoordinates(helperPose), [helperPose]);

  const lifecycleHealth = useMemo(
    () => computeLifecycleHealth(availableServices, lifecycleChecks, messageTimestamps),
    [availableServices, lifecycleChecks, messageTimestamps],
  );
  const tfHealth = useMemo(() => computeTfHealth(tfGraphSnapshot, connectionStatus), [connectionStatus, tfGraphSnapshot]);
  const knownTfFrames = useMemo(() => ros2Bridge.getKnownTfFrames(), [tfGraphSnapshot]);
  const validationSummary = useMemo(
    () => computeValidationSummary(connectionStatus, preflightStatus, goalState, tfHealth, lifecycleHealth),
    [connectionStatus, goalState, lifecycleHealth, preflightStatus, tfHealth],
  );

  const topicHealth = useMemo(
    () => computeTopicHealth(availableTopics, messageTimestamps),
    [availableTopics, messageTimestamps],
  );
  const planPointCount = useMemo(() => getPathPointCount(planMessage), [planMessage]);
  const transformedPlanPointCount = useMemo(
    () => getPathPointCount(transformedPlanMessage),
    [transformedPlanMessage],
  );
  const cmdVelSummary = useMemo(() => getTwistSummary(cmdVelNavMessage), [cmdVelNavMessage]);
  const debugSections = useMemo(
    () =>
      createDebugSections({
        sendGoalResponse,
        actionStatusMessage,
        actionFeedbackMessage,
        resultResponse,
        lifecycleChecks,
        tfGraphSnapshot,
        planMessage,
        transformedPlanMessage,
        stopStatusMessage,
      }),
    [
      actionFeedbackMessage,
      actionStatusMessage,
      lifecycleChecks,
      planMessage,
      resultResponse,
      sendGoalResponse,
      stopStatusMessage,
      tfGraphSnapshot,
      transformedPlanMessage,
    ],
  );

  const requiredLifecycleFailures = lifecycleHealth.filter((entry) => entry.required && entry.status === "error");
  const feedbackDistanceRemaining = getFeedbackMetric(activeGoalFeedback, "distance_remaining");
  const feedbackRecoveries = getFeedbackMetric(activeGoalFeedback, "number_of_recoveries");
  const feedbackNavigationTime = getFeedbackMetric(activeGoalFeedback, "navigation_time");
  const feedbackEta = getFeedbackMetric(activeGoalFeedback, "estimated_time_remaining");
  const handoffGeneratedAt = useMemo(() => new Date().toISOString(), [
    actionFeedbackMessage,
    actionStatusMessage,
    activeGoal,
    availableServices,
    availableTopics,
    cmdVelNavMessage,
    connectionStatus,
    goalState,
    lifecycleChecks,
    messageTimestamps,
    planMessage,
    poseMessage,
    preflightStatus,
    requestState,
    resultResponse,
    sendGoalResponse,
    stopStatusMessage,
    tfGraphSnapshot,
    transformedPlanMessage,
  ]);
  const llmHandoffBundle = useMemo(
    () =>
      createLlmHandoffBundle({
        generatedAt: handoffGeneratedAt,
        connectionStatus,
        validationSummary,
        preflightStatus,
        goalState,
        activeGoal,
        goalInput: { x: goalX, y: goalY, yaw: goalYaw },
        sendGoalResponse,
        activeGoalStatusEntry,
        activeGoalFeedback,
        resultResponse,
        requestState,
        lifecycleHealth,
        tfHealth,
        tfGraphSnapshot,
        knownTfFrames,
        topicHealth,
        planPointCount,
        transformedPlanPointCount,
        cmdVelSummary,
        messageTimestamps,
      }),
    [
      activeGoal,
      activeGoalFeedback,
      activeGoalStatusEntry,
      cmdVelSummary,
      connectionStatus,
      goalState,
      goalX,
      goalY,
      goalYaw,
      handoffGeneratedAt,
      knownTfFrames,
      lifecycleHealth,
      messageTimestamps,
      planPointCount,
      preflightStatus,
      requestState,
      resultResponse,
      sendGoalResponse,
      tfGraphSnapshot,
      tfHealth,
      topicHealth,
      transformedPlanPointCount,
      validationSummary,
    ],
  );
  const llmSummaryText = useMemo(
    () =>
      createLlmSummary({
        generatedAt: handoffGeneratedAt,
        validationSummary,
        connectionStatus,
        preflightStatus,
        goalState,
        activeGoal,
        goalInput: { x: goalX, y: goalY, yaw: goalYaw },
        sendGoalResponse,
        requestState,
        lifecycleHealth,
        tfHealth,
        activeGoalStatusEntry,
        activeGoalFeedback,
        resultResponse,
        topicHealth,
        planPointCount,
        transformedPlanPointCount,
        cmdVelSummary,
        messageTimestamps,
      }),
    [
      activeGoal,
      activeGoalFeedback,
      activeGoalStatusEntry,
      cmdVelSummary,
      connectionStatus,
      goalState,
      goalX,
      goalY,
      goalYaw,
      handoffGeneratedAt,
      lifecycleHealth,
      messageTimestamps,
      planPointCount,
      preflightStatus,
      requestState,
      resultResponse,
      sendGoalResponse,
      tfHealth,
      topicHealth,
      transformedPlanPointCount,
      validationSummary,
    ],
  );
  const llmJsonText = useMemo(() => JSON.stringify(llmHandoffBundle, null, 2), [llmHandoffBundle]);
  const rawDebugBundle = useMemo(() => createRawDebugBundle(debugSections), [debugSections]);
  const rawDebugJsonText = useMemo(() => JSON.stringify(rawDebugBundle, null, 2), [rawDebugBundle]);
  const rawDebugText = useMemo(() => createRawDebugText(debugSections), [debugSections]);

  useEffect(() => {
    if (copyFeedback.type === "idle") {
      return;
    }
    const timer = window.setTimeout(() => {
      setCopyFeedback({ type: "idle", message: "" });
    }, 3_000);
    return () => clearTimeout(timer);
  }, [copyFeedback]);

  async function waitForGoalResult(goal: ActiveGoal, token: number) {
    try {
      const response =
        (await ros2Bridge.callService<Record<string, unknown>>(
          GET_RESULT_SERVICE,
          { goal_id: { uuid: goal.uuid } },
          { timeoutMs: 300_000 },
        )) ?? null;
      if (activeGoalTokenRef.current !== token) {
        return;
      }
      setResultResponse(response);
      const terminalGoalState = mapResultStatusToGoalState(response);
      if (terminalGoalState) {
        setGoalState(terminalGoalState);
      }
    } catch (error) {
      if (activeGoalTokenRef.current !== token) {
        return;
      }
      const message = error instanceof Error ? error.message : String(error);
      setResultResponse({ error: message });
      setRequestState({
        type: "error",
        message: `get_result failed: ${message}`,
      });
      setGoalState("unknown");
    }
  }

  async function handleSendGoal() {
    const x = Number(goalX);
    const y = Number(goalY);
    const yaw = Number(goalYaw);

    if (![x, y, yaw].every(Number.isFinite)) {
      setRequestState({ type: "error", message: "Goal fields must be valid numbers." });
      return;
    }
    if (preflightStatus.state !== "ready") {
      setRequestState({
        type: "error",
        message: "NavigateToPose validation is blocked until all hidden action endpoints are advertised.",
      });
      return;
    }

    const uuid = createGoalUuid();
    const goal: ActiveGoal = { uuid, key: uuidToKey(uuid)! };
    const token = activeGoalTokenRef.current + 1;
    activeGoalTokenRef.current = token;

    setIsSendingGoal(true);
    setGoalState("sending");
    setActiveGoal(goal);
    setSendGoalResponse(null);
    setActionFeedbackMessage(null);
    setActionStatusMessage(null);
    setResultResponse(null);
    setRequestState({ type: "idle", message: "" });

    try {
      const response =
        (await ros2Bridge.callService<Record<string, unknown>>(
          SEND_GOAL_SERVICE,
          {
            goal_id: { uuid },
            goal: {
              pose: {
                header: {
                  frame_id: "map",
                  stamp: { sec: 0, nanosec: 0 },
                },
                pose: {
                  position: { x, y, z: 0 },
                  orientation: degreesToQuaternion(yaw),
                },
              },
              behavior_tree: "",
            },
          },
          { timeoutMs: 10_000 },
        )) ?? null;

      setSendGoalResponse(response);
      const accepted = Boolean(response?.accepted);
      if (!accepted) {
        setGoalState("rejected");
        setRequestState({
          type: "error",
          message: "send_goal rejected the validation request.",
        });
        return;
      }

      setGoalState("accepted");
      setRequestState({
        type: "info",
        message: "NavigateToPose goal accepted. Waiting for feedback, status, and terminal result.",
      });
      void waitForGoalResult(goal, token);
    } catch (error) {
      setGoalState("unknown");
      setRequestState({
        type: "error",
        message: error instanceof Error ? error.message : String(error),
      });
    } finally {
      setIsSendingGoal(false);
    }
  }

  async function handleCancelGoal() {
    if (!activeGoal) {
      setRequestState({ type: "error", message: "No active goal is available to cancel." });
      return;
    }
    setIsCancelingGoal(true);
    setGoalState("canceling");
    try {
      const response =
        (await ros2Bridge.callService<Record<string, unknown>>(
          CANCEL_GOAL_SERVICE,
          {
            goal_info: {
              goal_id: { uuid: activeGoal.uuid },
              stamp: { sec: 0, nanosec: 0 },
            },
          },
          { timeoutMs: 10_000 },
        )) ?? null;
      const goalsCanceling = Array.isArray(response?.goals_canceling) ? response.goals_canceling.length : 0;
      setRequestState({
        type: goalsCanceling > 0 ? "info" : "error",
        message:
          goalsCanceling > 0
            ? "cancel_goal accepted the request. Waiting for terminal canceled state."
            : "cancel_goal returned without any goals_canceling entries.",
      });
    } catch (error) {
      setRequestState({
        type: "error",
        message: error instanceof Error ? error.message : String(error),
      });
    } finally {
      setIsCancelingGoal(false);
    }
  }

  function useCurrentPose() {
    if (!currentMapCoordinates) {
      setRequestState({ type: "error", message: "Current map pose is not available yet." });
      return;
    }
    setGoalX(currentMapCoordinates.x.toFixed(2));
    setGoalY(currentMapCoordinates.y.toFixed(2));
    setGoalYaw((currentMapCoordinates.yaw ?? 0).toFixed(1));
    setRequestState({ type: "info", message: "Filled goal fields from the robot's current map pose." });
  }

  function useSmallForwardTestGoal() {
    if (!currentMapCoordinates) {
      setRequestState({ type: "error", message: "Current map pose is not available yet." });
      return;
    }
    setGoalX((currentMapCoordinates.x + 0.5).toFixed(2));
    setGoalY(currentMapCoordinates.y.toFixed(2));
    setGoalYaw((currentMapCoordinates.yaw ?? 0).toFixed(1));
    setRequestState({
      type: "info",
      message: "Filled a small absolute test goal 0.5 m ahead on map X. Adjust if your scene uses a different axis convention.",
    });
  }

  async function handleCopyLlmSummary() {
    try {
      await clipboard.copy(llmSummaryText);
      setCopyFeedback({ type: "info", message: "Copied compact LLM summary." });
    } catch (error) {
      setCopyFeedback({
        type: "error",
        message: error instanceof Error ? error.message : String(error),
      });
    }
  }

  async function handleCopyLlmJson() {
    try {
      await clipboard.copy(llmJsonText);
      setCopyFeedback({ type: "info", message: "Copied structured LLM JSON bundle." });
    } catch (error) {
      setCopyFeedback({
        type: "error",
        message: error instanceof Error ? error.message : String(error),
      });
    }
  }

  async function handleCopyAllRawDebug() {
    try {
      await clipboard.copy(rawDebugText);
      setCopyFeedback({ type: "info", message: "Copied all raw debug payloads as a combined text dump." });
    } catch (error) {
      setCopyFeedback({
        type: "error",
        message: error instanceof Error ? error.message : String(error),
      });
    }
  }

  async function handleCopyAllRawDebugJson() {
    try {
      await clipboard.copy(rawDebugJsonText);
      setCopyFeedback({ type: "info", message: "Copied all raw debug payloads as structured JSON." });
    } catch (error) {
      setCopyFeedback({
        type: "error",
        message: error instanceof Error ? error.message : String(error),
      });
    }
  }

  return (
    <ConnectionSettingsProvider>
      <div className="nav2-root">
        <header className="nav2-header">
          <div>
            <p className="nav2-eyebrow">TurtleBot4 / Nav2 Validation</p>
            <h1>NavigateToPose validation panel</h1>
            <p className="nav2-subtitle">
              Developer surface for proving <code>NavigateToPose</code> works end-to-end, including action lifecycle,
              lifecycle health, TF frame state, and supporting navigation traffic.
            </p>
          </div>
          <div className="nav2-header__status">
            <span className={getGoalStateClassName(goalState)}>{goalState}</span>
            <span className={`nav2-connection nav2-connection--${connectionStatus}`}>{connectionStatus}</span>
            <ConnectionSettingsTrigger />
          </div>
        </header>

        <div className={getValidationSummaryClassName(validationSummary.state)}>
          <strong>{validationSummary.title}</strong>
          <span>{validationSummary.detail}</span>
        </div>

        {VM_CONFIG_ID !== "turtlebot4" && (
          <div className="nav2-banner">
            This panel is tuned for the <code>turtlebot4</code> VM preset. Current config:{" "}
            <code>{VM_CONFIG_ID || "unknown"}</code>.
          </div>
        )}

        {preflightStatus.state === "blocked" && (
          <div className="nav2-banner nav2-banner--error">
            <strong>Hidden action endpoints are missing.</strong>
            <span>
              Foxglove must advertise hidden interfaces for <code>NavigateToPose</code>. Expected bridge prerequisite:{" "}
              <code>INCLUDE_HIDDEN=true</code>.
            </span>
            <ul className="nav2-inline-list">
              {preflightStatus.missingServices.map((service) => (
                <li key={service}>
                  Missing service: <code>{service}</code>
                </li>
              ))}
              {preflightStatus.missingTopics.map((topic) => (
                <li key={topic}>
                  Missing topic: <code>{topic}</code>
                </li>
              ))}
            </ul>
          </div>
        )}

        {requestState.type !== "idle" && (
          <div className={`nav2-banner nav2-banner--${requestState.type}`}>{requestState.message}</div>
        )}

        <section className="nav2-grid">
          <article className="nav2-card">
            <h2>Goal control</h2>
            <p className="nav2-card__hint">
              Sends a real action request through <code>{SEND_GOAL_SERVICE}</code>. No <code>/goal_pose</code> fallback
              is used in this panel.
            </p>
            <div className="nav2-helper-row">
              <div className="nav2-helper-box">
                <span className="nav2-helper-box__label">Current pose helper</span>
                <strong>{formatPose(helperPose)}</strong>
                <span className="nav2-helper-box__meta">Source: {helperPoseSource}</span>
              </div>
              <div className="nav2-helper-actions">
                <button className="nav2-button nav2-button--secondary" onClick={useCurrentPose} disabled={!currentMapCoordinates}>
                  Use current pose
                </button>
                <button className="nav2-button nav2-button--secondary" onClick={useSmallForwardTestGoal} disabled={!currentMapCoordinates}>
                  Small test goal
                </button>
              </div>
            </div>
            <div className="nav2-form">
              <label>
                <span>X (m)</span>
                <input type="number" value={goalX} onChange={(event) => setGoalX(event.target.value)} />
              </label>
              <label>
                <span>Y (m)</span>
                <input type="number" value={goalY} onChange={(event) => setGoalY(event.target.value)} />
              </label>
              <label>
                <span>Yaw (deg)</span>
                <input type="number" value={goalYaw} onChange={(event) => setGoalYaw(event.target.value)} />
              </label>
            </div>
            <div className="nav2-actions">
              <button onClick={handleSendGoal} disabled={isSendingGoal || preflightStatus.state !== "ready" || connectionStatus !== "connected"}>
                {isSendingGoal ? "Sending..." : "Send validation goal"}
              </button>
              <button
                className="nav2-button nav2-button--secondary"
                onClick={handleCancelGoal}
                disabled={isCancelingGoal || !activeGoal || !["accepted", "executing", "canceling"].includes(goalState)}
              >
                {isCancelingGoal ? "Canceling..." : "Cancel goal"}
              </button>
            </div>
            <dl className="nav2-facts nav2-facts--compact">
              <div>
                <dt>Active goal id</dt>
                <dd>{activeGoal?.key ?? "none"}</dd>
              </div>
              <div>
                <dt>Preflight</dt>
                <dd>{preflightStatus.state}</dd>
              </div>
            </dl>
          </article>

          <article className="nav2-card">
            <h2>Action validation</h2>
            <div className="nav2-kpis">
              <div className="nav2-kpi">
                <span>Action state</span>
                <strong>{goalState}</strong>
              </div>
              <div className="nav2-kpi">
                <span>Goal accepted</span>
                <strong>{sendGoalResponse?.accepted === true ? "yes" : sendGoalResponse ? "no" : "n/a"}</strong>
              </div>
              <div className="nav2-kpi">
                <span>Distance remaining</span>
                <strong>{formatNumber(feedbackDistanceRemaining)}</strong>
              </div>
              <div className="nav2-kpi">
                <span>Recoveries</span>
                <strong>{feedbackRecoveries ?? "n/a"}</strong>
              </div>
              <div className="nav2-kpi">
                <span>Navigation time</span>
                <strong>{formatRosDuration(feedbackNavigationTime)}</strong>
              </div>
              <div className="nav2-kpi">
                <span>ETA</span>
                <strong>{formatRosDuration(feedbackEta)}</strong>
              </div>
            </div>
            <dl className="nav2-facts">
              <div>
                <dt>Robot pose</dt>
                <dd>{formatPose(robotPose)}</dd>
              </div>
              <div>
                <dt>Current map pose</dt>
                <dd>{formatPose(currentMapPose)}</dd>
              </div>
              <div>
                <dt>cmd_vel_nav</dt>
                <dd>{cmdVelSummary}</dd>
              </div>
              <div>
                <dt>Latest action status</dt>
                <dd>{activeGoalStatusEntry ? JSON.stringify(activeGoalStatusEntry) : "No matching status entry yet."}</dd>
              </div>
            </dl>
          </article>
        </section>

        <section className="nav2-grid nav2-grid--bottom">
          <article className="nav2-card">
            <div className="nav2-card__header">
              <h2>Lifecycle health</h2>
              <span className="nav2-card__hint">Required lifecycle state for planner, controller, and BT navigator.</span>
            </div>
            <div className="nav2-health-list">
              {lifecycleHealth.map((entry) => (
                <div className="nav2-health-row" key={entry.node}>
                  <div>
                    <div className="nav2-health-row__label">
                      {entry.node}
                      {!entry.required && <span className="nav2-inline-tag">optional</span>}
                    </div>
                    <code>{entry.serviceName}</code>
                  </div>
                  <div className="nav2-health-row__meta nav2-health-row__meta--stacked">
                    <span className={getHealthChipClassName(entry.status)}>{entry.status}</span>
                    <span>{entry.detail}</span>
                  </div>
                </div>
              ))}
            </div>
          </article>

          <article className="nav2-card">
            <div className="nav2-card__header">
              <h2>TF frame state</h2>
              <span className="nav2-card__hint">Validates the usable chain from map to odom to base frame.</span>
            </div>
            <div className="nav2-health-row">
              <div>
                <div className="nav2-health-row__label">TF chain</div>
                <div>{tfHealth.detail}</div>
              </div>
              <div className="nav2-health-row__meta">
                <span className={getHealthChipClassName(tfHealth.status)}>{tfHealth.status}</span>
                <span>{tfHealth.baseFrame ? `base frame: ${tfHealth.baseFrame}` : "base frame unavailable"}</span>
              </div>
            </div>
            <dl className="nav2-facts nav2-facts--compact">
              <div>
                <dt>Known TF frames</dt>
                <dd>{knownTfFrames.join(", ") || "none"}</dd>
              </div>
              <div>
                <dt>TF graph updated</dt>
                <dd>{formatTimeAgo(tfGraphSnapshot.lastUpdatedAt)}</dd>
              </div>
              <div>
                <dt>Dynamic edges</dt>
                <dd>{tfGraphSnapshot.dynamicEdges.length}</dd>
              </div>
              <div>
                <dt>Static edges</dt>
                <dd>{tfGraphSnapshot.staticEdges.length}</dd>
              </div>
            </dl>
          </article>
        </section>

        <section className="nav2-grid nav2-grid--bottom">
          <article className="nav2-card">
            <div className="nav2-card__header">
              <h2>Supporting traffic</h2>
              <span className="nav2-card__hint">Secondary evidence for plans, costmaps, pose, and command output.</span>
            </div>
            <div className="nav2-kpis">
              <div className="nav2-kpi">
                <span>Global plan points</span>
                <strong>{planPointCount || "n/a"}</strong>
              </div>
              <div className="nav2-kpi">
                <span>Transformed plan points</span>
                <strong>{transformedPlanPointCount || "n/a"}</strong>
              </div>
              <div className="nav2-kpi">
                <span>Action status freshness</span>
                <strong>{formatTimeAgo(getRecentTimestamp(messageTimestamps, ACTION_STATUS_TOPIC))}</strong>
              </div>
              <div className="nav2-kpi">
                <span>Action feedback freshness</span>
                <strong>{formatTimeAgo(getRecentTimestamp(messageTimestamps, ACTION_FEEDBACK_TOPIC))}</strong>
              </div>
              <div className="nav2-kpi">
                <span>Nav cmd_vel freshness</span>
                <strong>{formatTimeAgo(getRecentTimestamp(messageTimestamps, CMD_VEL_NAV_TOPIC))}</strong>
              </div>
              <div className="nav2-kpi">
                <span>Required lifecycle errors</span>
                <strong>{requiredLifecycleFailures.length}</strong>
              </div>
            </div>
          </article>

          <article className="nav2-card">
            <div className="nav2-card__header">
              <h2>Topic health</h2>
              <span className="nav2-card__hint">Advertised vs receiving vs stale for required supporting surfaces.</span>
            </div>
            <div className="nav2-health-list">
              {topicHealth.map((topic) => (
                <div className="nav2-health-row" key={topic.topic}>
                  <div>
                    <div className="nav2-health-row__label">{topic.label}</div>
                    <code>{topic.topic}</code>
                  </div>
                  <div className="nav2-health-row__meta">
                    <span className={getTopicChipClassName(topic.status)}>{topic.status}</span>
                    <span>{formatTimeAgo(topic.lastMessageAt)}</span>
                  </div>
                </div>
              ))}
            </div>
          </article>
        </section>

        <section className="nav2-grid nav2-grid--bottom">
          <article className="nav2-card nav2-card--full">
            <div className="nav2-card__header">
              <div>
                <h2>LLM handoff</h2>
                <p className="nav2-card__hint">
                  Curated export for debugging with an LLM. Paste the summary first, then send the JSON bundle only if
                  the model needs more structure.
                </p>
              </div>
              <div className="nav2-handoff-actions">
                <button className="nav2-button nav2-button--secondary" onClick={handleCopyLlmSummary}>
                  Copy LLM summary
                </button>
                <button className="nav2-button nav2-button--secondary" onClick={handleCopyLlmJson}>
                  Copy LLM JSON
                </button>
              </div>
            </div>
            <div className="nav2-banner nav2-banner--info nav2-handoff-guidance">
              Best screenshot crop: validation summary banner, Lifecycle health, TF frame state, and this handoff card.
              That gives the model structured state plus visual context without the full raw dump.
            </div>
            {copyFeedback.type !== "idle" && (
              <div className={`nav2-banner nav2-banner--${copyFeedback.type} nav2-handoff-copy-state`}>
                {copyFeedback.message}
              </div>
            )}
            <pre className="nav2-handoff-preview">{llmSummaryText}</pre>
            <details className="nav2-debug-section">
              <summary>Preview structured LLM JSON bundle</summary>
              <pre>{llmJsonText}</pre>
            </details>
          </article>
        </section>

        <section className="nav2-grid nav2-grid--bottom">
          <article className="nav2-card nav2-card--full">
            <div className="nav2-card__header">
              <div>
                <h2>Raw debug payloads</h2>
                <span className="nav2-card__hint">Collapsed by default. Expand when you need exact wire payloads.</span>
              </div>
              <div className="nav2-handoff-actions">
                <button className="nav2-button nav2-button--secondary" onClick={handleCopyAllRawDebug}>
                  Copy all raw payloads
                </button>
                <button className="nav2-button nav2-button--secondary" onClick={handleCopyAllRawDebugJson}>
                  Copy raw payloads JSON
                </button>
              </div>
            </div>
            <div className="nav2-debug-stack">
              {debugSections.map((section) => (
                <details className="nav2-debug-section" key={section.id}>
                  <summary>{section.title}</summary>
                  <pre>{section.payload ? JSON.stringify(section.payload, null, 2) : "No payload received yet."}</pre>
                </details>
              ))}
            </div>
          </article>
        </section>
      </div>
    </ConnectionSettingsProvider>
  );
}
