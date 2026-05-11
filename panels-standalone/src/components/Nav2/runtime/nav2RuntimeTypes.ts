import type { Subscription } from "../../../ros2-bridge";
import type { TfEdgeSnapshot, TfGraphSnapshot } from "tensorfleet-util/ros/ros-bridge-api";

export type GoalState =
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

export type ValidationSummaryState = "blocked" | "pass" | "fail" | "in-progress";
export type HealthSeverity = "healthy" | "warning" | "error" | "unknown" | "pending";

export type TopicHealthConfig = {
  topic: string;
  label: string;
  type: string;
  staleAfterMs: number;
  isStatic?: boolean;
};

export type TopicHealth = TopicHealthConfig & {
  advertised: boolean;
  lastMessageAt: number | null;
  status: "missing" | "advertised" | "receiving" | "stale";
};

export type LifecycleSpec = {
  node: string;
  required: boolean;
  evidenceTopics: string[];
};

export type LifecycleCheck = {
  pending: boolean;
  lastCheckedAt: number | null;
  response: Record<string, unknown> | null;
  error: string | null;
};

export type LifecycleHealth = {
  node: string;
  required: boolean;
  serviceName: string;
  status: HealthSeverity;
  detail: string;
  response: Record<string, unknown> | null;
};

export type TfHealth = {
  status: "healthy" | "error" | "pending";
  detail: string;
  baseFrame: string | null;
  missingFrames: string[];
  missingEdges: string[];
  staleEdges: string[];
};

export type ValidationSummary = {
  state: ValidationSummaryState;
  title: string;
  detail: string;
};

export type ActiveGoal = {
  uuid: number[];
  key: string;
};

export type PreflightStatus = {
  state: "pending" | "ready" | "blocked";
  missingTopics: string[];
  missingServices: string[];
};

export type Nav2RequestState = {
  type: "idle" | "info" | "error";
  message: string;
};

export type GoalCoordinates = {
  x: number;
  y: number;
  yaw: number;
};

export type PoseCoordinates = {
  x: number;
  y: number;
  yaw: number | null;
};

export type Nav2RuntimeState = {
  connectionStatus: "connected" | "connecting" | "disconnected";
  connectedAt: number | null;
  availableTopics: Subscription[];
  availableServices: string[];
  messageTimestamps: Record<string, number | null>;
  odomMessage: Record<string, unknown> | null;
  poseMessage: Record<string, unknown> | null;
  planMessage: Record<string, unknown> | null;
  transformedPlanMessage: Record<string, unknown> | null;
  cmdVelNavMessage: Record<string, unknown> | null;
  stopStatusMessage: Record<string, unknown> | null;
  actionStatusMessage: Record<string, unknown> | null;
  actionFeedbackMessage: Record<string, unknown> | null;
  tfGraphSnapshot: TfGraphSnapshot;
  goalState: GoalState;
  activeGoal: ActiveGoal | null;
  sendGoalResponse: Record<string, unknown> | null;
  resultResponse: Record<string, unknown> | null;
  lifecycleChecks: Record<string, LifecycleCheck>;
  requestState: Nav2RequestState;
  isSendingGoal: boolean;
  isCancelingGoal: boolean;
  preflightStatus: PreflightStatus;
  activeGoalStatusEntry: Record<string, unknown> | null;
  activeGoalFeedback: Record<string, unknown> | null;
  robotPose: Record<string, unknown> | null;
  currentMapPose: Record<string, unknown> | null;
  helperPose: Record<string, unknown> | null;
  helperPoseSource: string;
  currentMapCoordinates: PoseCoordinates | null;
  lifecycleHealth: LifecycleHealth[];
  tfHealth: TfHealth;
  knownTfFrames: string[];
  validationSummary: ValidationSummary;
  topicHealth: TopicHealth[];
  planPointCount: number;
  transformedPlanPointCount: number;
  cmdVelSummary: string;
  feedbackDistanceRemaining: unknown;
  feedbackRecoveries: unknown;
  feedbackNavigationTime: unknown;
  feedbackEta: unknown;
  sendGoal: (goal: GoalCoordinates) => Promise<void>;
  cancelGoal: () => Promise<void>;
  fillGoalFromCurrentPose: () => GoalCoordinates | null;
  fillSmallForwardTestGoal: () => GoalCoordinates | null;
  clearRequestState: () => void;
};

export type { Subscription, TfEdgeSnapshot, TfGraphSnapshot };
