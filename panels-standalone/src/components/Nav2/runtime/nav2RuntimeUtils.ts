import {
  REQUIRED_ACTION_SERVICES,
  REQUIRED_ACTION_TOPICS,
  TF_EDGE_STALE_AFTER_MS,
  TOPIC_HEALTH_CONFIGS,
  PRE_FLIGHT_DISCOVERY_GRACE_MS,
  LIFECYCLE_SPECS,
} from "./nav2RuntimeConstants";
import type {
  GoalState,
  LifecycleCheck,
  LifecycleHealth,
  PoseCoordinates,
  PreflightStatus,
  Subscription,
  TfEdgeSnapshot,
  TfGraphSnapshot,
  TfHealth,
  TopicHealth,
  ValidationSummary,
} from "./nav2RuntimeTypes";

export function nowMs(): number {
  return Date.now();
}

type ServiceDiscoveryEntry = string | { service?: unknown; name?: unknown };

export function normalizeServiceNames(services: unknown): string[] {
  if (!Array.isArray(services)) {
    return [];
  }
  return services.flatMap((entry: ServiceDiscoveryEntry) => {
    if (typeof entry === "string") {
      return entry;
    }
    const serviceName = typeof entry.service === "string" ? entry.service : typeof entry.name === "string" ? entry.name : null;
    return serviceName ? serviceName : [];
  });
}

function snakeToCamel(value: string): string {
  return value.replace(/_([a-z])/g, (_, letter: string) => letter.toUpperCase());
}

export function getRecordEntry(record: Record<string, unknown>, key: string): unknown {
  if (key in record) {
    return record[key];
  }
  const camelKey = snakeToCamel(key);
  if (camelKey in record) {
    return record[camelKey];
  }
  return null;
}

export function getNestedRecordEntry(value: unknown, keys: string[]): unknown {
  let current: unknown = value;
  for (const key of keys) {
    if (!current || typeof current !== "object") {
      return null;
    }
    current = getRecordEntry(current as Record<string, unknown>, key);
  }
  return current;
}

export function normalizeRosMessage(message: unknown): Record<string, unknown> | null {
  if (!message || typeof message !== "object") {
    return null;
  }
  const record = message as Record<string, unknown>;
  if (record.msg && typeof record.msg === "object") {
    return record.msg as Record<string, unknown>;
  }
  return record;
}

export function formatTimeAgo(timestamp: number | null): string {
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

export function formatRosDuration(value: unknown): string {
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

export function formatNumber(value: unknown, digits = 2): string {
  const numeric = typeof value === "string" ? Number(value) : value;
  if (typeof numeric !== "number" || Number.isNaN(numeric)) {
    return "n/a";
  }
  return numeric.toFixed(digits);
}

export function degreesToQuaternion(yawDegrees: number) {
  const radians = (yawDegrees * Math.PI) / 180;
  const halfYaw = radians / 2;
  return {
    x: 0,
    y: 0,
    z: Math.sin(halfYaw),
    w: Math.cos(halfYaw),
  };
}

export function quaternionToYawDegrees(orientation: unknown): number | null {
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

export function formatPose(pose: unknown): string {
  if (!pose || typeof pose !== "object") {
    return "n/a";
  }
  const record = pose as Record<string, unknown>;
  const position = (record.position ?? {}) as Record<string, unknown>;
  const orientation = (record.orientation ?? {}) as Record<string, unknown>;
  const yaw = quaternionToYawDegrees(orientation);
  return `x ${formatNumber(position.x)}  y ${formatNumber(position.y)}  yaw ${yaw == null ? "n/a" : yaw.toFixed(1)}deg`;
}

export function extractStampedPose(message: Record<string, unknown> | null): Record<string, unknown> | null {
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

export function getPoseCoordinates(
  pose: Record<string, unknown> | null,
): PoseCoordinates | null {
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

export function computeTopicHealth(
  topics: Subscription[],
  messageTimestamps: Record<string, number | null>,
): TopicHealth[] {
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

export function getRecentTimestamp(
  messageTimestamps: Record<string, number | null>,
  topic: string,
): number | null {
  return messageTimestamps[topic] ?? null;
}

function isFresh(timestamp: number | null, staleAfterMs: number): boolean {
  return timestamp != null && nowMs() - timestamp <= staleAfterMs;
}

export function getPathPointCount(message: Record<string, unknown> | null): number {
  const poses = message?.poses;
  return Array.isArray(poses) ? poses.length : 0;
}

export function getTwistSummary(message: Record<string, unknown> | null): string {
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

export function createGoalUuid(): number[] {
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

export function uuidToKey(value: unknown): string | null {
  const uuidArray = extractUuidArray(value);
  if (!uuidArray) {
    return null;
  }
  return uuidArray
    .map((byte) => ((byte % 256) + 256) % 256)
    .map((byte) => byte.toString(16).padStart(2, "0"))
    .join("");
}

export function extractUuidArray(value: unknown): number[] | null {
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

export function getGoalStatusEntry(
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

export function getFeedbackForGoal(
  feedbackMessage: Record<string, unknown> | null,
  activeGoalKey: string | null,
): Record<string, unknown> | null {
  if (!feedbackMessage || !activeGoalKey) {
    return null;
  }
  const goalId = getRecordEntry(feedbackMessage, "goal_id");
  return uuidToKey(goalId) === activeGoalKey ? feedbackMessage : null;
}

export function mapActionStatusCodeToGoalState(statusCode: unknown): GoalState | null {
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

export function mapResultStatusToGoalState(resultMessage: Record<string, unknown> | null): GoalState | null {
  if (!resultMessage) {
    return null;
  }
  return mapActionStatusCodeToGoalState(resultMessage.status);
}

export function getLifecycleServiceName(node: string): string {
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

export function computeLifecycleHealth(
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
        detail: "Lifecycle service not advertised.",
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
    if (
      [
        "inactive",
        "configuring",
        "activating",
        "deactivating",
        "cleaningup",
        "shuttingdown",
        "unconfigured",
      ].includes(normalizedLabel)
    ) {
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

export function findTfEdge(
  edges: TfEdgeSnapshot[],
  parentFrame: string,
  childFrame: string,
): TfEdgeSnapshot | null {
  return edges.find((edge) => edge.parentFrame === parentFrame && edge.childFrame === childFrame) ?? null;
}

function edgeIsStale(edge: TfEdgeSnapshot | null): boolean {
  return !!edge && !edge.isStatic && nowMs() - edge.lastMessageAt > TF_EDGE_STALE_AFTER_MS;
}

export function computeTfHealth(
  snapshot: TfGraphSnapshot,
  connectionStatus: "connected" | "connecting" | "disconnected",
  knownTfFrames: string[],
): TfHealth {
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

  const frameSet = new Set(knownTfFrames);
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

export function computePreflightStatus(
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
    (nowMs() - connectedAt >= PRE_FLIGHT_DISCOVERY_GRACE_MS ||
      availableTopics.length > 0 ||
      availableServices.length > 0);

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

export function computeValidationSummary(
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

export function getFeedbackMetric(feedbackMessage: Record<string, unknown> | null, key: string): unknown {
  if (!feedbackMessage) {
    return null;
  }
  const feedback = getRecordEntry(feedbackMessage, "feedback");
  if (!feedback || typeof feedback !== "object") {
    return null;
  }
  return getRecordEntry(feedback as Record<string, unknown>, key);
}

export function createFeedbackSnippet(
  feedbackMessage: Record<string, unknown> | null,
): Record<string, unknown> | null {
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
