import React, { useEffect, useMemo, useState } from "react";
import { ros2Bridge, type Subscription } from "../../ros2-bridge";
import { ConnectionSettingsProvider, ConnectionSettingsTrigger } from "../ConnectionSettingsProvider";
import "./Nav2Panel.css";

type GoalState = "idle" | "accepted" | "executing" | "canceling" | "succeeded" | "canceled" | "aborted" | "unknown";

type TopicHealthConfig = {
  topic: string;
  label: string;
  type: string;
  staleAfterMs: number;
};

type TopicHealth = TopicHealthConfig & {
  advertised: boolean;
  lastMessageAt: number | null;
  status: "missing" | "advertised" | "receiving" | "stale";
};

const VM_CONFIG_ID = (typeof window !== "undefined" ? (window as any).TENSORFLEET_VM_CONFIG_ID : "") ?? "";
const GOAL_POSE_TOPIC = "/turtlebot4/goal_pose";
const PLAN_TOPIC = "/turtlebot4/plan";
const TRANSFORMED_GLOBAL_PLAN_TOPIC = "/turtlebot4/transformed_global_plan";
const CMD_VEL_NAV_TOPIC = "/turtlebot4/cmd_vel_nav";
const STOP_STATUS_TOPIC = "/turtlebot4/stop_status";

const TOPIC_HEALTH_CONFIGS: TopicHealthConfig[] = [
  { topic: "/turtlebot4/map", label: "SLAM map", type: "nav_msgs/msg/OccupancyGrid", staleAfterMs: 15000 },
  { topic: "/turtlebot4/scan", label: "Lidar scan", type: "sensor_msgs/msg/LaserScan", staleAfterMs: 5000 },
  { topic: "/turtlebot4/odom", label: "Odometry", type: "nav_msgs/msg/Odometry", staleAfterMs: 5000 },
  { topic: "/turtlebot4/pose", label: "Localized pose", type: "geometry_msgs/msg/PoseWithCovarianceStamped", staleAfterMs: 5000 },
  { topic: "/turtlebot4/tf", label: "TF", type: "tf2_msgs/msg/TFMessage", staleAfterMs: 5000 },
  { topic: "/turtlebot4/local_costmap/costmap", label: "Local costmap", type: "nav_msgs/msg/OccupancyGrid", staleAfterMs: 15000 },
  { topic: "/turtlebot4/global_costmap/costmap", label: "Global costmap", type: "nav_msgs/msg/OccupancyGrid", staleAfterMs: 15000 },
  { topic: GOAL_POSE_TOPIC, label: "Goal pose input", type: "geometry_msgs/msg/PoseStamped", staleAfterMs: 15000 },
  { topic: PLAN_TOPIC, label: "Global plan", type: "nav_msgs/msg/Path", staleAfterMs: 5000 },
  { topic: TRANSFORMED_GLOBAL_PLAN_TOPIC, label: "Transformed global plan", type: "nav_msgs/msg/Path", staleAfterMs: 5000 },
  { topic: CMD_VEL_NAV_TOPIC, label: "Nav cmd_vel", type: "geometry_msgs/msg/TwistStamped", staleAfterMs: 5000 },
  { topic: STOP_STATUS_TOPIC, label: "Stop status", type: "irobot_create_msgs/msg/StopStatus", staleAfterMs: 15000 },
];

function nowMs(): number {
  return Date.now();
}

function formatTimeAgo(timestamp: number | null): string {
  if (timestamp == null) {
    return "never";
  }
  const deltaMs = Math.max(0, nowMs() - timestamp);
  if (deltaMs < 1000) {
    return "just now";
  }
  if (deltaMs < 60000) {
    return `${Math.floor(deltaMs / 1000)}s ago`;
  }
  return `${Math.floor(deltaMs / 60000)}m ago`;
}

function formatRosDuration(value: unknown): string {
  if (!value || typeof value !== "object") {
    return "n/a";
  }
  const record = value as Record<string, unknown>;
  const sec = Number(record.sec ?? record.secs ?? 0);
  const nanosec = Number(record.nanosec ?? record.nsecs ?? 0);
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
  if (typeof value !== "number" || Number.isNaN(value)) {
    return "n/a";
  }
  return value.toFixed(digits);
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
      status = currentTime - lastMessageAt > config.staleAfterMs ? "stale" : "receiving";
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

function inferGoalState(messageTimestamps: Record<string, number | null>): GoalState {
  const goalFresh = isFresh(getRecentTimestamp(messageTimestamps, GOAL_POSE_TOPIC), 15_000);
  const transformedPlanFresh = isFresh(getRecentTimestamp(messageTimestamps, TRANSFORMED_GLOBAL_PLAN_TOPIC), 5_000);
  const planFresh = isFresh(getRecentTimestamp(messageTimestamps, PLAN_TOPIC), 5_000);
  const cmdVelFresh = isFresh(getRecentTimestamp(messageTimestamps, CMD_VEL_NAV_TOPIC), 5_000);

  if (!goalFresh && !planFresh && !cmdVelFresh) {
    return "idle";
  }
  if (goalFresh && (transformedPlanFresh || cmdVelFresh)) {
    return "executing";
  }
  if (goalFresh || planFresh) {
    return "accepted";
  }
  return "unknown";
}

function getTopicChipClassName(state: TopicHealth["status"]): string {
  return `nav2-chip nav2-chip--${state}`;
}

function getGoalStateClassName(state: GoalState): string {
  return `nav2-goal-state nav2-goal-state--${state}`;
}

export function Nav2Panel(): React.JSX.Element {
  const [connectionStatus, setConnectionStatus] = useState<"connected" | "connecting" | "disconnected">("connecting");
  const [availableTopics, setAvailableTopics] = useState<Subscription[]>([]);
  const [messageTimestamps, setMessageTimestamps] = useState<Record<string, number | null>>({});
  const [odomMessage, setOdomMessage] = useState<Record<string, unknown> | null>(null);
  const [poseMessage, setPoseMessage] = useState<Record<string, unknown> | null>(null);
  const [goalPoseMessage, setGoalPoseMessage] = useState<Record<string, unknown> | null>(null);
  const [planMessage, setPlanMessage] = useState<Record<string, unknown> | null>(null);
  const [transformedPlanMessage, setTransformedPlanMessage] = useState<Record<string, unknown> | null>(null);
  const [cmdVelNavMessage, setCmdVelNavMessage] = useState<Record<string, unknown> | null>(null);
  const [stopStatusMessage, setStopStatusMessage] = useState<Record<string, unknown> | null>(null);
  const [goalX, setGoalX] = useState("0.0");
  const [goalY, setGoalY] = useState("0.0");
  const [goalYaw, setGoalYaw] = useState("0.0");
  const [requestState, setRequestState] = useState<{ type: "idle" | "info" | "error"; message: string }>({
    type: "idle",
    message: "",
  });
  const [isSendingGoal, setIsSendingGoal] = useState(false);

  useEffect(() => {
    const updateConnectionStatus = () => {
      setConnectionStatus(ros2Bridge.isConnected() ? "connected" : "disconnected");
    };

    updateConnectionStatus();
    const connectionTimer = window.setInterval(updateConnectionStatus, 1000);
    const unsubscribeTopics = ros2Bridge.onAvailableTopicsChanged((topics) => {
      setAvailableTopics(topics);
    });

    const markTopicMessage = (topic: string) => {
      setMessageTimestamps((current) => ({ ...current, [topic]: nowMs() }));
    };

    const unsubscribers = TOPIC_HEALTH_CONFIGS.map((config) =>
      ros2Bridge.subscribe({ topic: config.topic, type: config.type }, (message) => {
        markTopicMessage(config.topic);
        if (config.topic === "/turtlebot4/odom") {
          setOdomMessage(message as Record<string, unknown>);
        } else if (config.topic === "/turtlebot4/pose") {
          setPoseMessage(message as Record<string, unknown>);
        } else if (config.topic === GOAL_POSE_TOPIC) {
          setGoalPoseMessage(message as Record<string, unknown>);
        } else if (config.topic === PLAN_TOPIC) {
          setPlanMessage(message as Record<string, unknown>);
        } else if (config.topic === TRANSFORMED_GLOBAL_PLAN_TOPIC) {
          setTransformedPlanMessage(message as Record<string, unknown>);
        } else if (config.topic === CMD_VEL_NAV_TOPIC) {
          setCmdVelNavMessage(message as Record<string, unknown>);
        } else if (config.topic === STOP_STATUS_TOPIC) {
          setStopStatusMessage(message as Record<string, unknown>);
        }
      }),
    );

    return () => {
      unsubscribeTopics();
      unsubscribers.forEach((unsubscribe) => unsubscribe());
      clearInterval(connectionTimer);
    };
  }, []);

  const goalState = useMemo<GoalState>(() => inferGoalState(messageTimestamps), [messageTimestamps]);

  const robotPose = useMemo(() => extractStampedPose(odomMessage), [odomMessage]);
  const currentMapPose = useMemo(() => extractStampedPose(poseMessage), [poseMessage]);
  const helperPose = currentMapPose ?? robotPose;
  const helperPoseSource = currentMapPose ? "localized pose" : robotPose ? "odometry fallback" : "unavailable";
  const currentMapCoordinates = useMemo(() => getPoseCoordinates(helperPose), [helperPose]);

  const goalPose = useMemo(() => goalPoseMessage?.pose ?? null, [goalPoseMessage]);
  const planPointCount = useMemo(() => getPathPointCount(planMessage), [planMessage]);
  const transformedPlanPointCount = useMemo(
    () => getPathPointCount(transformedPlanMessage),
    [transformedPlanMessage],
  );
  const cmdVelSummary = useMemo(() => getTwistSummary(cmdVelNavMessage), [cmdVelNavMessage]);
  const topicHealth = useMemo(
    () => computeTopicHealth(availableTopics, messageTimestamps),
    [availableTopics, messageTimestamps],
  );

  function handleSendGoal() {
    const x = Number(goalX);
    const y = Number(goalY);
    const yaw = Number(goalYaw);

    if (![x, y, yaw].every(Number.isFinite)) {
      setRequestState({ type: "error", message: "Goal fields must be valid numbers." });
      return;
    }

    setIsSendingGoal(true);
    setRequestState({ type: "idle", message: "" });

    try {
      ros2Bridge.publish(GOAL_POSE_TOPIC, "geometry_msgs/msg/PoseStamped", {
        header: {
          frame_id: "map",
          stamp: { sec: 0, nanosec: 0 },
        },
        pose: {
          position: { x, y, z: 0 },
          orientation: degreesToQuaternion(yaw),
        },
      });
      setRequestState({
        type: "info",
        message: "Published goal to /turtlebot4/goal_pose. Foxglove bridge does not expose the hidden NavigateToPose action service, so acceptance/result are not available here.",
      });
    } catch (error) {
      setRequestState({
        type: "error",
        message: error instanceof Error ? error.message : String(error),
      });
    } finally {
      setIsSendingGoal(false);
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
      message: "Filled a small absolute test goal 0.5 m ahead on map X. Adjust if the map axes differ from your scene.",
    });
  }

  return (
    <ConnectionSettingsProvider>
      <div className="nav2-root">
        <header className="nav2-header">
          <div>
            <p className="nav2-eyebrow">TurtleBot4 / Nav2</p>
            <h1>Nav2 operator panel</h1>
            <p className="nav2-subtitle">
              Uses the visible Nav2 bridge surfaces: <code>/turtlebot4/goal_pose</code>, <code>/turtlebot4/plan</code>,
              and <code>/turtlebot4/transformed_global_plan</code>.
            </p>
          </div>
          <div className="nav2-header__status">
            <span className={getGoalStateClassName(goalState)}>{goalState}</span>
            <span className={`nav2-connection nav2-connection--${connectionStatus}`}>{connectionStatus}</span>
            <ConnectionSettingsTrigger />
          </div>
        </header>

        {VM_CONFIG_ID !== "turtlebot4" && (
          <div className="nav2-banner">
            This panel is tuned for the <code>turtlebot4</code> VM preset. Current config:{" "}
            <code>{VM_CONFIG_ID || "unknown"}</code>.
          </div>
        )}

        <div className="nav2-banner">
          This Foxglove bridge does not advertise the hidden <code>NavigateToPose</code> action topics/services. Goal
          send works via the visible <code>/turtlebot4/goal_pose</code> topic; cancel/result/action feedback are not
          available in the current transport.
        </div>

        <div className="nav2-banner">
          Goal fields are absolute <code>map</code>-frame coordinates, not relative robot moves. Use the current pose
          helper below before sending a small test goal.
        </div>

        {requestState.type !== "idle" && (
          <div className={`nav2-banner nav2-banner--${requestState.type}`}>{requestState.message}</div>
        )}

        <section className="nav2-grid">
          <article className="nav2-card">
            <h2>Goal control</h2>
            <p className="nav2-card__hint">Publish a map-frame goal to the visible <code>/turtlebot4/goal_pose</code> topic.</p>
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
              <button onClick={handleSendGoal} disabled={isSendingGoal || connectionStatus !== "connected"}>
                {isSendingGoal ? "Sending..." : "Send goal"}
              </button>
              <button className="nav2-button nav2-button--secondary" disabled title="Hidden action cancel service is not advertised by this Foxglove bridge.">
                Cancel unavailable
              </button>
            </div>
          </article>

          <article className="nav2-card">
            <h2>Navigation state</h2>
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
                <span>Goal freshness</span>
                <strong>{formatTimeAgo(getRecentTimestamp(messageTimestamps, GOAL_POSE_TOPIC))}</strong>
              </div>
              <div className="nav2-kpi">
                <span>Nav cmd_vel</span>
                <strong>{formatTimeAgo(getRecentTimestamp(messageTimestamps, CMD_VEL_NAV_TOPIC))}</strong>
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
                <dt>Helper pose source</dt>
                <dd>{helperPoseSource}</dd>
              </div>
              <div>
                <dt>Goal pose</dt>
                <dd>{formatPose(goalPose)}</dd>
              </div>
              <div>
                <dt>cmd_vel_nav</dt>
                <dd>{cmdVelSummary}</dd>
              </div>
              <div>
                <dt>Stop status</dt>
                <dd>{stopStatusMessage ? "received" : "n/a"}</dd>
              </div>
            </dl>
          </article>
        </section>

        <section className="nav2-grid nav2-grid--bottom">
          <article className="nav2-card">
            <div className="nav2-card__header">
              <h2>Topic health</h2>
              <span className="nav2-card__hint">Advertised vs. receiving vs. stale for the visible Nav2 bridge surfaces.</span>
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

          <article className="nav2-card">
            <h2>Visible Nav2 traffic</h2>
            <div className="nav2-debug-section">
              <h3>Latest pose</h3>
              <pre>{poseMessage ? JSON.stringify(poseMessage, null, 2) : "No pose received yet."}</pre>
            </div>
            <div className="nav2-debug-section">
              <h3>Latest goal_pose</h3>
              <pre>{goalPoseMessage ? JSON.stringify(goalPoseMessage, null, 2) : "No goal pose published yet."}</pre>
            </div>
            <div className="nav2-debug-section">
              <h3>Latest plan</h3>
              <pre>{planMessage ? JSON.stringify(planMessage, null, 2) : "No plan received yet."}</pre>
            </div>
            <div className="nav2-debug-section">
              <h3>Latest transformed_global_plan</h3>
              <pre>{transformedPlanMessage ? JSON.stringify(transformedPlanMessage, null, 2) : "No transformed plan received yet."}</pre>
            </div>
            <div className="nav2-debug-section">
              <h3>Latest stop_status</h3>
              <pre>{stopStatusMessage ? JSON.stringify(stopStatusMessage, null, 2) : "No stop status received yet."}</pre>
            </div>
          </article>
        </section>
      </div>
    </ConnectionSettingsProvider>
  );
}
