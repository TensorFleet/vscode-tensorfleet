import React, { useEffect, useMemo, useState } from "react";
import clipboard from "@lichtblick/suite-base/util/clipboard";
import type { TfGraphSnapshot } from "tensorfleet-util/ros/ros-bridge-api";
import {
  ACTION_FEEDBACK_TOPIC,
  ACTION_STATUS_TOPIC,
  CMD_VEL_NAV_TOPIC,
  PLAN_TOPIC,
  SEND_GOAL_SERVICE,
  TRANSFORMED_GLOBAL_PLAN_TOPIC,
  VM_CONFIG_ID,
} from "./runtime/nav2RuntimeConstants";
import {
  createFeedbackSnippet,
  findTfEdge,
  formatNumber,
  formatPose,
  formatRosDuration,
  formatTimeAgo,
  getRecentTimestamp,
} from "./runtime/nav2RuntimeUtils";
import type {
  ActiveGoal,
  GoalState,
  HealthSeverity,
  LifecycleCheck,
  LifecycleHealth,
  PreflightStatus,
  TfHealth,
  TopicHealth,
  ValidationSummary,
  ValidationSummaryState,
} from "./runtime/nav2RuntimeTypes";
import { useNav2Runtime } from "./runtime/useNav2Runtime";
import { ConnectionSettingsProvider, ConnectionSettingsTrigger } from "../ConnectionSettingsProvider";
import "./Nav2Panel.css";

type DebugSection = {
  id: string;
  title: string;
  payload: unknown;
};

type CopyFeedback = {
  type: "idle" | "info" | "error";
  message: string;
};

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
  const [goalX, setGoalX] = useState("0.0");
  const [goalY, setGoalY] = useState("0.0");
  const [goalYaw, setGoalYaw] = useState("0.0");
  const [copyFeedback, setCopyFeedback] = useState<CopyFeedback>({ type: "idle", message: "" });
  const {
    connectionStatus,
    messageTimestamps,
    planMessage,
    transformedPlanMessage,
    stopStatusMessage,
    actionStatusMessage,
    actionFeedbackMessage,
    tfGraphSnapshot,
    goalState,
    activeGoal,
    sendGoalResponse,
    resultResponse,
    lifecycleChecks,
    requestState,
    isSendingGoal,
    isCancelingGoal,
    preflightStatus,
    activeGoalStatusEntry,
    activeGoalFeedback,
    robotPose,
    currentMapPose,
    helperPose,
    helperPoseSource,
    currentMapCoordinates,
    lifecycleHealth,
    tfHealth,
    knownTfFrames,
    validationSummary,
    topicHealth,
    planPointCount,
    transformedPlanPointCount,
    cmdVelSummary,
    feedbackDistanceRemaining,
    feedbackRecoveries,
    feedbackNavigationTime,
    feedbackEta,
    sendGoal,
    cancelGoal,
    fillGoalFromCurrentPose,
    fillSmallForwardTestGoal,
  } = useNav2Runtime();

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
  const handoffGeneratedAt = useMemo(() => new Date().toISOString(), [
    actionFeedbackMessage,
    actionStatusMessage,
    activeGoal,
    connectionStatus,
    goalState,
    lifecycleChecks,
    messageTimestamps,
    planMessage,
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

  async function handleSendGoal() {
    const x = Number(goalX);
    const y = Number(goalY);
    const yaw = Number(goalYaw);
    await sendGoal({ x, y, yaw });
  }

  async function handleCancelGoal() {
    await cancelGoal();
  }

  function useCurrentPose() {
    const goal = fillGoalFromCurrentPose();
    if (!goal) {
      return;
    }
    setGoalX(goal.x.toFixed(2));
    setGoalY(goal.y.toFixed(2));
    setGoalYaw(goal.yaw.toFixed(1));
  }

  function useSmallForwardTestGoal() {
    const goal = fillSmallForwardTestGoal();
    if (!goal) {
      return;
    }
    setGoalX(goal.x.toFixed(2));
    setGoalY(goal.y.toFixed(2));
    setGoalYaw(goal.yaw.toFixed(1));
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
                <strong>
                  {typeof feedbackRecoveries === "number" || typeof feedbackRecoveries === "string"
                    ? String(feedbackRecoveries)
                    : "n/a"}
                </strong>
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
