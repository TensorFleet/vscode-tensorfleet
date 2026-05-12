import { useEffect, useMemo, useRef, useState } from "react";
import { ros2Bridge } from "tensorfleet-ros";
import {
  ACTION_FEEDBACK_TOPIC,
  ACTION_STATUS_TOPIC,
  CANCEL_GOAL_SERVICE,
  CMD_VEL_NAV_TOPIC,
  GET_RESULT_SERVICE,
  LIFECYCLE_SPECS,
  LIFECYCLE_POLL_MS,
  PLAN_TOPIC,
  SEND_GOAL_SERVICE,
  STOP_STATUS_TOPIC,
  TOPIC_HEALTH_CONFIGS,
  TRANSFORMED_GLOBAL_PLAN_TOPIC,
} from "./nav2RuntimeConstants";
import type {
  ActiveGoal,
  GoalCoordinates,
  GoalState,
  LifecycleCheck,
  Nav2RuntimeState,
  Nav2RequestState,
} from "./nav2RuntimeTypes";
import {
  computeLifecycleHealth,
  computePreflightStatus,
  computeTfHealth,
  computeTopicHealth,
  computeValidationSummary,
  createGoalUuid,
  degreesToQuaternion,
  extractStampedPose,
  getFeedbackForGoal,
  getFeedbackMetric,
  getGoalStatusEntry,
  getPathPointCount,
  getPoseCoordinates,
  getTwistSummary,
  getLifecycleServiceName,
  mapActionStatusCodeToGoalState,
  mapResultStatusToGoalState,
  normalizeServiceNames,
  normalizeRosMessage,
  nowMs,
  uuidToKey,
} from "./nav2RuntimeUtils";

export function useNav2Runtime(): Nav2RuntimeState {
  const [connectionStatus, setConnectionStatus] = useState<"connected" | "connecting" | "disconnected">("connecting");
  const [connectedAt, setConnectedAt] = useState<number | null>(null);
  const [availableTopics, setAvailableTopics] = useState(ros2Bridge.getAvailableTopics());
  const [availableServices, setAvailableServices] = useState<string[]>(() =>
    normalizeServiceNames(ros2Bridge.getAvailableServices()),
  );
  const [messageTimestamps, setMessageTimestamps] = useState<Record<string, number | null>>({});
  const [odomMessage, setOdomMessage] = useState<Record<string, unknown> | null>(null);
  const [poseMessage, setPoseMessage] = useState<Record<string, unknown> | null>(null);
  const [planMessage, setPlanMessage] = useState<Record<string, unknown> | null>(null);
  const [transformedPlanMessage, setTransformedPlanMessage] = useState<Record<string, unknown> | null>(null);
  const [cmdVelNavMessage, setCmdVelNavMessage] = useState<Record<string, unknown> | null>(null);
  const [stopStatusMessage, setStopStatusMessage] = useState<Record<string, unknown> | null>(null);
  const [actionStatusMessage, setActionStatusMessage] = useState<Record<string, unknown> | null>(null);
  const [actionFeedbackMessage, setActionFeedbackMessage] = useState<Record<string, unknown> | null>(null);
  const [tfGraphSnapshot, setTfGraphSnapshot] = useState(ros2Bridge.getTfGraphSnapshot());
  const [goalState, setGoalState] = useState<GoalState>("blocked");
  const [activeGoal, setActiveGoal] = useState<ActiveGoal | null>(null);
  const [sendGoalResponse, setSendGoalResponse] = useState<Record<string, unknown> | null>(null);
  const [resultResponse, setResultResponse] = useState<Record<string, unknown> | null>(null);
  const [lifecycleChecks, setLifecycleChecks] = useState<Record<string, LifecycleCheck>>({});
  const [requestState, setRequestState] = useState<Nav2RequestState>({ type: "idle", message: "" });
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
      setAvailableServices(normalizeServiceNames(services));
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
        LIFECYCLE_SPECS.map(async ({ node }) => {
          const serviceName = getLifecycleServiceName(node);
          const serviceAdvertised = availableServices.includes(serviceName);

          if (!serviceAdvertised) {
            setLifecycleChecks((current) => ({
              ...current,
              [node]: {
                pending: false,
                lastCheckedAt: current[node]?.lastCheckedAt ?? null,
                response: current[node]?.response ?? null,
                error: null,
              },
            }));
            return;
          }

          setLifecycleChecks((current) => ({
            ...current,
            [node]: {
              pending: true,
              lastCheckedAt: current[node]?.lastCheckedAt ?? null,
              response: current[node]?.response ?? null,
              error: null,
            },
          }));

          try {
            const response =
              (await ros2Bridge.callService<Record<string, unknown>>(serviceName, {}, { timeoutMs: 5_000 })) ?? null;
            if (cancelled) {
              return;
            }
            setLifecycleChecks((current) => ({
              ...current,
              [node]: {
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
              [node]: {
                pending: false,
                lastCheckedAt: nowMs(),
                response: current[node]?.response ?? null,
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
  const knownTfFrames = useMemo(() => ros2Bridge.getKnownTfFrames(), [tfGraphSnapshot]);
  const lifecycleHealth = useMemo(
    () => computeLifecycleHealth(availableServices, lifecycleChecks, messageTimestamps),
    [availableServices, lifecycleChecks, messageTimestamps],
  );
  const tfHealth = useMemo(
    () => computeTfHealth(tfGraphSnapshot, connectionStatus, knownTfFrames),
    [connectionStatus, knownTfFrames, tfGraphSnapshot],
  );
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
  const feedbackDistanceRemaining = getFeedbackMetric(activeGoalFeedback, "distance_remaining");
  const feedbackRecoveries = getFeedbackMetric(activeGoalFeedback, "number_of_recoveries");
  const feedbackNavigationTime = getFeedbackMetric(activeGoalFeedback, "navigation_time");
  const feedbackEta = getFeedbackMetric(activeGoalFeedback, "estimated_time_remaining");

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

  async function sendGoal(goalInput: GoalCoordinates) {
    const { x, y, yaw } = goalInput;
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

  async function cancelGoal() {
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

  function fillGoalFromCurrentPose(): GoalCoordinates | null {
    if (!currentMapCoordinates) {
      setRequestState({ type: "error", message: "Current map pose is not available yet." });
      return null;
    }
    const goal = {
      x: currentMapCoordinates.x,
      y: currentMapCoordinates.y,
      yaw: currentMapCoordinates.yaw ?? 0,
    };
    setRequestState({ type: "info", message: "Filled goal fields from the robot's current map pose." });
    return goal;
  }

  function fillSmallForwardTestGoal(): GoalCoordinates | null {
    if (!currentMapCoordinates) {
      setRequestState({ type: "error", message: "Current map pose is not available yet." });
      return null;
    }
    const goal = {
      x: currentMapCoordinates.x + 0.5,
      y: currentMapCoordinates.y,
      yaw: currentMapCoordinates.yaw ?? 0,
    };
    setRequestState({
      type: "info",
      message: "Filled a small absolute test goal 0.5 m ahead on map X. Adjust if your scene uses a different axis convention.",
    });
    return goal;
  }

  function clearRequestState() {
    setRequestState({ type: "idle", message: "" });
  }

  return {
    connectionStatus,
    connectedAt,
    availableTopics,
    availableServices,
    messageTimestamps,
    odomMessage,
    poseMessage,
    planMessage,
    transformedPlanMessage,
    cmdVelNavMessage,
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
    clearRequestState,
  };
}
