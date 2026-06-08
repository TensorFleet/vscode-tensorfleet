import assert from "node:assert/strict";
import { readFileSync, readdirSync, statSync } from "node:fs";
import { resolve } from "node:path";
import {
  CANCEL_GOAL_SERVICE,
  SEND_GOAL_SERVICE,
} from "../panels-standalone/src/components/Nav2/runtime/nav2RuntimeConstants";
import type { Nav2RuntimeState } from "../panels-standalone/src/components/Nav2/runtime/nav2RuntimeTypes";
import {
  VACUUM_CAPABILITY_NAMES,
  createUnsupportedCapabilities,
} from "../panels-standalone/src/vacuum-adapter/capabilities";
import type { VacuumCommand, VacuumCommandName } from "../panels-standalone/src/vacuum-adapter/commands";
import {
  buildVacuumMapMetadata,
  parseVacuumMapGrid,
} from "../panels-standalone/src/vacuum-adapter/mapGrid";
import {
  normalizeServiceNames,
} from "../panels-standalone/src/components/Nav2/runtime/nav2RuntimeUtils";
import {
  dispatchTurtleBot4Nav2Command,
} from "../panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/commandDispatcher";
import {
  MAPPING_SERVICE_NAMES,
  MAP_ANNOTATION_SERVICE_NAMES,
  MAPPING_STATUS_TOPIC,
  MISSION_SERVICE_NAMES,
  MISSION_STATUS_TOPIC,
  mapTurtleBot4Nav2Capabilities,
} from "../panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/capabilityMapper";
import {
  mapTurtleBot4Nav2State,
} from "../panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/stateMapper";
import {
  hasMigratedLocalPrototypeMapAnnotations,
  readLocalPrototypeMapAnnotations,
} from "../panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/localAnnotationMigration";
import {
  mapVacuumCommandToValetudoRequest,
  mapValetudoRuntimeCommandResult,
} from "../panels-standalone/src/vacuum-adapter/backends/valetudo/commandMapper";
import {
  mapValetudoCapabilities,
} from "../panels-standalone/src/vacuum-adapter/backends/valetudo/capabilityMapper";
import {
  isValetudoRuntimeSnapshot,
  mapValetudoRuntimeSnapshotToBoundary,
  mapValetudoRuntimeUnavailable,
  mapValetudoState,
} from "../panels-standalone/src/vacuum-adapter/backends/valetudo/stateMapper";
import {
  buildCleanAreaCoverageSnapshot,
  buildCleanAreaCoverageTarget,
  markCleanAreaCoveredCells,
} from "../panels-standalone/src/components/VacuumControl/cleanAreaCoverage";
import {
  buildLawnmowerWaypoints,
} from "../panels-standalone/src/components/VacuumControl/cleanAreaPlanner";
import {
  buildCleanAreaCoverageRuntimeConfig,
} from "../panels-standalone/src/components/VacuumControl/cleanAreaProfile";

const repoRoot = resolve(import.meta.dir, "..");
const valetudoRawCapabilityNames = [
  "BasicControlCapability",
  "BatteryStateCapability",
  "ConsumableMonitoringCapability",
  "FanSpeedControlCapability",
  "GoToLocationCapability",
  "MapSegmentationCapability",
  "WaterUsageControlCapability",
  "ZoneCleaningCapability",
] as const;

function collectFiles(dir: string, predicate: (path: string) => boolean): string[] {
  const files: string[] = [];
  for (const entry of readdirSync(dir)) {
    const path = resolve(dir, entry);
    const stat = statSync(path);
    if (stat.isDirectory()) {
      files.push(...collectFiles(path, predicate));
      continue;
    }
    if (predicate(path)) {
      files.push(path);
    }
  }
  return files;
}

function installMockLocalStorage(): Map<string, string> {
  const storage = new Map<string, string>();
  Object.defineProperty(globalThis, "window", {
    configurable: true,
    value: {
      localStorage: {
        getItem: (key: string) => storage.get(key) ?? null,
        setItem: (key: string, value: string) => {
          storage.set(key, value);
        },
        removeItem: (key: string) => {
          storage.delete(key);
        },
      },
    },
  });
  return storage;
}

function createRuntime(overrides: Partial<Nav2RuntimeState> = {}): Nav2RuntimeState {
  return {
    connectionStatus: "connected",
    connectedAt: 1,
    availableTopics: [{ topic: MISSION_STATUS_TOPIC, type: "std_msgs/msg/String" }],
    availableServices: [
      SEND_GOAL_SERVICE,
      CANCEL_GOAL_SERVICE,
      ...Object.values(MAPPING_SERVICE_NAMES),
      ...Object.values(MISSION_SERVICE_NAMES),
      ...Object.values(MAP_ANNOTATION_SERVICE_NAMES),
    ],
    messageTimestamps: {},
    odomMessage: null,
    poseMessage: null,
    planMessage: {
      poses: [
        { pose: { position: { x: 1, y: 2 }, orientation: { w: 1 } } },
        { pose: { pose: { position: { x: 3, y: 4 }, orientation: { w: 1 } } } },
      ],
    },
    transformedPlanMessage: null,
    cmdVelNavMessage: null,
    stopStatusMessage: null,
    actionStatusMessage: null,
    actionFeedbackMessage: null,
    tfGraphSnapshot: { edges: [], nodes: [] } as Nav2RuntimeState["tfGraphSnapshot"],
    goalState: "ready",
    activeGoal: null,
    sendGoalResponse: null,
    resultResponse: null,
    lifecycleChecks: {},
    requestState: { type: "idle", message: "" },
    isSendingGoal: false,
    isCancelingGoal: false,
    preflightStatus: { state: "ready", missingTopics: [], missingServices: [] },
    activeGoalStatusEntry: null,
    activeGoalFeedback: null,
    robotPose: null,
    currentMapPose: null,
    helperPose: null,
    helperPoseSource: "test",
    currentMapCoordinates: { x: 0, y: 0, yaw: 0 },
    lifecycleHealth: [],
    tfHealth: { status: "healthy", detail: "ok", baseFrame: "base_link", missingFrames: [], missingEdges: [], staleEdges: [] },
    knownTfFrames: ["map", "odom", "base_link"],
    validationSummary: { state: "pass", title: "Ready", detail: "Runtime ready." },
    topicHealth: [
      {
        topic: "/map",
        label: "SLAM map",
        type: "nav_msgs/msg/OccupancyGrid",
        staleAfterMs: 15_000,
        advertised: true,
        lastMessageAt: Date.now(),
        status: "receiving",
      },
    ],
    planPointCount: 2,
    transformedPlanPointCount: 0,
    cmdVelSummary: "n/a",
    feedbackDistanceRemaining: 1.2,
    feedbackRecoveries: 0,
    feedbackNavigationTime: { sec: 1, nanosec: 0 },
    feedbackEta: { sec: 2, nanosec: 0 },
    sendGoal: async () => undefined,
    cancelGoal: async () => undefined,
    fillGoalFromCurrentPose: () => null,
    fillSmallForwardTestGoal: () => null,
    clearRequestState: () => undefined,
    ...overrides,
  };
}

async function testTurtleBot4Commands(): Promise<void> {
  const sentTargets: unknown[] = [];
  let cancelCount = 0;
  let currentTarget: unknown = null;
  let initialDistance: number | null = null;
  const runtime = createRuntime({
    sendGoal: async (target) => {
      sentTargets.push(target);
    },
    cancelGoal: async () => {
      cancelCount += 1;
    },
  });
  const snapshot = mapTurtleBot4Nav2State({ runtime, currentTarget: null, initialDistance: null });

  const target = { x: 2, y: 0, yaw: 90 };
  const goResult = await dispatchTurtleBot4Nav2Command(
    { command: "go_to_location", target },
    {
      runtime,
      snapshot,
      setCurrentTarget: (value) => {
        currentTarget = value;
      },
      setInitialDistance: (value) => {
        initialDistance = value;
      },
    },
  );

  assert.equal(goResult.ok, true);
  assert.deepEqual(sentTargets, [target]);
  assert.deepEqual(currentTarget, target);
  assert.equal(initialDistance, 2);

  const serviceCalls: string[] = [];
  const missionResult = await dispatchTurtleBot4Nav2Command(
    { command: "start_navigation", target },
    {
      runtime: {
        ...runtime,
        callService: async (name) => {
          serviceCalls.push(name);
          return name === MISSION_SERVICE_NAMES.setParameters
            ? { results: [{ successful: true }] }
            : { success: true, message: name };
        },
      },
      snapshot,
      setCurrentTarget: (value) => {
        currentTarget = value;
      },
      setInitialDistance: (value) => {
        initialDistance = value;
      },
    },
  );

  assert.equal(missionResult.ok, true);
  assert.equal(sentTargets.length, 1);
  assert.deepEqual(serviceCalls, [MISSION_SERVICE_NAMES.setParameters, MISSION_SERVICE_NAMES.startNavigation]);

  const cancelResult = await dispatchTurtleBot4Nav2Command(
    { command: "cancel_mission" },
    {
      runtime: {
        ...runtime,
        callService: async (name) => {
          serviceCalls.push(name);
          return { success: true, message: name };
        },
      },
      snapshot,
      setCurrentTarget: () => undefined,
      setInitialDistance: () => undefined,
    },
  );

  assert.equal(cancelResult.ok, true);
  assert.equal(cancelCount, 0);
  assert.equal(serviceCalls.at(-1), MISSION_SERVICE_NAMES.cancel);

  const coverageResult = await dispatchTurtleBot4Nav2Command(
    {
      command: "start_coverage",
      area: { shape: "rectangle", minX: 0, minY: 0, maxX: 1, maxY: 1 },
    },
    {
      runtime: {
        ...runtime,
        callService: async (name) => {
          serviceCalls.push(name);
          return name === MISSION_SERVICE_NAMES.setParameters
            ? { results: [{ successful: true }] }
            : { success: true, message: name };
        },
      },
      snapshot,
      setCurrentTarget: () => undefined,
      setInitialDistance: () => undefined,
    },
  );

  assert.equal(coverageResult.ok, true);
  assert.deepEqual(serviceCalls.slice(-2), [MISSION_SERVICE_NAMES.setParameters, MISSION_SERVICE_NAMES.startCoverage]);

  const roomCleaningResult = await dispatchTurtleBot4Nav2Command(
    {
      command: "start_room_cleaning",
      annotation: {
        id: "room-1",
        kind: "room",
        name: "Kitchen",
        area: { shape: "rectangle", minX: 0, minY: 0, maxX: 1, maxY: 1 },
        mapId: "lab-map",
        createdAt: 1,
        updatedAt: 1,
      },
    },
    {
      runtime: {
        ...runtime,
        callService: async (name) => {
          serviceCalls.push(name);
          return name === MISSION_SERVICE_NAMES.setParameters
            ? { results: [{ successful: true }] }
            : { success: true, message: name };
        },
      },
      snapshot,
      setCurrentTarget: () => undefined,
      setInitialDistance: () => undefined,
    },
  );

  assert.equal(roomCleaningResult.ok, true);
  assert.deepEqual(serviceCalls.slice(-2), [MISSION_SERVICE_NAMES.setParameters, MISSION_SERVICE_NAMES.startCoverage]);

  for (const command of [
    { command: "pause_mission" },
    { command: "resume_mission" },
    { command: "retry_mission_step" },
    { command: "skip_mission_step" },
  ] satisfies VacuumCommand[]) {
    const result = await dispatchTurtleBot4Nav2Command(command, {
      runtime: {
        ...runtime,
        callService: async (name) => {
          serviceCalls.push(name);
          return { success: true, message: name };
        },
      },
      snapshot,
      setCurrentTarget: () => undefined,
      setInitialDistance: () => undefined,
    });
    assert.equal(result.ok, true, `${command.command} should dispatch`);
  }
}

async function testTurtleBot4MappingCommands(): Promise<void> {
  const serviceCalls: string[] = [];
  const runtime = createRuntime({
    availableTopics: [{ topic: MAPPING_STATUS_TOPIC, type: "std_msgs/msg/String" }],
  });
  const snapshot = mapTurtleBot4Nav2State({ runtime, currentTarget: null, initialDistance: null });

  for (const command of [
    { command: "start_mapping", mode: "auto" },
    { command: "start_mapping", mode: "manual" },
    { command: "pause_mapping" },
    { command: "resume_mapping" },
    { command: "finish_mapping" },
    { command: "discard_mapping" },
    { command: "accept_map" },
  ] satisfies VacuumCommand[]) {
    const result = await dispatchTurtleBot4Nav2Command(command, {
      runtime: {
        ...runtime,
        callService: async (name) => {
          serviceCalls.push(name);
          return { success: true, message: name };
        },
      },
      snapshot,
      setCurrentTarget: () => undefined,
      setInitialDistance: () => undefined,
    });
    assert.equal(result.ok, true, `${command.command} should dispatch`);
  }

  assert.deepEqual(serviceCalls, [
    MAPPING_SERVICE_NAMES.startAuto,
    MAPPING_SERVICE_NAMES.startManual,
    MAPPING_SERVICE_NAMES.pause,
    MAPPING_SERVICE_NAMES.resume,
    MAPPING_SERVICE_NAMES.finish,
    MAPPING_SERVICE_NAMES.discard,
    MAPPING_SERVICE_NAMES.accept,
  ]);
}

async function testTurtleBot4UnsupportedCommands(): Promise<void> {
  const runtime = createRuntime();
  const snapshot = mapTurtleBot4Nav2State({ runtime, currentTarget: null, initialDistance: null });
  const unsupportedCommands: VacuumCommand[] = [
    { command: "start_cleaning" },
    { command: "pause" },
    { command: "resume" },
    { command: "return_to_dock" },
    { command: "segment_cleaning" },
    { command: "zone_cleaning" },
    { command: "set_fan_speed", value: "balanced" },
    { command: "set_water_usage", value: "medium" },
  ];

  for (const command of unsupportedCommands) {
    const result = await dispatchTurtleBot4Nav2Command(command, {
      runtime,
      snapshot,
      setCurrentTarget: () => undefined,
      setInitialDistance: () => undefined,
    });
    assert.equal(result.ok, false, `${command.command} should fail explicitly`);
    if (!result.ok) {
      assert.equal(result.error.code, "unsupported", `${command.command} should be unsupported`);
      assert.equal(result.error.command, command.command);
    }
  }
}

function testCapabilityCoverage(): void {
  const unsupported = createUnsupportedCapabilities();
  assert.deepEqual(Object.keys(unsupported).sort(), [...VACUUM_CAPABILITY_NAMES].sort());

  const supportedNav2 = mapTurtleBot4Nav2Capabilities(createRuntime());
  assert.equal(supportedNav2.go_to_location.supported, true);
  assert.equal(supportedNav2.start_navigation.supported, true);
  assert.equal(supportedNav2.cancel_mission.supported, true);
  assert.equal(supportedNav2.cancel_navigation.supported, true);
  assert.equal(supportedNav2.go_to_location.backendCapability, "nav2_msgs/action/NavigateToPose");
  assert.equal(supportedNav2.mapping_session.supported, true);
  assert.equal(supportedNav2.auto_mapping.supported, false);
  assert.equal(supportedNav2.coverage_mission.supported, true);
  assert.equal(supportedNav2.map_annotations.supported, true);
  assert.equal(supportedNav2.room_semantics.supported, true);
  assert.equal(supportedNav2.zone_semantics.supported, true);
  assert.equal(supportedNav2.room_cleaning.supported, true);
  assert.equal(supportedNav2.zone_cleaning.supported, true);
  assert.equal(supportedNav2.start_coverage.supported, true);
  assert.equal(supportedNav2.pause_mission.supported, true);
  assert.equal(supportedNav2.resume_mission.supported, true);
  assert.equal(supportedNav2.retry_mission_step.supported, true);
  assert.equal(supportedNav2.skip_mission_step.supported, true);

  const mappingNav2 = mapTurtleBot4Nav2Capabilities(
    createRuntime({ availableTopics: [{ topic: MAPPING_STATUS_TOPIC, type: "std_msgs/msg/String" }] }),
  );
  assert.equal(mappingNav2.auto_mapping.supported, true);

  const blockedNav2 = mapTurtleBot4Nav2Capabilities(createRuntime({ availableServices: [] }));
  assert.equal(blockedNav2.go_to_location.supported, false);
  assert.equal(blockedNav2.start_navigation.supported, false);
  assert.equal(blockedNav2.cancel_mission.supported, false);
  assert.equal(blockedNav2.cancel_navigation.supported, false);
  assert.equal(blockedNav2.mapping_session.supported, false);
  assert.equal(blockedNav2.start_coverage.supported, false);
  assert.equal(blockedNav2.map_annotations.supported, false);
  assert.equal(blockedNav2.room_semantics.supported, false);
  assert.equal(blockedNav2.zone_semantics.supported, false);
  assert.equal(blockedNav2.room_cleaning.supported, false);
  assert.equal(blockedNav2.zone_cleaning.supported, false);

  const valetudo = mapValetudoCapabilities([
    "BasicControlCapability",
    "BatteryStateCapability",
    "ConsumableMonitoringCapability",
    "GoToLocationCapability",
    "FanSpeedControlCapability",
    "WaterUsageControlCapability",
    "MapSegmentationCapability",
    "ZoneCleaningCapability",
  ]);
  assert.equal(valetudo.start_cleaning.supported, true);
  assert.equal(valetudo.start_cleaning.status, "supported");
  assert.equal(valetudo.start_cleaning.available, true);
  assert.equal(valetudo.pause.supported, true);
  assert.equal(valetudo.stop.supported, true);
  assert.equal(valetudo.return_to_dock.supported, true);
  assert.equal(valetudo.battery.supported, true);
  assert.equal(valetudo.go_to_location.supported, false);
  assert.equal(valetudo.fan_speed.supported, false);
  assert.equal(valetudo.water_usage.supported, false);
  assert.equal(valetudo.consumables.supported, false);
  assert.equal(valetudo.segment_cleaning.supported, false);
  assert.equal(valetudo.zone_cleaning.supported, false);
  assert.equal(valetudo.map.supported, false);
  assert.equal(valetudo.pose.supported, false);
  assert.equal(valetudo.zone_cleaning.supported, false);
  assert.equal(valetudo.room_cleaning.supported, false);
  assert.equal(valetudo.map_annotations.supported, false);
  assert.equal(valetudo.room_semantics.supported, false);
  assert.equal(valetudo.zone_semantics.supported, false);
  assert.equal(valetudo.resume.supported, false);
  assert.equal(valetudo.auto_mapping.supported, false);

  for (const name of ["fan_speed", "water_usage", "consumables", "segment_cleaning", "zone_cleaning"] as const) {
    assert.equal(valetudo[name].source, "valetudo");
    assert.equal(valetudo[name].status, "detected_not_ready");
    assert.equal(valetudo[name].available, false);
    assert.equal(
      valetudo[name].notes,
      "Valetudo capability detected, but product workflow is not implemented in this slice.",
    );
  }

  const withoutBasicControl = mapValetudoCapabilities(["BatteryStateCapability"]);
  assert.equal(withoutBasicControl.start_cleaning.supported, false);
  assert.equal(withoutBasicControl.pause.supported, false);
  assert.equal(withoutBasicControl.stop.supported, false);
  assert.equal(withoutBasicControl.return_to_dock.supported, false);
  assert.equal(withoutBasicControl.battery.supported, true);
}

function testServiceDiscoveryNormalization(): void {
  assert.deepEqual(
    normalizeServiceNames([
      SEND_GOAL_SERVICE,
      { service: CANCEL_GOAL_SERVICE, type: "action_msgs/srv/CancelGoal" },
      { name: MAPPING_SERVICE_NAMES.startAuto, type: "std_srvs/srv/Trigger" },
      { service: 42, type: "invalid" },
    ]),
    [SEND_GOAL_SERVICE, CANCEL_GOAL_SERVICE, MAPPING_SERVICE_NAMES.startAuto],
  );
}

function testStateMapping(): void {
  const idle = mapTurtleBot4Nav2State({ runtime: createRuntime({ goalState: "ready" }), currentTarget: null });
  assert.equal(idle.mission.state, "idle");
  assert.equal(idle.activity?.status, "idle");
  assert.equal(idle.navigation.state, "idle");
  assert.equal(idle.health?.runtimeStatus, "online");
  assert.equal(idle.source?.kind, "turtlebot4_nav2");
  assert.equal(idle.source?.status, "reachable");
  assert.equal(idle.dock?.supported, false);
  assert.equal(idle.dock?.state, "unknown");
  assert.equal(idle.diagnostics?.backend, "turtlebot4_nav2");
  assert.equal((idle.diagnostics?.map as { topic?: string } | undefined)?.topic, "/map");
  assert.equal((idle.diagnostics?.pose as { source?: string } | undefined)?.source, "test");
  assert.equal(
    (idle.diagnostics?.navigation as { backendGoalState?: string } | undefined)?.backendGoalState,
    "ready",
  );
  assert.equal(
    ((idle.diagnostics?.capabilities as { backendCapabilities?: Record<string, string> } | undefined)?.backendCapabilities ?? {})
      .go_to_location,
    "nav2_msgs/action/NavigateToPose",
  );
  assert.deepEqual(idle.navigation.planPath, [
    { x: 1, y: 2 },
    { x: 3, y: 4 },
  ]);
  assert.deepEqual(idle.map.annotations, []);

  const annotated = mapTurtleBot4Nav2State({
    runtime: createRuntime({ goalState: "ready" }),
    annotations: [
      {
        id: "room-1",
        kind: "room",
        name: "Kitchen",
        area: { shape: "rectangle", minX: 0, minY: 0, maxX: 1, maxY: 1 },
        mapId: "live-map",
        createdAt: 1,
        updatedAt: 1,
      },
    ],
  });
  assert.equal(annotated.map.annotations[0]?.name, "Kitchen");

  const navigating = mapTurtleBot4Nav2State({
    runtime: createRuntime({ goalState: "executing" }),
    currentTarget: { x: 3, y: 4, yaw: 0 },
    initialDistance: 5,
  });
  assert.equal(navigating.mission.state, "navigating");
  assert.equal(navigating.activity?.status, "navigating");
  assert.equal(navigating.activeMission?.type, "navigation");
  assert.equal(navigating.activeMission?.status, "running");
  assert.equal(navigating.activeMission?.requestedCommand, "start_navigation");
  assert.deepEqual(navigating.missions.active, navigating.activeMission);
  assert.equal(navigating.navigation.active, true);
  assert.deepEqual(navigating.navigation.currentTarget, { x: 3, y: 4, yaw: 0 });
  assert.equal(navigating.navigation.progress.initialDistance, 5);

  const hydratedNavigation = mapTurtleBot4Nav2State({
    runtime: createRuntime({ goalState: "ready" }),
    mission: {
      id: "navigation-1",
      type: "navigation",
      status: "running",
      backendSource: "turtlebot4_nav2",
      startedAt: 1,
      updatedAt: 2,
      requestedCommand: "start_navigation",
      phase: "navigating",
      progress: {
        percent: 0.5,
        currentStep: null,
        totalSteps: null,
        distanceRemaining: 0.8,
        areaCoveredSqM: null,
        areaRemainingSqM: null,
      },
      availableActions: ["cancel_mission", "pause_mission"],
      result: null,
      error: null,
      target: { x: 4, y: 5, yaw: 90 },
    },
  });
  assert.equal(hydratedNavigation.mission.state, "navigating");
  assert.equal(hydratedNavigation.navigation.active, true);
  assert.equal(hydratedNavigation.navigation.state, "active");
  assert.deepEqual(hydratedNavigation.navigation.currentTarget, { x: 4, y: 5, yaw: 90 });
  assert.deepEqual(hydratedNavigation.activeMission?.availableActions, ["cancel_mission"]);

  const hydratedCoverage = mapTurtleBot4Nav2State({
    runtime: createRuntime({ goalState: "ready" }),
    mission: {
      id: "coverage-1",
      type: "coverage",
      status: "running",
      backendSource: "turtlebot4_nav2",
      startedAt: 1,
      updatedAt: 2,
      requestedCommand: "start_coverage",
      phase: "navigating_step",
      progress: {
        percent: 0.25,
        currentStep: 2,
        totalSteps: 8,
        distanceRemaining: 1.1,
        areaCoveredSqM: 0.5,
        areaRemainingSqM: 1.5,
      },
      availableActions: ["pause_mission", "cancel_mission", "retry_mission_step", "skip_mission_step"],
      result: null,
      error: null,
      target: {
        area: { shape: "rectangle", minX: 0, minY: 0, maxX: 1, maxY: 1 },
        route: [{ x: 0.5, y: 0.5, yaw: 0 }],
      },
    },
  });
  assert.equal(hydratedCoverage.mission.state, "cleaning");
  assert.equal(hydratedCoverage.activity?.status, "covering");
  assert.equal(hydratedCoverage.activeMission?.type, "coverage");
  assert.equal(hydratedCoverage.activeMission?.status, "running");
  assert.deepEqual(hydratedCoverage.activeMission?.availableActions, [
    "pause_mission",
    "cancel_mission",
    "retry_mission_step",
    "skip_mission_step",
  ]);

  const hydratedCoverageWithAcceptedMap = mapTurtleBot4Nav2State({
    runtime: createRuntime({ goalState: "ready" }),
    mapping: {
      state: "accepted",
      mode: "auto",
      stateReason: "Map accepted.",
      knownRatio: 1,
      unknownRatio: 0,
      frontierCount: 0,
      visitedGoalCount: 4,
      failedGoalCount: 0,
      activeGoal: null,
      lastError: null,
      updatedAt: 30,
      persistence: "session",
      acceptedSessionLevel: true,
      savedMapPath: null,
      loadedMapPath: null,
      lastSavedAt: null,
      saveError: null,
      loadError: null,
      activeMapName: null,
      savedMaps: [],
    },
    mission: {
      ...hydratedCoverage.activeMission!,
      id: "coverage-accepted-map",
      status: "running",
      result: null,
    },
  });
  assert.equal(hydratedCoverageWithAcceptedMap.mission.state, "cleaning");
  assert.equal(hydratedCoverageWithAcceptedMap.activeMission?.type, "coverage");

  const hydratedPausedRoomCleaning = mapTurtleBot4Nav2State({
    runtime: createRuntime({ goalState: "ready" }),
    mission: {
      id: "room-cleaning-paused",
      type: "room_cleaning",
      status: "paused",
      backendSource: "turtlebot4_nav2",
      startedAt: 100,
      updatedAt: 150,
      requestedCommand: "start_room_cleaning",
      phase: "paused",
      progress: {
        percent: 0.5,
        currentStep: 2,
        totalSteps: 4,
        distanceRemaining: 0.5,
        areaCoveredSqM: 1.5,
        areaRemainingSqM: 1.5,
      },
      availableActions: ["resume_mission", "cancel_mission"],
      result: null,
      error: null,
      target: {
        area: { shape: "rectangle", minX: 0, minY: 0, maxX: 2, maxY: 2 },
        annotation: { id: "room-1", kind: "room", name: "Kitchen", mapId: "lab-map" },
      },
    },
  });
  assert.equal(hydratedPausedRoomCleaning.mission.state, "paused");
  assert.equal(hydratedPausedRoomCleaning.activity?.status, "paused");
  assert.equal(hydratedPausedRoomCleaning.activeMission?.type, "room_cleaning");
  assert.equal(hydratedPausedRoomCleaning.activeMission?.status, "paused");
  assert.deepEqual(hydratedPausedRoomCleaning.activeMission?.availableActions, ["resume_mission", "cancel_mission"]);
  assert.deepEqual((hydratedPausedRoomCleaning.activeMission?.target as { annotation?: unknown }).annotation, {
    id: "room-1",
    kind: "room",
    name: "Kitchen",
    mapId: "lab-map",
  });

  const hydratedAssistanceZoneCleaning = mapTurtleBot4Nav2State({
    runtime: createRuntime({ goalState: "ready" }),
    mission: {
      ...hydratedPausedRoomCleaning.activeMission!,
      id: "zone-cleaning-assistance",
      type: "zone_cleaning",
      status: "needs_assistance",
      requestedCommand: "start_zone_cleaning",
      phase: "needs_assistance",
      availableActions: ["retry_mission_step", "skip_mission_step", "cancel_mission"],
      error: {
        code: "coverage_step_failed",
        message: "Coverage navigation step failed.",
        recoverable: true,
      },
      target: {
        area: { shape: "rectangle", minX: 0, minY: 0, maxX: 2, maxY: 2 },
        annotation: { id: "zone-1", kind: "zone", name: "Kitchen spill", mapId: "lab-map" },
      },
    },
  });
  assert.equal(hydratedAssistanceZoneCleaning.mission.state, "paused");
  assert.equal(hydratedAssistanceZoneCleaning.activeMission?.type, "zone_cleaning");
  assert.deepEqual(hydratedAssistanceZoneCleaning.activeMission?.availableActions, [
    "retry_mission_step",
    "skip_mission_step",
    "cancel_mission",
  ]);

  const hydratedRecentRoomCleaning = mapTurtleBot4Nav2State({
    runtime: createRuntime({ goalState: "ready" }),
    recentMissions: [
      {
        id: "room-cleaning-1",
        type: "room_cleaning",
        status: "completed",
        backendSource: "turtlebot4_nav2",
        startedAt: 100,
        updatedAt: 200,
        requestedCommand: "start_room_cleaning",
        phase: "completed",
        progress: {
          percent: 1,
          currentStep: 4,
          totalSteps: 4,
          distanceRemaining: 0,
          areaCoveredSqM: 3,
          areaRemainingSqM: 0,
        },
        availableActions: [],
        result: {
          status: "completed",
          completedAt: 200,
          summary: "Kitchen partially cleaned. 2.5 m² cleaned, 0.5 m² remaining, 0.2 m² skipped.",
          details: {
            featureState: "partially_cleaned",
            cleanedAreaSqM: 2.5,
            remainingAreaSqM: 0.5,
            skippedAreaSqM: 0.2,
            skippedReasons: { occupied: 2, unknown: 1, outOfBounds: 0, tooSmall: 0 },
            routeCompleted: true,
            coverageThresholdReached: false,
          },
        },
        error: null,
        target: {
          area: { shape: "rectangle", minX: 0, minY: 0, maxX: 2, maxY: 2 },
          annotation: { id: "room-1", kind: "room", name: "Kitchen", mapId: "lab-map" },
          coverage: {
            completionThreshold: 0.95,
            targetCells: 10,
            coveredCells: 8,
            remainingCells: 2,
            cleanableAreaSqM: 3,
            coveredAreaSqM: 2.5,
            remainingAreaSqM: 0.5,
            skippedAreaSqM: 0.2,
            skippedReasons: { occupied: 2, unknown: 1, outOfBounds: 0, tooSmall: 0 },
          },
        },
      },
    ],
  });
  assert.equal(hydratedRecentRoomCleaning.activeMission, null);
  assert.equal(hydratedRecentRoomCleaning.missions.recent.length, 1);
  assert.equal(hydratedRecentRoomCleaning.missions.recent[0]?.type, "room_cleaning");
  assert.equal(
    hydratedRecentRoomCleaning.missions.recent[0]?.result?.summary,
    "Kitchen partially cleaned. 2.5 m² cleaned, 0.5 m² remaining, 0.2 m² skipped.",
  );
  assert.deepEqual(hydratedRecentRoomCleaning.missions.recent[0]?.result?.details?.skippedReasons, {
    occupied: 2,
    unknown: 1,
    outOfBounds: 0,
    tooSmall: 0,
  });

  const hydratedTerminalRoomCleaning = mapTurtleBot4Nav2State({
    runtime: createRuntime({ goalState: "ready" }),
    mission: {
      ...hydratedPausedRoomCleaning.activeMission!,
      status: "canceled",
      updatedAt: 250,
      phase: "canceled",
      availableActions: [],
      result: {
        status: "canceled",
        completedAt: 250,
        summary: "Kitchen canceled.",
      },
    },
  });
  assert.equal(hydratedTerminalRoomCleaning.activeMission?.status, "canceled");
  assert.equal(hydratedTerminalRoomCleaning.missions.recent[0]?.id, "room-cleaning-paused");
  assert.equal(hydratedTerminalRoomCleaning.missions.recent[0]?.result?.summary, "Kitchen canceled.");

  const mapping = mapTurtleBot4Nav2State({
    runtime: createRuntime({ goalState: "ready" }),
    mapping: {
      state: "auto_mapping",
      mode: "auto",
      stateReason: "Exploring.",
      knownRatio: 0.25,
      unknownRatio: 0.75,
      frontierCount: 3,
      visitedGoalCount: 1,
      failedGoalCount: 0,
      activeGoal: { x: 1, y: 2, yaw: 0 },
      lastError: null,
      updatedAt: 10,
      persistence: "session",
      acceptedSessionLevel: false,
      savedMapPath: null,
      loadedMapPath: null,
      lastSavedAt: null,
      saveError: null,
      loadError: null,
      activeMapName: null,
      savedMaps: [],
    },
  });
  assert.equal(mapping.mission.state, "mapping");
  assert.equal(mapping.activity?.status, "mapping");
  assert.equal(mapping.activeMission?.type, "mapping");
  assert.equal(mapping.activeMission?.status, "running");
  assert.equal(mapping.activeMission?.progress.percent, 0.25);
  assert.deepEqual(mapping.activeMission?.availableActions, ["pause_mapping", "finish_mapping", "discard_mapping"]);
  assert.equal(mapping.mapping.frontierCount, 3);

  const mappingWithSavedPaths = mapTurtleBot4Nav2State({
    runtime: createRuntime({ goalState: "ready" }),
    mapping: {
      state: "idle",
      mode: null,
      stateReason: "Saved map inventory loaded.",
      knownRatio: 1,
      unknownRatio: 0,
      frontierCount: 0,
      visitedGoalCount: 0,
      failedGoalCount: 0,
      activeGoal: null,
      lastError: null,
      updatedAt: 20,
      persistence: "persistent",
      acceptedSessionLevel: false,
      savedMapPath: "/maps/lab.yaml",
      loadedMapPath: "/maps/lab.yaml",
      lastSavedAt: 20,
      saveError: null,
      loadError: null,
      activeMapName: "lab",
      savedMaps: [
        {
          id: "lab",
          name: "Lab",
          yamlPath: "/maps/lab.yaml",
          imagePath: "/maps/lab.pgm",
          poseGraphPath: "/maps/lab.posegraph",
          loadable: true,
          loadUnavailableReason: null,
          modifiedAt: 20,
          sizeBytes: 123,
          active: true,
        },
      ],
    },
  });
  assert.equal(mappingWithSavedPaths.mapping.savedMaps[0]?.yamlPath, "/maps/lab.yaml");
  assert.deepEqual(
    (mappingWithSavedPaths.diagnostics?.mapping as {
      savedMapPaths?: {
        savedMapPath?: string | null;
        loadedMapPath?: string | null;
        savedMaps?: Array<{ id: string; yamlPath: string; imagePath: string | null; poseGraphPath: string | null }>;
      };
    } | undefined)?.savedMapPaths,
    {
      savedMapPath: "/maps/lab.yaml",
      loadedMapPath: "/maps/lab.yaml",
      savedMaps: [
        {
          id: "lab",
          yamlPath: "/maps/lab.yaml",
          imagePath: "/maps/lab.pgm",
          poseGraphPath: "/maps/lab.posegraph",
        },
      ],
    },
  );

  const offline = mapTurtleBot4Nav2State({
    runtime: createRuntime({ connectionStatus: "disconnected" }),
    currentTarget: null,
  });
  assert.equal(offline.activity?.status, "unavailable");
  assert.equal(offline.mission.state, "idle");
}

function testLocalPrototypeAnnotationMigrationParsing(): void {
  const storage = installMockLocalStorage();
  storage.set(
    "tensorfleet:vacuums:turtlebot4-nav2:map-annotations:lab-map",
    JSON.stringify({
      annotations: [
        {
          id: "room-1",
          kind: "room",
          name: "Kitchen",
          area: { shape: "rectangle", minX: 0, minY: 0, maxX: 1, maxY: 1 },
          mapId: "lab-map",
          createdAt: 1,
          updatedAt: 2,
        },
        {
          id: "wrong-map-room",
          kind: "room",
          name: "Wrong map",
          area: { shape: "rectangle", minX: 0, minY: 0, maxX: 1, maxY: 1 },
          mapId: "other-map",
          createdAt: 1,
          updatedAt: 2,
        },
      ],
    }),
  );
  assert.equal(readLocalPrototypeMapAnnotations("lab-map").length, 1);
  assert.equal(readLocalPrototypeMapAnnotations("lab-map")[0]?.name, "Kitchen");
  assert.equal(readLocalPrototypeMapAnnotations("other-map").length, 0);

  storage.set(
    "tensorfleet:vacuums:turtlebot4-nav2:map-annotations:legacy-array-map",
    JSON.stringify([
      {
        id: "zone-1",
        kind: "zone",
        name: "Entry",
        area: { shape: "rectangle", minX: 1, minY: 1, maxX: 2, maxY: 2 },
        mapId: null,
        createdAt: 3,
        updatedAt: 4,
      },
    ]),
  );
  const legacyArrayAnnotations = readLocalPrototypeMapAnnotations("legacy-array-map");
  assert.equal(legacyArrayAnnotations.length, 1);
  assert.equal(legacyArrayAnnotations[0]?.mapId, "legacy-array-map");

  assert.equal(hasMigratedLocalPrototypeMapAnnotations("lab-map"), false);
  storage.set("tensorfleet:vacuums:turtlebot4-nav2:map-annotations-migrated:lab-map", "true");
  assert.equal(hasMigratedLocalPrototypeMapAnnotations("lab-map"), true);
}

function testMapMetadata(): void {
  const grid = parseVacuumMapGrid({
    info: {
      width: 3,
      height: 2,
      resolution: 0.5,
      origin: { position: { x: -1, y: -2 }, orientation: { w: 1 } },
    },
    header: { frame_id: "map" },
    data: Int8Array.from([-1, 0, 10, 80, 100, -1]),
  });
  assert.ok(grid);
  assert.equal(grid.width, 3);
  assert.equal(grid.originX, -1);
  const metadata = buildVacuumMapMetadata(grid, 123);
  assert.equal(metadata.totalCells, 6);
  assert.equal(metadata.freeCells, 2);
  assert.equal(metadata.occupiedCells, 2);
  assert.equal(metadata.unknownCells, 2);
  assert.equal(metadata.knownRatio, 4 / 6);
  assert.equal(metadata.lastUpdateAt, 123);
}

function testCleanAreaCoverageAndPlanning(): void {
  const grid = parseVacuumMapGrid({
    info: {
      width: 20,
      height: 20,
      resolution: 0.1,
      origin: { position: { x: 0, y: 0 }, orientation: { w: 1 } },
    },
    header: { frame_id: "map" },
    data: Array.from({ length: 400 }, () => 0),
  });
  assert.ok(grid);

  const rect = { minX: 0.2, minY: 0.2, maxX: 1.2, maxY: 1.5 };
  const target = buildCleanAreaCoverageTarget(rect, grid);
  assert.ok(target);
  assert.equal(target.occupiedCells.length, 0);
  assert.equal(target.unknownCells.length, 0);
  assert.ok(target.cleanableCells.length > 100);

  const waypoints = buildLawnmowerWaypoints({
    rect,
    spacing: 0.24,
    swathWidth: 0.3,
    target,
  });
  assert.ok(waypoints.length >= 8, "planner should create dense overlapping passes");
  assert.ok(Math.min(...waypoints.map((point) => point.x)) <= rect.minX + 0.16, "first lane should cover left edge");
  assert.ok(Math.max(...waypoints.map((point) => point.x)) >= rect.maxX - 0.16, "last lane should cover right edge");
  assert.ok(Math.min(...waypoints.map((point) => point.y)) <= rect.minY + 0.16, "pass endpoints should cover lower edge");
  assert.ok(Math.max(...waypoints.map((point) => point.y)) >= rect.maxY - 0.16, "pass endpoints should cover upper edge");

  const compactRect = { minX: 0.3, minY: 0.3, maxX: 1.0, maxY: 1.0 };
  const compactTarget = buildCleanAreaCoverageTarget(compactRect, grid);
  assert.ok(compactTarget);
  const compactWaypoints = buildLawnmowerWaypoints({
    rect: compactRect,
    spacing: 0.12,
    swathWidth: 0.3,
    goalCompletionTolerance: 0.28,
    target: compactTarget,
  });
  assert.ok(compactWaypoints.length >= 12, "small square clean areas should get dense passes");
  assert.ok(Math.min(...compactWaypoints.map((point) => point.y)) <= compactRect.minY + 0.01);
  assert.ok(Math.max(...compactWaypoints.map((point) => point.y)) >= compactRect.maxY - 0.01);
  assert.ok(Math.min(...compactWaypoints.map((point) => point.x)) <= compactRect.minX - 0.25);
  assert.ok(Math.max(...compactWaypoints.map((point) => point.x)) >= compactRect.maxX + 0.25);

  const covered = markCleanAreaCoveredCells({
    target,
    coveredCellKeys: new Set(),
    previousPose: { x: 0.35, y: 0.35, yaw: 0 },
    currentPose: { x: 1.05, y: 0.35, yaw: 0 },
    swathWidth: 0.3,
  });
  const snapshot = buildCleanAreaCoverageSnapshot({ target, coveredCellKeys: covered, swathWidth: 0.3 });
  assert.ok(snapshot);
  assert.ok(snapshot.coveredCells > 0);
  assert.equal(snapshot.remainingCells + snapshot.coveredCells, snapshot.targetCells);
  assert.ok(snapshot.overlayCells.every((cell) => Number.isFinite(cell.minX) && Number.isFinite(cell.maxY)));
}

function testCoverageProfileAndDecomposition(): void {
  const runtimeConfig = buildCleanAreaCoverageRuntimeConfig({
    mapMetadata: {
      hasMap: true,
      width: 20,
      height: 20,
      resolution: 0.05,
      freeCells: 300,
      occupiedCells: 20,
      unknownCells: 80,
      knownCells: 320,
      totalCells: 400,
      freeRatio: 0.75,
      occupiedRatio: 0.05,
      unknownRatio: 0.2,
      knownRatio: 0.8,
      knownAreaSqM: 0.8,
      lastUpdateAt: 1,
      poseAvailable: true,
      readiness: "Map active",
    },
  });
  assert.equal(runtimeConfig.cleaningSwathWidthM, 0.3);
  assert.equal(runtimeConfig.completionThreshold, 0.95);
  assert.equal(runtimeConfig.laneSpacingM, 0.12);
  assert.equal(runtimeConfig.boundaryExtensionM, 0.28);

  const data = Array.from({ length: 100 }, () => 100);
  for (const [x, y] of [
    [1, 1],
    [1, 2],
    [2, 1],
    [7, 7],
  ]) {
    data[x + y * 10] = 0;
  }
  data[4 + 4 * 10] = -1;
  const grid = parseVacuumMapGrid({
    info: {
      width: 10,
      height: 10,
      resolution: 0.1,
      origin: { position: { x: 0, y: 0 }, orientation: { w: 1 } },
    },
    header: { frame_id: "map" },
    data,
  });
  assert.ok(grid);

  const target = buildCleanAreaCoverageTarget(
    { minX: 0, minY: 0, maxX: 1, maxY: 1 },
    grid,
    { minimumUsefulCleanableRegionSqM: 0.02 },
  );
  assert.ok(target);
  assert.equal(target.cleanableRegions.length, 1);
  assert.equal(target.skippedSmallRegionCount, 1);
  assert.equal(target.skippedSmallRegionCells.length, 1);
  assert.equal(target.unknownCells.length, 1);
  assert.equal(target.cleanableCells.length, 3);

  const snapshot = buildCleanAreaCoverageSnapshot({
    target,
    coveredCellKeys: new Set(),
    swathWidth: runtimeConfig.cleaningSwathWidthM,
  });
  assert.ok(snapshot);
  assert.equal(snapshot.cleanableRegionCount, 1);
  assert.equal(snapshot.skippedSmallRegionCount, 1);
  assert.equal(snapshot.skippedSmallRegionCells, 1);
}

function testValetudoCommandStub(): void {
  const capabilities = mapValetudoCapabilities([
    "BasicControlCapability",
    "GoToLocationCapability",
    "WaterUsageControlCapability",
  ]);

  assert.deepEqual(mapVacuumCommandToValetudoRequest({ command: "start_cleaning" }, capabilities), {
    ok: true,
    command: "start_cleaning",
    request: { type: "basic_control", action: "start" },
  });
  assert.deepEqual(mapVacuumCommandToValetudoRequest({ command: "pause" }, capabilities), {
    ok: true,
    command: "pause",
    request: { type: "basic_control", action: "pause" },
  });
  assert.deepEqual(mapVacuumCommandToValetudoRequest({ command: "stop" }, capabilities), {
    ok: true,
    command: "stop",
    request: { type: "basic_control", action: "stop" },
  });
  assert.deepEqual(mapVacuumCommandToValetudoRequest({ command: "return_to_dock" }, capabilities), {
    ok: true,
    command: "return_to_dock",
    request: { type: "basic_control", action: "home" },
  });
  assert.equal(
    mapVacuumCommandToValetudoRequest({ command: "go_to_location", target: { x: 1, y: 2, yaw: 0 } }, capabilities).ok,
    false,
  );
  assert.equal(mapVacuumCommandToValetudoRequest({ command: "set_water_usage", value: "medium" }, capabilities).ok, false);
  assert.equal(mapVacuumCommandToValetudoRequest({ command: "set_fan_speed", value: "turbo" }, capabilities).ok, false);
  const zoneResult = mapVacuumCommandToValetudoRequest({
    command: "start_zone_cleaning",
    annotation: {
      id: "zone-1",
      kind: "zone",
      name: "Entryway",
      area: { shape: "rectangle", minX: 0, minY: 0, maxX: 1, maxY: 1 },
      mapId: null,
      createdAt: 1,
      updatedAt: 1,
    },
  }, capabilities);
  assert.equal(zoneResult.ok, false);
  const annotationResult = mapVacuumCommandToValetudoRequest({
    command: "save_map_annotation",
    annotation: {
      id: "room-1",
      kind: "room",
      name: "Kitchen",
      area: { shape: "rectangle", minX: 0, minY: 0, maxX: 1, maxY: 1 },
      mapId: null,
    },
  }, capabilities);
  assert.equal(annotationResult.ok, false);
  const mappingResult = mapVacuumCommandToValetudoRequest({ command: "start_mapping", mode: "auto" }, capabilities);
  assert.equal(mappingResult.ok, false);

  const missingBasicControl = mapValetudoCapabilities([]);
  for (const command of ["start_cleaning", "pause", "stop", "return_to_dock"] as const) {
    const result = mapVacuumCommandToValetudoRequest({ command }, missingBasicControl);
    assert.equal(result.ok, false, `${command} should be unsupported without BasicControlCapability`);
  }

  assert.deepEqual(
    mapValetudoRuntimeCommandResult("start_cleaning", {
      ok: false,
      status: "unsupported",
      command: "start_cleaning",
      message: "not implemented",
      updatedAt: 1,
    }),
    {
      ok: false,
      command: "start_cleaning",
      error: { command: "start_cleaning", code: "unsupported", message: "not implemented" },
    },
  );
  assert.deepEqual(
    mapValetudoRuntimeCommandResult("start_cleaning", {
      ok: false,
      status: "unavailable",
      command: "start_cleaning",
      message: "source down",
      reason: "source_unreachable",
      code: "source_unreachable",
      updatedAt: 1,
    }),
    {
      ok: false,
      command: "start_cleaning",
      error: { command: "start_cleaning", code: "source_unreachable", message: "source down" },
    },
  );
  assert.deepEqual(
    mapValetudoRuntimeCommandResult("pause", {
      ok: false,
      status: "unavailable",
      command: "pause",
      message: "stale",
      reason: "stale_source",
      updatedAt: 1,
    }),
    {
      ok: false,
      command: "pause",
      error: { command: "pause", code: "stale_source", message: "stale" },
    },
  );
  assert.deepEqual(
    mapValetudoRuntimeCommandResult("pause", {
      ok: false,
      status: "failed",
      command: "pause",
      message: "cannot pause from docked",
      reason: "invalid_state",
      code: "invalid_state",
      updatedAt: 1,
    }),
    {
      ok: false,
      command: "pause",
      error: { command: "pause", code: "invalid_state", message: "cannot pause from docked" },
    },
  );
  assert.deepEqual(
    mapValetudoRuntimeCommandResult("pause", {
      ok: false,
      status: "unsupported",
      command: "pause",
      message: "legacy invalid state",
      reason: "command_invalid_state",
      code: "command_invalid_state",
      updatedAt: 1,
    }),
    {
      ok: false,
      command: "pause",
      error: { command: "pause", code: "invalid_state", message: "legacy invalid state" },
    },
  );
  assert.deepEqual(
    mapValetudoRuntimeCommandResult("start_cleaning", {
      ok: false,
      status: "unsupported",
      command: "start_cleaning",
      message: "missing basic control",
      reason: "capability_unavailable",
      code: "capability_unavailable",
      updatedAt: 1,
    }),
    {
      ok: false,
      command: "start_cleaning",
      error: { command: "start_cleaning", code: "unsupported", message: "missing basic control" },
    },
  );
  assert.deepEqual(
    mapValetudoRuntimeCommandResult("start_cleaning", {
      ok: false,
      status: "failed",
      command: "start_cleaning",
      message: "source rejected command",
      reason: "source_command_failed",
      code: "source_command_failed",
      updatedAt: 1,
    }),
    {
      ok: false,
      command: "start_cleaning",
      error: { command: "start_cleaning", code: "backend_error", message: "source rejected command" },
    },
  );
  assert.deepEqual(
    mapValetudoRuntimeCommandResult("start_cleaning", {
      ok: false,
      status: "failed",
      command: "",
      message: "Missing command",
      reason: "invalid_request",
      code: "missing_command",
      updatedAt: 1,
    }),
    {
      ok: false,
      command: "start_cleaning",
      error: { command: "start_cleaning", code: "invalid_request", message: "Missing command" },
    },
  );
}

function testValetudoStateAwareCommandAvailability(): void {
  const snapshotFor = (overrides: {
    state?: { value: string; label: string; started: boolean; paused: boolean };
    dock?: { state: string; docked: boolean };
    battery?: { level: number; charging: boolean };
    runtimeStatus?: "online" | "degraded" | "offline";
    sourceStatus?: "reachable" | "unreachable" | "unknown";
    sourceStale?: boolean;
    reachable?: boolean;
    online?: boolean;
    commands?: Record<string, { available: boolean; reason?: string }>;
  }) =>
    mapValetudoState(
      mapValetudoRuntimeSnapshotToBoundary({
        runtime: {
          id: "tensorfleet-valetudo-runtime",
          version: "0.6.0",
          status: overrides.runtimeStatus ?? "online",
        },
        backend: "valetudo",
        robot: { id: "valetudo-command-rules", name: "Valetudo Command Rules" },
        source: {
          kind: "valetudo_mock",
          status: overrides.sourceStatus ?? "reachable",
          stale: overrides.sourceStale ?? false,
          lastSeenAt: 1,
        },
        connectivity: {
          reachable: overrides.reachable ?? true,
          online: overrides.online ?? true,
        },
        state: overrides.state ?? { value: "idle", label: "Idle", started: false, paused: false },
        battery: overrides.battery ?? { level: 82, charging: false },
        dock: overrides.dock ?? { state: "available", docked: false },
        capabilities: {
          commands: overrides.commands ?? {
            start_cleaning: { available: true },
            pause: { available: true },
            stop: { available: true },
            return_to_dock: { available: true },
          },
          diagnostics: [],
        },
        diagnostics: { mode: "valetudo_mock", rawCapabilityNames: ["BasicControlCapability"] },
        updatedAt: 1,
      }),
    );

  const assertAvailability = (
    label: string,
    snapshot: ReturnType<typeof snapshotFor>,
    expected: Record<"start_cleaning" | "pause" | "stop" | "return_to_dock", { available: boolean; reason?: string }>,
  ) => {
    for (const command of ["start_cleaning", "pause", "stop", "return_to_dock"] as const) {
      assert.equal(
        snapshot.capabilities[command].available,
        expected[command].available,
        `${label}: ${command} availability`,
      );
      assert.equal(
        snapshot.capabilities[command].availabilityReason,
        expected[command].reason,
        `${label}: ${command} reason`,
      );
      const result = mapVacuumCommandToValetudoRequest({ command }, snapshot.capabilities);
      assert.equal(result.ok, expected[command].available, `${label}: ${command} dispatch gate`);
      if (!result.ok) {
        assert.equal(result.reason, expected[command].reason ?? "unavailable", `${label}: ${command} result reason`);
      }
    }
  };

  assertAvailability("idle away from dock", snapshotFor({}), {
    start_cleaning: { available: true },
    pause: { available: false, reason: "invalid_state" },
    stop: { available: false, reason: "invalid_state" },
    return_to_dock: { available: true },
  });

  assertAvailability("idle docked", snapshotFor({ dock: { state: "available", docked: true } }), {
    start_cleaning: { available: true },
    pause: { available: false, reason: "invalid_state" },
    stop: { available: false, reason: "invalid_state" },
    return_to_dock: { available: false, reason: "invalid_state" },
  });

  assertAvailability(
    "cleaning",
    snapshotFor({ state: { value: "cleaning", label: "Cleaning", started: true, paused: false } }),
    {
      start_cleaning: { available: false, reason: "invalid_state" },
      pause: { available: true },
      stop: { available: true },
      return_to_dock: { available: true },
    },
  );

  assertAvailability(
    "paused",
    snapshotFor({ state: { value: "paused", label: "Paused", started: true, paused: true } }),
    {
      start_cleaning: { available: false, reason: "invalid_state" },
      pause: { available: false, reason: "invalid_state" },
      stop: { available: true },
      return_to_dock: { available: true },
    },
  );

  assertAvailability(
    "returning",
    snapshotFor({
      state: { value: "returning_to_dock", label: "Returning", started: false, paused: false },
      dock: { state: "returning", docked: false },
    }),
    {
      start_cleaning: { available: false, reason: "invalid_state" },
      pause: { available: false, reason: "invalid_state" },
      stop: { available: true },
      return_to_dock: { available: false, reason: "invalid_state" },
    },
  );

  assertAvailability(
    "stopped away from dock",
    snapshotFor({ state: { value: "stopped", label: "Stopped", started: false, paused: false } }),
    {
      start_cleaning: { available: true },
      pause: { available: false, reason: "invalid_state" },
      stop: { available: false, reason: "invalid_state" },
      return_to_dock: { available: true },
    },
  );

  for (const [label, overrides, reason] of [
    ["stale source", { sourceStale: true }, "stale_source"],
    ["unreachable source", { sourceStatus: "unreachable", reachable: false }, "source_unreachable"],
    ["offline runtime", { runtimeStatus: "offline", online: false, reachable: false }, "runtime_offline"],
    ["degraded runtime", { runtimeStatus: "degraded" }, "degraded_runtime"],
  ] as const) {
    assertAvailability(label, snapshotFor(overrides), {
      start_cleaning: { available: false, reason },
      pause: { available: false, reason },
      stop: { available: false, reason },
      return_to_dock: { available: false, reason },
    });
  }

  const runtimeBlocked = snapshotFor({
    commands: {
      start_cleaning: { available: false, reason: "unavailable" },
      pause: { available: false, reason: "unavailable" },
      stop: { available: false, reason: "unavailable" },
      return_to_dock: { available: false, reason: "unavailable" },
    },
  });
  assert.equal(runtimeBlocked.capabilities.start_cleaning.status, "unavailable");
  assert.equal(runtimeBlocked.capabilities.start_cleaning.availabilityReason, "unavailable");
  assert.equal(runtimeBlocked.capabilities.start_cleaning.reasons?.[0]?.code, "unavailable");
  assert.equal(runtimeBlocked.capabilities.start_cleaning.reasons?.[0]?.message, "Currently unavailable.");

  const withoutBasicControl = mapValetudoCapabilities([]);
  for (const command of ["start_cleaning", "pause", "stop", "return_to_dock"] as const) {
    const result = mapVacuumCommandToValetudoRequest({ command }, withoutBasicControl);
    assert.equal(withoutBasicControl[command].supported, false, `${command} should be unsupported`);
    assert.equal(result.ok, false, `${command} should be blocked without BasicControlCapability`);
    if (!result.ok) {
      assert.equal(result.reason, "unsupported", `${command} should return unsupported`);
    }
  }

  const withoutReturnHome = snapshotFor({
    commands: {
      start_cleaning: { available: true },
      pause: { available: true },
      stop: { available: true },
    },
  });
  assert.equal(withoutReturnHome.capabilities.return_to_dock.supported, false);
  assert.equal(withoutReturnHome.capabilities.return_to_dock.status, "unsupported");

  const detectedButNotReady = mapValetudoCapabilities(["FanSpeedControlCapability", "WaterUsageControlCapability"]);
  assert.equal(detectedButNotReady.fan_speed.status, "detected_not_ready");
  assert.equal(detectedButNotReady.water_usage.status, "detected_not_ready");
  assert.equal(mapValetudoCapabilities(["BasicControlCapability"]).resume.supported, false);

  const staleSource = snapshotFor({ sourceStale: true });
  assert.equal(staleSource.capabilities.start_cleaning.reasons?.[0]?.code, "stale_source");
  assert.equal(staleSource.capabilities.start_cleaning.reasons?.[0]?.message, "Robot state is stale.");
}

function testValetudoRuntimeSnapshotMapping(): void {
  const boundary = mapValetudoRuntimeSnapshotToBoundary({
    runtime: { id: "tensorfleet-valetudo-runtime-fixed-mock", version: "0.1.0-layer6a-m1", status: "online" },
    backend: "valetudo",
    robot: { id: "valetudo-fixed-mock-001", name: "Valetudo Fixed Mock" },
    source: { kind: "fixed_mock", status: "reachable", stale: false, lastSeenAt: 1 },
    connectivity: { reachable: true, online: true },
    state: { value: "idle", label: "Idle", started: false, paused: false },
    battery: { level: 82, charging: false },
    dock: { state: "available", docked: true },
    capabilities: {
      commands: {
        start_cleaning: { available: true },
        pause: { available: true },
        stop: { available: true },
        return_to_dock: { available: true },
      },
      diagnostics: [
        { name: "FanSpeedControlCapability", detected: true, implemented: false, scope: "diagnostics" },
        { name: "GoToLocationCapability", detected: true, implemented: false, scope: "diagnostics" },
        { name: "ZoneCleaningCapability", detected: true, implemented: false, scope: "diagnostics" },
      ],
    },
    diagnostics: {
      mode: "fixed_mock",
      rawCapabilityNames: [
        "BasicControlCapability",
        "BatteryStateCapability",
        "FanSpeedControlCapability",
        "GoToLocationCapability",
        "ZoneCleaningCapability",
      ],
    },
    updatedAt: 1,
  });
  const snapshot = mapValetudoState(boundary);
  assert.equal(snapshot.identity.label, "Valetudo Fixed Mock");
  assert.equal(snapshot.availability.status, "online");
  assert.equal(snapshot.health?.runtimeStatus, "online");
  assert.equal(snapshot.source?.kind, "fixed_mock");
  assert.equal(snapshot.source?.status, "reachable");
  assert.equal(snapshot.source?.stale, false);
  assert.equal(snapshot.dock?.supported, true);
  assert.equal(snapshot.dock?.state, "docked");
  assert.equal(snapshot.diagnostics?.backend, "valetudo");
  assert.equal(snapshot.capabilities.start_cleaning.supported, true);
  assert.equal(snapshot.capabilities.start_cleaning.status, "supported");
  assert.equal(snapshot.capabilities.start_cleaning.available, true);
  assert.equal(snapshot.capabilities.pause.supported, true);
  assert.equal(snapshot.capabilities.stop.supported, true);
  assert.equal(snapshot.capabilities.return_to_dock.supported, true);
  assert.equal(snapshot.capabilities.battery.supported, true);
  assert.equal(snapshot.capabilities.go_to_location.supported, false);
  assert.equal(snapshot.capabilities.fan_speed.supported, false);
  assert.equal(snapshot.capabilities.zone_cleaning.supported, false);
  assert.equal(snapshot.capabilities.map.supported, false);
  assert.equal(snapshot.capabilities.pose.supported, false);
  assert.equal(snapshot.capabilities.navigation_status.supported, false);
  assert.equal(snapshot.capabilities.mapping_session.supported, false);
  assert.equal(snapshot.capabilities.auto_mapping.supported, false);
  assert.equal(snapshot.capabilities.coverage_mission.supported, false);
  assert.equal(snapshot.capabilities.start_coverage.supported, false);
  assert.equal(snapshot.capabilities.map_annotations.supported, false);
  assert.equal(snapshot.capabilities.room_semantics.supported, false);
  assert.equal(snapshot.capabilities.zone_semantics.supported, false);
  assert.equal(snapshot.capabilities.room_cleaning.supported, false);
  assert.equal(snapshot.capabilities.manual_control.supported, false);
  assert.equal(snapshot.map.grid, null);
  assert.equal(snapshot.map.metadata.hasMap, false);
  assert.equal(snapshot.map.metadata.totalCells, 0);
  assert.equal(snapshot.map.receiving, false);
  assert.equal(snapshot.map.readiness, "unavailable");
  assert.deepEqual(snapshot.map.annotations, []);
  assert.equal(snapshot.pose.available, false);
  assert.equal(snapshot.pose.coordinates, null);
  assert.equal(snapshot.pose.readiness, "unavailable");
  assert.equal(snapshot.pose.source, undefined);
  assert.equal(snapshot.navigation.active, false);
  assert.equal(snapshot.navigation.currentTarget, null);
  assert.equal(snapshot.navigation.backendGoalState, null);
  assert.equal(snapshot.navigation.planPath, null);
  assert.equal(snapshot.mapping.persistence, "unsupported");
  assert.equal(snapshot.mapping.state, "idle");
  assert.equal(snapshot.mapping.savedMaps.length, 0);
  assert.equal(snapshot.readiness.ready, true);
  assert.equal(snapshot.fault.readiness, "ready");
  assert.equal(snapshot.battery.percentage, 82);
  // A docked-but-idle robot that is not charging must read as idle with no active mission.
  assert.equal(snapshot.mission.state, "idle");
  assert.equal(snapshot.activity?.status, "docked");
  assert.deepEqual(snapshot.activity?.availableActions, ["start_cleaning"]);
  assert.equal(snapshot.battery.charging, false);
  assert.equal(snapshot.activeMission, null);
  assert.equal(snapshot.missions.active, null);
  assert.equal((snapshot.diagnostics?.raw as { valetudoState?: string } | undefined)?.valetudoState, "idle");
  assert.deepEqual(
    (snapshot.diagnostics?.capabilities as { rawCapabilityNames?: string[] } | undefined)?.rawCapabilityNames,
    [
      "BasicControlCapability",
      "BatteryStateCapability",
      "FanSpeedControlCapability",
      "GoToLocationCapability",
      "ZoneCleaningCapability",
    ],
  );
  assert.equal((snapshot.diagnostics?.map as { supported?: boolean } | undefined)?.supported, false);
  assert.equal((snapshot.diagnostics?.pose as { supported?: boolean } | undefined)?.supported, false);
  assert.equal((snapshot.diagnostics?.navigation as { supported?: boolean } | undefined)?.supported, false);
  assert.equal((snapshot.diagnostics?.mapping as { supported?: boolean } | undefined)?.supported, false);

  const mqttBoundary = mapValetudoRuntimeSnapshotToBoundary({
    runtime: { id: "tensorfleet-valetudo-runtime", version: "0.6.0-layer6a-m6", status: "online" },
    backend: "valetudo",
    robot: { id: "valetudo-mqtt-robot", name: "Valetudo MQTT Source" },
    source: { kind: "valetudo_mock", status: "reachable", stale: false, lastSeenAt: 2 },
    connectivity: { reachable: true, online: true },
    state: { value: "cleaning", label: "Cleaning", started: true, paused: false },
    battery: { level: 64, charging: true },
    dock: { state: "available", docked: false },
    capabilities: {
      commands: {
        start_cleaning: { available: true },
        pause: { available: true },
        stop: { available: true },
        return_to_dock: { available: true },
      },
      diagnostics: [],
    },
    diagnostics: {
      mode: "valetudo_mock_mqtt",
      rawCapabilityNames: ["BasicControlCapability", "BatteryStateCapability"],
      transports: [
        { name: "http", enabled: true, status: "available", stale: false },
        {
          name: "mqtt",
          enabled: true,
          status: "reachable",
          stale: false,
          messageCount: 4,
          subscriptions: ["valetudo/robot/#"],
        },
      ],
    },
    updatedAt: 2,
  });
  const mqttSnapshot = mapValetudoState(mqttBoundary);
  assert.equal(mqttSnapshot.identity.label, "Valetudo MQTT Source");
  assert.equal(mqttSnapshot.availability.status, "online");
  assert.equal(mqttSnapshot.source?.kind, "valetudo_mock");
  assert.equal(mqttSnapshot.dock?.state, "charging");
  assert.equal(mqttSnapshot.mission.state, "cleaning");
  assert.equal(mqttSnapshot.activity?.status, "cleaning");
  assert.equal(mqttSnapshot.capabilities.start_cleaning.supported, true);
  assert.equal(mqttSnapshot.capabilities.battery.supported, true);

  const missingBasic = mapValetudoState(
    mapValetudoRuntimeSnapshotToBoundary({
      runtime: { id: "tensorfleet-valetudo-runtime-fixed-mock", version: "0.1.0-layer6a-m1", status: "online" },
      backend: "valetudo",
      robot: { id: "valetudo-fixed-mock-001", name: "Valetudo Fixed Mock" },
      source: { kind: "fixed_mock", status: "reachable", stale: false, lastSeenAt: 1 },
      connectivity: { reachable: true, online: true },
      state: { value: "idle", label: "Idle", started: false, paused: false },
      battery: { level: 82, charging: false },
      dock: { state: "available", docked: true },
      capabilities: {
        commands: {
          start_cleaning: { available: false, reason: "capability_unavailable" },
          pause: { available: false, reason: "capability_unavailable" },
          stop: { available: false, reason: "capability_unavailable" },
          return_to_dock: { available: false, reason: "capability_unavailable" },
        },
        diagnostics: [
          { name: "BasicControlCapability", detected: true, implemented: true, scope: "diagnostics" },
          { name: "FanSpeedControlCapability", detected: true, implemented: false, scope: "diagnostics" },
        ],
      },
      diagnostics: { mode: "fixed_mock", rawCapabilityNames: ["BasicControlCapability", "FanSpeedControlCapability"] },
      updatedAt: 1,
    }),
  );
  assert.equal(missingBasic.capabilities.start_cleaning.supported, true);
  assert.equal(missingBasic.capabilities.start_cleaning.status, "unavailable");
  assert.equal(missingBasic.capabilities.start_cleaning.available, false);
  assert.equal(missingBasic.capabilities.pause.supported, true);
  assert.equal(missingBasic.capabilities.pause.status, "unavailable");
  assert.equal(missingBasic.capabilities.stop.supported, true);
  assert.equal(missingBasic.capabilities.stop.status, "unavailable");
  assert.equal(missingBasic.capabilities.return_to_dock.supported, true);
  assert.equal(missingBasic.capabilities.return_to_dock.status, "unavailable");
  assert.equal(missingBasic.capabilities.battery.supported, true);
  assert.equal(missingBasic.capabilities.fan_speed.supported, false);

  const unreachable = mapValetudoState(
    mapValetudoRuntimeSnapshotToBoundary({
      runtime: { id: "tensorfleet-valetudo-runtime", version: "0.6.0", status: "online" },
      backend: "valetudo",
      robot: { id: "valetudo-source-down", name: "Valetudo Source Down" },
      source: { kind: "real_robot", status: "unreachable", stale: false, lastSeenAt: 10 },
      connectivity: { reachable: false, online: true },
      state: { value: "idle", label: "Idle", started: false, paused: false },
      battery: { level: 72, charging: false },
      dock: { state: "available", docked: true },
      capabilities: {
        commands: {
          start_cleaning: { available: false, reason: "source_unreachable" },
          pause: { available: false, reason: "source_unreachable" },
          stop: { available: false, reason: "source_unreachable" },
          return_to_dock: { available: false, reason: "source_unreachable" },
        },
        diagnostics: [],
      },
      diagnostics: { mode: "real_robot", rawCapabilityNames: ["BasicControlCapability"] },
      updatedAt: 10,
    }),
  );
  assert.equal(unreachable.availability.status, "online");
  assert.equal(unreachable.source?.status, "unreachable");
  assert.equal(unreachable.source?.reason, "source_unreachable");
  assert.equal(unreachable.activity?.status, "unavailable");
  assert.equal(unreachable.activity?.reason, "source_unreachable");
  assert.equal(unreachable.capabilities.start_cleaning.supported, true);
  assert.equal(unreachable.capabilities.start_cleaning.status, "unavailable");
  assert.equal(unreachable.capabilities.start_cleaning.available, false);
  assert.equal(unreachable.capabilities.start_cleaning.availabilityReason, "source_unreachable");
  const unreachableCommand = mapVacuumCommandToValetudoRequest(
    { command: "start_cleaning" },
    unreachable.capabilities,
  );
  assert.equal(unreachableCommand.ok, false);
  assert.equal(unreachableCommand.reason, "source_unreachable");

  const runtimeOffline = mapValetudoState(
    mapValetudoRuntimeSnapshotToBoundary({
      runtime: { id: "tensorfleet-valetudo-runtime", version: "0.6.0", status: "offline" },
      backend: "valetudo",
      robot: { id: "valetudo-runtime-down", name: "Valetudo Runtime Down" },
      source: { kind: "real_robot", status: "unreachable", stale: false, lastSeenAt: 10 },
      connectivity: { reachable: false, online: false },
      state: { value: "idle", label: "Idle", started: false, paused: false },
      capabilities: {
        commands: {
          start_cleaning: { available: false, reason: "source_unreachable" },
          pause: { available: false, reason: "source_unreachable" },
          stop: { available: false, reason: "source_unreachable" },
          return_to_dock: { available: false, reason: "source_unreachable" },
        },
        diagnostics: [],
      },
      diagnostics: { mode: "real_robot", rawCapabilityNames: ["BasicControlCapability"] },
      updatedAt: 10,
    }),
  );
  assert.equal(runtimeOffline.availability.status, "offline");
  assert.equal(runtimeOffline.health?.runtimeStatus, "offline");
  assert.equal(runtimeOffline.source?.reason, "runtime_offline");
  assert.equal(runtimeOffline.activity?.status, "unavailable");
  assert.equal(runtimeOffline.capabilities.start_cleaning.availabilityReason, "runtime_offline");
  assert.deepEqual(runtimeOffline.fault.faults, ["Valetudo integration runtime is offline."]);

  const stale = mapValetudoState(
    mapValetudoRuntimeSnapshotToBoundary({
      runtime: { id: "tensorfleet-valetudo-runtime", version: "0.6.0", status: "online" },
      backend: "valetudo",
      robot: { id: "valetudo-stale", name: "Valetudo Stale" },
      source: { kind: "valetudo_mock", status: "reachable", stale: true, lastSeenAt: 11 },
      connectivity: { reachable: true, online: true },
      state: { value: "idle", label: "Idle", started: false, paused: false },
      capabilities: {
        commands: {
          start_cleaning: { available: false, reason: "stale_source" },
        },
        diagnostics: [{ name: "BasicControlCapability", detected: true, implemented: true, scope: "control" }],
      },
      diagnostics: { mode: "valetudo_mock", rawCapabilityNames: ["BasicControlCapability"] },
      updatedAt: 11,
    }),
  );
  assert.equal(stale.source?.status, "stale");
  assert.equal(stale.source?.stale, true);
  assert.equal(stale.activity?.status, "idle");
  assert.equal(stale.activity?.reason, "stale_source");
  assert.equal(stale.capabilities.start_cleaning.supported, true);
  assert.equal(stale.capabilities.start_cleaning.status, "unavailable");
  assert.equal(stale.capabilities.start_cleaning.availabilityReason, "stale_source");
}

function testValetudoRuntimeMissionStateMapping(): void {
  const createSnapshot = (state: {
    value: string;
    label: string;
    started: boolean;
    paused: boolean;
  }) =>
    mapValetudoState(
      mapValetudoRuntimeSnapshotToBoundary({
        runtime: { id: "tensorfleet-valetudo-runtime-fixed-mock", version: "0.1.0-layer6a-m1", status: "online" },
        backend: "valetudo",
        robot: { id: "valetudo-fixed-mock-001", name: "Valetudo Fixed Mock" },
        source: { kind: "fixed_mock", status: "reachable", stale: false, lastSeenAt: 1 },
        connectivity: { reachable: true, online: true },
        state,
        battery: { level: 82, charging: false },
        dock: { state: "available", docked: false },
        capabilities: {
          commands: {
            start_cleaning: { available: true },
            pause: { available: true },
            stop: { available: true },
            return_to_dock: { available: true },
          },
          diagnostics: [],
        },
        diagnostics: { mode: "fixed_mock", rawCapabilityNames: ["BasicControlCapability"] },
        updatedAt: 1,
      }),
    );

  const cleaning = createSnapshot({ value: "cleaning", label: "Cleaning", started: true, paused: false });
  assert.equal(cleaning.mission.state, "cleaning");
  assert.equal(cleaning.activity?.status, "cleaning");
  assert.equal(cleaning.activeMission?.type, "hardware_cleaning");
  assert.deepEqual(cleaning.activity?.availableActions, ["pause", "stop", "return_to_dock"]);

  const paused = createSnapshot({ value: "paused", label: "Paused", started: true, paused: true });
  assert.equal(paused.mission.state, "paused");
  assert.equal(paused.activity?.status, "paused");

  const stopped = createSnapshot({ value: "stopped", label: "Stopped", started: false, paused: false });
  assert.equal(stopped.mission.state, "idle");
  assert.equal(stopped.activity?.status, "idle");
  assert.equal(stopped.activeMission, null);

  const returning = createSnapshot({ value: "returning_to_dock", label: "Returning to dock", started: false, paused: false });
  assert.equal(returning.mission.state, "returning");
  assert.equal(returning.activity?.status, "returning");

  const faulted = createSnapshot({ value: "error", label: "Error", started: false, paused: false });
  assert.equal(faulted.activity?.status, "faulted");
}

function testValetudoChargingAndOfflineMapping(): void {
  const chargingBoundary = mapValetudoRuntimeSnapshotToBoundary({
    runtime: { id: "rt", version: "v", status: "online" },
    backend: "valetudo",
    robot: { id: "valetudo-fixed-mock-001", name: "Valetudo Fixed Mock" },
    source: { kind: "fixed_mock", status: "reachable", stale: false, lastSeenAt: 1 },
    connectivity: { reachable: true, online: true },
    state: { value: "docked", label: "Docked", started: false, paused: false },
    battery: { level: 50, charging: true },
    dock: { state: "charging", docked: true },
    capabilities: { commands: { start_cleaning: { available: true } }, diagnostics: [] },
    diagnostics: { mode: "fixed_mock", rawCapabilityNames: ["BasicControlCapability"] },
    updatedAt: 1,
  });
  const chargingSnapshot = mapValetudoState(chargingBoundary);
  assert.equal(chargingSnapshot.mission.state, "charging");
  assert.equal(chargingSnapshot.activity?.status, "charging");
  assert.equal(chargingSnapshot.battery.charging, true);

  // Malformed payloads (e.g. a proxy error body) must be rejected so the adapter
  // can fall back to an offline snapshot instead of crashing the UI.
  assert.equal(isValetudoRuntimeSnapshot({ error: "Tensorfleet VM service is unavailable" }), false);
  assert.equal(
    isValetudoRuntimeSnapshot({
      backend: "valetudo",
      robot: { id: "valetudo-fixed-mock-001" },
      connectivity: { online: true },
      state: { value: "idle" },
      capabilities: { commands: {} },
    }),
    false,
  );
  assert.equal(isValetudoRuntimeSnapshot(null), false);
  assert.equal(isValetudoRuntimeSnapshot("offline"), false);

  const offline = mapValetudoState(mapValetudoRuntimeUnavailable("runtime stopped"));
  assert.equal(offline.availability.connected, false);
  assert.equal(offline.availability.status, "offline");
  assert.equal(offline.map.grid, null);
  assert.equal(offline.map.metadata.hasMap, false);
  assert.equal(offline.pose.available, false);
  assert.equal(offline.navigation.active, false);
  assert.equal(offline.mapping.persistence, "unsupported");
  assert.equal(offline.activity?.status, "unavailable");
  assert.equal(offline.activeMission, null);
}

function testAdvancedSurfaceOptionality(): void {
  const grid = parseVacuumMapGrid({
    info: {
      width: 2,
      height: 2,
      resolution: 0.5,
      origin: { position: { x: -1, y: -1 }, orientation: { w: 1 } },
    },
    header: { frame_id: "map" },
    data: [0, 0, 100, -1],
  });
  assert.ok(grid);
  const metadata = buildVacuumMapMetadata(grid, 50);
  const nav2 = mapTurtleBot4Nav2State({
    runtime: createRuntime({
      currentMapCoordinates: { x: 0.25, y: 0.5, yaw: 90 },
      helperPoseSource: "amcl",
      goalState: "executing",
    }),
    currentTarget: { x: 1, y: 1, yaw: 0 },
    initialDistance: 2,
    mapGrid: grid,
    mapMetadata: metadata,
  });
  assert.equal(nav2.capabilities.map.supported, true);
  assert.equal(nav2.map.metadata.hasMap, true);
  assert.deepEqual(nav2.map.grid, grid);
  assert.equal(nav2.pose.available, true);
  assert.deepEqual(nav2.pose.coordinates, { x: 0.25, y: 0.5, yaw: 90 });
  assert.equal(nav2.navigation.active, true);
  assert.equal(nav2.mapping.knownRatio, metadata.knownRatio);
  assert.equal(nav2.capabilities.coverage_mission.supported, true);

  const noMapValetudo = mapValetudoState(
    mapValetudoRuntimeSnapshotToBoundary({
      runtime: { id: "rt", version: "v", status: "online" },
      backend: "valetudo",
      robot: { id: "real-valetudo", name: "Real Valetudo" },
      source: { kind: "real_robot", status: "reachable", stale: false, lastSeenAt: 100 },
      connectivity: { reachable: true, online: true },
      state: { value: "idle", label: "Idle", started: false, paused: false },
      battery: { level: 70, charging: false },
      dock: { state: "available", docked: false },
      capabilities: {
        commands: {
          start_cleaning: { available: true },
          pause: { available: true },
          stop: { available: true },
          return_to_dock: { available: true },
        },
        diagnostics: [
          { name: "GoToLocationCapability", detected: true, implemented: false, scope: "diagnostics" },
          { name: "MapSegmentationCapability", detected: true, implemented: false, scope: "diagnostics" },
          { name: "ZoneCleaningCapability", detected: true, implemented: false, scope: "diagnostics" },
        ],
      },
      diagnostics: {
        mode: "real_robot",
        rawCapabilityNames: [
          "BasicControlCapability",
          "GoToLocationCapability",
          "MapSegmentationCapability",
          "ZoneCleaningCapability",
        ],
      },
      updatedAt: 100,
    }),
  );
  assert.equal(noMapValetudo.capabilities.map.supported, false);
  assert.equal(noMapValetudo.map.grid, null);
  assert.equal(noMapValetudo.map.metadata.hasMap, false);
  assert.deepEqual(noMapValetudo.map.annotations, []);
  assert.equal(noMapValetudo.pose.available, false);
  assert.equal(noMapValetudo.pose.coordinates, null);
  assert.equal(noMapValetudo.navigation.active, false);
  assert.equal(noMapValetudo.navigation.state, "idle");
  assert.equal(noMapValetudo.navigation.backendGoalState, null);
  assert.equal(noMapValetudo.capabilities.go_to_location.status, "detected_not_ready");
  assert.equal(noMapValetudo.capabilities.navigation_status.supported, false);
  assert.equal(noMapValetudo.mapping.persistence, "unsupported");
  assert.equal(noMapValetudo.capabilities.coverage_mission.supported, false);
  assert.equal(noMapValetudo.capabilities.start_coverage.supported, false);
  assert.equal(noMapValetudo.capabilities.map_annotations.supported, false);
  assert.equal(noMapValetudo.capabilities.room_semantics.supported, false);
  assert.equal(noMapValetudo.capabilities.zone_semantics.supported, false);
  assert.equal(noMapValetudo.capabilities.room_cleaning.supported, false);
  assert.equal(noMapValetudo.capabilities.zone_cleaning.status, "detected_not_ready");
  assert.equal(noMapValetudo.capabilities.manual_control.supported, false);
  assert.equal(noMapValetudo.activity?.status, "idle");
  assert.deepEqual(noMapValetudo.activity?.availableActions, ["start_cleaning", "return_to_dock"]);
  assert.equal(noMapValetudo.fault.readiness, "ready");
  assert.deepEqual(noMapValetudo.fault.faults, []);
  assert.deepEqual(
    (noMapValetudo.diagnostics?.raw as { rawCapabilityNames?: string[] } | undefined)?.rawCapabilityNames,
    [
      "BasicControlCapability",
      "GoToLocationCapability",
      "MapSegmentationCapability",
      "ZoneCleaningCapability",
    ],
  );
  assert.equal((noMapValetudo.diagnostics?.map as { supported?: boolean } | undefined)?.supported, false);
  assert.equal((noMapValetudo.diagnostics?.pose as { supported?: boolean } | undefined)?.supported, false);
  assert.equal((noMapValetudo.diagnostics?.navigation as { supported?: boolean } | undefined)?.supported, false);
  assert.equal((noMapValetudo.diagnostics?.mapping as { supported?: boolean } | undefined)?.supported, false);
}

function testPublicContractAndUiBoundary(): void {
  const publicFiles = ["adapter.ts", "capabilities.ts", "commands.ts", "errors.ts", "mapGrid.ts", "state.ts"];
  for (const file of publicFiles) {
    const contents = readFileSync(resolve(repoRoot, "panels-standalone/src/vacuum-adapter", file), "utf8");
    assert.equal(/components\/Nav2|nav2Runtime|nav_msgs\/msg|geometry_msgs\/msg/.test(contents), false, file);
    for (const capabilityName of valetudoRawCapabilityNames) {
      assert.equal(contents.includes(capabilityName), false, `${file} should not expose ${capabilityName}`);
    }
  }

  const panelContents = readFileSync(
    resolve(repoRoot, "panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx"),
    "utf8",
  );
  for (const backendName of ["turtlebot4_nav2", "valetudo"]) {
    assert.equal(panelContents.includes(backendName), false, `Vacuum Control should not branch on ${backendName}`);
  }
  assert.equal(/identity\.source|snapshot\.identity\.source/.test(panelContents), false);
  assert.equal(panelContents.includes("returning_to_dock"), false, "Vacuum Control should not branch on raw Valetudo state");
  assert.equal(
    panelContents.includes("Map unavailable") && panelContents.includes("This backend does not expose a product map yet."),
    true,
    "Vacuum Control should render a no-map placeholder in the reserved map area",
  );
  assert.equal(
    /mapSurfaceAvailable \? \([\s\S]*?<MapCanvas[\s\S]*?\) : \([\s\S]*?<NoMapCanvasPlaceholder/.test(panelContents),
    true,
    "MapCanvas should be replaced by a no-map placeholder when map support is false",
  );
  assert.equal(
    /health=\{snapshot\.health\}/.test(panelContents) &&
      /source=\{snapshot\.source\}/.test(panelContents) &&
      /activity=\{snapshot\.activity\}/.test(panelContents) &&
      /dock=\{snapshot\.dock\}/.test(panelContents),
    true,
    "No-map sidebar should render normalized health, source, activity, and dock state",
  );
  assert.equal(
    /battery=\{snapshot\.battery\}/.test(panelContents) && /fault=\{snapshot\.fault\}/.test(panelContents),
    true,
    "No-map sidebar should render normalized battery and fault state",
  );
  assert.equal(
    /supportedControls = controls\.filter\(\(control\) => control\.capability\.supported\)/.test(panelContents),
    true,
    "Unsupported basic commands should not appear as active controls",
  );
  assert.equal(
    /control\.capability\.available === false/.test(panelContents) &&
      /formatCapabilityReason\([\s\S]*control\.capability[\s\S]*control\.key/.test(panelContents) &&
      /vacuum-action-hint--disabled/.test(panelContents),
    true,
    "Basic controls should render disabled state and visible reasons from normalized availability",
  );
  assert.equal(
      panelContents.includes("Robot is not cleaning.") &&
      panelContents.includes("Nothing is running.") &&
      panelContents.includes("Already docked.") &&
      panelContents.includes("Robot state is stale.") &&
      panelContents.includes("Runtime offline.") &&
      panelContents.includes("Source unreachable.") &&
      panelContents.includes("Not supported by this backend."),
    true,
    "UI should translate raw availability reason codes into readable operator copy",
  );
  assert.equal(
    /isBasicRobotProfile/.test(panelContents) && /UnavailableWorkflowsCard/.test(panelContents),
    true,
    "No-map basic profiles should show advanced workflows as unavailable instead of dominant mode tabs",
  );
  assert.equal(
    /!mapSurfaceAvailable && !isBasicRobotProfile[\s\S]*?<BasicControlsCard/.test(panelContents),
    false,
    "No-map advanced profiles should not duplicate the basic command controls beside the status surface",
  );
  assert.equal(
    /const mapSurfaceAvailable = mapSupported/.test(panelContents),
    true,
    "MapCanvas should be gated by the normalized map capability",
  );
  assert.equal(
    /disabled=\{!navigationSupported/.test(panelContents),
    true,
    "Navigation controls should be gated by normalized navigation capability",
  );
  assert.equal(
    /disabled=\{!cleanAreaSupported/.test(panelContents),
    true,
    "Clean Area controls should be gated by normalized coverage capability",
  );
  assert.equal(
    /disabled=\{!roomsZonesSupported/.test(panelContents),
    true,
    "Rooms/Zones controls should be gated by normalized room and zone capabilities",
  );
  assert.equal(
    /manualControlSupported \? \(/.test(panelContents),
    true,
    "Teleop should be gated by normalized manual_control capability",
  );
  assert.equal(panelContents.includes("rawCapabilityNames"), false, "Vacuum Control should not read raw backend capability diagnostics");
  assert.equal(panelContents.includes("map.topic"), false, "Vacuum Control should not branch on backend map topics");
  assert.equal(panelContents.includes("pose.source"), false, "Vacuum Control should not branch on backend pose sources");
  assert.equal(panelContents.includes("imagePath"), false, "Vacuum Control should not branch on saved-map image paths");
  assert.equal(panelContents.includes("backendGoalState"), false, "Vacuum Control should not branch on Nav2 backend goal state");
  assert.equal(panelContents.includes("yamlPath"), false, "Vacuum Control should not branch on saved-map YAML paths");
  assert.equal(panelContents.includes("poseGraphPath"), false, "Vacuum Control should not branch on saved-map pose graph paths");

  const componentFiles = collectFiles(
    resolve(repoRoot, "panels-standalone/src/components/VacuumControl"),
    (path) => path.endsWith(".ts") || path.endsWith(".tsx"),
  );
  for (const file of componentFiles) {
    const contents = readFileSync(file, "utf8");
    for (const capabilityName of valetudoRawCapabilityNames) {
      assert.equal(contents.includes(capabilityName), false, `${file} should not branch on ${capabilityName}`);
    }
    assert.equal(contents.includes("rawCapabilityNames"), false, `${file} should not branch on raw backend capability diagnostics`);
    assert.equal(contents.includes("map.topic"), false, `${file} should not branch on backend map topics`);
    assert.equal(contents.includes("pose.source"), false, `${file} should not branch on backend pose sources`);
    assert.equal(contents.includes("backendGoalState"), false, `${file} should not branch on backend navigation states`);
    assert.equal(contents.includes("yamlPath"), false, `${file} should not branch on saved-map YAML paths`);
    assert.equal(contents.includes("imagePath"), false, `${file} should not branch on saved-map image paths`);
    assert.equal(contents.includes("poseGraphPath"), false, `${file} should not branch on saved-map pose graph paths`);
    assert.equal(contents.includes("identity.source"), false, `${file} should not branch on backend identity source`);
    assert.equal(contents.includes("snapshot.identity.source"), false, `${file} should not branch on backend identity source`);
  }
}

function assertCommandNamesHandled(): void {
  const commandNames: VacuumCommandName[] = [
    "start_navigation",
    "go_to_location",
    "cancel_navigation",
    "manual_control",
    "start_mapping",
    "pause_mapping",
    "resume_mapping",
    "finish_mapping",
    "discard_mapping",
    "accept_map",
    "load_map",
    "save_map_annotation",
    "delete_map_annotation",
    "start_coverage",
    "start_room_cleaning",
    "start_zone_cleaning",
    "pause_mission",
    "resume_mission",
    "cancel_mission",
    "retry_mission_step",
    "skip_mission_step",
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
  assert.equal(new Set(commandNames).size, commandNames.length);
}

async function main(): Promise<void> {
  assertCommandNamesHandled();
  testCapabilityCoverage();
  testStateMapping();
  testLocalPrototypeAnnotationMigrationParsing();
  testMapMetadata();
  testCleanAreaCoverageAndPlanning();
  testCoverageProfileAndDecomposition();
  testValetudoStateAwareCommandAvailability();
  testValetudoRuntimeSnapshotMapping();
  testValetudoRuntimeMissionStateMapping();
  testValetudoChargingAndOfflineMapping();
  testAdvancedSurfaceOptionality();
  testValetudoCommandStub();
  testPublicContractAndUiBoundary();
  testServiceDiscoveryNormalization();
  await testTurtleBot4Commands();
  await testTurtleBot4MappingCommands();
  await testTurtleBot4UnsupportedCommands();
  console.log("vacuum_adapter regression harness passed");
}

await main();
