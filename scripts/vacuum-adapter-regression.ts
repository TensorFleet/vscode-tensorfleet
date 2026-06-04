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
  assert.equal(idle.navigation.state, "idle");
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
  assert.equal(mapping.activeMission?.type, "mapping");
  assert.equal(mapping.activeMission?.status, "running");
  assert.equal(mapping.activeMission?.progress.percent, 0.25);
  assert.deepEqual(mapping.activeMission?.availableActions, ["pause_mapping", "finish_mapping", "discard_mapping"]);
  assert.equal(mapping.mapping.frontierCount, 3);
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
  assert.equal(snapshot.capabilities.start_cleaning.supported, true);
  assert.equal(snapshot.capabilities.pause.supported, true);
  assert.equal(snapshot.capabilities.stop.supported, true);
  assert.equal(snapshot.capabilities.return_to_dock.supported, true);
  assert.equal(snapshot.capabilities.battery.supported, true);
  assert.equal(snapshot.capabilities.go_to_location.supported, false);
  assert.equal(snapshot.capabilities.fan_speed.supported, false);
  assert.equal(snapshot.capabilities.zone_cleaning.supported, false);
  assert.equal(snapshot.capabilities.map.supported, false);
  assert.equal(snapshot.map.grid, null);
  assert.equal(snapshot.pose.available, false);
  assert.equal(snapshot.battery.percentage, 82);
  // A docked-but-idle robot that is not charging must read as idle with no active mission.
  assert.equal(snapshot.mission.state, "idle");
  assert.equal(snapshot.battery.charging, false);
  assert.equal(snapshot.activeMission, null);
  assert.equal(snapshot.missions.active, null);

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
  assert.equal(mqttSnapshot.mission.state, "cleaning");
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
  assert.equal(missingBasic.capabilities.start_cleaning.supported, false);
  assert.equal(missingBasic.capabilities.pause.supported, false);
  assert.equal(missingBasic.capabilities.stop.supported, false);
  assert.equal(missingBasic.capabilities.return_to_dock.supported, false);
  assert.equal(missingBasic.capabilities.battery.supported, true);
  assert.equal(missingBasic.capabilities.fan_speed.supported, false);
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

  assert.equal(
    createSnapshot({ value: "cleaning", label: "Cleaning", started: true, paused: false }).mission.state,
    "cleaning",
  );
  assert.equal(
    createSnapshot({ value: "paused", label: "Paused", started: true, paused: true }).mission.state,
    "paused",
  );
  assert.equal(
    createSnapshot({ value: "stopped", label: "Stopped", started: false, paused: false }).mission.state,
    "idle",
  );
  assert.equal(
    createSnapshot({ value: "returning_to_dock", label: "Returning to dock", started: false, paused: false }).mission
      .state,
    "returning",
  );
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
  assert.equal(chargingSnapshot.battery.charging, true);

  // Malformed payloads (e.g. a proxy error body) must be rejected so the adapter
  // can fall back to an offline snapshot instead of crashing the UI.
  assert.equal(isValetudoRuntimeSnapshot({ error: "Tensorfleet VM service is unavailable" }), false);
  assert.equal(isValetudoRuntimeSnapshot(null), false);
  assert.equal(isValetudoRuntimeSnapshot("offline"), false);

  const offline = mapValetudoState(mapValetudoRuntimeUnavailable("runtime stopped"));
  assert.equal(offline.availability.connected, false);
  assert.equal(offline.availability.status, "offline");
  assert.equal(offline.map.grid, null);
  assert.equal(offline.activeMission, null);
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

  const componentFiles = collectFiles(
    resolve(repoRoot, "panels-standalone/src/components/VacuumControl"),
    (path) => path.endsWith(".ts") || path.endsWith(".tsx"),
  );
  for (const file of componentFiles) {
    const contents = readFileSync(file, "utf8");
    for (const capabilityName of valetudoRawCapabilityNames) {
      assert.equal(contents.includes(capabilityName), false, `${file} should not branch on ${capabilityName}`);
    }
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
  testValetudoRuntimeSnapshotMapping();
  testValetudoRuntimeMissionStateMapping();
  testValetudoChargingAndOfflineMapping();
  testValetudoCommandStub();
  testPublicContractAndUiBoundary();
  testServiceDiscoveryNormalization();
  await testTurtleBot4Commands();
  await testTurtleBot4MappingCommands();
  await testTurtleBot4UnsupportedCommands();
  console.log("vacuum_adapter regression harness passed");
}

await main();
