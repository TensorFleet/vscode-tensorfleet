import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
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
  dispatchTurtleBot4Nav2Command,
} from "../panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/commandDispatcher";
import {
  MAPPING_SERVICE_NAMES,
  MAPPING_STATUS_TOPIC,
  mapTurtleBot4Nav2Capabilities,
} from "../panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/capabilityMapper";
import {
  mapTurtleBot4Nav2State,
} from "../panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/stateMapper";
import {
  mapVacuumCommandToValetudoRequest,
} from "../panels-standalone/src/vacuum-adapter/backends/valetudo/commandMapper";
import {
  mapValetudoCapabilities,
} from "../panels-standalone/src/vacuum-adapter/backends/valetudo/capabilityMapper";

const repoRoot = resolve(import.meta.dir, "..");

function createRuntime(overrides: Partial<Nav2RuntimeState> = {}): Nav2RuntimeState {
  return {
    connectionStatus: "connected",
    connectedAt: 1,
    availableTopics: [],
    availableServices: [SEND_GOAL_SERVICE, CANCEL_GOAL_SERVICE, ...Object.values(MAPPING_SERVICE_NAMES)],
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

  const cancelResult = await dispatchTurtleBot4Nav2Command(
    { command: "cancel_navigation" },
    {
      runtime,
      snapshot,
      setCurrentTarget: () => undefined,
      setInitialDistance: () => undefined,
    },
  );

  assert.equal(cancelResult.ok, true);
  assert.equal(cancelCount, 1);
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
  assert.equal(supportedNav2.cancel_navigation.supported, true);
  assert.equal(supportedNav2.go_to_location.backendCapability, "nav2_msgs/action/NavigateToPose");
  assert.equal(supportedNav2.mapping_session.supported, true);
  assert.equal(supportedNav2.auto_mapping.supported, false);

  const mappingNav2 = mapTurtleBot4Nav2Capabilities(
    createRuntime({ availableTopics: [{ topic: MAPPING_STATUS_TOPIC, type: "std_msgs/msg/String" }] }),
  );
  assert.equal(mappingNav2.auto_mapping.supported, true);

  const blockedNav2 = mapTurtleBot4Nav2Capabilities(createRuntime({ availableServices: [] }));
  assert.equal(blockedNav2.go_to_location.supported, false);
  assert.equal(blockedNav2.cancel_navigation.supported, false);
  assert.equal(blockedNav2.mapping_session.supported, false);

  const valetudo = mapValetudoCapabilities([
    "BasicControlCapability",
    "GoToLocationCapability",
    "FanSpeedControlCapability",
  ]);
  assert.equal(valetudo.start_cleaning.supported, true);
  assert.equal(valetudo.return_to_dock.supported, true);
  assert.equal(valetudo.go_to_location.supported, true);
  assert.equal(valetudo.fan_speed.supported, true);
  assert.equal(valetudo.zone_cleaning.supported, false);
  assert.equal(valetudo.resume.supported, false);
  assert.equal(valetudo.auto_mapping.supported, false);
}

function testStateMapping(): void {
  const idle = mapTurtleBot4Nav2State({ runtime: createRuntime({ goalState: "ready" }), currentTarget: null });
  assert.equal(idle.mission.state, "idle");
  assert.equal(idle.navigation.state, "idle");
  assert.deepEqual(idle.navigation.planPath, [
    { x: 1, y: 2 },
    { x: 3, y: 4 },
  ]);

  const navigating = mapTurtleBot4Nav2State({
    runtime: createRuntime({ goalState: "executing" }),
    currentTarget: { x: 3, y: 4, yaw: 0 },
    initialDistance: 5,
  });
  assert.equal(navigating.mission.state, "navigating");
  assert.equal(navigating.navigation.active, true);
  assert.deepEqual(navigating.navigation.currentTarget, { x: 3, y: 4, yaw: 0 });
  assert.equal(navigating.navigation.progress.initialDistance, 5);

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
      lastSavedAt: null,
      saveError: null,
    },
  });
  assert.equal(mapping.mission.state, "mapping");
  assert.equal(mapping.mapping.frontierCount, 3);
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
  assert.deepEqual(
    mapVacuumCommandToValetudoRequest({ command: "go_to_location", target: { x: 1, y: 2, yaw: 0 } }, capabilities),
    {
      ok: true,
      command: "go_to_location",
      request: { type: "go_to_location", target: { x: 1, y: 2, yaw: 0 } },
    },
  );
  assert.deepEqual(mapVacuumCommandToValetudoRequest({ command: "set_water_usage", value: "medium" }, capabilities), {
    ok: true,
    command: "set_water_usage",
    request: { type: "set_water_usage", value: "medium" },
  });
  const zoneResult = mapVacuumCommandToValetudoRequest({ command: "zone_cleaning" }, capabilities);
  assert.equal(zoneResult.ok, false);
  const mappingResult = mapVacuumCommandToValetudoRequest({ command: "start_mapping", mode: "auto" }, capabilities);
  assert.equal(mappingResult.ok, false);
}

function testPublicContractAndUiBoundary(): void {
  const publicFiles = ["adapter.ts", "capabilities.ts", "commands.ts", "errors.ts", "mapGrid.ts", "state.ts"];
  for (const file of publicFiles) {
    const contents = readFileSync(resolve(repoRoot, "panels-standalone/src/vacuum-adapter", file), "utf8");
    assert.equal(/components\/Nav2|nav2Runtime|nav_msgs\/msg|geometry_msgs\/msg/.test(contents), false, file);
  }

  const panelContents = readFileSync(
    resolve(repoRoot, "panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx"),
    "utf8",
  );
  for (const backendName of ["turtlebot4_nav2", "valetudo"]) {
    assert.equal(panelContents.includes(backendName), false, `Vacuum Control should not branch on ${backendName}`);
  }
  assert.equal(/identity\.source|snapshot\.identity\.source/.test(panelContents), false);
}

function assertCommandNamesHandled(): void {
  const commandNames: VacuumCommandName[] = [
    "go_to_location",
    "cancel_navigation",
    "manual_control",
    "start_mapping",
    "pause_mapping",
    "resume_mapping",
    "finish_mapping",
    "discard_mapping",
    "accept_map",
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
  testMapMetadata();
  testValetudoCommandStub();
  testPublicContractAndUiBoundary();
  await testTurtleBot4Commands();
  await testTurtleBot4MappingCommands();
  await testTurtleBot4UnsupportedCommands();
  console.log("vacuum_adapter regression harness passed");
}

await main();
