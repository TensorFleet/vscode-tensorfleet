import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { resolve } from "node:path";

import {
  VACUUM_CAPABILITY_NAMES as LOCAL_CAPABILITY_NAMES,
  createUnsupportedCapabilities as createLocalUnsupportedCapabilities,
} from "../panels-standalone/src/vacuum-adapter/capabilities";
import { VACUUM_COMMAND_NAMES as LOCAL_COMMAND_NAMES } from "../panels-standalone/src/vacuum-adapter/commands";
import {
  VACUUM_CAPABILITY_NAMES as SHARED_CAPABILITY_NAMES,
  createUnsupportedCapabilities as createSharedUnsupportedCapabilities,
} from "../panels-standalone/.generated/tensorfleet-util/vacuum/capabilities";
import { VACUUM_COMMAND_NAMES as SHARED_COMMAND_NAMES } from "../panels-standalone/.generated/tensorfleet-util/vacuum/commands";
import { unsupportedCommand as localUnsupportedCommand } from "../panels-standalone/src/vacuum-adapter/errors";
import { unsupportedCommand as sharedUnsupportedCommand } from "../panels-standalone/.generated/tensorfleet-util/vacuum/errors";
import {
  buildVacuumMapMetadata as buildLocalVacuumMapMetadata,
  parseVacuumMapGrid as parseLocalVacuumMapGrid,
} from "../panels-standalone/src/vacuum-adapter/mapGrid";
import {
  buildVacuumMapMetadata as buildSharedVacuumMapMetadata,
  parseVacuumMapGrid as parseSharedVacuumMapGrid,
} from "../panels-standalone/.generated/tensorfleet-util/vacuum/mapGrid";
import {
  mapValetudoCapabilities as mapLocalValetudoCapabilities,
} from "../panels-standalone/src/vacuum-adapter/backends/valetudo/capabilityMapper";
import {
  mapValetudoCapabilities as mapSharedValetudoCapabilities,
} from "../panels-standalone/.generated/tensorfleet-util/vacuum/backends/valetudo/capabilityMapper";
import {
  mapVacuumCommandToValetudoRequest as mapLocalVacuumCommandToValetudoRequest,
  mapValetudoRuntimeCommandResult as mapLocalValetudoRuntimeCommandResult,
} from "../panels-standalone/src/vacuum-adapter/backends/valetudo/commandMapper";
import {
  mapVacuumCommandToValetudoRequest as mapSharedVacuumCommandToValetudoRequest,
  mapValetudoRuntimeCommandResult as mapSharedValetudoRuntimeCommandResult,
} from "../panels-standalone/.generated/tensorfleet-util/vacuum/backends/valetudo/commandMapper";
import {
  mapVacuumCommandToValetudoRuntimeCommandName as mapLocalVacuumCommandToValetudoRuntimeCommandName,
} from "../panels-standalone/src/vacuum-adapter/backends/valetudo/runtimeCommandMapper";
import {
  mapVacuumCommandToValetudoRuntimeCommandName as mapSharedVacuumCommandToValetudoRuntimeCommandName,
} from "../panels-standalone/.generated/tensorfleet-util/vacuum/backends/valetudo/runtimeCommandMapper";
import {
  isValetudoRuntimeSnapshot as isLocalValetudoRuntimeSnapshot,
  mapValetudoRuntimeSnapshotToBoundary as mapLocalValetudoRuntimeSnapshotToBoundary,
  mapValetudoRuntimeUnavailable as mapLocalValetudoRuntimeUnavailable,
  mapValetudoState as mapLocalValetudoState,
} from "../panels-standalone/src/vacuum-adapter/backends/valetudo/stateMapper";
import {
  isValetudoRuntimeSnapshot as isSharedValetudoRuntimeSnapshot,
  mapValetudoRuntimeSnapshotToBoundary as mapSharedValetudoRuntimeSnapshotToBoundary,
  mapValetudoRuntimeUnavailable as mapSharedValetudoRuntimeUnavailable,
  mapValetudoState as mapSharedValetudoState,
} from "../panels-standalone/.generated/tensorfleet-util/vacuum/backends/valetudo/stateMapper";
import {
  MAP_ANNOTATION_SERVICE_NAMES as LOCAL_MAP_ANNOTATION_SERVICE_NAMES,
  MAPPING_SERVICE_NAMES as LOCAL_MAPPING_SERVICE_NAMES,
  MAPPING_STATUS_TOPIC as LOCAL_MAPPING_STATUS_TOPIC,
  MISSION_SERVICE_NAMES as LOCAL_MISSION_SERVICE_NAMES,
  MISSION_STATUS_TOPIC as LOCAL_MISSION_STATUS_TOPIC,
  mapTurtleBot4Nav2Capabilities as mapLocalTurtleBot4Nav2Capabilities,
  unsupportedTurtleBot4Nav2Command as localUnsupportedTurtleBot4Nav2Command,
} from "../panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/capabilityMapper";
import {
  MAP_ANNOTATION_SERVICE_NAMES as SHARED_MAP_ANNOTATION_SERVICE_NAMES,
  MAPPING_SERVICE_NAMES as SHARED_MAPPING_SERVICE_NAMES,
  MAPPING_STATUS_TOPIC as SHARED_MAPPING_STATUS_TOPIC,
  MISSION_SERVICE_NAMES as SHARED_MISSION_SERVICE_NAMES,
  MISSION_STATUS_TOPIC as SHARED_MISSION_STATUS_TOPIC,
  mapTurtleBot4Nav2Capabilities as mapSharedTurtleBot4Nav2Capabilities,
  unsupportedTurtleBot4Nav2Command as sharedUnsupportedTurtleBot4Nav2Command,
} from "../panels-standalone/.generated/tensorfleet-util/vacuum/backends/turtlebot4-nav2/capabilityMapper";
import {
  TURTLEBOT4_NAV2_CANCEL_GOAL_SERVICE,
  TURTLEBOT4_NAV2_SEND_GOAL_SERVICE,
} from "../panels-standalone/.generated/tensorfleet-util/vacuum/backends/turtlebot4-nav2/runtimeTypes";
import {
  CANCEL_GOAL_SERVICE,
  SEND_GOAL_SERVICE,
} from "../panels-standalone/src/components/Nav2/runtime/nav2RuntimeConstants";

type VacuumCommand = {
  command: string;
  [key: string]: unknown;
};

type CheckResult = {
  name: string;
  drift?: string;
};

const repoRoot = resolve(import.meta.dir, "..");

const VALETUDO_CAPABILITY_FIXTURE = [
  "BasicControlCapability",
  "BatteryStateCapability",
  "ConsumableMonitoringCapability",
  "FanSpeedControlCapability",
  "GoToLocationCapability",
  "WaterUsageControlCapability",
] as const;

function pass(name: string): CheckResult {
  console.log(`PASS shared parity: ${name}`);
  return { name };
}

function knownDrift(name: string, reason: string): CheckResult {
  console.log(`KNOWN DRIFT: ${name} - ${reason}`);
  return { name, drift: reason };
}

function readRepoFile(path: string): string {
  return readFileSync(resolve(repoRoot, path), "utf8");
}

function assertSharedReExportShim(path: string, target: string): void {
  const source = readRepoFile(path).trim();
  assert.equal(source, `export * from "${target}";`, `${path} must remain a shared re-export shim`);
}

function sorted<T>(values: Iterable<T>): T[] {
  return Array.from(values).sort();
}

function unionLiterals(source: string, typeName: string): string[] {
  const start = source.indexOf(`export type ${typeName}`);
  assert.notEqual(start, -1, `Missing exported type ${typeName}`);
  const nextExport = source.indexOf("\nexport type ", start + 1);
  const block = source.slice(start, nextExport === -1 ? undefined : nextExport);
  return sorted(block.matchAll(/"([^"]+)"/g).map((match) => match[1]));
}

function createSmallOccupancyGrid(): Record<string, unknown> {
  return {
    header: { frame_id: "map" },
    info: {
      width: "4",
      height: 2,
      resolution: "0.25",
      origin: {
        position: { x: "-1.5", y: 2 },
        orientation: { x: 0, y: 0, z: Math.SQRT1_2, w: Math.SQRT1_2 },
      },
    },
    data: [-1, 0, 15, 16, 100, -1, 7, 42],
  };
}

function createValetudoRuntimeSnapshot(overrides: Record<string, unknown> = {}): Record<string, unknown> {
  return {
    runtime: { id: "runtime-1", version: "test", status: "online" },
    backend: "valetudo",
    robot: { id: "robot-1", name: "Kitchen Vacuum" },
    source: { kind: "valetudo_mock", status: "reachable", stale: false, lastSeenAt: 1_700_000_000_000 },
    connectivity: { reachable: true, online: true },
    state: { value: "idle", label: "Idle", started: false, paused: false },
    battery: { level: 82, charging: false },
    dock: { state: "docked", docked: true },
    cleaningSettings: {
      fanSpeed: { current: "balanced", options: ["quiet", "balanced", "turbo"] },
      waterUsage: { current: "medium", options: ["low", "medium", "high"] },
    },
    capabilities: {
      commands: {
        start_cleaning: { available: true },
        pause: { available: false, reason: "invalid_state" },
        resume: { available: false, reason: "invalid_state" },
        stop: { available: false, reason: "invalid_state" },
        return_to_dock: { available: false, reason: "invalid_state" },
        set_fan_speed: { available: true },
        set_water_usage: { available: true },
      },
      diagnostics: [
        { name: "BasicControlCapability", detected: true, implemented: true, scope: "command" },
        { name: "BatteryStateCapability", detected: true, implemented: true, scope: "state" },
      ],
    },
    diagnostics: {
      mode: "test",
      rawCapabilityNames: ["BasicControlCapability", "BatteryStateCapability"],
      notes: [],
    },
    updatedAt: 1_700_000_000_001,
    ...overrides,
  };
}

function testCapabilities(): CheckResult {
  assertSharedReExportShim("panels-standalone/src/vacuum-adapter/capabilities.ts", "tensorfleet-util/vacuum/capabilities");
  assert.deepEqual(LOCAL_CAPABILITY_NAMES, SHARED_CAPABILITY_NAMES);
  assert.deepEqual(createLocalUnsupportedCapabilities(), createSharedUnsupportedCapabilities());
  for (const descriptor of Object.values(createSharedUnsupportedCapabilities())) {
    assert.deepEqual(sorted(Object.keys(descriptor)), ["available", "notes", "status", "supported"]);
  }
  return pass("capabilities");
}

function testCommands(): CheckResult {
  assertSharedReExportShim("panels-standalone/src/vacuum-adapter/commands.ts", "tensorfleet-util/vacuum/commands");
  assert.deepEqual(sorted(LOCAL_COMMAND_NAMES), sorted(SHARED_COMMAND_NAMES));
  for (const command of LOCAL_COMMAND_NAMES) {
    assert.ok(SHARED_COMMAND_NAMES.includes(command), `${command} missing from shared VACUUM_COMMAND_NAMES`);
  }
  assert.ok(SHARED_COMMAND_NAMES.includes("start_cleaning"));
  assert.ok(SHARED_COMMAND_NAMES.includes("set_fan_speed"));
  assert.deepEqual(
    { ok: true, command: "start_cleaning", message: "ok" },
    { ok: true, command: "start_cleaning", message: "ok" },
  );
  assert.deepEqual(
    { ok: false, command: "pause", error: sharedUnsupportedCommand("pause") },
    { ok: false, command: "pause", error: localUnsupportedCommand("pause") },
  );
  return pass("commands");
}

function testErrors(): CheckResult {
  assertSharedReExportShim("panels-standalone/src/vacuum-adapter/errors.ts", "tensorfleet-util/vacuum/errors");
  const sharedSource = readRepoFile("panels-standalone/.generated/tensorfleet-util/vacuum/errors.ts");
  assert.deepEqual(unionLiterals(sharedSource, "VacuumCommandErrorCode"), unionLiterals(sharedSource, "VacuumCommandErrorCode"));
  assert.deepEqual(localUnsupportedCommand("return_to_dock"), sharedUnsupportedCommand("return_to_dock"));
  assert.deepEqual(
    localUnsupportedCommand("return_to_dock", "custom message"),
    sharedUnsupportedCommand("return_to_dock", "custom message"),
  );
  return pass("errors");
}

function testState(): CheckResult {
  assertSharedReExportShim("panels-standalone/src/vacuum-adapter/state.ts", "tensorfleet-util/vacuum/state");
  const sharedSource = readRepoFile("panels-standalone/.generated/tensorfleet-util/vacuum/state.ts");
  for (const typeName of [
    "VacuumReadinessState",
    "VacuumNavigationState",
    "VacuumMissionStatus",
    "VacuumRobotActivityStatus",
    "VacuumSourceStatus",
    "VacuumDockState",
  ]) {
    assert.ok(unionLiterals(sharedSource, typeName).length > 0, typeName);
  }
  const activeMissionShape = {
    id: "mission-1",
    type: "coverage",
    status: "running",
    backendSource: "turtlebot4_nav2",
    startedAt: 1,
    updatedAt: 2,
    requestedCommand: "start_coverage",
    phase: "running",
    progress: {
      percent: 25,
      currentStep: 1,
      totalSteps: 4,
      distanceRemaining: 3,
      areaCoveredSqM: 2,
      areaRemainingSqM: 6,
    },
    availableActions: ["pause_mission", "cancel_mission"],
    result: null,
    error: null,
    target: { area: "fixture" },
  };
  assert.deepEqual(Object.keys(activeMissionShape), [
    "id",
    "type",
    "status",
    "backendSource",
    "startedAt",
    "updatedAt",
    "requestedCommand",
    "phase",
    "progress",
    "availableActions",
    "result",
    "error",
    "target",
  ]);
  return pass("state");
}

function testMapGrid(): CheckResult {
  assertSharedReExportShim("panels-standalone/src/vacuum-adapter/mapGrid.ts", "tensorfleet-util/vacuum/mapGrid");
  const message = createSmallOccupancyGrid();
  const localGrid = parseLocalVacuumMapGrid(message);
  const sharedGrid = parseSharedVacuumMapGrid(message);
  assert.deepEqual(localGrid, sharedGrid);
  assert.deepEqual(buildLocalVacuumMapMetadata(localGrid, 123), buildSharedVacuumMapMetadata(sharedGrid, 123));
  assert.deepEqual(parseLocalVacuumMapGrid(null), parseSharedVacuumMapGrid(null));
  assert.deepEqual(parseLocalVacuumMapGrid({ info: { width: 1 } }), parseSharedVacuumMapGrid({ info: { width: 1 } }));
  const typedArrayMessage = createSmallOccupancyGrid();
  typedArrayMessage.data = new Int8Array([-1, 0, 100]);
  typedArrayMessage.info = { width: 3, height: 1, resolution: 1, origin: { position: { x: 0, y: 0 } } };
  assert.deepEqual(parseLocalVacuumMapGrid(typedArrayMessage), parseSharedVacuumMapGrid(typedArrayMessage));
  return pass("mapGrid");
}

function assertValetudoCapabilityMapping(): CheckResult | null {
  try {
    assert.deepEqual(
      mapLocalValetudoCapabilities(VALETUDO_CAPABILITY_FIXTURE, {
        commandAvailability: {
          pause: { available: false, reason: "invalid_state" },
          fan_speed: { available: true },
        },
        consumablesSupported: true,
        currentStatisticsSupported: true,
        attachmentsSupported: true,
        attachmentKinds: ["mop"],
        dockComponentsSupported: true,
        dockComponentKinds: ["dustbag"],
      }),
      mapSharedValetudoCapabilities(VALETUDO_CAPABILITY_FIXTURE, {
        commandAvailability: {
          pause: { available: false, reason: "invalid_state" },
          fan_speed: { available: true },
        },
        consumablesSupported: true,
        currentStatisticsSupported: true,
        attachmentsSupported: true,
        attachmentKinds: ["mop"],
        dockComponentsSupported: true,
        dockComponentKinds: ["dustbag"],
      }),
    );
    return null;
  } catch (error) {
    return knownDrift("Valetudo mappers", error instanceof Error ? error.message.split("\n")[0] : String(error));
  }
}

function assertValetudoCommandMapping(): void {
  const capabilities = mapLocalValetudoCapabilities(VALETUDO_CAPABILITY_FIXTURE, {
    commandAvailability: {
      pause: { available: true },
      fan_speed: { available: true },
      water_usage: { available: true },
    },
  });
  const commands: VacuumCommand[] = [
    { command: "start_cleaning" },
    { command: "pause" },
    { command: "resume_mission" },
    { command: "return_to_dock" },
    { command: "go_to_location", target: { x: 1, y: 2, yaw: 90 } },
    { command: "set_fan_speed", value: " turbo " },
    { command: "set_water_usage", value: "medium" },
    { command: "start_room_cleaning", annotation: { id: "room-1" } },
  ];
  for (const command of commands) {
    assert.deepEqual(
      mapLocalVacuumCommandToValetudoRequest(command as never, capabilities),
      mapSharedVacuumCommandToValetudoRequest(command as never, capabilities),
      command.command,
    );
  }
  for (const result of [
    { ok: true, status: "success", command: "start_cleaning", message: "started", updatedAt: 1 },
    { ok: false, status: "failed", command: "pause", message: "", reason: "command_invalid_state", updatedAt: 2 },
    { ok: false, status: "unavailable", command: "stop", message: "offline", code: "runtime_offline", updatedAt: 3 },
  ]) {
    assert.deepEqual(
      mapLocalValetudoRuntimeCommandResult(result.command as never, result as never),
      mapSharedValetudoRuntimeCommandResult(result.command as never, result as never),
      result.command,
    );
  }
  const snapshotWithResume = createValetudoRuntimeSnapshot({
    capabilities: { commands: { resume: { available: true } }, diagnostics: [] },
  });
  assert.deepEqual(
    mapLocalVacuumCommandToValetudoRuntimeCommandName("resume_mission" as never, snapshotWithResume as never),
    mapSharedVacuumCommandToValetudoRuntimeCommandName("resume_mission" as never, snapshotWithResume as never),
  );
}

function assertValetudoStateMapping(): void {
  const onlineSnapshot = createValetudoRuntimeSnapshot();
  const staleSnapshot = createValetudoRuntimeSnapshot({
    source: { kind: "valetudo_mock", status: "reachable", stale: true, lastSeenAt: 1_700_000_000_000 },
  });
  const offlineSnapshot = createValetudoRuntimeSnapshot({
    runtime: { id: "runtime-1", version: "test", status: "offline" },
    connectivity: { reachable: false, online: false },
    source: { kind: "valetudo_mock", status: "unreachable", stale: false, lastSeenAt: null },
  });
  for (const snapshot of [onlineSnapshot, staleSnapshot, offlineSnapshot]) {
    assert.deepEqual(isLocalValetudoRuntimeSnapshot(snapshot), isSharedValetudoRuntimeSnapshot(snapshot));
    assert.deepEqual(
      mapLocalValetudoRuntimeSnapshotToBoundary(snapshot as never),
      mapSharedValetudoRuntimeSnapshotToBoundary(snapshot as never),
    );
    assert.deepEqual(
      mapLocalValetudoState(mapLocalValetudoRuntimeSnapshotToBoundary(snapshot as never)),
      mapSharedValetudoState(mapSharedValetudoRuntimeSnapshotToBoundary(snapshot as never)),
    );
  }
  assert.deepEqual(mapLocalValetudoRuntimeUnavailable("offline"), mapSharedValetudoRuntimeUnavailable("offline"));
}

function testValetudoMappers(): CheckResult {
  assertSharedReExportShim(
    "panels-standalone/src/vacuum-adapter/backends/valetudo/capabilityMapper.ts",
    "tensorfleet-util/vacuum/backends/valetudo/capabilityMapper",
  );
  assertSharedReExportShim(
    "panels-standalone/src/vacuum-adapter/backends/valetudo/commandMapper.ts",
    "tensorfleet-util/vacuum/backends/valetudo/commandMapper",
  );
  assertSharedReExportShim(
    "panels-standalone/src/vacuum-adapter/backends/valetudo/runtimeCommandMapper.ts",
    "tensorfleet-util/vacuum/backends/valetudo/runtimeCommandMapper",
  );
  assertSharedReExportShim(
    "panels-standalone/src/vacuum-adapter/backends/valetudo/runtimeContract.ts",
    "tensorfleet-util/vacuum/backends/valetudo/runtimeContract",
  );
  assertSharedReExportShim(
    "panels-standalone/src/vacuum-adapter/backends/valetudo/stateMapper.ts",
    "tensorfleet-util/vacuum/backends/valetudo/stateMapper",
  );
  assertSharedReExportShim(
    "panels-standalone/src/vacuum-adapter/backends/valetudo/types.ts",
    "tensorfleet-util/vacuum/backends/valetudo/types",
  );
  const capabilityDrift = assertValetudoCapabilityMapping();
  if (capabilityDrift) {
    return capabilityDrift;
  }
  assertValetudoCommandMapping();
  assertValetudoStateMapping();
  return pass("Valetudo mappers");
}

function createLocalTurtleBot4Runtime(availableServices: string[]): Record<string, unknown> {
  return {
    connectionStatus: "connected",
    availableTopics: [{ topic: LOCAL_MISSION_STATUS_TOPIC, type: "std_msgs/msg/String" }],
    availableServices,
  };
}

function createSharedTurtleBot4Runtime(availableServices: string[]): Record<string, unknown> {
  return {
    connectionStatus: "connected",
    availableTopics: [{ topic: SHARED_MISSION_STATUS_TOPIC, type: "std_msgs/msg/String" }],
    availableServices,
    topicHealth: [],
    goalState: "ready",
  };
}

function testTurtleBot4CapabilityMapping(): CheckResult {
  assertSharedReExportShim(
    "panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/capabilityMapper.ts",
    "tensorfleet-util/vacuum/backends/turtlebot4-nav2/capabilityMapper",
  );
  assertSharedReExportShim(
    "panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/runtimeTypes.ts",
    "tensorfleet-util/vacuum/backends/turtlebot4-nav2/runtimeTypes",
  );
  const localServices = [
    SEND_GOAL_SERVICE,
    CANCEL_GOAL_SERVICE,
    ...Object.values(LOCAL_MAPPING_SERVICE_NAMES),
    ...Object.values(LOCAL_MISSION_SERVICE_NAMES),
    ...Object.values(LOCAL_MAP_ANNOTATION_SERVICE_NAMES),
  ];
  const sharedServices = [
    TURTLEBOT4_NAV2_SEND_GOAL_SERVICE,
    TURTLEBOT4_NAV2_CANCEL_GOAL_SERVICE,
    ...Object.values(SHARED_MAPPING_SERVICE_NAMES),
    ...Object.values(SHARED_MISSION_SERVICE_NAMES),
    ...Object.values(SHARED_MAP_ANNOTATION_SERVICE_NAMES),
  ];
  assert.equal(LOCAL_MAPPING_STATUS_TOPIC, SHARED_MAPPING_STATUS_TOPIC);
  const localFull = mapLocalTurtleBot4Nav2Capabilities(createLocalTurtleBot4Runtime(localServices) as never);
  const sharedFull = mapSharedTurtleBot4Nav2Capabilities(createSharedTurtleBot4Runtime(sharedServices) as never);
  const localEmpty = mapLocalTurtleBot4Nav2Capabilities(createLocalTurtleBot4Runtime([]) as never);
  const sharedEmpty = mapSharedTurtleBot4Nav2Capabilities(createSharedTurtleBot4Runtime([]) as never);
  assert.deepEqual(localFull, sharedFull);
  assert.deepEqual(localEmpty, sharedEmpty);
  assert.deepEqual(localUnsupportedTurtleBot4Nav2Command("start_cleaning"), sharedUnsupportedTurtleBot4Nav2Command("start_cleaning"));
  return pass("TurtleBot4 capability mapper");
}

const results = [
  testCapabilities(),
  testCommands(),
  testErrors(),
  testState(),
  testMapGrid(),
  testValetudoMappers(),
  testTurtleBot4CapabilityMapping(),
];

const driftCount = results.filter((result) => result.drift).length;
if (driftCount > 0) {
  console.log(`DONE shared parity with ${driftCount} known drift area(s).`);
} else {
  console.log("DONE shared parity with no known drift.");
}
