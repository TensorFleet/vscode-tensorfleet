import assert from "node:assert/strict";
import http from "node:http";
import {
  VACUUM_MCP_TOOL_NAMES,
  normalizeCapabilities,
  normalizeMap,
  normalizeMissionState,
  normalizeNavigationState,
  normalizePose,
  normalizeSnapshot,
  validateCommandRequest,
  validateSettingValue,
} from "../src/mcp/vacuum-tools";
import { resolveMcpRuntimeConfig } from "../src/mcp/config";
import { createVacuumRuntimeContext } from "../src/mcp/vacuum-runtime";
import { fetchVacuumSnapshot } from "../src/mcp/vacuum-runtime";

const expectedTools = [
  "vacuum_get_health",
  "vacuum_get_snapshot",
  "vacuum_get_capabilities",
  "vacuum_get_map_targets",
  "vacuum_get_pose",
  "vacuum_get_map_summary",
  "vacuum_get_mission_state",
  "vacuum_get_navigation_state",
  "vacuum_start_cleaning",
  "vacuum_pause",
  "vacuum_resume",
  "vacuum_stop",
  "vacuum_return_to_dock",
  "vacuum_set_fan_speed",
  "vacuum_set_water_usage",
];

assert.deepEqual([...VACUUM_MCP_TOOL_NAMES], expectedTools);
for (const forbiddenTool of [
  "nav2_send_goal",
  "nav2_get_status",
  "ros_publish",
  "call_runtime_endpoint",
  "valetudo_post_command",
  "drone_takeoff",
  "gazebo_spawn",
]) {
  assert.equal(VACUUM_MCP_TOOL_NAMES.includes(forbiddenTool as never), false);
}

const missingAuthConfig = await resolveMcpRuntimeConfig(
  { TENSORFLEET_VM_MANAGER_URL: "http://vm-manager.example.test" },
  async () => null,
);
const missingAuth = createVacuumRuntimeContext(missingAuthConfig);
assert.equal(missingAuth.ok, false);
if (!missingAuth.ok) {
  assert.equal(missingAuth.result.status, "not_authenticated");
  assert.equal(missingAuth.result.reason, "missing_token");
}

const missingBackendConfig = await resolveMcpRuntimeConfig(
  {
    TENSORFLEET_VM_MANAGER_URL: "http://vm-manager.example.test",
    TENSORFLEET_JWT: "token",
  },
  async () => null,
);
const missingBackend = createVacuumRuntimeContext(missingBackendConfig);
assert.equal(missingBackend.ok, false);
if (!missingBackend.ok) {
  assert.equal(missingBackend.result.status, "invalid_state");
  assert.equal(missingBackend.result.reason, "missing_vacuum_backend");
}

const simulationEnvConfig = await resolveMcpRuntimeConfig(
  {
    TENSORFLEET_VM_MANAGER_URL: "http://vm-manager.example.test",
    TENSORFLEET_JWT: "token",
    TENSORFLEET_VACUUM_BACKEND: "simulation",
  },
  async () => null,
);
const simulationContext = createVacuumRuntimeContext(simulationEnvConfig);
assert.equal(simulationContext.ok, true);
if (simulationContext.ok) {
  assert.equal(simulationContext.backend, "turtlebot4_nav2");
}

const missingSimulationRouteServer = http.createServer((_request, response) => {
  response.writeHead(404, { "content-type": "application/json" });
  response.end(JSON.stringify({ error: "not found" }));
});
await new Promise<void>((resolve) => missingSimulationRouteServer.listen(0, "127.0.0.1", resolve));
try {
  const address = missingSimulationRouteServer.address();
  assert.ok(address && typeof address === "object");
  const missingRoute = await fetchVacuumSnapshot({
    ok: true,
    backend: "turtlebot4_nav2",
    config: {
      vmManagerUrl: `http://127.0.0.1:${address.port}`,
      token: "token",
      tokenAvailable: true,
      selectedBackend: "turtlebot4_nav2",
      source: "env",
    },
    options: {
      baseUrl: `http://127.0.0.1:${address.port}`,
      token: "token",
      timeoutMs: 1000,
    },
  });
  assert.equal(missingRoute.ok, false);
  assert.equal(missingRoute.status, "unavailable");
  assert.equal(missingRoute.reason, "simulation_snapshot_route_unavailable");
} finally {
  await new Promise<void>((resolve) => missingSimulationRouteServer.close(() => resolve()));
}

const envAuthBridgeBackendConfig = await resolveMcpRuntimeConfig(
  {
    TENSORFLEET_VM_MANAGER_URL: "http://vm-manager.example.test",
    TENSORFLEET_JWT: "token",
  },
  async () => ({
    selectedBackend: "turtlebot4_nav2",
  }),
);
const envAuthBridgeBackend = createVacuumRuntimeContext(envAuthBridgeBackendConfig);
assert.equal(envAuthBridgeBackend.ok, true);
if (envAuthBridgeBackend.ok) {
  assert.equal(envAuthBridgeBackend.backend, "turtlebot4_nav2");
  assert.equal(envAuthBridgeBackend.config.source, "env+bridge");
}

const bridgeValetudoConfig = await resolveMcpRuntimeConfig(
  {},
  async () => ({
    vmManagerUrl: "http://vm-manager.example.test",
    token: "token",
    tokenAvailable: true,
    selectedBackend: "valetudo",
  }),
);
const bridgeValetudo = createVacuumRuntimeContext(bridgeValetudoConfig);
assert.equal(bridgeValetudo.ok, true);
if (bridgeValetudo.ok) {
  assert.equal(bridgeValetudo.backend, "valetudo");
}

const unavailableRuntime = validateCommandRequest(
  {
    runtime: { status: "offline" },
    source: { status: "reachable", stale: false },
    connectivity: { online: false, reachable: true },
    capabilities: { commands: { pause: { available: true } } },
  },
  "pause",
  {},
);
assert.equal(unavailableRuntime.ok, false);
assert.equal(unavailableRuntime.status, "runtime_offline");

const unsupportedCommand = validateCommandRequest(
  {
    runtime: { status: "online" },
    source: { status: "reachable", stale: false },
    connectivity: { online: true, reachable: true },
    capabilities: { commands: {} },
  },
  "return_to_dock",
  {},
);
assert.equal(unsupportedCommand.ok, false);
assert.equal(unsupportedCommand.status, "unsupported");

const unsupportedSimulationFanSpeed = validateCommandRequest(
  {
    availability: { status: "online", connected: true },
    source: { kind: "turtlebot4_nav2", status: "reachable", stale: false },
    capabilities: {
      fan_speed: {
        supported: false,
        available: false,
        status: "unsupported",
        notes: "Not supported by the TurtleBot4/Nav2 adapter slice.",
      },
    },
  },
  "set_fan_speed",
  { value: "turbo" },
);
assert.equal(unsupportedSimulationFanSpeed.ok, false);
assert.equal(unsupportedSimulationFanSpeed.status, "unsupported");
assert.equal(unsupportedSimulationFanSpeed.reason, "unsupported_command");

const unsupportedSimulationPause = validateCommandRequest(
  {
    availability: { status: "online", connected: true },
    source: { kind: "turtlebot4_nav2", status: "reachable", stale: false },
    capabilities: {
      pause: {
        supported: false,
        available: false,
        status: "unsupported",
      },
    },
  },
  "pause",
  {},
);
assert.equal(unsupportedSimulationPause.ok, false);
assert.equal(unsupportedSimulationPause.status, "unsupported");

const invalidFanSpeed = validateSettingValue(["balanced", "turbo"], "fan speed", "set_fan_speed", {
  value: "max",
});
assert.equal(invalidFanSpeed.ok, false);
assert.equal(invalidFanSpeed.status, "invalid_request");
assert.equal(invalidFanSpeed.reason, "invalid_value");

const validFanSpeed = validateCommandRequest(
  {
    runtime: { status: "online" },
    source: { status: "reachable", stale: false },
    connectivity: { online: true, reachable: true },
    cleaningSettings: { fanSpeed: { options: ["balanced", "turbo"] } },
    capabilities: { commands: { set_fan_speed: { available: true } } },
  },
  "set_fan_speed",
  { value: "turbo" },
);
assert.equal(validFanSpeed.ok, true);

const simulationReadSnapshot = {
  identity: {
    id: "simulation-vacuum",
    label: "Simulation Vacuum",
    source: "turtlebot4_nav2",
    model: "TurtleBot4 simulation",
  },
  availability: { status: "online", connected: true, detail: "Simulation runtime online." },
  source: { kind: "turtlebot4_nav2", status: "reachable", stale: false, lastSeenAt: 1800000000000 },
  updatedAt: 1800000000001,
  capabilities: {
    map: { supported: true, available: true, status: "supported", backendCapability: "/map", attributes: ["nav_msgs/msg/OccupancyGrid"] },
    pose: { supported: true, available: true, status: "supported", backendCapability: "/pose", attributes: ["geometry_msgs/msg/Pose"] },
    navigation_status: { supported: true, available: true, status: "supported", notes: "Backed by Nav2 action status." },
    mission_state: { supported: true, available: true, status: "supported" },
    start_navigation: { supported: true, available: true, status: "supported", commands: ["start_navigation"] },
    go_to_location: { supported: true, status: "supported", commands: ["go_to_location"], notes: "Backed by NavigateToPose." },
    start_coverage: { supported: true, available: false, availabilityReason: "map_required", commands: ["start_coverage"] },
    pause_mission: { supported: true, available: false, availabilityReason: "no_active_mission", commands: ["pause_mission"] },
    resume_mission: { supported: true, available: false, availabilityReason: "no_paused_mission", commands: ["resume_mission"] },
    cancel_mission: { supported: true, available: true, commands: ["cancel_mission"] },
    retry_mission_step: { supported: true, available: false, availabilityReason: "no_failed_step", commands: ["retry_mission_step"] },
    skip_mission_step: { supported: true, available: false, availabilityReason: "no_active_step", commands: ["skip_mission_step"] },
    mapping_session: { supported: true, available: false, availabilityReason: "mapping_idle", commands: ["start_mapping"] },
    auto_mapping: { supported: true, available: false, availabilityReason: "mapping_idle" },
    map_annotations: { supported: true, available: true, commands: ["save_map_annotation", "delete_map_annotation"] },
    room_semantics: { supported: true, available: true },
    zone_semantics: { supported: true, available: true },
    room_cleaning: { supported: true, available: false, availabilityReason: "no_room_selected", commands: ["start_room_cleaning"] },
    zone_cleaning: { supported: true, available: false, availabilityReason: "no_zone_selected", commands: ["start_zone_cleaning"] },
    fan_speed: { supported: false, available: false, status: "unsupported" },
    water_usage: { supported: false, available: false, status: "unsupported" },
  },
  map: {
    readiness: "ready",
    receiving: true,
    grid: {
      width: 3,
      height: 2,
      resolution: 0.05,
      originX: 0,
      originY: 0,
      originYaw: 0,
      frameId: "map",
      data: [0, 0, 100, -1, 0, 0],
    },
    metadata: {
      hasMap: true,
      width: 3,
      height: 2,
      resolution: 0.05,
      freeCells: 4,
      occupiedCells: 1,
      unknownCells: 1,
      knownCells: 5,
      totalCells: 6,
      freeRatio: 4 / 6,
      occupiedRatio: 1 / 6,
      unknownRatio: 1 / 6,
      knownRatio: 5 / 6,
      knownAreaSqM: 0.0125,
      lastUpdateAt: 1800000000000,
    },
    targets: {
      segments: [{ id: "seg-1", label: "Segment 1", available: true }],
      zones: [{ id: "zone-1", label: "Zone 1", available: true, geometry: { type: "rectangle", bounds: { x: 0, y: 0, width: 1, height: 1 } } }],
    },
    annotations: [
      { id: "room-1", kind: "room", name: "Lab", area: { shape: "rectangle", minX: 0, minY: 0, maxX: 1, maxY: 1 } },
      { id: "zone-2", kind: "zone", name: "Bench", area: { shape: "rectangle", minX: 1, minY: 1, maxX: 2, maxY: 2 } },
    ],
    detail: "Map is ready.",
  },
  pose: {
    readiness: "ready",
    available: true,
    coordinates: { x: 1.2, y: -0.4, yaw: 1.57 },
    detail: "Pose is available.",
  },
  navigation: {
    state: "active",
    active: true,
    isSending: false,
    isCanceling: false,
    currentTarget: { x: 2, y: 3, yaw: 0 },
    terminalState: null,
    planPath: [{ x: 1, y: 1 }, { x: 2, y: 3 }],
    progress: {
      distanceRemaining: 1.4,
      initialDistance: 3,
      recoveries: 0,
      navigationTime: { sec: 2 },
      estimatedTimeRemaining: { sec: 6 },
    },
    detail: "Navigating.",
  },
  mission: { state: "navigating", detail: "Robot is navigating." },
  activeMission: {
    id: "mission-1",
    type: "navigation",
    status: "running",
    phase: "en_route",
    progress: { percent: 0.5, currentStep: 1, totalSteps: 2, distanceRemaining: 1.4, areaCoveredSqM: null, areaRemainingSqM: null },
    availableActions: ["cancel_mission"],
    result: null,
    error: null,
    target: { x: 2, y: 3, yaw: 0 },
    startedAt: 1799999999000,
    updatedAt: 1800000000001,
  },
  missions: {
    active: null,
    recent: [
      {
        id: "mission-0",
        type: "coverage",
        status: "completed",
        phase: "done",
        progress: { percent: 1, currentStep: 2, totalSteps: 2, distanceRemaining: 0, areaCoveredSqM: 4, areaRemainingSqM: 0 },
        result: { status: "completed", completedAt: 1799999900000, summary: "Coverage completed." },
        updatedAt: 1799999900000,
      },
    ],
  },
  mapping: {
    state: "idle",
    mode: null,
    stateReason: "No mapping session active.",
    knownRatio: 5 / 6,
    unknownRatio: 1 / 6,
    frontierCount: 0,
    activeMapName: "lab-map",
    updatedAt: 1800000000000,
  },
  readiness: { ready: true, blockingReasons: [] },
};

const simulationCapabilities = normalizeCapabilities(simulationReadSnapshot, "turtlebot4_nav2");
for (const capabilityName of [
  "map",
  "pose",
  "navigation_status",
  "mission_state",
  "start_navigation",
  "go_to_location",
  "start_coverage",
  "pause_mission",
  "resume_mission",
  "cancel_mission",
  "retry_mission_step",
  "skip_mission_step",
  "mapping_session",
  "auto_mapping",
  "map_annotations",
  "room_semantics",
  "zone_semantics",
  "room_cleaning",
  "zone_cleaning",
]) {
  assert.ok((simulationCapabilities.features as Record<string, unknown>)[capabilityName], capabilityName);
}
assert.equal((simulationCapabilities.features as any).start_navigation.available, true);
assert.equal((simulationCapabilities.features as any).go_to_location.available, false);
assert.equal((simulationCapabilities.features as any).go_to_location.reason, "availability_not_reported");
assert.equal((simulationCapabilities.features as any).start_coverage.available, false);
assert.equal((simulationCapabilities.features as any).start_coverage.reason, "map_required");
assert.equal((simulationCapabilities.features as any).fan_speed.supported, false);
assert.equal((simulationCapabilities.features as any).water_usage.supported, false);
assert.doesNotMatch(JSON.stringify(simulationCapabilities), /\/map|\/pose|NavigateToPose|geometry_msgs|nav_msgs|Foxglove|topic|service/i);

const mapSummary = normalizeMap(simulationReadSnapshot);
assert.equal(mapSummary.available, true);
assert.equal((mapSummary.dimensions as any).width, 3);
assert.equal((mapSummary.annotations as any).rooms, 1);
assert.equal((mapSummary.targets as any).zoneCount, 1);
assert.equal("grid" in mapSummary, false);
assert.equal(JSON.stringify(mapSummary).includes("[0,0,100"), false);
const mapSummaryWithGrid = normalizeMap(simulationReadSnapshot, { includeGrid: true });
assert.ok((mapSummaryWithGrid.grid as any).data.length > 0);

const pose = normalizePose(simulationReadSnapshot);
assert.equal(pose.available, true);
assert.deepEqual(pose.coordinates, { x: 1.2, y: -0.4, yaw: 1.57 });
const missingPose = normalizePose({ ...simulationReadSnapshot, pose: { readiness: "waiting", available: false, coordinates: null, detail: "Waiting for pose." } });
assert.equal(missingPose.available, false);
assert.equal(missingPose.status, "unavailable");
assert.equal(missingPose.reason, "Waiting for pose.");

const missionState = normalizeMissionState(simulationReadSnapshot);
assert.equal((missionState.active as any).id, "mission-1");
assert.equal((missionState.active as any).requestedCommand, undefined);
assert.equal(missionState.recentCount, 1);

const navigationState = normalizeNavigationState(simulationReadSnapshot);
assert.equal(navigationState.state, "active");
assert.equal((navigationState.path as any).pointCount, 2);
assert.equal((navigationState.controls as any).cancel, true);
assert.equal(JSON.stringify(navigationState).includes("backendGoalState"), false);
const navigationFromMission = normalizeNavigationState({ ...simulationReadSnapshot, navigation: undefined });
assert.equal(navigationFromMission.available, false);
assert.equal(navigationFromMission.reason, "navigation_state_unavailable");
assert.equal((navigationFromMission.relatedMission as any).id, "mission-1");

const normalizedSnapshot = normalizeSnapshot(simulationReadSnapshot, {
  backend: "turtlebot4_nav2",
  includeDiagnostics: false,
  includeRawDiagnostics: false,
  includeMapPreview: false,
});
assert.equal((normalizedSnapshot.readiness as any).selectedBackend, "turtlebot4_nav2");
assert.equal((normalizedSnapshot.readiness as any).movementReady, true);
assert.equal("diagnostics" in normalizedSnapshot, false);
assert.equal(JSON.stringify(normalizedSnapshot).includes("[0,0,100"), false);

console.log("MCP vacuum regression passed.");
