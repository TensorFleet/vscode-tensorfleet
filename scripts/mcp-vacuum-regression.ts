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
  createVacuumTools,
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
  "vacuum_check_navigation_readiness",
  "vacuum_check_clean_area_readiness",
  "vacuum_get_supported_actions",
  "vacuum_start_cleaning",
  "vacuum_pause",
  "vacuum_resume",
  "vacuum_stop",
  "vacuum_return_to_dock",
  "vacuum_set_fan_speed",
  "vacuum_set_water_usage",
  "vacuum_pause_mission",
  "vacuum_resume_mission",
  "vacuum_cancel_mission",
  "vacuum_retry_mission_step",
  "vacuum_skip_mission_step",
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
  "vacuum_start_navigation",
  "vacuum_go_to_location",
  "vacuum_start_clean_area",
  "vacuum_start_room_cleaning",
  "vacuum_start_zone_cleaning",
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

function toolResult(response: any): any {
  if (Array.isArray(response?.content)) {
    return JSON.parse(response.content[0]?.text ?? "{}");
  }
  return response;
}

async function withToolEnv<T>(
  env: Record<string, string | undefined>,
  callback: () => Promise<T>,
): Promise<T> {
  const previous = {
    TENSORFLEET_VM_MANAGER_URL: process.env.TENSORFLEET_VM_MANAGER_URL,
    TENSORFLEET_JWT: process.env.TENSORFLEET_JWT,
    TENSORFLEET_VACUUM_BACKEND: process.env.TENSORFLEET_VACUUM_BACKEND,
  };
  for (const [key, value] of Object.entries(env)) {
    if (value == null) {
      delete process.env[key];
    } else {
      process.env[key] = value;
    }
  }
  try {
    return await callback();
  } finally {
    for (const [key, value] of Object.entries(previous)) {
      if (value == null) {
        delete process.env[key];
      } else {
        process.env[key] = value;
      }
    }
  }
}

async function withMockVacuumServer<T>(
  handler: (request: http.IncomingMessage, response: http.ServerResponse) => void,
  callback: (baseUrl: string) => Promise<T>,
): Promise<T> {
  const server = http.createServer(handler);
  await new Promise<void>((resolve) => server.listen(0, "127.0.0.1", resolve));
  try {
    const address = server.address();
    assert.ok(address && typeof address === "object");
    return await callback(`http://127.0.0.1:${address.port}`);
  } finally {
    await new Promise<void>((resolve) => server.close(() => resolve()));
  }
}

function jsonResponse(response: http.ServerResponse, status: number, body: unknown): void {
  response.writeHead(status, { "content-type": "application/json" });
  response.end(JSON.stringify(body));
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

const tools = createVacuumTools();
for (const toolName of [
  "vacuum_check_navigation_readiness",
  "vacuum_check_clean_area_readiness",
  "vacuum_get_supported_actions",
  "vacuum_pause_mission",
  "vacuum_resume_mission",
  "vacuum_cancel_mission",
  "vacuum_retry_mission_step",
  "vacuum_skip_mission_step",
]) {
  assert.ok(tools.has(toolName), toolName);
}

const readyNavigationSnapshot = {
  ...simulationReadSnapshot,
  navigation: { state: "idle", active: false, detail: "Idle." },
  mission: { state: "idle", detail: "No active mission." },
  activeMission: null,
  missions: { active: null, recent: [] },
};

const readyCleanAreaSnapshot = {
  ...readyNavigationSnapshot,
  capabilities: {
    ...(readyNavigationSnapshot.capabilities as Record<string, unknown>),
    start_coverage: { supported: true, available: true, status: "supported", commands: ["start_coverage"] },
  },
};

await withToolEnv(
  {
    TENSORFLEET_VM_MANAGER_URL: "http://vm-manager.example.test",
    TENSORFLEET_JWT: "token",
    TENSORFLEET_VACUUM_BACKEND: undefined,
  },
  async () => {
    const result = toolResult(await tools.get("vacuum_check_navigation_readiness")!.execute({}));
    assert.equal(result.ok, false);
    assert.equal(result.status, "invalid_state");
    assert.equal(result.reason, "missing_vacuum_backend");
  },
);

await withToolEnv(
  {
    TENSORFLEET_VM_MANAGER_URL: "http://vm-manager.example.test",
    TENSORFLEET_JWT: "token",
    TENSORFLEET_VACUUM_BACKEND: "valetudo",
  },
  async () => {
    const navigation = toolResult(await tools.get("vacuum_check_navigation_readiness")!.execute({}));
    assert.equal(navigation.ok, false);
    assert.equal(navigation.status, "unsupported");
    assert.equal(navigation.reason, "unsupported_backend");
    assert.equal(navigation.data.backend, "valetudo");

    const cleanArea = toolResult(await tools.get("vacuum_check_clean_area_readiness")!.execute({}));
    assert.equal(cleanArea.ok, false);
    assert.equal(cleanArea.status, "unsupported");
    assert.equal(cleanArea.reason, "unsupported_backend");
    assert.equal(cleanArea.data.backend, "valetudo");
  },
);

await withMockVacuumServer(
  (request, response) => {
    if (request.method === "GET" && request.url === "/vms/self/tensorfleet/api/v1/vacuum/snapshot") {
      jsonResponse(response, 404, { error: "not found" });
      return;
    }
    jsonResponse(response, 404, { error: "not found" });
  },
  async (baseUrl) => withToolEnv(
    {
      TENSORFLEET_VM_MANAGER_URL: baseUrl,
      TENSORFLEET_JWT: "token",
      TENSORFLEET_VACUUM_BACKEND: "simulation",
    },
    async () => {
      const result = toolResult(await tools.get("vacuum_check_navigation_readiness")!.execute({}));
      assert.equal(result.ok, false);
      assert.equal(result.status, "unavailable");
      assert.equal(result.reason, "simulation_snapshot_route_unavailable");
      assert.equal(result.data.backend, "turtlebot4_nav2");
      assert.equal(result.data.status, "unavailable");
    },
  ),
);

for (const [toolName, snapshot, expectedReason] of [
  [
    "vacuum_check_navigation_readiness",
    {
      ...readyNavigationSnapshot,
      map: {
        ...(readyNavigationSnapshot.map as Record<string, unknown>),
        readiness: "unavailable",
        metadata: { hasMap: false },
        detail: "Map is not available.",
      },
    },
    "map_unavailable",
  ],
  [
    "vacuum_check_clean_area_readiness",
    {
      ...readyCleanAreaSnapshot,
      map: {
        ...(readyCleanAreaSnapshot.map as Record<string, unknown>),
        readiness: "unavailable",
        metadata: { hasMap: false },
        detail: "Map is not available.",
      },
    },
    "map_unavailable",
  ],
  [
    "vacuum_check_navigation_readiness",
    {
      ...readyNavigationSnapshot,
      pose: { readiness: "waiting", available: false, coordinates: null, detail: "Waiting for pose." },
    },
    "pose_unavailable",
  ],
  [
    "vacuum_check_clean_area_readiness",
    {
      ...readyCleanAreaSnapshot,
      pose: { readiness: "waiting", available: false, coordinates: null, detail: "Waiting for pose." },
    },
    "pose_unavailable",
  ],
] as const) {
  await withMockVacuumServer(
    (request, response) => {
      if (request.method === "GET" && request.url === "/vms/self/tensorfleet/api/v1/vacuum/snapshot") {
        jsonResponse(response, 200, snapshot);
        return;
      }
      jsonResponse(response, 404, { error: "not found" });
    },
    async (baseUrl) => withToolEnv(
      {
        TENSORFLEET_VM_MANAGER_URL: baseUrl,
        TENSORFLEET_JWT: "token",
        TENSORFLEET_VACUUM_BACKEND: "simulation",
      },
      async () => {
        const args = toolName === "vacuum_check_navigation_readiness"
          ? { target: { x: 0, y: 0, theta: 0, frameId: "map" } }
          : { area: { type: "rectangle", x: 0, y: 0, width: 1, height: 1, frameId: "map" } };
        const result = toolResult(await tools.get(toolName)!.execute(args));
        assert.equal(result.ok, true);
        assert.equal(result.data.ready, false);
        assert.equal(result.data.status, "blocked");
        assert.ok(result.data.blockingReasons.includes(expectedReason), `${toolName} should include ${expectedReason}`);
      },
    ),
  );
}

await withMockVacuumServer(
  (request, response) => {
    if (request.method === "GET" && request.url === "/vms/self/tensorfleet/api/v1/vacuum/snapshot") {
      jsonResponse(response, 200, simulationReadSnapshot);
      return;
    }
    jsonResponse(response, 404, { error: "not found" });
  },
  async (baseUrl) => withToolEnv(
    {
      TENSORFLEET_VM_MANAGER_URL: baseUrl,
      TENSORFLEET_JWT: "token",
      TENSORFLEET_VACUUM_BACKEND: "simulation",
    },
    async () => {
      const result = toolResult(await tools.get("vacuum_check_navigation_readiness")!.execute({
        target: { x: 0, y: 0, theta: 0, frameId: "map" },
      }));
      assert.equal(result.ok, true);
      assert.equal(result.data.ready, false);
      assert.equal(result.data.status, "blocked");
      assert.ok(result.data.blockingReasons.includes("active_mission_incompatible"));
    },
  ),
);

await withMockVacuumServer(
  (request, response) => {
    if (request.method === "GET" && request.url === "/vms/self/tensorfleet/api/v1/vacuum/snapshot") {
      jsonResponse(response, 200, readyNavigationSnapshot);
      return;
    }
    jsonResponse(response, 404, { error: "not found" });
  },
  async (baseUrl) => withToolEnv(
    {
      TENSORFLEET_VM_MANAGER_URL: baseUrl,
      TENSORFLEET_JWT: "token",
      TENSORFLEET_VACUUM_BACKEND: "simulation",
    },
    async () => {
      const result = toolResult(await tools.get("vacuum_check_navigation_readiness")!.execute({}));
      assert.equal(result.ok, true);
      assert.equal(result.data.ready, false);
      assert.equal(result.data.status, "needs_input");
      assert.deepEqual(result.data.requiredInputs, ["target"]);
    },
  ),
);

await withMockVacuumServer(
  (request, response) => {
    if (request.method === "GET" && request.url === "/vms/self/tensorfleet/api/v1/vacuum/snapshot") {
      jsonResponse(response, 200, readyCleanAreaSnapshot);
      return;
    }
    jsonResponse(response, 404, { error: "not found" });
  },
  async (baseUrl) => withToolEnv(
    {
      TENSORFLEET_VM_MANAGER_URL: baseUrl,
      TENSORFLEET_JWT: "token",
      TENSORFLEET_VACUUM_BACKEND: "simulation",
    },
    async () => {
      const result = toolResult(await tools.get("vacuum_check_clean_area_readiness")!.execute({}));
      assert.equal(result.ok, true);
      assert.equal(result.data.ready, false);
      assert.equal(result.data.status, "needs_input");
      assert.deepEqual(result.data.requiredInputs, ["area"]);
    },
  ),
);

await withToolEnv(
  {
    TENSORFLEET_VM_MANAGER_URL: "http://vm-manager.example.test",
    TENSORFLEET_JWT: "token",
    TENSORFLEET_VACUUM_BACKEND: "simulation",
  },
  async () => {
    const invalidTarget = toolResult(await tools.get("vacuum_check_navigation_readiness")!.execute({ target: { x: "bad", y: 0 } }));
    assert.equal(invalidTarget.ok, false);
    assert.equal(invalidTarget.status, "invalid_request");
    assert.equal(invalidTarget.reason, "invalid_target");

    const invalidArea = toolResult(await tools.get("vacuum_check_clean_area_readiness")!.execute({
      area: { type: "rectangle", x: 0, y: 0, width: 0, height: 1 },
    }));
    assert.equal(invalidArea.ok, false);
    assert.equal(invalidArea.status, "invalid_request");
    assert.equal(invalidArea.reason, "invalid_area_dimensions");
  },
);

let preflightDispatchCount = 0;
await withMockVacuumServer(
  (request, response) => {
    if (request.method === "GET" && request.url === "/vms/self/tensorfleet/api/v1/vacuum/snapshot") {
      jsonResponse(response, 200, readyCleanAreaSnapshot);
      return;
    }
    if (request.method === "POST" && request.url === "/vms/self/tensorfleet/api/v1/vacuum/command") {
      preflightDispatchCount += 1;
      jsonResponse(response, 500, { error: "preflight must not dispatch" });
      return;
    }
    jsonResponse(response, 404, { error: "not found" });
  },
  async (baseUrl) => withToolEnv(
    {
      TENSORFLEET_VM_MANAGER_URL: baseUrl,
      TENSORFLEET_JWT: "token",
      TENSORFLEET_VACUUM_BACKEND: "simulation",
    },
    async () => {
      const navigation = toolResult(await tools.get("vacuum_check_navigation_readiness")!.execute({
        target: { x: 0, y: 0, theta: 0, frameId: "map" },
      }));
      assert.equal(navigation.ok, true);
      assert.equal(navigation.data.ready, true);
      assert.equal(navigation.data.status, "ready");

      const cleanArea = toolResult(await tools.get("vacuum_check_clean_area_readiness")!.execute({
        area: { type: "rectangle", x: 0, y: 0, width: 1, height: 1, frameId: "map" },
      }));
      assert.equal(cleanArea.ok, true);
      assert.equal(cleanArea.data.ready, true);
      assert.equal(cleanArea.data.status, "ready");
      assert.equal(preflightDispatchCount, 0);
      assert.doesNotMatch(JSON.stringify({ navigation, cleanArea }), /\/map|\/pose|NavigateToPose|geometry_msgs|nav_msgs|Foxglove|topic|service|http:\/\/|10\.\d+\.\d+\.\d+/i);
    },
  ),
);

await withMockVacuumServer(
  (request, response) => {
    if (request.method === "GET" && request.url === "/vms/self/tensorfleet/api/v1/vacuum/snapshot") {
      jsonResponse(response, 200, readyCleanAreaSnapshot);
      return;
    }
    jsonResponse(response, 404, { error: "not found" });
  },
  async (baseUrl) => withToolEnv(
    {
      TENSORFLEET_VM_MANAGER_URL: baseUrl,
      TENSORFLEET_JWT: "token",
      TENSORFLEET_VACUUM_BACKEND: "simulation",
    },
    async () => {
      const result = toolResult(await tools.get("vacuum_get_supported_actions")!.execute({}));
      assert.equal(result.ok, true);
      assert.deepEqual(result.data.callableMovementWriteTools, []);
      assert.ok(result.data.deferredActions.includes("vacuum_start_navigation"));
      assert.ok(result.data.futureMovementActions.navigation.supported);
      assert.doesNotMatch(JSON.stringify(result), /\/map|\/pose|NavigateToPose|geometry_msgs|nav_msgs|Foxglove|topic|service|http:\/\/|10\.\d+\.\d+\.\d+/i);
    },
  ),
);

await withToolEnv(
  {
    TENSORFLEET_VM_MANAGER_URL: "http://vm-manager.example.test",
    TENSORFLEET_JWT: "token",
    TENSORFLEET_VACUUM_BACKEND: "valetudo",
  },
  async () => {
    const result = toolResult(await tools.get("vacuum_cancel_mission")!.execute({}));
    assert.equal(result.ok, false);
    assert.equal(result.status, "unsupported");
    assert.equal(result.reason, "unsupported_backend");
    assert.equal(result.data.backend, "valetudo");
  },
);

await withToolEnv(
  {
    TENSORFLEET_VM_MANAGER_URL: "http://vm-manager.example.test",
    TENSORFLEET_JWT: "token",
    TENSORFLEET_VACUUM_BACKEND: undefined,
  },
  async () => {
    const missingBackendResult = createVacuumRuntimeContext(await resolveMcpRuntimeConfig(
      {
        TENSORFLEET_VM_MANAGER_URL: "http://vm-manager.example.test",
        TENSORFLEET_JWT: "token",
      },
      async () => null,
    ));
    assert.equal(missingBackendResult.ok, false);
    if (!missingBackendResult.ok) {
      assert.equal(missingBackendResult.result.status, "invalid_state");
      assert.equal(missingBackendResult.result.reason, "missing_vacuum_backend");
    }
  },
);

const noActiveMissionSnapshot = {
  ...simulationReadSnapshot,
  activeMission: null,
  missions: { active: null, recent: [] },
};
await withMockVacuumServer(
  (request, response) => {
    if (request.method === "GET" && request.url === "/vms/self/tensorfleet/api/v1/vacuum/snapshot") {
      jsonResponse(response, 200, noActiveMissionSnapshot);
      return;
    }
    jsonResponse(response, 404, { error: "not found" });
  },
  async (baseUrl) => withToolEnv(
    {
      TENSORFLEET_VM_MANAGER_URL: baseUrl,
      TENSORFLEET_JWT: "token",
      TENSORFLEET_VACUUM_BACKEND: "simulation",
    },
    async () => {
      const result = toolResult(await tools.get("vacuum_cancel_mission")!.execute({}));
      assert.equal(result.ok, false);
      assert.equal(result.status, "invalid_state");
      assert.equal(result.reason, "missing_active_mission");
      assert.equal(result.data.blockingGate, "active_mission");
    },
  ),
);

const unavailableActionSnapshot = {
  ...simulationReadSnapshot,
  activeMission: {
    ...(simulationReadSnapshot.activeMission as Record<string, unknown>),
    status: "running",
    availableActions: [],
  },
};
await withMockVacuumServer(
  (request, response) => {
    if (request.method === "GET" && request.url === "/vms/self/tensorfleet/api/v1/vacuum/snapshot") {
      jsonResponse(response, 200, unavailableActionSnapshot);
      return;
    }
    jsonResponse(response, 404, { error: "not found" });
  },
  async (baseUrl) => withToolEnv(
    {
      TENSORFLEET_VM_MANAGER_URL: baseUrl,
      TENSORFLEET_JWT: "token",
      TENSORFLEET_VACUUM_BACKEND: "simulation",
    },
    async () => {
      const result = toolResult(await tools.get("vacuum_cancel_mission")!.execute({}));
      assert.equal(result.ok, false);
      assert.equal(result.status, "unavailable");
      assert.equal(result.reason, "mission_action_unavailable");
      assert.equal(result.data.blockingGate, "available_actions");
      assert.deepEqual(result.data.availableActions, []);
    },
  ),
);

let dispatchedCommand: unknown;
await withMockVacuumServer(
  (request, response) => {
    if (request.method === "GET" && request.url === "/vms/self/tensorfleet/api/v1/vacuum/snapshot") {
      jsonResponse(response, 200, simulationReadSnapshot);
      return;
    }
    if (request.method === "POST" && request.url === "/vms/self/tensorfleet/api/v1/vacuum/command") {
      const chunks: Buffer[] = [];
      request.on("data", (chunk) => chunks.push(chunk));
      request.on("end", () => {
        dispatchedCommand = JSON.parse(Buffer.concat(chunks).toString("utf8"));
        jsonResponse(response, 200, { ok: true, status: "success", command: "cancel_mission", message: "Canceled." });
      });
      return;
    }
    jsonResponse(response, 404, { error: "not found" });
  },
  async (baseUrl) => withToolEnv(
    {
      TENSORFLEET_VM_MANAGER_URL: baseUrl,
      TENSORFLEET_JWT: "token",
      TENSORFLEET_VACUUM_BACKEND: "simulation",
    },
    async () => {
      const result = toolResult(await tools.get("vacuum_cancel_mission")!.execute({}));
      assert.equal(result.ok, true);
      assert.equal(result.data.actionRequested, "cancel_mission");
      assert.equal(result.data.previousActiveMission.id, "mission-1");
      assert.equal(result.data.refreshedActiveMission.id, "mission-1");
      assert.deepEqual(dispatchedCommand, { command: "cancel_mission" });
    },
  ),
);

await withMockVacuumServer(
  (request, response) => {
    if (request.method === "GET" && request.url === "/vms/self/tensorfleet/api/v1/vacuum/snapshot") {
      jsonResponse(response, 200, simulationReadSnapshot);
      return;
    }
    if (request.method === "POST" && request.url === "/vms/self/tensorfleet/api/v1/vacuum/command") {
      jsonResponse(response, 404, { error: "not found" });
      return;
    }
    jsonResponse(response, 404, { error: "not found" });
  },
  async (baseUrl) => withToolEnv(
    {
      TENSORFLEET_VM_MANAGER_URL: baseUrl,
      TENSORFLEET_JWT: "token",
      TENSORFLEET_VACUUM_BACKEND: "simulation",
    },
    async () => {
      const result = toolResult(await tools.get("vacuum_cancel_mission")!.execute({}));
      assert.equal(result.ok, false);
      assert.equal(result.status, "unavailable");
      assert.equal(result.reason, "simulation_command_route_unavailable");
      assert.equal(result.data.blockingGate, "dispatch");
    },
  ),
);

console.log("MCP vacuum regression passed.");
