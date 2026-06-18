import assert from "node:assert/strict";
import { VACUUM_MCP_TOOL_NAMES, validateCommandRequest, validateSettingValue } from "../src/mcp/vacuum-tools";
import { resolveMcpRuntimeConfig } from "../src/mcp/config";
import { createVacuumRuntimeContext } from "../src/mcp/vacuum-runtime";

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

console.log("MCP vacuum regression passed.");
