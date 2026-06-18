import assert from "node:assert/strict";
import { VACUUM_MCP_TOOL_NAMES, validateCommandRequest, validateSettingValue } from "../src/mcp/vacuum-tools";
import { resolveMcpRuntimeConfig } from "../src/mcp/config";
import { createVacuumRuntimeContext } from "../src/mcp/vacuum-runtime";

const expectedTools = [
  "vacuum_get_health",
  "vacuum_get_snapshot",
  "vacuum_get_capabilities",
  "vacuum_get_map_targets",
  "vacuum_start_cleaning",
  "vacuum_pause",
  "vacuum_resume",
  "vacuum_stop",
  "vacuum_return_to_dock",
  "vacuum_set_fan_speed",
  "vacuum_set_water_usage",
];

assert.deepEqual([...VACUUM_MCP_TOOL_NAMES], expectedTools);

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
