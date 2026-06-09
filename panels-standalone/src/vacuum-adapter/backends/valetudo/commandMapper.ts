import type { VacuumCommand } from "../../commands";
import type { VacuumCapabilities } from "../../capabilities";
import type { VacuumCommandErrorCode } from "../../errors";
import type { VacuumCommandResult } from "../../commands";
import type { ValetudoRuntimeCommandResult } from "./runtimeContract";
import type { ValetudoCommandMappingResult } from "./types";

function blocked(
  command: VacuumCommand["command"],
  reason: VacuumCommandErrorCode,
  message: string,
): ValetudoCommandMappingResult {
  return {
    ok: false,
    command,
    reason,
    message,
  };
}

function unsupported(command: VacuumCommand["command"], message: string): ValetudoCommandMappingResult {
  return blocked(command, "unsupported", message);
}

function isVacuumCommandErrorCode(value: string | undefined): value is VacuumCommandErrorCode {
  return Boolean(value) && RICH_RUNTIME_ERROR_CODES.has(value as VacuumCommandErrorCode);
}

function isCapabilityUsable(capabilities: VacuumCapabilities, name: keyof VacuumCapabilities): boolean {
  const capability = capabilities[name];
  return capability.supported && capability.available !== false && capability.status !== "unavailable";
}

function unavailableReasonCode(capabilities: VacuumCapabilities, name: keyof VacuumCapabilities): VacuumCommandErrorCode {
  const capability = capabilities[name];
  const structuredCode = capability.reasons?.find((reason) => isVacuumCommandErrorCode(reason.code))?.code;
  if (structuredCode) {
    return structuredCode;
  }
  if (isVacuumCommandErrorCode(capability.availabilityReason)) {
    return capability.availabilityReason;
  }
  return "unavailable";
}

function unavailableMessage(capabilities: VacuumCapabilities, name: keyof VacuumCapabilities, fallback: string): string {
  const capability = capabilities[name];
  const structuredMessage = capability.reasons?.[0]?.message;
  if (structuredMessage && structuredMessage !== capability.reasons?.[0]?.code) {
    return structuredMessage;
  }
  return humanizeRuntimeReason(capability.availabilityReason) ?? capability.notes ?? fallback;
}

function supportedOrUnavailable(
  command: VacuumCommand,
  capabilities: VacuumCapabilities,
  name: keyof VacuumCapabilities,
  request: NonNullable<Extract<ValetudoCommandMappingResult, { ok: true }>["request"]>,
  unsupportedMessage: string,
): ValetudoCommandMappingResult {
  const capability = capabilities[name];
  if (!capability.supported) {
    return unsupported(command.command, unsupportedMessage);
  }
  if (!isCapabilityUsable(capabilities, name)) {
    return blocked(
      command.command,
      unavailableReasonCode(capabilities, name),
      unavailableMessage(capabilities, name, unsupportedMessage),
    );
  }
  return { ok: true, command: command.command, request };
}

function commandValue(command: VacuumCommand): string | null {
  if (command.command !== "set_fan_speed" && command.command !== "set_water_usage") {
    return null;
  }
  return typeof command.value === "string" && command.value.trim() !== "" ? command.value.trim() : null;
}

function capabilityOptionValues(capabilities: VacuumCapabilities, name: keyof VacuumCapabilities): string[] {
  return (capabilities[name].attributes ?? [])
    .map((attribute) => attribute.match(/^option:(.+)$/)?.[1])
    .filter((value): value is string => Boolean(value));
}

function validateSelectedOption(
  command: VacuumCommand,
  capabilities: VacuumCapabilities,
  name: keyof VacuumCapabilities,
  value: string,
): ValetudoCommandMappingResult | null {
  const options = capabilityOptionValues(capabilities, name);
  if (options.length > 0 && !options.includes(value)) {
    return blocked(command.command, "invalid_request", "Selected cleaning setting is not available.");
  }
  return null;
}

const RICH_RUNTIME_ERROR_CODES = new Set<VacuumCommandErrorCode>([
  "unsupported",
  "unavailable",
  "invalid_state",
  "stale_source",
  "runtime_offline",
  "source_unreachable",
  "backend_timeout",
  "malformed_backend_response",
  "degraded_runtime",
  "needs_assistance",
  "not_ready",
  "backend_error",
  "invalid_request",
]);

const RUNTIME_ERROR_CODE_ALIASES: Record<string, VacuumCommandErrorCode> = {
  unsupported_command: "unsupported",
  capability_unavailable: "unsupported",
  command_invalid_state: "invalid_state",
  invalid_json: "invalid_request",
  invalid_value: "invalid_request",
  missing_command: "invalid_request",
  missing_value: "invalid_request",
  source_command_failed: "backend_error",
  mqtt_publish_failure: "backend_error",
  malformed_snapshot: "malformed_backend_response",
  malformed_command_response: "malformed_backend_response",
};

function humanizeRuntimeReason(value: string | undefined): string | null {
  if (!value) {
    return null;
  }
  const knownReasons: Record<string, string> = {
    unsupported: "Not supported by this backend.",
    unsupported_command: "Not supported by this backend.",
    capability_unavailable: "Not supported by this backend.",
    invalid_state: "Robot state does not allow this action.",
    command_invalid_state: "Robot state does not allow this action.",
    source_unreachable: "Source unreachable.",
    stale_source: "Robot state is stale.",
    runtime_offline: "Runtime offline.",
    degraded_runtime: "Runtime degraded.",
    invalid_request: "Invalid command request.",
    invalid_json: "Invalid command payload.",
    missing_command: "Missing command.",
    source_command_failed: "Backend command failed.",
    backend_error: "Backend command failed.",
    malformed_backend_response: "Malformed backend response.",
  };
  return knownReasons[value] ?? null;
}

function normalizeRuntimeErrorCode(result: ValetudoRuntimeCommandResult): VacuumCommandErrorCode {
  if (result.code && RUNTIME_ERROR_CODE_ALIASES[result.code]) {
    return RUNTIME_ERROR_CODE_ALIASES[result.code];
  }
  if (result.reason && RUNTIME_ERROR_CODE_ALIASES[result.reason]) {
    return RUNTIME_ERROR_CODE_ALIASES[result.reason];
  }
  if (result.code && RICH_RUNTIME_ERROR_CODES.has(result.code as VacuumCommandErrorCode)) {
    return result.code as VacuumCommandErrorCode;
  }
  if (result.reason && RICH_RUNTIME_ERROR_CODES.has(result.reason as VacuumCommandErrorCode)) {
    return result.reason as VacuumCommandErrorCode;
  }
  if (result.status === "unsupported") {
    return "unsupported";
  }
  if (result.status === "unavailable") {
    return "unavailable";
  }
  return "backend_error";
}

export function mapValetudoRuntimeCommandResult(
  command: VacuumCommand["command"],
  result: ValetudoRuntimeCommandResult,
): VacuumCommandResult {
  if (result.ok && result.status === "success") {
    return {
      ok: true,
      command,
      message: result.message,
    };
  }
  return {
    ok: false,
    command,
    error: {
      command,
      code: normalizeRuntimeErrorCode(result),
      message: result.message || humanizeRuntimeReason(result.code) || humanizeRuntimeReason(result.reason) || `Valetudo runtime command ${command} failed.`,
    },
  };
}

export function mapVacuumCommandToValetudoRequest(
  command: VacuumCommand,
  capabilities: VacuumCapabilities,
): ValetudoCommandMappingResult {
  if (command.command === "start_cleaning") {
    return supportedOrUnavailable(command, capabilities, "start_cleaning", { type: "basic_control", action: "start" }, "Valetudo basic cleaning control is not available.");
  }
  if (command.command === "pause") {
    return supportedOrUnavailable(command, capabilities, "pause", { type: "basic_control", action: "pause" }, "Valetudo pause support is not available.");
  }
  if (command.command === "pause_mission") {
    return supportedOrUnavailable(command, capabilities, "pause_mission", { type: "basic_control", action: "pause" }, "Valetudo mission pause support is not available.");
  }
  if (command.command === "stop") {
    return supportedOrUnavailable(command, capabilities, "stop", { type: "basic_control", action: "stop" }, "Valetudo stop support is not available.");
  }
  if (command.command === "cancel_mission") {
    return supportedOrUnavailable(command, capabilities, "cancel_mission", { type: "basic_control", action: "stop" }, "Valetudo mission cancel support is not available.");
  }
  if (command.command === "return_to_dock") {
    return supportedOrUnavailable(command, capabilities, "return_to_dock", { type: "basic_control", action: "home" }, "Valetudo return-to-dock support is not available.");
  }
  if (command.command === "start_navigation" || command.command === "go_to_location") {
    return capabilities.go_to_location.supported
      ? { ok: true, command: command.command, request: { type: "go_to_location", target: command.target } }
      : unsupported(command.command, "Valetudo go-to execution is not available.");
  }
  if (command.command === "set_fan_speed") {
    const value = commandValue(command);
    if (!value) {
      return blocked(command.command, "invalid_request", "Missing fan speed value.");
    }
    const invalid = validateSelectedOption(command, capabilities, "fan_speed", value);
    if (invalid) {
      return invalid;
    }
    return supportedOrUnavailable(
      command,
      capabilities,
      "fan_speed",
      { type: "set_fan_speed", value },
      "Valetudo fan speed support is not available.",
    );
  }
  if (command.command === "set_water_usage") {
    const value = commandValue(command);
    if (!value) {
      return blocked(command.command, "invalid_request", "Missing water usage value.");
    }
    const invalid = validateSelectedOption(command, capabilities, "water_usage", value);
    if (invalid) {
      return invalid;
    }
    return supportedOrUnavailable(
      command,
      capabilities,
      "water_usage",
      { type: "set_water_usage", value },
      "Valetudo water usage support is not available.",
    );
  }
  if (command.command === "segment_cleaning") {
    return unsupported(command.command, "segment_cleaning requires explicit segment ids in a later command payload.");
  }
  if (command.command === "start_room_cleaning") {
    return unsupported(command.command, "Room cleaning requires explicit Valetudo segment or annotation mapping in Layer 6.");
  }
  if (command.command === "start_zone_cleaning" || command.command === "zone_cleaning") {
    return unsupported(command.command, "Zone cleaning requires explicit Valetudo zone geometry mapping in Layer 6.");
  }
  if (command.command === "save_map_annotation" || command.command === "delete_map_annotation") {
    return unsupported(command.command, "Map annotation sessions are not implemented for the Valetudo backend stub.");
  }
  if (
    command.command === "start_mapping" ||
    command.command === "pause_mapping" ||
    command.command === "resume_mapping" ||
    command.command === "finish_mapping" ||
    command.command === "discard_mapping" ||
    command.command === "accept_map" ||
    command.command === "load_map"
  ) {
    return unsupported(command.command, "Mapping sessions are not implemented for the Valetudo backend stub.");
  }
  if (
    command.command === "start_coverage" ||
    command.command === "resume_mission" ||
    command.command === "retry_mission_step" ||
    command.command === "skip_mission_step"
  ) {
    return unsupported(command.command, "Mission lifecycle commands require a runtime-owned mission executor.");
  }
  return unsupported(command.command, `Command ${command.command} is not mapped for the Valetudo backend stub.`);
}
