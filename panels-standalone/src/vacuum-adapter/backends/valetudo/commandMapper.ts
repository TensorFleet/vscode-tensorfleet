import type { VacuumCommand } from "../../commands";
import type { VacuumCapabilities } from "../../capabilities";
import type { ValetudoCommandMappingResult } from "./types";

function unsupported(command: VacuumCommand["command"], message: string): ValetudoCommandMappingResult {
  return {
    ok: false,
    command,
    reason: "unsupported",
    message,
  };
}

export function mapVacuumCommandToValetudoRequest(
  command: VacuumCommand,
  capabilities: VacuumCapabilities,
): ValetudoCommandMappingResult {
  if (command.command === "start_cleaning") {
    return capabilities.start_cleaning.supported
      ? { ok: true, command: command.command, request: { type: "basic_control", action: "start" } }
      : unsupported(command.command, "Valetudo BasicControlCapability is not available.");
  }
  if (command.command === "pause") {
    return capabilities.pause.supported
      ? { ok: true, command: command.command, request: { type: "basic_control", action: "pause" } }
      : unsupported(command.command, "Valetudo pause support is not available.");
  }
  if (command.command === "pause_mission") {
    return capabilities.pause_mission.supported
      ? { ok: true, command: command.command, request: { type: "basic_control", action: "pause" } }
      : unsupported(command.command, "Valetudo mission pause support is not available.");
  }
  if (command.command === "stop") {
    return capabilities.stop.supported
      ? { ok: true, command: command.command, request: { type: "basic_control", action: "stop" } }
      : unsupported(command.command, "Valetudo stop support is not available.");
  }
  if (command.command === "cancel_mission") {
    return capabilities.cancel_mission.supported
      ? { ok: true, command: command.command, request: { type: "basic_control", action: "stop" } }
      : unsupported(command.command, "Valetudo mission cancel support is not available.");
  }
  if (command.command === "return_to_dock") {
    return capabilities.return_to_dock.supported
      ? { ok: true, command: command.command, request: { type: "basic_control", action: "home" } }
      : unsupported(command.command, "Valetudo return-to-dock support is not available.");
  }
  if (command.command === "start_navigation" || command.command === "go_to_location") {
    return capabilities.go_to_location.supported
      ? { ok: true, command: command.command, request: { type: "go_to_location", target: command.target } }
      : unsupported(command.command, "Valetudo GoToLocationCapability is not available.");
  }
  if (command.command === "set_fan_speed") {
    return capabilities.fan_speed.supported
      ? { ok: true, command: command.command, request: { type: "set_fan_speed", value: command.value } }
      : unsupported(command.command, "Valetudo fan speed support is not available.");
  }
  if (command.command === "set_water_usage") {
    return capabilities.water_usage.supported
      ? { ok: true, command: command.command, request: { type: "set_water_usage", value: command.value } }
      : unsupported(command.command, "Valetudo water usage support is not available.");
  }
  if (command.command === "segment_cleaning") {
    return unsupported(command.command, "segment_cleaning requires explicit segment ids in a later command payload.");
  }
  if (command.command === "zone_cleaning") {
    return unsupported(command.command, "zone_cleaning requires explicit zone geometry in a later command payload.");
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
