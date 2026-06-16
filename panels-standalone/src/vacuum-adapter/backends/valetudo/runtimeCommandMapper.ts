import type { VacuumCommand } from "../../commands";
import type { ValetudoRuntimeCommandName, ValetudoRuntimeSnapshot } from "./runtimeContract";

function hasRuntimeCommand(snapshot: ValetudoRuntimeSnapshot | null, command: string): boolean {
  return Boolean(snapshot?.capabilities.commands[command]);
}

export function mapVacuumCommandToValetudoRuntimeCommandName(
  command: VacuumCommand["command"],
  snapshot: ValetudoRuntimeSnapshot | null,
): ValetudoRuntimeCommandName {
  if (command === "pause_mission") {
    return "pause";
  }
  if (command === "resume_mission" || command === "resume") {
    return hasRuntimeCommand(snapshot, "resume") ? "resume" : "start_cleaning";
  }
  if (command === "cancel_mission") {
    return "stop";
  }
  return command;
}
