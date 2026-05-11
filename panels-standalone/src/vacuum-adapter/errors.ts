import type { VacuumCommandName } from "./commands";

export type VacuumCommandErrorCode =
  | "unsupported"
  | "not_ready"
  | "backend_error"
  | "invalid_request";

export type VacuumCommandError = {
  code: VacuumCommandErrorCode;
  command: VacuumCommandName;
  message: string;
};

export function unsupportedCommand(command: VacuumCommandName, message?: string): VacuumCommandError {
  return {
    code: "unsupported",
    command,
    message: message ?? `Command ${command} is not supported by this backend.`,
  };
}

