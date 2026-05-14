import type { VacuumCommand, VacuumCommandResult } from "./commands";
import type { VacuumAdapterSnapshot } from "./state";

export type VacuumAdapter = {
  snapshot: VacuumAdapterSnapshot;
  sendCommand: (command: VacuumCommand) => Promise<VacuumCommandResult>;
};
