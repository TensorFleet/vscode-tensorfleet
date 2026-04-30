import { useCallback, useMemo, useRef, useState } from "react";
import { useNav2Runtime } from "../../../components/Nav2/runtime/useNav2Runtime";
import type { VacuumAdapter } from "../../adapter";
import type { VacuumCommand, VacuumCommandResult } from "../../commands";
import type { VacuumGoalCoordinates } from "../../state";
import { dispatchTurtleBot4Nav2Command } from "./commandDispatcher";
import { mapTurtleBot4Nav2State } from "./stateMapper";

export function useTurtleBot4Nav2Adapter(): VacuumAdapter {
  const runtime = useNav2Runtime();
  const [currentTarget, setCurrentTarget] = useState<VacuumGoalCoordinates | null>(null);
  const [initialDistance, setInitialDistance] = useState<number | null>(null);
  const runtimeRef = useRef(runtime);
  runtimeRef.current = runtime;

  const snapshot = useMemo(
    () => mapTurtleBot4Nav2State({ runtime, currentTarget, initialDistance }),
    [currentTarget, initialDistance, runtime],
  );

  const sendCommand = useCallback(
    async (command: VacuumCommand): Promise<VacuumCommandResult> => {
      return await dispatchTurtleBot4Nav2Command(command, {
        runtime: runtimeRef.current,
        snapshot,
        setCurrentTarget,
        setInitialDistance,
      });
    },
    [snapshot.capabilities, snapshot.readiness],
  );

  return {
    snapshot,
    sendCommand,
  };
}
