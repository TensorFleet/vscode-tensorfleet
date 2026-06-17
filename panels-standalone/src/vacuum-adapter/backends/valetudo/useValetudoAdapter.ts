import { useCallback, useEffect, useMemo, useState } from "react";
import type { VacuumAdapter } from "../../adapter";
import type { VacuumCommand, VacuumCommandResult } from "../../commands";
import { mapVacuumCommandToValetudoRequest, mapValetudoRuntimeCommandResult } from "./commandMapper";
import { mapVacuumCommandToValetudoRuntimeCommandName } from "./runtimeCommandMapper";
import { createValetudoRuntimeClient, type ValetudoRuntimeClient } from "./runtimeClient";
import type { ValetudoRuntimeSnapshot } from "./runtimeContract";
import {
  isValetudoRuntimeSnapshot,
  mapValetudoRuntimeSnapshotToBoundary,
  mapValetudoRuntimeUnavailable,
  mapValetudoState,
} from "./stateMapper";

const POLL_INTERVAL_MS = 3000;

function runtimeCommandParams(mappedRequest: Extract<ReturnType<typeof mapVacuumCommandToValetudoRequest>, { ok: true }>["request"]): Record<string, unknown> | undefined {
  if ("value" in mappedRequest) {
    return { value: mappedRequest.value };
  }
  return undefined;
}

export function useValetudoAdapter(client?: ValetudoRuntimeClient): VacuumAdapter {
  const runtimeClient = useMemo(() => client ?? createValetudoRuntimeClient(), [client]);
  const [runtimeSnapshot, setRuntimeSnapshot] = useState<ValetudoRuntimeSnapshot | null>(null);
  const [lastError, setLastError] = useState<string | null>(null);

  const refresh = useCallback(async () => {
    try {
      const nextSnapshot = await runtimeClient.getSnapshot();
      if (!isValetudoRuntimeSnapshot(nextSnapshot)) {
        throw new Error("Valetudo integration runtime returned an unexpected snapshot shape.");
      }
      setRuntimeSnapshot(nextSnapshot);
      setLastError(null);
    } catch (error) {
      setLastError(error instanceof Error ? error.message : String(error));
    }
  }, [runtimeClient]);

  useEffect(() => {
    let active = true;
    const refreshIfActive = async () => {
      try {
        const nextSnapshot = await runtimeClient.getSnapshot();
        if (!isValetudoRuntimeSnapshot(nextSnapshot)) {
          throw new Error("Valetudo integration runtime returned an unexpected snapshot shape.");
        }
        if (active) {
          setRuntimeSnapshot(nextSnapshot);
          setLastError(null);
        }
      } catch (error) {
        if (active) {
          setLastError(error instanceof Error ? error.message : String(error));
        }
      }
    };
    void refreshIfActive();
    const interval = setInterval(() => void refreshIfActive(), POLL_INTERVAL_MS);
    return () => {
      active = false;
      clearInterval(interval);
    };
  }, [runtimeClient]);

  const snapshot = useMemo(() => {
    if (!runtimeSnapshot) {
      return mapValetudoState(
        mapValetudoRuntimeUnavailable(lastError ?? "Waiting for Valetudo integration runtime snapshot."),
      );
    }
    if (lastError) {
      return mapValetudoState(mapValetudoRuntimeUnavailable(lastError));
    }
    return mapValetudoState(mapValetudoRuntimeSnapshotToBoundary(runtimeSnapshot));
  }, [lastError, runtimeSnapshot]);

  const sendCommand = useCallback(
    async (command: VacuumCommand): Promise<VacuumCommandResult> => {
      const mapped = mapVacuumCommandToValetudoRequest(command, snapshot.capabilities);
      if (!mapped.ok) {
        return {
          ok: false,
          command: command.command,
          error: {
            command: command.command,
            code: mapped.reason,
            message: mapped.message,
          },
        };
      }
      try {
        const result = await runtimeClient.sendCommand({
          command: mapVacuumCommandToValetudoRuntimeCommandName(command.command, runtimeSnapshot),
          params: runtimeCommandParams(mapped.request),
        });
        await refresh();
        return mapValetudoRuntimeCommandResult(command.command, result);
      } catch (error) {
        await refresh();
        return {
          ok: false,
          command: command.command,
          error: {
            command: command.command,
            code: "runtime_offline",
            message: error instanceof Error ? error.message : String(error),
          },
        };
      }
    },
    [refresh, runtimeClient, runtimeSnapshot, snapshot.capabilities],
  );

  return useMemo(
    () => ({
      snapshot,
      sendCommand,
    }),
    [sendCommand, snapshot],
  );
}
