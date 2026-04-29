import { useCallback, useMemo, useRef, useState } from "react";
import { useNav2Runtime } from "../../../components/Nav2/runtime/useNav2Runtime";
import type { VacuumAdapter } from "../../adapter";
import type { VacuumCommand, VacuumCommandResult } from "../../commands";
import { unsupportedCommand } from "../../errors";
import type { VacuumGoalCoordinates } from "../../state";
import { TURTLEBOT4_NAV2_UNSUPPORTED_COMMANDS } from "./capabilityMapper";
import { mapTurtleBot4Nav2State } from "./stateMapper";

function distanceBetween(
  a: { x: number; y: number } | null | undefined,
  b: { x: number; y: number } | null | undefined,
): number | null {
  if (!a || !b) {
    return null;
  }
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  if (!Number.isFinite(dx) || !Number.isFinite(dy)) {
    return null;
  }
  return Math.hypot(dx, dy);
}

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
      const current = runtimeRef.current;
      if (command.command === "go_to_location") {
        if (!snapshot.capabilities.go_to_location.supported) {
          return {
            ok: false,
            command: command.command,
            error: unsupportedCommand(
              command.command,
              "go_to_location is not supported yet: NavigateToPose send_goal service is not advertised.",
            ),
          };
        }
        if (!snapshot.readiness.ready) {
          return {
            ok: false,
            command: command.command,
            error: {
              code: "not_ready",
              command: command.command,
              message: `Adapter not ready to dispatch: ${snapshot.readiness.blockingReasons.join(" ")}`,
            },
          };
        }
        try {
          setCurrentTarget(command.target);
          setInitialDistance(distanceBetween(current.currentMapCoordinates, command.target));
          await current.sendGoal(command.target);
          return {
            ok: true,
            command: command.command,
            message: "NavigateToPose goal dispatched through the TurtleBot4/Nav2 adapter.",
          };
        } catch (error) {
          return {
            ok: false,
            command: command.command,
            error: {
              code: "backend_error",
              command: command.command,
              message: error instanceof Error ? error.message : String(error),
            },
          };
        }
      }

      if (command.command === "cancel_navigation") {
        if (!snapshot.capabilities.cancel_navigation.supported) {
          return {
            ok: false,
            command: command.command,
            error: unsupportedCommand(
              command.command,
              "cancel_navigation is not supported: NavigateToPose cancel_goal service is not advertised.",
            ),
          };
        }
        try {
          await current.cancelGoal();
          return {
            ok: true,
            command: command.command,
            message: "Cancel request dispatched through the TurtleBot4/Nav2 adapter.",
          };
        } catch (error) {
          return {
            ok: false,
            command: command.command,
            error: {
              code: "backend_error",
              command: command.command,
              message: error instanceof Error ? error.message : String(error),
            },
          };
        }
      }

      if (command.command === "manual_control") {
        return {
          ok: false,
          command: command.command,
          error: {
            code: "invalid_request",
            command: command.command,
            message:
              "manual_control is exposed through the TeleopCard publisher for /cmd_vel_raw and is not routed through sendCommand in the current adapter slice.",
          },
        };
      }

      if (TURTLEBOT4_NAV2_UNSUPPORTED_COMMANDS.includes(command.command)) {
        return {
          ok: false,
          command: command.command,
          error: unsupportedCommand(
            command.command,
            `Command ${command.command} is not supported by the TurtleBot4/Nav2 adapter.`,
          ),
        };
      }

      return {
        ok: false,
        command: command.command,
        error: unsupportedCommand(command.command),
      };
    },
    [snapshot.capabilities, snapshot.readiness],
  );

  return {
    snapshot,
    sendCommand,
  };
}
