import type { Nav2RuntimeState } from "../../../components/Nav2/runtime/nav2RuntimeTypes";
import type { VacuumAdapterSnapshot } from "../../state";
import type { VacuumCommand, VacuumCommandResult } from "../../commands";
import { unsupportedCommand } from "../../errors";
import type { VacuumGoalCoordinates } from "../../state";
import { TURTLEBOT4_NAV2_UNSUPPORTED_COMMANDS } from "./capabilityMapper";

export type TurtleBot4Nav2CommandRuntime = Pick<
  Nav2RuntimeState,
  "currentMapCoordinates" | "sendGoal" | "cancelGoal"
>;

export type TurtleBot4Nav2CommandDispatchContext = {
  runtime: TurtleBot4Nav2CommandRuntime;
  snapshot: Pick<VacuumAdapterSnapshot, "capabilities" | "readiness">;
  setCurrentTarget: (target: VacuumGoalCoordinates | null) => void;
  setInitialDistance: (distance: number | null) => void;
};

export function distanceBetween(
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

export async function dispatchTurtleBot4Nav2Command(
  command: VacuumCommand,
  context: TurtleBot4Nav2CommandDispatchContext,
): Promise<VacuumCommandResult> {
  const { runtime, snapshot } = context;

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
      context.setCurrentTarget(command.target);
      context.setInitialDistance(distanceBetween(runtime.currentMapCoordinates, command.target));
      await runtime.sendGoal(command.target);
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
      await runtime.cancelGoal();
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
}
