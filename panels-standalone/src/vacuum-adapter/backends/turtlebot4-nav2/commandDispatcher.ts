import type { Nav2RuntimeState } from "../../../components/Nav2/runtime/nav2RuntimeTypes";
import type { VacuumAdapterSnapshot } from "../../state";
import type { VacuumCommand, VacuumCommandResult } from "../../commands";
import { unsupportedCommand } from "../../errors";
import type { VacuumGoalCoordinates } from "../../state";
import { MAPPING_SERVICE_NAMES, MISSION_SERVICE_NAMES, TURTLEBOT4_NAV2_UNSUPPORTED_COMMANDS } from "./capabilityMapper";

export type TurtleBot4Nav2CommandRuntime = Pick<
  Nav2RuntimeState,
  "currentMapCoordinates" | "sendGoal" | "cancelGoal"
> & {
  callService?: (name: string, request: Record<string, unknown>, opts?: { timeoutMs?: number }) => Promise<Record<string, unknown> | null>;
};

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

  async function callMappingService(
    serviceName: string,
    commandName: VacuumCommand["command"],
  ): Promise<VacuumCommandResult> {
    if (!runtime.callService) {
      return {
        ok: false,
        command: commandName,
        error: unsupportedCommand(commandName, "Mapping service calls are not available in this runtime."),
      };
    }
    try {
      const response = await runtime.callService(serviceName, {}, { timeoutMs: 10_000 });
      const success = response == null || response.success !== false;
      if (!success) {
        return {
          ok: false,
          command: commandName,
          error: {
            code: "backend_error",
            command: commandName,
            message: typeof response?.message === "string" ? response.message : `${serviceName} returned failure.`,
          },
        };
      }
      return {
        ok: true,
        command: commandName,
        message: typeof response?.message === "string" ? response.message : `Dispatched ${commandName}.`,
      };
    } catch (error) {
      return {
        ok: false,
        command: commandName,
        error: {
          code: "backend_error",
          command: commandName,
          message: error instanceof Error ? error.message : String(error),
        },
      };
    }
  }

  async function callTriggerService(
    serviceName: string,
    commandName: VacuumCommand["command"],
    unavailableMessage: string,
  ): Promise<VacuumCommandResult> {
    if (!runtime.callService) {
      return {
        ok: false,
        command: commandName,
        error: unsupportedCommand(commandName, unavailableMessage),
      };
    }
    try {
      const response = await runtime.callService(serviceName, {}, { timeoutMs: 10_000 });
      const success = response == null || response.success !== false;
      if (!success) {
        return {
          ok: false,
          command: commandName,
          error: {
            code: "backend_error",
            command: commandName,
            message: typeof response?.message === "string" ? response.message : `${serviceName} returned failure.`,
          },
        };
      }
      return {
        ok: true,
        command: commandName,
        message: typeof response?.message === "string" ? response.message : `Dispatched ${commandName}.`,
      };
    } catch (error) {
      return {
        ok: false,
        command: commandName,
        error: {
          code: "backend_error",
          command: commandName,
          message: error instanceof Error ? error.message : String(error),
        },
      };
    }
  }

  async function setMapBasename(name: string | undefined): Promise<VacuumCommandResult | null> {
    const trimmed = name?.trim();
    if (!trimmed) {
      return null;
    }
    if (!runtime.callService) {
      return {
        ok: false,
        command: command.command,
        error: unsupportedCommand(command.command, "Map naming requires parameter service calls in this runtime."),
      };
    }
    try {
      const response = await runtime.callService(
        "/vacuum_frontier_explorer/set_parameters",
        {
          parameters: [
            {
              name: "map_basename",
              value: {
                type: 4,
                string_value: trimmed,
              },
            },
          ],
        },
        { timeoutMs: 5_000 },
      );
      const results = Array.isArray(response?.results) ? response.results : [];
      const failed = results.find((entry) => entry && typeof entry === "object" && (entry as { successful?: boolean }).successful === false);
      if (failed) {
        return {
          ok: false,
          command: command.command,
          error: {
            code: "backend_error",
            command: command.command,
            message:
              typeof (failed as { reason?: unknown }).reason === "string"
                ? (failed as { reason: string }).reason
                : "Map name parameter update failed.",
          },
        };
      }
      return null;
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

  async function setNavigationRequest(target: VacuumGoalCoordinates): Promise<VacuumCommandResult | null> {
    if (!runtime.callService) {
      return {
        ok: false,
        command: "start_navigation",
        error: unsupportedCommand("start_navigation", "Navigation mission service calls are not available in this runtime."),
      };
    }
    try {
      const response = await runtime.callService(
        MISSION_SERVICE_NAMES.setParameters,
        {
          parameters: [
            {
              name: "navigation_request",
              value: {
                type: 4,
                string_value: JSON.stringify({ target }),
              },
            },
          ],
        },
        { timeoutMs: 5_000 },
      );
      const results = Array.isArray(response?.results) ? response.results : [];
      const failed = results.find((entry) => entry && typeof entry === "object" && (entry as { successful?: boolean }).successful === false);
      if (failed) {
        return {
          ok: false,
          command: "start_navigation",
          error: {
            code: "backend_error",
            command: "start_navigation",
            message:
              typeof (failed as { reason?: unknown }).reason === "string"
                ? (failed as { reason: string }).reason
                : "Navigation request parameter update failed.",
          },
        };
      }
      return null;
    } catch (error) {
      return {
        ok: false,
        command: "start_navigation",
        error: {
          code: "backend_error",
          command: "start_navigation",
          message: error instanceof Error ? error.message : String(error),
        },
      };
    }
  }

  if (command.command === "start_navigation") {
    if (!snapshot.capabilities.start_navigation.supported) {
      return {
        ok: false,
        command: command.command,
        error: unsupportedCommand(command.command, "start_navigation requires the VM navigation mission runtime."),
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
    context.setCurrentTarget(command.target);
    context.setInitialDistance(distanceBetween(runtime.currentMapCoordinates, command.target));
    const setRequestError = await setNavigationRequest(command.target);
    if (setRequestError) {
      return setRequestError;
    }
    return await callTriggerService(
      MISSION_SERVICE_NAMES.startNavigation,
      command.command,
      "Navigation mission service calls are not available in this runtime.",
    );
  }

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

  if (command.command === "cancel_mission") {
    if (!snapshot.capabilities.cancel_mission.supported) {
      return {
        ok: false,
        command: command.command,
        error: unsupportedCommand(command.command, "cancel_mission requires the VM mission runtime."),
      };
    }
    return await callTriggerService(
      MISSION_SERVICE_NAMES.cancel,
      command.command,
      "Mission cancel service calls are not available in this runtime.",
    );
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

  if (command.command === "start_mapping") {
    const setNameError = await setMapBasename(command.name);
    if (setNameError) {
      return setNameError;
    }
    if (command.mode === "auto" && !snapshot.capabilities.auto_mapping.supported) {
      return {
        ok: false,
        command: command.command,
        error: unsupportedCommand(command.command, "auto_mapping is not supported by the current backend."),
      };
    }
    if (command.mode === "manual" && !snapshot.capabilities.mapping_session.supported) {
      return {
        ok: false,
        command: command.command,
        error: unsupportedCommand(command.command, "mapping_session is not supported by the current backend."),
      };
    }
    return await callMappingService(
      command.mode === "auto" ? MAPPING_SERVICE_NAMES.startAuto : MAPPING_SERVICE_NAMES.startManual,
      command.command,
    );
  }

  if (
    command.command === "pause_mapping" ||
    command.command === "resume_mapping" ||
    command.command === "finish_mapping" ||
    command.command === "discard_mapping" ||
    command.command === "accept_map" ||
    command.command === "load_map"
  ) {
    if (!snapshot.capabilities.mapping_session.supported) {
      return {
        ok: false,
        command: command.command,
        error: unsupportedCommand(command.command, "mapping_session is not supported by the current backend."),
      };
    }
    if (command.command === "accept_map") {
      const setNameError = await setMapBasename(command.name);
      if (setNameError) {
        return setNameError;
      }
    }
    if (command.command === "load_map") {
      const setNameError = await setMapBasename(command.name);
      if (setNameError) {
        return setNameError;
      }
    }
    const serviceByCommand = {
      pause_mapping: MAPPING_SERVICE_NAMES.pause,
      resume_mapping: MAPPING_SERVICE_NAMES.resume,
      finish_mapping: MAPPING_SERVICE_NAMES.finish,
      discard_mapping: MAPPING_SERVICE_NAMES.discard,
      accept_map: MAPPING_SERVICE_NAMES.accept,
      load_map: MAPPING_SERVICE_NAMES.loadMap,
    } as const;
    return await callMappingService(serviceByCommand[command.command], command.command);
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
