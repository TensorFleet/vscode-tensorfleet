// simulation_controller.ts
/**
 * High-level simulation controller:
 *  - Receives simulation-control intents from the app layer
 *  - Uses ros2Bridge to call a simulator service if available
 */

import { ros2Bridge } from "tensorfleet-ros";

export class SimulationController {
  /**
   * Restart the simulator. Placeholder implementation:
   *  - If Foxglove services are available, call "/sim/restart".
   *  - Otherwise, log and resolve.
   */
  async restart(): Promise<void> {
    // Guard: attempt service call when connected and callService exists
    try {
      const restartService = "/simulation_manager/start_simulation";
      const canCall =
        typeof (ros2Bridge as any).callService === "function" &&
        ros2Bridge.isConnected();

      if (canCall) {
        if (typeof (ros2Bridge as any).waitForService === "function") {
          await (ros2Bridge as any).waitForService(restartService, 3_000);
        }
        await (ros2Bridge as any).callService(restartService, {});
        return;
      }

      console.log("[SimulationController] restart requested (placeholder handled)");
    } catch (err) {
      if (
        err instanceof Error &&
        (err.message.includes("not advertised") || err.message.includes("Timeout waiting for service"))
      ) {
        throw new Error("Simulation manager service is unavailable on this VM.");
      }
      // Surface a clear error to the caller/bridge
      throw (err instanceof Error ? err : new Error(String(err)));
    }
  }
}
