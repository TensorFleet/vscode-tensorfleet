// simulation_controller.ts
/**
 * High-level simulation controller:
 *  - Receives simulation-control intents from the app layer
 *  - Uses ros2Bridge to call a simulator service if available
 */

import { ros2Bridge } from "@/ros2-bridge";

export class SimulationController {
  /**
   * Restart the simulator. Placeholder implementation:
   *  - If Foxglove services are available, call "/sim/restart".
   *  - Otherwise, log and resolve.
   */
  async restart(): Promise<void> {
    // Guard: attempt service call when connected and callService exists
    try {
      const canCall =
        typeof (ros2Bridge as any).callService === "function" &&
        ros2Bridge.isConnected();

      if (canCall) {
        await (ros2Bridge as any).callService("/simulation_manager/start_simulation", {});
        return;
      }

      console.log("[SimulationController] restart requested (placeholder handled)");
    } catch (err) {
      // Surface a clear error to the caller/bridge
      throw (err instanceof Error ? err : new Error(String(err)));
    }
  }
}
