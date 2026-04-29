import type { VacuumAdapter } from "./adapter";
import { useTurtleBot4Nav2Adapter } from "./backends/turtlebot4-nav2/useTurtleBot4Nav2Adapter";

export type VacuumAdapterBackendId = "turtlebot4_nav2" | "valetudo";

export type UseVacuumAdapterOptions = {
  backend?: VacuumAdapterBackendId;
};

export function useVacuumAdapter(options: UseVacuumAdapterOptions = {}): VacuumAdapter {
  const backend = options.backend ?? "turtlebot4_nav2";
  if (backend !== "turtlebot4_nav2") {
    throw new Error(
      `Vacuum adapter backend "${backend}" is not available yet. Only "turtlebot4_nav2" is implemented in this slice.`,
    );
  }
  return useTurtleBot4Nav2Adapter();
}
