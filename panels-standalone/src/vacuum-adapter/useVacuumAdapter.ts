import { useRef } from "react";
import type { VacuumAdapter } from "./adapter";
import { useTurtleBot4Nav2Adapter } from "./backends/turtlebot4-nav2/useTurtleBot4Nav2Adapter";
import { useValetudoAdapter } from "./backends/valetudo/useValetudoAdapter";

export type VacuumAdapterBackendId = "turtlebot4_nav2" | "valetudo";

export const VACUUM_ADAPTER_BACKENDS = {
  simulation: "turtlebot4_nav2",
  valetudoRuntime: "valetudo",
} as const satisfies Record<string, VacuumAdapterBackendId>;

export type UseVacuumAdapterOptions = {
  backend?: VacuumAdapterBackendId;
};

export function normalizeVacuumAdapterBackend(value: unknown): VacuumAdapterBackendId | null {
  return value === VACUUM_ADAPTER_BACKENDS.valetudoRuntime || value === VACUUM_ADAPTER_BACKENDS.simulation
    ? value
    : null;
}

export function readConfiguredVacuumAdapterBackend(): VacuumAdapterBackendId {
  if (typeof window !== "undefined") {
    const globalBackend = (window as unknown as { TENSORFLEET_VACUUM_BACKEND?: string }).TENSORFLEET_VACUUM_BACKEND;
    const queryBackend = new URLSearchParams(window.location.search).get("vacuumBackend");
    const storedBackend = window.localStorage.getItem("tensorfleet:vacuums:adapter-backend");
    const backend = normalizeVacuumAdapterBackend(globalBackend ?? queryBackend ?? storedBackend);
    if (backend) {
      return backend;
    }
  }
  return VACUUM_ADAPTER_BACKENDS.simulation;
}

export function useVacuumAdapter(options: UseVacuumAdapterOptions = {}): VacuumAdapter {
  // The backend is chosen once per mount. To switch backends, callers must remount
  // (VacuumControlPanel does this via `key={backend}`). We pin the backend on first
  // render so the conditional hook call below keeps a constant call order for the
  // lifetime of this component instance, satisfying the Rules of Hooks even if a
  // caller passes a changing `backend` prop without remounting.
  const backendRef = useRef<VacuumAdapterBackendId | null>(null);
  if (backendRef.current === null) {
    backendRef.current = options.backend ?? readConfiguredVacuumAdapterBackend();
  }
  const backend = backendRef.current;

  // Exactly one of these adapter hooks runs for this component instance, and the
  // pinned backend guarantees the choice never changes between renders.
  if (backend === VACUUM_ADAPTER_BACKENDS.valetudoRuntime) {
    // eslint-disable-next-line react-hooks/rules-of-hooks
    return useValetudoAdapter();
  }
  // eslint-disable-next-line react-hooks/rules-of-hooks
  return useTurtleBot4Nav2Adapter();
}
