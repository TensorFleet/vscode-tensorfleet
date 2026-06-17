import type {
  VacuumAdapterSnapshot,
  VacuumBatteryState,
  VacuumDockStatus,
  VacuumFaultState,
  VacuumRobotActivity,
  VacuumRuntimeHealth,
  VacuumSourceState,
} from "./state";

export type VacuumPrimaryRobotState =
  | "offline"
  | "unavailable"
  | "idle"
  | "docked"
  | "charging"
  | "cleaning"
  | "paused"
  | "returning_to_dock"
  | "error";

export type VacuumPrimaryRobotStateSummary = {
  state: VacuumPrimaryRobotState;
  label: string;
  detail: string;
  reason?: string;
};

function hasFault(fault: VacuumFaultState | undefined, activity: VacuumRobotActivity | undefined, dock: VacuumDockStatus | undefined): boolean {
  return (
    fault?.readiness === "unavailable" ||
    activity?.status === "faulted" ||
    dock?.state === "error"
  );
}

function isSourceUnavailable(source: VacuumSourceState | undefined): boolean {
  return source?.reason === "source_unreachable" || source?.status === "unreachable";
}

function isSnapshotUnavailable(snapshot: Pick<VacuumAdapterSnapshot, "source" | "activity">): boolean {
  return (
    snapshot.activity?.status === "unavailable" ||
    snapshot.source?.stale === true ||
    snapshot.source?.status === "stale" ||
    isSourceUnavailable(snapshot.source)
  );
}

function primaryStateLabel(state: VacuumPrimaryRobotState): string {
  switch (state) {
    case "offline":
      return "Offline";
    case "unavailable":
      return "Unavailable";
    case "idle":
      return "Idle";
    case "docked":
      return "Docked";
    case "charging":
      return "Charging";
    case "cleaning":
      return "Cleaning";
    case "paused":
      return "Paused";
    case "returning_to_dock":
      return "Returning to dock";
    case "error":
      return "Needs attention";
  }
}

const UNKNOWN_BATTERY: VacuumBatteryState = {
  readiness: "waiting",
  percentage: null,
  charging: null,
  detail: "Battery state unavailable.",
};

function batteryText(battery: VacuumBatteryState | undefined): string {
  return battery?.percentage == null ? "battery unknown" : `${Math.round(battery.percentage)}% battery`;
}

function supportingDetail(
  state: VacuumPrimaryRobotState,
  battery: VacuumBatteryState | undefined,
  dock: VacuumDockStatus | undefined,
  health: VacuumRuntimeHealth | undefined,
  source: VacuumSourceState | undefined,
  activity: VacuumRobotActivity | undefined,
): string {
  if (state === "offline") {
    return health?.detail ?? "Adapter runtime is offline.";
  }
  if (state === "unavailable") {
    if (source?.stale) {
      return "Robot state is stale.";
    }
    if (isSourceUnavailable(source)) {
      return "Robot source is unreachable.";
    }
    return activity?.label ?? "Robot state is not available.";
  }
  if (state === "error") {
    return activity?.label ?? dock?.detail ?? "Robot reported a fault.";
  }
  if (state === "returning_to_dock") {
    return `Returning to dock with ${batteryText(battery)}.`;
  }
  if (state === "charging") {
    return `Charging at ${batteryText(battery)}.`;
  }
  if (state === "docked") {
    return `Docked with ${batteryText(battery)}.`;
  }
  if (state === "cleaning") {
    return `Cleaning with ${batteryText(battery)}.`;
  }
  if (state === "paused") {
    return `Paused with ${batteryText(battery)}.`;
  }
  const dockLabel = dock?.state === "undocked" ? "undocked" : dock?.state === "docked" ? "docked" : null;
  return [dockLabel, batteryText(battery)].filter(Boolean).join(", ") || "Robot is idle.";
}

export function deriveVacuumPrimaryRobotState(
  snapshot: Pick<VacuumAdapterSnapshot, "availability" | "health" | "source" | "activity" | "dock" | "fault"> & {
    battery?: VacuumBatteryState;
  },
): VacuumPrimaryRobotStateSummary {
  let state: VacuumPrimaryRobotState;
  const battery = snapshot.battery ?? UNKNOWN_BATTERY;

  if (snapshot.availability.status === "offline" || snapshot.health?.runtimeStatus === "offline") {
    state = "offline";
  } else if (hasFault(snapshot.fault, snapshot.activity, snapshot.dock)) {
    state = "error";
  } else if (isSnapshotUnavailable(snapshot)) {
    state = "unavailable";
  } else if (snapshot.activity?.status === "returning" || snapshot.dock?.state === "returning") {
    state = "returning_to_dock";
  } else if (snapshot.activity?.status === "paused") {
    state = "paused";
  } else if (snapshot.activity?.status === "cleaning" || snapshot.activity?.status === "covering") {
    state = "cleaning";
  } else if (snapshot.activity?.status === "charging" || snapshot.dock?.state === "charging" || battery.charging === true) {
    state = "charging";
  } else if (snapshot.activity?.status === "docked" || snapshot.dock?.state === "docked") {
    state = "docked";
  } else {
    state = "idle";
  }

  return {
    state,
    label: primaryStateLabel(state),
    detail: supportingDetail(state, battery, snapshot.dock, snapshot.health, snapshot.source, snapshot.activity),
    reason: snapshot.source?.reason ?? snapshot.activity?.reason,
  };
}
