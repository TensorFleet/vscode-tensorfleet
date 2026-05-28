export type ValetudoRuntimeStatus = "online" | "degraded" | "offline";

export type ValetudoRuntimeSourceKind = "fixed_mock" | "valetudo_mock" | "real_robot";

export type ValetudoRuntimeSourceStatus = "reachable" | "unreachable" | "unknown";

export type ValetudoRuntimeCommandStatus = "success" | "unsupported" | "unavailable" | "failed";

export type ValetudoRuntimeCommandName =
  | "start_cleaning"
  | "pause"
  | "stop"
  | "return_to_dock"
  | string;

export type ValetudoRuntimeHealth = {
  runtime: {
    id: string;
    version: string;
    status: ValetudoRuntimeStatus;
  };
  source: {
    kind: ValetudoRuntimeSourceKind;
    status: ValetudoRuntimeSourceStatus;
    stale: boolean;
    lastSeenAt: number | null;
  };
  updatedAt: number;
};

export type ValetudoRuntimeSnapshot = {
  runtime: ValetudoRuntimeHealth["runtime"];
  backend: "valetudo";
  robot: {
    id: string;
    name: string;
  };
  source: ValetudoRuntimeHealth["source"];
  connectivity: {
    reachable: boolean;
    online: boolean;
  };
  state: {
    value: string;
    label: string;
    started: boolean;
    paused: boolean;
  };
  battery?: {
    level: number;
    charging: boolean;
  };
  dock?: {
    state: string;
    docked: boolean;
  };
  capabilities: {
    commands: Record<string, { available: boolean; reason?: string }>;
    diagnostics: ValetudoRuntimeCapabilityDiagnostic[];
  };
  diagnostics: {
    mode: string;
    rawCapabilityNames: string[];
    notes?: string[];
  };
  rawDiagnostics?: Record<string, unknown>;
  updatedAt: number;
};

export type ValetudoRuntimeCommandRequest = {
  command: ValetudoRuntimeCommandName;
  params?: Record<string, unknown>;
};

export type ValetudoRuntimeCommandResult = {
  ok: boolean;
  status: ValetudoRuntimeCommandStatus;
  command: string;
  message: string;
  reason?: string;
  code?: string;
  updatedAt: number;
  diagnostics?: Record<string, unknown>;
};

export type ValetudoRuntimeCapabilityDiagnostic = {
  name: string;
  detected: boolean;
  implemented: boolean;
  scope: string;
  note?: string;
};
