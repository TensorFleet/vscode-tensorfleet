export type ValetudoRuntimeStatus = "online" | "degraded" | "offline";

export type ValetudoRuntimeSourceKind =
  | "fixed_mock"
  | "valetudo_mock"
  | "valetudo_http"
  | "real_robot"
  | "unknown";

export type ValetudoRuntimeSourceStatus = "reachable" | "unreachable" | "unknown";

export type ValetudoRuntimeCommandStatus = "success" | "unsupported" | "unavailable" | "failed";

export type ValetudoRuntimeCommandName =
  | "start_cleaning"
  | "pause"
  | "stop"
  | "return_to_dock"
  | "segment_cleaning"
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
  cleaningSettings?: {
    fanSpeed?: {
      current?: string;
      options: string[];
    };
    waterUsage?: {
      current?: string;
      options: string[];
    };
  };
  maintenance?: {
    consumables?: ValetudoRuntimeConsumable[];
  };
  statistics?: {
    current?: ValetudoRuntimeCurrentStatistics;
  };
  capabilities: {
    commands: Record<string, { available: boolean; reason?: string }>;
    diagnostics: ValetudoRuntimeCapabilityDiagnostic[];
  };
  diagnostics: {
    mode: string;
    rawCapabilityNames: string[];
    source?: ValetudoRuntimeSourceDiagnostic;
    readiness?: ValetudoRuntimeReadinessSummary;
    lastCommand?: ValetudoRuntimeCommandAudit;
    capabilityTiers?: ValetudoRuntimeCapabilityTier[];
    transports?: ValetudoRuntimeTransportDiagnostic[];
    notes?: string[];
  };
  rawDiagnostics?: Record<string, unknown>;
  updatedAt: number;
};

export type ValetudoRuntimeReadinessSummary = {
  runtimeOnline: boolean;
  sourceReachable: boolean;
  sourceStale: boolean;
  supportedCapabilities: string[];
  detectedNotProductReady: string[];
  basicCommandsAvailable: boolean;
  segmentTargetsAvailable: boolean;
  segmentTargetCount: number;
  missingProductRequirements?: string[];
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

export type ValetudoRuntimeConsumable = {
  id: string;
  label: string;
  remainingPercent?: number;
  remainingMinutes?: number;
  usedMinutes?: number;
  totalMinutes?: number;
  status?: "ok" | "warning" | "replace_soon" | "replace_now" | "unknown";
  detail?: string;
};

export type ValetudoRuntimeCurrentStatistics = {
  durationSeconds?: number;
  areaSquareMeters?: number;
  startedAt?: number | string;
  updatedAt?: number | string;
  detail?: string;
};

export type ValetudoRuntimeCapabilityDiagnostic = {
  name: string;
  detected: boolean;
  implemented: boolean;
  scope: string;
  note?: string;
};

export type ValetudoRuntimeTransportDiagnostic = {
  name: "http" | "mqtt" | string;
  enabled: boolean;
  status: string;
  stale: boolean;
  lastSeenAt?: number | null;
  lastSuccessAt?: number | null;
  staleReason?: string;
  detail?: string;
  messageCount?: number;
  lastError?: string | null;
  subscriptions?: string[];
};

export type ValetudoRuntimeSourceDiagnostic = {
  kind: ValetudoRuntimeSourceKind;
  status: ValetudoRuntimeSourceStatus;
  stale: boolean;
  staleReason?: string;
  lastPollAt?: number | null;
  lastSuccessfulUpdateAt?: number | null;
  lastError?: string | null;
  sourceUrl?: string;
  mqttBrokerUrl?: string;
  mqttTopicRoot?: string;
};

export type ValetudoRuntimeCommandAudit = {
  ok: boolean;
  status: ValetudoRuntimeCommandStatus;
  command: string;
  message: string;
  reason?: string;
  code?: string;
  updatedAt: number;
};

export type ValetudoRuntimeCapabilityTier = {
  tier: number;
  label: string;
  implemented: boolean;
  names: string[];
  note?: string;
};
