import type {
  VacuumAdapterDiagnostics,
  VacuumAttachmentKind,
  VacuumAttachmentState,
  VacuumAttachmentStatus,
  VacuumAttachmentsState,
  VacuumCleaningSettingOption,
  VacuumCleaningSettingsState,
  VacuumConsumableState,
  VacuumDockComponentKind,
  VacuumDockComponentState,
  VacuumDockComponentStatus,
  VacuumAdapterSnapshot,
  VacuumDockState,
  VacuumLayeredMapPreview,
  VacuumLayeredMapMetadata,
  VacuumMapEntity,
  VacuumMapEntityKind,
  VacuumMapLayer,
  VacuumMapLayerKind,
  VacuumMapRun,
  VacuumMaintenanceState,
  VacuumMapTarget,
  VacuumMapTargetGeometry,
  VacuumMapTargets,
  VacuumMissionSnapshot,
  VacuumMissionState,
  VacuumNavigationStatus,
  VacuumRobotActivity,
  VacuumRobotActivityAction,
  VacuumRobotActivityStatus,
  VacuumRuntimeHealth,
  VacuumSourceState,
  VacuumStatisticsState,
} from "../../state";
import type { VacuumCapabilities } from "../../capabilities";
import { buildVacuumMapMetadata } from "../../mapGrid";
import type { ValetudoBackendCapability } from "./capabilityMapper";
import { mapValetudoCapabilities } from "./capabilityMapper";
import type { ValetudoRuntimeMapEntity, ValetudoRuntimeMapLayer, ValetudoRuntimeMapTarget, ValetudoRuntimeSnapshot } from "./runtimeContract";
import type { ValetudoRuntimeBoundary } from "./types";

const EMPTY_NAVIGATION: VacuumNavigationStatus = {
  state: "idle",
  backendGoalState: null,
  active: false,
  isSending: false,
  isCanceling: false,
  currentTarget: null,
  terminalState: null,
  planPath: null,
  progress: {
    distanceRemaining: null,
    initialDistance: null,
    recoveries: null,
    navigationTime: null,
    estimatedTimeRemaining: null,
  },
  detail: "Valetudo navigation/go-to is unsupported in the current product contract.",
};

const RUNTIME_COMMAND_TO_BACKEND_CAPABILITY: Record<string, ValetudoBackendCapability> = {
  start_cleaning: "BasicControlCapability",
  pause: "BasicControlCapability",
  resume: "BasicControlCapability",
  stop: "BasicControlCapability",
  return_to_dock: "BasicControlCapability",
  set_fan_speed: "FanSpeedControlCapability",
  set_water_usage: "WaterUsageControlCapability",
};

const VALETUDO_BASIC_COMMANDS = ["start_cleaning", "pause", "resume", "stop", "return_to_dock"] as const;

type ValetudoBasicCommandName = (typeof VALETUDO_BASIC_COMMANDS)[number];

const KNOWN_VALETUDO_BACKEND_CAPABILITIES = new Set<ValetudoBackendCapability>([
  "BasicControlCapability",
  "BatteryStateCapability",
  "ConsumableMonitoringCapability",
  "FanSpeedControlCapability",
  "GoToLocationCapability",
  "MapSegmentationCapability",
  "WaterUsageControlCapability",
  "ZoneCleaningCapability",
]);

function isKnownValetudoBackendCapability(value: string): value is ValetudoBackendCapability {
  return KNOWN_VALETUDO_BACKEND_CAPABILITIES.has(value as ValetudoBackendCapability);
}

function isDiagnosticOnlyCapability(value: ValetudoBackendCapability): boolean {
  return (
    value !== "BasicControlCapability" &&
    value !== "BatteryStateCapability" &&
    value !== "FanSpeedControlCapability" &&
    value !== "WaterUsageControlCapability"
  );
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return value != null && typeof value === "object";
}

function isStringArray(value: unknown): value is string[] {
  return Array.isArray(value) && value.every((item) => typeof item === "string");
}

function normalizeSourceStatus(snapshot: ValetudoRuntimeSnapshot): VacuumSourceState["status"] {
  if (snapshot.source.stale) {
    return "stale";
  }
  if (snapshot.source.status === "reachable") {
    return "reachable";
  }
  if (snapshot.source.status === "unreachable") {
    return "unreachable";
  }
  return "unknown";
}

function normalizeSourceKind(kind: string): VacuumSourceState["kind"] {
  if (kind === "fixed_mock") {
    return "fixed_mock";
  }
  if (kind === "valetudo_mock") {
    return "valetudo_mock";
  }
  if (kind === "valetudo_http") {
    return "valetudo_http";
  }
  if (kind === "real_robot") {
    return "real_robot";
  }
  return "unknown";
}

function sourceUnavailableReason(snapshot: ValetudoRuntimeSnapshot): string | undefined {
  if (snapshot.source.stale) {
    return "stale_source";
  }
  if (!snapshot.connectivity.online) {
    return "runtime_offline";
  }
  if (!snapshot.connectivity.reachable || snapshot.source.status === "unreachable") {
    return "source_unreachable";
  }
  if (snapshot.runtime.status === "degraded") {
    return "degraded_runtime";
  }
  return undefined;
}

function isBasicCommandName(value: string): value is ValetudoBasicCommandName {
  return (VALETUDO_BASIC_COMMANDS as readonly string[]).includes(value);
}

function normalizeStateValue(snapshot: ValetudoRuntimeSnapshot): string {
  return snapshot.state.value.toLowerCase();
}

function hasStateToken(snapshot: ValetudoRuntimeSnapshot, token: string): boolean {
  return normalizeStateValue(snapshot).includes(token);
}

function isRobotCleaning(snapshot: ValetudoRuntimeSnapshot): boolean {
  const value = normalizeStateValue(snapshot);
  return (snapshot.state.started && !snapshot.state.paused) || value.includes("clean");
}

function isRobotPaused(snapshot: ValetudoRuntimeSnapshot): boolean {
  return snapshot.state.paused || hasStateToken(snapshot, "pause");
}

function isRobotReturningToDock(snapshot: ValetudoRuntimeSnapshot): boolean {
  return hasStateToken(snapshot, "return") || mapDockState(snapshot) === "returning";
}

function isRobotDockedOrCharging(snapshot: ValetudoRuntimeSnapshot): boolean {
  const dockState = mapDockState(snapshot);
  return dockState === "docked" || dockState === "charging" || snapshot.battery?.charging === true;
}

function stateUnavailableReason(
  snapshot: ValetudoRuntimeSnapshot,
  command: ValetudoBasicCommandName,
): string | undefined {
  if (hasStateToken(snapshot, "fault") || hasStateToken(snapshot, "error")) {
    return "invalid_state";
  }
  if (command === "start_cleaning") {
    if (isRobotCleaning(snapshot) || isRobotPaused(snapshot) || isRobotReturningToDock(snapshot)) {
      return "invalid_state";
    }
    return undefined;
  }
  if (command === "pause") {
    return isRobotCleaning(snapshot) && !isRobotPaused(snapshot) && !isRobotReturningToDock(snapshot)
      ? undefined
      : "invalid_state";
  }
  if (command === "resume") {
    return isRobotPaused(snapshot) ? undefined : "invalid_state";
  }
  if (command === "stop") {
    return isRobotCleaning(snapshot) || isRobotPaused(snapshot) || isRobotReturningToDock(snapshot)
      ? undefined
      : "invalid_state";
  }
  if (command === "return_to_dock") {
    return isRobotDockedOrCharging(snapshot) || isRobotReturningToDock(snapshot) ? "invalid_state" : undefined;
  }
  return undefined;
}

function commandUnavailableReason(
  snapshot: ValetudoRuntimeSnapshot,
  command: ValetudoBasicCommandName,
  commandAvailable: boolean,
  commandReason: string | undefined,
): string | undefined {
  const sourceReason = sourceUnavailableReason(snapshot);
  if (sourceReason) {
    return sourceReason;
  }
  if (!commandAvailable) {
    return commandReason ?? "unavailable";
  }
  return stateUnavailableReason(snapshot, command);
}

function mapDockState(snapshot: ValetudoRuntimeSnapshot): VacuumDockState {
  if (snapshot.battery?.charging) {
    return "charging";
  }
  const state = snapshot.dock?.state.toLowerCase() ?? "";
  if (state.includes("return")) {
    return "returning";
  }
  if (state.includes("error") || state.includes("fault")) {
    return "error";
  }
  if (snapshot.dock?.docked) {
    return "docked";
  }
  if (snapshot.dock) {
    return "undocked";
  }
  return "unknown";
}

function mapHealth(snapshot: ValetudoRuntimeSnapshot): VacuumRuntimeHealth {
  return {
    runtimeStatus: snapshot.runtime.status,
    updatedAt: snapshot.updatedAt,
    detail:
      snapshot.runtime.status === "online"
        ? "Valetudo integration runtime is online."
        : snapshot.runtime.status === "degraded"
          ? "Valetudo integration runtime is degraded."
          : "Valetudo integration runtime is offline.",
  };
}

function mapDiagnostics(snapshot: ValetudoRuntimeSnapshot): VacuumAdapterDiagnostics {
  const warnings = [
    ...(snapshot.diagnostics.notes ?? []),
    ...(snapshot.source.stale ? ["Valetudo runtime source state is stale."] : []),
  ];
  return {
    backend: "valetudo",
    runtime: snapshot.runtime,
    source: snapshot.diagnostics.source ?? snapshot.source,
    capabilities: {
      detected: snapshot.capabilities.diagnostics,
      rawCapabilityNames: snapshot.diagnostics.rawCapabilityNames,
    },
    map: {
      supported: false,
      reason: "Valetudo map data is diagnostics-only until product map rendering is implemented.",
    },
    pose: {
      supported: false,
      reason: "Valetudo pose data is not exposed as a product navigation surface in this adapter slice.",
    },
    navigation: {
      supported: false,
      reason: "Valetudo go-to/navigation data is diagnostics-only until the normalized workflow is implemented.",
    },
    mapping: {
      supported: false,
      reason: "Valetudo mapping sessions are unsupported by this adapter slice.",
    },
    warnings,
    raw: {
      valetudoState: snapshot.state.value,
      rawCapabilityNames: snapshot.diagnostics.rawCapabilityNames,
      readiness: snapshot.diagnostics.readiness,
      capabilityTiers: snapshot.diagnostics.capabilityTiers,
      transports: snapshot.diagnostics.transports,
      lastCommand: snapshot.diagnostics.lastCommand,
      rawDiagnostics: snapshot.rawDiagnostics,
    },
  };
}

function titleCasePreset(value: string): string {
  return value
    .split(/[_\s-]+/)
    .filter(Boolean)
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(" ");
}

function mapPresetOptions(options: string[] | undefined): VacuumCleaningSettingOption[] {
  return (options ?? []).map((value) => ({
    value,
    label: titleCasePreset(value),
  }));
}

function humanizeAvailabilityReason(reason: string): string {
  const knownReasons: Record<string, string> = {
    degraded_runtime: "Runtime degraded.",
    runtime_offline: "Runtime offline.",
    source_unreachable: "Source unreachable.",
    stale_source: "Robot state is stale.",
    unavailable: "Currently unavailable.",
  };
  return knownReasons[reason] ?? titleCasePreset(reason);
}

function mapCleaningSettings(snapshot: ValetudoRuntimeSnapshot): VacuumCleaningSettingsState | undefined {
  const sourceReason = sourceUnavailableReason(snapshot);
  const settings: VacuumCleaningSettingsState = {};
  const fanSpeed = snapshot.cleaningSettings?.fanSpeed;
  const waterUsage = snapshot.cleaningSettings?.waterUsage;
  if (fanSpeed && fanSpeed.options.length > 0) {
    settings.fanSpeed = {
      current: fanSpeed.current || undefined,
      options: mapPresetOptions(fanSpeed.options),
      readiness: sourceReason ? "unavailable" : "ready",
      status: sourceReason ?? "ready",
      detail: sourceReason ? humanizeAvailabilityReason(sourceReason) : "Fan speed is available.",
    };
  }
  if (waterUsage && waterUsage.options.length > 0) {
    settings.waterUsage = {
      current: waterUsage.current || undefined,
      options: mapPresetOptions(waterUsage.options),
      readiness: sourceReason ? "unavailable" : "ready",
      status: sourceReason ?? "ready",
      detail: sourceReason ? humanizeAvailabilityReason(sourceReason) : "Water usage is available.",
    };
  }
  return settings.fanSpeed || settings.waterUsage ? settings : undefined;
}

function mapMaintenance(snapshot: ValetudoRuntimeSnapshot): VacuumMaintenanceState | undefined {
  const consumables = (snapshot.maintenance?.consumables ?? [])
    .filter((item): item is VacuumConsumableState => (
      typeof item.id === "string" &&
      item.id.trim() !== "" &&
      typeof item.label === "string" &&
      item.label.trim() !== ""
    ))
    .map((item) => ({
      id: item.id,
      label: item.label,
      remainingPercent: typeof item.remainingPercent === "number" ? item.remainingPercent : undefined,
      remainingMinutes: typeof item.remainingMinutes === "number" ? item.remainingMinutes : undefined,
      usedMinutes: typeof item.usedMinutes === "number" ? item.usedMinutes : undefined,
      totalMinutes: typeof item.totalMinutes === "number" ? item.totalMinutes : undefined,
      status: item.status,
      detail: item.detail,
    }));
  return consumables.length > 0 ? { consumables } : undefined;
}

function mapStatistics(snapshot: ValetudoRuntimeSnapshot): VacuumStatisticsState | undefined {
  if (sourceUnavailableReason(snapshot)) {
    return undefined;
  }
  const current = snapshot.statistics?.current;
  if (!current) {
    return undefined;
  }
  const durationSeconds = typeof current.durationSeconds === "number" && Number.isFinite(current.durationSeconds)
    ? current.durationSeconds
    : undefined;
  const areaSquareMeters = typeof current.areaSquareMeters === "number" && Number.isFinite(current.areaSquareMeters)
    ? current.areaSquareMeters
    : undefined;
  const startedAt = typeof current.startedAt === "number" || typeof current.startedAt === "string"
    ? current.startedAt
    : undefined;
  const updatedAt = typeof current.updatedAt === "number" || typeof current.updatedAt === "string"
    ? current.updatedAt
    : undefined;
  const detail = typeof current.detail === "string" && current.detail.trim() !== ""
    ? current.detail
    : undefined;
  if (
    durationSeconds == null &&
    areaSquareMeters == null &&
    startedAt == null &&
    updatedAt == null &&
    detail == null
  ) {
    return undefined;
  }
  return {
    current: {
      durationSeconds,
      areaSquareMeters,
      startedAt,
      updatedAt,
      detail,
    },
  };
}

const ATTACHMENT_KINDS = new Set<VacuumAttachmentKind>(["dustbin", "water_tank", "mop", "detergent", "unknown"]);
const ATTACHMENT_STATUSES = new Set<VacuumAttachmentStatus>([
  "installed",
  "missing",
  "full",
  "empty",
  "low",
  "ok",
  "unknown",
  "error",
]);
const DOCK_COMPONENT_KINDS = new Set<VacuumDockComponentKind>([
  "freshwater",
  "wastewater",
  "detergent",
  "dustbag",
  "unknown",
]);
const DOCK_COMPONENT_STATUSES = new Set<VacuumDockComponentStatus>([
  "ok",
  "missing",
  "full",
  "empty",
  "low",
  "unknown",
  "error",
]);

function normalizePercent(value: unknown): number | undefined {
  if (typeof value !== "number" || !Number.isFinite(value) || value < 0 || value > 100) {
    return undefined;
  }
  return value;
}

function normalizeTimestamp(value: unknown): number | string | undefined {
  return typeof value === "number" || typeof value === "string" ? value : undefined;
}

function normalizeAttachmentKind(value: unknown): VacuumAttachmentKind {
  return ATTACHMENT_KINDS.has(value as VacuumAttachmentKind) ? (value as VacuumAttachmentKind) : "unknown";
}

function normalizeAttachmentStatus(value: unknown): VacuumAttachmentStatus {
  return ATTACHMENT_STATUSES.has(value as VacuumAttachmentStatus) ? (value as VacuumAttachmentStatus) : "unknown";
}

function normalizeDockComponentKind(value: unknown): VacuumDockComponentKind {
  return DOCK_COMPONENT_KINDS.has(value as VacuumDockComponentKind) ? (value as VacuumDockComponentKind) : "unknown";
}

function normalizeDockComponentStatus(value: unknown): VacuumDockComponentStatus {
  return DOCK_COMPONENT_STATUSES.has(value as VacuumDockComponentStatus) ? (value as VacuumDockComponentStatus) : "unknown";
}

function hasMeaningfulIdentity(item: unknown): item is Record<string, unknown> {
  return (
    isRecord(item) &&
    typeof item.id === "string" &&
    item.id.trim() !== "" &&
    typeof item.label === "string" &&
    item.label.trim() !== ""
  );
}

function mapAttachments(snapshot: ValetudoRuntimeSnapshot): VacuumAttachmentsState | undefined {
  if (sourceUnavailableReason(snapshot)) {
    return undefined;
  }
  const seen = new Set<string>();
  const items = (snapshot.attachments?.items ?? [])
    .filter(hasMeaningfulIdentity)
    .map((item): VacuumAttachmentState => ({
      id: item.id.trim(),
      label: item.label.trim(),
      kind: normalizeAttachmentKind(item.kind),
      status: normalizeAttachmentStatus(item.status),
      available: typeof item.available === "boolean" ? item.available : undefined,
      levelPercent: normalizePercent(item.levelPercent),
      detail: typeof item.detail === "string" && item.detail.trim() !== "" ? item.detail : undefined,
      updatedAt: normalizeTimestamp(item.updatedAt),
    }))
    .filter((item) => {
      if (seen.has(item.id)) {
        return false;
      }
      seen.add(item.id);
      return true;
    });
  return items.length > 0 ? { items } : undefined;
}

function mapDockComponents(snapshot: ValetudoRuntimeSnapshot): VacuumDockComponentState[] | undefined {
  if (sourceUnavailableReason(snapshot)) {
    return undefined;
  }
  const seen = new Set<string>();
  const components = (snapshot.dock?.components ?? [])
    .filter(hasMeaningfulIdentity)
    .map((item): VacuumDockComponentState => ({
      id: item.id.trim(),
      label: item.label.trim(),
      kind: normalizeDockComponentKind(item.kind),
      status: normalizeDockComponentStatus(item.status),
      levelPercent: normalizePercent(item.levelPercent),
      detail: typeof item.detail === "string" && item.detail.trim() !== "" ? item.detail : undefined,
      updatedAt: normalizeTimestamp(item.updatedAt),
    }))
    .filter((item) => {
      if (seen.has(item.id)) {
        return false;
      }
      seen.add(item.id);
      return true;
    });
  return components.length > 0 ? components : undefined;
}

function normalizeMapSource(value: string | undefined): VacuumLayeredMapMetadata["source"] {
  if (value === "fixed_mock") {
    return "fixed_mock";
  }
  if (value === "valetudo_mock") {
    return "valetudo_mock";
  }
  if (value === "valetudo_http") {
    return "valetudo_http";
  }
  if (value === "real_robot") {
    return "real_robot";
  }
  return "unknown";
}

function normalizeMapCoordinateSystem(value: unknown): VacuumLayeredMapMetadata["coordinateSystem"] {
  return value === "valetudo_pixel" ? "valetudo_pixel" : "unknown";
}

function normalizeMapMetadata(snapshot: ValetudoRuntimeSnapshot): VacuumLayeredMapMetadata | undefined {
  if (sourceUnavailableReason(snapshot) || snapshot.map?.available !== true || !snapshot.map.metadata) {
    return undefined;
  }
  const metadata = snapshot.map.metadata;
  return {
    id: typeof metadata.id === "string" && metadata.id.trim() !== "" ? metadata.id.trim() : undefined,
    width: normalizePositiveInteger(metadata.width),
    height: normalizePositiveInteger(metadata.height),
    pixelSize: normalizePositiveInteger(metadata.pixelSize),
    coordinateSystem: normalizeMapCoordinateSystem(metadata.coordinateSystem),
    layerCount: normalizeNonNegativeInteger(metadata.layerCount),
    entityCount: normalizeNonNegativeInteger(metadata.entityCount),
    segmentCount: normalizeNonNegativeInteger(metadata.segmentCount),
    zoneCount: normalizeNonNegativeInteger(metadata.zoneCount),
    updatedAt: normalizeTimestamp(snapshot.map.updatedAt),
    source: normalizeMapSource(snapshot.map.source),
  };
}

function normalizeMapPreview(snapshot: ValetudoRuntimeSnapshot): VacuumLayeredMapPreview | undefined {
  if (sourceUnavailableReason(snapshot) || snapshot.map?.available !== true || !snapshot.map.metadata || !snapshot.map.preview) {
    return undefined;
  }
  const width = normalizePositiveInteger(snapshot.map.metadata.width);
  const height = normalizePositiveInteger(snapshot.map.metadata.height);
  if (width == null || height == null) {
    return undefined;
  }
  const layers = normalizeMapPreviewLayers(snapshot.map.preview.layers);
  const entities = normalizeMapPreviewEntities(snapshot.map.preview.entities);
  if (layers.length === 0 && entities.length === 0) {
    return undefined;
  }
  return {
    width,
    height,
    pixelSize: normalizePositiveInteger(snapshot.map.metadata.pixelSize),
    coordinateSystem: normalizeMapCoordinateSystem(snapshot.map.metadata.coordinateSystem),
    layers,
    entities,
    updatedAt: normalizeTimestamp(snapshot.map.updatedAt),
  };
}

function normalizeMapPreviewLayers(values: ValetudoRuntimeMapLayer[] | undefined): VacuumMapLayer[] {
  const seen = new Set<string>();
  return (values ?? [])
    .filter((item): item is ValetudoRuntimeMapLayer => isRecord(item))
    .map((item): VacuumMapLayer | null => {
      const id = typeof item.id === "string" && item.id.trim() !== "" ? item.id.trim() : null;
      if (!id) {
        return null;
      }
      const kind = normalizeMapLayerKind(item.kind);
      const runs = normalizeMapRuns(item.runs);
      const points = normalizeMapPoints(item.points);
      if (!runs && !points) {
        return null;
      }
      const layer: VacuumMapLayer = {
        id,
        kind,
      };
      if (typeof item.label === "string" && item.label.trim() !== "") {
        layer.label = item.label.trim();
      }
      if (typeof item.segmentId === "string" && item.segmentId.trim() !== "") {
        layer.segmentId = item.segmentId.trim();
      }
      if (runs) {
        layer.runs = runs;
      }
      if (points) {
        layer.points = points;
      }
      return layer;
    })
    .filter((item): item is VacuumMapLayer => {
      if (!item || seen.has(item.id)) {
        return false;
      }
      seen.add(item.id);
      return true;
    });
}

function normalizeMapPreviewEntities(values: ValetudoRuntimeMapEntity[] | undefined): VacuumMapEntity[] {
  const seen = new Set<string>();
  return (values ?? [])
    .filter((item): item is ValetudoRuntimeMapEntity => isRecord(item))
    .map((item): VacuumMapEntity | null => {
      const id = typeof item.id === "string" && item.id.trim() !== "" ? item.id.trim() : null;
      if (!id) {
        return null;
      }
      const kind = normalizeMapEntityKind(item.kind);
      const points = normalizeMapPoints(item.points);
      const requiredPoints =
        kind === "zone" || kind === "no_go_area" || kind === "no_mop_area"
          ? 3
          : kind === "path" || kind === "virtual_wall"
            ? 2
            : 1;
      if (!points || points.length < requiredPoints) {
        return null;
      }
      const entity: VacuumMapEntity = {
        id,
        kind,
        points,
      };
      if (typeof item.label === "string" && item.label.trim() !== "") {
        entity.label = item.label.trim();
      }
      if (typeof item.angle === "number" && Number.isFinite(item.angle)) {
        entity.angle = item.angle;
      }
      if (typeof item.detail === "string" && item.detail.trim() !== "") {
        entity.detail = item.detail.trim();
      }
      return entity;
    })
    .filter((item): item is VacuumMapEntity => {
      if (!item || seen.has(item.id)) {
        return false;
      }
      seen.add(item.id);
      return true;
    });
}

function normalizeMapLayerKind(value: unknown): VacuumMapLayerKind {
  return value === "floor" || value === "wall" || value === "segment" || value === "path" ? value : "unknown";
}

function normalizeMapEntityKind(value: unknown): VacuumMapEntityKind {
  return (
    value === "robot" ||
    value === "charger" ||
    value === "path" ||
    value === "zone" ||
    value === "no_go_area" ||
    value === "no_mop_area" ||
    value === "virtual_wall" ||
    value === "obstacle"
  )
    ? value
    : "unknown";
}

function normalizeMapRuns(value: unknown): VacuumMapRun[] | undefined {
  if (!Array.isArray(value)) {
    return undefined;
  }
  const runs = value
    .filter((run): run is Record<string, unknown> => isRecord(run))
    .map((run) => ({
      x: typeof run.x === "number" && Number.isFinite(run.x) ? run.x : null,
      y: typeof run.y === "number" && Number.isFinite(run.y) ? run.y : null,
      count: typeof run.count === "number" && Number.isFinite(run.count) && run.count > 0 ? run.count : null,
    }))
    .filter((run): run is VacuumMapRun => run.x != null && run.y != null && run.count != null);
  return runs.length > 0 ? runs : undefined;
}

function normalizeMapTargets(snapshot: ValetudoRuntimeSnapshot): VacuumMapTargets | undefined {
  if (sourceUnavailableReason(snapshot) || snapshot.map?.available !== true) {
    return undefined;
  }
  const segments = normalizeMapTargetList(snapshot.map.targets?.segments, "segment");
  const zones = normalizeMapTargetList(snapshot.map.targets?.zones, "zone");
  return segments.length > 0 || zones.length > 0 ? { segments, zones } : undefined;
}

function normalizeMapTargetList(
  values: ValetudoRuntimeMapTarget[] | undefined,
  expectedKind: "segment" | "zone",
): VacuumMapTarget[] {
  const seen = new Set<string>();
  return (values ?? [])
    .filter((item): item is ValetudoRuntimeMapTarget => (
      item != null &&
      typeof item.id === "string" &&
      item.id.trim() !== "" &&
      typeof item.label === "string" &&
      item.label.trim() !== ""
    ))
    .map((item): VacuumMapTarget => ({
      id: item.id.trim(),
      label: item.label.trim(),
      kind: expectedKind,
      source: "runtime",
      available: item.available === true,
      geometry: normalizeMapTargetGeometry(item.geometry),
      detail: typeof item.detail === "string" && item.detail.trim() !== "" ? item.detail : undefined,
    }))
    .filter((item) => {
      if (seen.has(item.id)) {
        return false;
      }
      seen.add(item.id);
      return true;
    });
}

function normalizeMapTargetGeometry(value: unknown): VacuumMapTargetGeometry | undefined {
  if (!isRecord(value) || typeof value.type !== "string") {
    return undefined;
  }
  const points = normalizeMapPoints(value.points);
  const bounds = normalizeMapBounds(value.bounds);
  if (value.type === "polygon" && points && points.length >= 3) {
    return { type: "polygon", points };
  }
  if (value.type === "rectangle" && bounds) {
    return { type: "rectangle", bounds };
  }
  if (value.type === "unknown" && (points || bounds)) {
    const geometry: VacuumMapTargetGeometry = { type: "unknown" };
    if (points) {
      geometry.points = points;
    }
    if (bounds) {
      geometry.bounds = bounds;
    }
    return geometry;
  }
  return undefined;
}

function normalizeMapPoints(value: unknown): Array<{ x: number; y: number }> | undefined {
  if (!Array.isArray(value)) {
    return undefined;
  }
  const points = value
    .filter((point): point is Record<string, unknown> => isRecord(point))
    .map((point) => ({
      x: typeof point.x === "number" && Number.isFinite(point.x) ? point.x : null,
      y: typeof point.y === "number" && Number.isFinite(point.y) ? point.y : null,
    }))
    .filter((point): point is { x: number; y: number } => point.x != null && point.y != null);
  return points.length > 0 ? points : undefined;
}

function normalizeMapBounds(value: unknown): { x: number; y: number; width: number; height: number } | undefined {
  if (!isRecord(value)) {
    return undefined;
  }
  const x = typeof value.x === "number" && Number.isFinite(value.x) ? value.x : null;
  const y = typeof value.y === "number" && Number.isFinite(value.y) ? value.y : null;
  const width = typeof value.width === "number" && Number.isFinite(value.width) && value.width > 0 ? value.width : null;
  const height = typeof value.height === "number" && Number.isFinite(value.height) && value.height > 0 ? value.height : null;
  return x != null && y != null && width != null && height != null ? { x, y, width, height } : undefined;
}

function normalizePositiveInteger(value: unknown): number | undefined {
  return typeof value === "number" && Number.isInteger(value) && value > 0 ? value : undefined;
}

function normalizeNonNegativeInteger(value: unknown): number | undefined {
  return typeof value === "number" && Number.isInteger(value) && value >= 0 ? value : undefined;
}

function runtimeCommandToCapabilityName(command: string): keyof VacuumCapabilities | null {
  if (command === "set_fan_speed") {
    return "fan_speed";
  }
  if (command === "set_water_usage") {
    return "water_usage";
  }
  if (command === "start_cleaning" || command === "pause" || command === "resume" || command === "stop" || command === "return_to_dock") {
    return command;
  }
  return null;
}

export function isValetudoRuntimeSnapshot(value: unknown): value is ValetudoRuntimeSnapshot {
  if (!isRecord(value)) {
    return false;
  }
  const candidate = value;
  const runtime = candidate.runtime;
  const robot = candidate.robot;
  const source = candidate.source;
  const state = candidate.state;
  const connectivity = candidate.connectivity;
  const capabilities = candidate.capabilities;
  const diagnostics = candidate.diagnostics;
  return (
    typeof candidate.backend === "string" &&
    isRecord(runtime) &&
    typeof runtime.id === "string" &&
    typeof runtime.version === "string" &&
    typeof runtime.status === "string" &&
    isRecord(robot) &&
    typeof robot.id === "string" &&
    typeof robot.name === "string" &&
    isRecord(source) &&
    typeof source.kind === "string" &&
    typeof source.status === "string" &&
    typeof source.stale === "boolean" &&
    (typeof source.lastSeenAt === "number" || source.lastSeenAt === null) &&
    isRecord(state) &&
    typeof state.value === "string" &&
    typeof state.label === "string" &&
    typeof state.started === "boolean" &&
    typeof state.paused === "boolean" &&
    isRecord(connectivity) &&
    typeof connectivity.reachable === "boolean" &&
    typeof connectivity.online === "boolean" &&
    isRecord(capabilities) &&
    isRecord(capabilities.commands) &&
    Array.isArray(capabilities.diagnostics) &&
    isRecord(diagnostics) &&
    typeof diagnostics.mode === "string" &&
    isStringArray(diagnostics.rawCapabilityNames) &&
    typeof candidate.updatedAt === "number"
  );
}

function normalizeMissionState(snapshot: ValetudoRuntimeSnapshot): VacuumMissionState {
  const value = snapshot.state.value.toLowerCase();
  if (snapshot.state.paused || value.includes("pause")) {
    return "paused";
  }
  if (value.includes("return")) {
    return "returning";
  }
  if (snapshot.state.started || value.includes("clean")) {
    return "cleaning";
  }
  // Only treat the robot as charging when the source actually reports charging.
  // Being docked while idle is not a mission and must map to "idle".
  if (snapshot.battery?.charging) {
    return "charging";
  }
  return "idle";
}

function normalizeActivityStatus(snapshot: ValetudoRuntimeSnapshot): VacuumRobotActivityStatus {
  const sourceReason = sourceUnavailableReason(snapshot);
  if (sourceReason === "runtime_offline" || sourceReason === "source_unreachable") {
    return "unavailable";
  }
  const value = normalizeStateValue(snapshot);
  if (value.includes("fault") || value.includes("error")) {
    return "faulted";
  }
  if (snapshot.state.paused || value.includes("pause")) {
    return "paused";
  }
  if (value.includes("return")) {
    return "returning";
  }
  if (snapshot.state.started || value.includes("clean")) {
    return "cleaning";
  }
  if (snapshot.battery?.charging) {
    return "charging";
  }
  if (mapDockState(snapshot) === "docked") {
    return "docked";
  }
  return "idle";
}

export function mapValetudoRuntimeSnapshotToBoundary(
  snapshot: ValetudoRuntimeSnapshot,
): ValetudoRuntimeBoundary {
  const capabilities = new Set<ValetudoBackendCapability>();
  const commandAvailability: ValetudoRuntimeBoundary["commandAvailability"] = {};
  const unsupportedCommands: ValetudoRuntimeBoundary["unsupportedCommands"] = {};
  const runtimeCommandNames = new Set(Object.keys(snapshot.capabilities.commands));
  for (const [command, availability] of Object.entries(snapshot.capabilities.commands)) {
    const backendCapability = RUNTIME_COMMAND_TO_BACKEND_CAPABILITY[command];
    if (
      backendCapability &&
      (backendCapability === "BasicControlCapability" || availability.reason !== "capability_unavailable")
    ) {
      capabilities.add(backendCapability);
    }
    if (isBasicCommandName(command)) {
      const reason = commandUnavailableReason(snapshot, command, availability.available, availability.reason);
      commandAvailability[command] = {
        available: reason ? false : true,
        reason,
      };
    } else {
      const capabilityName = runtimeCommandToCapabilityName(command);
      if (!capabilityName) {
        continue;
      }
      commandAvailability[capabilityName] = {
        available: availability.available && !sourceUnavailableReason(snapshot),
        reason: sourceUnavailableReason(snapshot) ?? availability.reason,
      };
    }
  }
  if (
    capabilities.has("BasicControlCapability") &&
    !runtimeCommandNames.has("resume") &&
    runtimeCommandNames.has("start_cleaning")
  ) {
    const startAvailability = snapshot.capabilities.commands.start_cleaning;
    const reason = commandUnavailableReason(
      snapshot,
      "resume",
      startAvailability?.available ?? false,
      startAvailability?.reason,
    );
    commandAvailability.resume = {
      available: reason ? false : true,
      reason,
    };
  }
  if (capabilities.has("BasicControlCapability")) {
    for (const command of VALETUDO_BASIC_COMMANDS) {
      if (!runtimeCommandNames.has(command)) {
        if (command === "resume" && commandAvailability.resume) {
          continue;
        }
        unsupportedCommands[command] = command === "resume"
          ? "Valetudo resume support is not reported by this runtime."
          : `Valetudo ${command.replace(/_/g, "-")} support is not reported by this runtime.`;
      }
    }
  }
  if (snapshot.battery) {
    capabilities.add("BatteryStateCapability");
  }
  for (const diagnostic of snapshot.capabilities.diagnostics) {
    if (
      diagnostic.detected &&
      isKnownValetudoBackendCapability(diagnostic.name) &&
      isDiagnosticOnlyCapability(diagnostic.name)
    ) {
      capabilities.add(diagnostic.name);
    }
  }

  const sourceReason = sourceUnavailableReason(snapshot);
  const cleaningSettings = mapCleaningSettings(snapshot);
  const maintenance = mapMaintenance(snapshot);
  const statistics = mapStatistics(snapshot);
  const attachments = mapAttachments(snapshot);
  const dockComponents = mapDockComponents(snapshot);
  const mapMetadata = normalizeMapMetadata(snapshot);
  const mapPreview = normalizeMapPreview(snapshot);
  const mapTargets = normalizeMapTargets(snapshot);
  const faults = [
    ...(normalizeActivityStatus(snapshot) === "faulted" ? [snapshot.state.label || "Valetudo runtime reported a robot fault."] : []),
    ...(snapshot.source.stale ? ["Valetudo runtime source state is stale."] : []),
    ...(sourceReason === "source_unreachable" ? ["Valetudo runtime source is unreachable."] : []),
    ...(snapshot.runtime.status === "degraded" ? ["Valetudo integration runtime is degraded."] : []),
  ];

  return {
    connectionStatus: snapshot.connectivity.online ? "online" : "offline",
    capabilities: [...capabilities],
    commandAvailability,
    unsupportedCommands,
    state: {
      id: snapshot.robot.id,
      label: snapshot.robot.name,
      mapAvailable: false,
      pose: null,
      batteryPercentage: snapshot.battery?.level ?? null,
      charging: snapshot.battery?.charging ?? null,
      missionState: normalizeMissionState(snapshot),
      activityStatus: normalizeActivityStatus(snapshot),
      activityLabel: snapshot.state.label,
      activityUpdatedAt: snapshot.updatedAt,
      faults,
    },
    health: mapHealth(snapshot),
    source: {
      kind: normalizeSourceKind(snapshot.source.kind),
      status: normalizeSourceStatus(snapshot),
      stale: snapshot.source.stale,
      lastSeenAt: snapshot.source.lastSeenAt,
      reason: sourceReason,
    },
    dock: {
      supported: snapshot.dock != null || snapshot.battery?.charging != null,
      state: mapDockState(snapshot),
      charging: snapshot.battery?.charging,
      detail: snapshot.dock?.state ?? (snapshot.battery?.charging ? "Charging." : "Dock state unknown."),
      components: dockComponents,
    },
    cleaningSettings,
    maintenance,
    statistics,
    attachments,
    map: mapMetadata || mapPreview || mapTargets
      ? {
          layeredMetadata: mapMetadata,
          layeredPreview: mapPreview,
          targets: mapTargets,
          detail: snapshot.map?.detail,
        }
      : undefined,
    diagnostics: mapDiagnostics(snapshot),
    lastError: snapshot.connectivity.online ? undefined : "Valetudo integration runtime is offline.",
  };
}

export function mapValetudoRuntimeUnavailable(message: string): ValetudoRuntimeBoundary {
  return {
    connectionStatus: "offline",
    capabilities: [],
    state: null,
    health: {
      runtimeStatus: "offline",
      detail: message,
    },
    source: {
      kind: "unknown",
      status: "unknown",
      stale: false,
      lastSeenAt: null,
      reason: "runtime_offline",
    },
    dock: {
      supported: false,
      state: "unknown",
      charging: undefined,
      detail: "Dock state is unavailable while the runtime is offline.",
    },
    cleaningSettings: undefined,
    maintenance: undefined,
    statistics: undefined,
    attachments: undefined,
    map: undefined,
    diagnostics: {
      backend: "valetudo",
      runtime: {
        status: "offline",
      },
      source: {
        status: "unknown",
        reason: "runtime_offline",
      },
      map: {
        supported: false,
        reason: "Valetudo map data is unavailable while the runtime is offline.",
      },
      pose: {
        supported: false,
        reason: "Valetudo pose data is unavailable while the runtime is offline.",
      },
      navigation: {
        supported: false,
        reason: "Valetudo navigation data is unavailable while the runtime is offline.",
      },
      mapping: {
        supported: false,
        reason: "Valetudo mapping data is unavailable while the runtime is offline.",
      },
      warnings: [message],
    },
    lastError: message,
  };
}

const VALETUDO_ACTIVITY_ACTIONS = [
  "start_cleaning",
  "pause",
  "resume",
  "stop",
  "return_to_dock",
] as const satisfies readonly VacuumRobotActivityAction[];

function availableValetudoActivityActions(capabilities: VacuumCapabilities): VacuumRobotActivityAction[] {
  return VALETUDO_ACTIVITY_ACTIONS.filter((action) => capabilities[action].supported && capabilities[action].available !== false);
}

function buildValetudoActivity(
  runtime: ValetudoRuntimeBoundary,
  capabilities: VacuumCapabilities,
): VacuumRobotActivity {
  const state = runtime.state;
  if (!state) {
    return {
      status: "unavailable",
      label: "Unavailable",
      source: "valetudo",
      reason: runtime.source?.reason ?? "runtime_offline",
      availableActions: [],
    };
  }
  const status =
    runtime.source?.reason === "runtime_offline" || runtime.source?.reason === "source_unreachable"
      ? "unavailable"
      : state.activityStatus;
  return {
    status,
    label: state.activityLabel,
    updatedAt: state.activityUpdatedAt,
    source: "valetudo",
    reason: runtime.source?.reason,
    availableActions: availableValetudoActivityActions(capabilities),
    details: {
      dock: runtime.dock?.state ?? "unknown",
      charging: state.charging,
    },
  };
}

function buildValetudoActiveMission(runtime: ValetudoRuntimeBoundary): VacuumMissionSnapshot | null {
  const state = runtime.state;
  if (!state || state.missionState === "idle") {
    return null;
  }
  const status = state.missionState === "paused"
    ? "paused"
    : state.missionState === "returning"
      ? "returning"
      : state.missionState === "charging"
        ? "charging"
        : "running";
  return {
    id: "valetudo:active",
    type: state.missionState === "returning" ? "return_to_dock" : "hardware_cleaning",
    status,
    backendSource: "valetudo",
    startedAt: null,
    updatedAt: null,
    requestedCommand: "runtime_state",
    phase: state.missionState,
    progress: {
      percent: null,
      currentStep: null,
      totalSteps: null,
      distanceRemaining: null,
      areaCoveredSqM: null,
      areaRemainingSqM: null,
    },
    availableActions: status === "running"
      ? ["pause_mission", "cancel_mission", "return_to_dock"]
      : status === "paused"
        ? ["resume_mission", "cancel_mission"]
        : [],
    result: null,
    error: null,
    target: null,
  };
}

export function mapValetudoState(runtime: ValetudoRuntimeBoundary): VacuumAdapterSnapshot {
  const connected = runtime.connectionStatus === "online";
  const state = runtime.state;
  const activeMission = buildValetudoActiveMission(runtime);
  const faults = [...(state?.faults ?? [])];
  if (runtime.lastError) {
    faults.push(runtime.lastError);
  }

  const capabilities = mapValetudoCapabilities(runtime.capabilities, {
    commandAvailability: runtime.commandAvailability,
    unsupportedCommands: runtime.unsupportedCommands,
    consumablesSupported: (runtime.maintenance?.consumables.length ?? 0) > 0,
    currentStatisticsSupported: runtime.statistics?.current != null,
    attachmentsSupported: (runtime.attachments?.items.length ?? 0) > 0,
    attachmentKinds: [...new Set((runtime.attachments?.items ?? []).map((item) => item.kind))],
    dockComponentsSupported: (runtime.dock?.components?.length ?? 0) > 0,
    dockComponentKinds: [...new Set((runtime.dock?.components ?? []).map((item) => item.kind))],
    unavailableReason:
      runtime.connectionStatus === "online" && runtime.source?.reason
        ? runtime.source.reason
        : runtime.connectionStatus === "online"
          ? undefined
          : "runtime_offline",
  });
  if (runtime.cleaningSettings?.fanSpeed && capabilities.fan_speed.supported) {
    capabilities.fan_speed.attributes = runtime.cleaningSettings.fanSpeed.options.map((option) => `option:${option.value}`);
  }
  if (runtime.cleaningSettings?.waterUsage && capabilities.water_usage.supported) {
    capabilities.water_usage.attributes = runtime.cleaningSettings.waterUsage.options.map((option) => `option:${option.value}`);
  }
  const activity = buildValetudoActivity(runtime, capabilities);

  return {
    identity: {
      id: state?.id ?? "valetudo",
      label: state?.label ?? "Valetudo Vacuum",
      source: "valetudo",
      model: state?.model,
    },
    availability: {
      status: runtime.connectionStatus,
      connected,
      detail: connected ? "Valetudo integration runtime is online." : "Valetudo integration runtime is not online.",
    },
    capabilities,
    health: runtime.health,
    source: runtime.source,
    dock: runtime.dock,
    diagnostics: runtime.diagnostics,
    cleaningSettings: runtime.cleaningSettings,
    maintenance: runtime.maintenance,
    statistics: runtime.statistics,
    attachments: runtime.attachments,
    map: {
      readiness: "unavailable",
      receiving: false,
      detail: runtime.map?.detail ?? "Map is unsupported for this Valetudo adapter state; no product map is available.",
      grid: null,
      metadata: buildVacuumMapMetadata(null, null),
      layeredMetadata: runtime.map?.layeredMetadata,
      layeredPreview: runtime.map?.layeredPreview,
      targets: runtime.map?.targets,
      annotations: [],
    },
    pose: {
      readiness: "unavailable",
      available: false,
      coordinates: null,
      detail: "Pose is unsupported for this Valetudo adapter state; no product pose is available.",
    },
    navigation: EMPTY_NAVIGATION,
    activity,
    mission: {
      state: state?.missionState ?? "idle",
      detail: state ? `Valetudo mission state: ${state.missionState}.` : "Waiting for Valetudo mission state.",
      lastTerminalNavigation: null,
    },
    activeMission,
    missions: {
      active: activeMission,
      recent: [],
    },
    mapping: {
      state: "idle",
      mode: null,
      stateReason: "Auto mapping is not implemented for this backend.",
      knownRatio: 0,
      unknownRatio: 1,
      frontierCount: 0,
      visitedGoalCount: 0,
      failedGoalCount: 0,
      activeGoal: null,
      lastError: null,
      updatedAt: null,
      persistence: "unsupported",
      acceptedSessionLevel: false,
      savedMapPath: null,
      loadedMapPath: null,
      lastSavedAt: null,
      saveError: null,
      loadError: null,
      activeMapName: null,
      savedMaps: [],
    },
    readiness: {
      ready: connected && state != null,
      blockingReasons: connected && state != null ? [] : ["Valetudo runtime state is not available."],
    },
    fault: {
      readiness: faults.length > 0 ? "degraded" : connected ? "ready" : "unavailable",
      faults,
      detail: faults.length > 0 ? faults.join(" ") : "No Valetudo faults reported.",
    },
    battery: {
      readiness: state?.batteryPercentage == null ? (connected ? "waiting" : "unavailable") : "ready",
      percentage: state?.batteryPercentage ?? null,
      charging: state?.charging ?? null,
      detail:
        state?.batteryPercentage == null
          ? "Waiting for Valetudo battery state."
          : `Battery ${state.batteryPercentage}%.`,
    },
  };
}
