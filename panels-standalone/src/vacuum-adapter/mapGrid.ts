import type { VacuumMapGrid, VacuumMapMetadata } from "./state";

function toFiniteNumber(value: unknown): number | null {
  const numeric = typeof value === "string" ? Number(value) : value;
  return typeof numeric === "number" && Number.isFinite(numeric) ? numeric : null;
}

function getRecordEntry(record: Record<string, unknown>, key: string): unknown {
  return record[key];
}

function toNumericArray(value: unknown): number[] | null {
  if (Array.isArray(value)) {
    return value.map((entry) => Number(entry));
  }
  if (typeof ArrayBuffer !== "undefined" && ArrayBuffer.isView(value) && !(value instanceof DataView)) {
    return Array.from(value as unknown as ArrayLike<number>, (entry) => Number(entry));
  }
  return null;
}

function quaternionToYawDegrees(orientation: Record<string, unknown> | null): number {
  if (!orientation) {
    return 0;
  }
  const x = toFiniteNumber(getRecordEntry(orientation, "x")) ?? 0;
  const y = toFiniteNumber(getRecordEntry(orientation, "y")) ?? 0;
  const z = toFiniteNumber(getRecordEntry(orientation, "z")) ?? 0;
  const w = toFiniteNumber(getRecordEntry(orientation, "w")) ?? 1;
  const sinyCosp = 2 * (w * z + x * y);
  const cosyCosp = 1 - 2 * (y * y + z * z);
  return (Math.atan2(sinyCosp, cosyCosp) * 180) / Math.PI;
}

export function parseVacuumMapGrid(message: Record<string, unknown> | null): VacuumMapGrid | null {
  if (!message) {
    return null;
  }

  const info = getRecordEntry(message, "info");
  if (!info || typeof info !== "object") {
    return null;
  }

  const infoRecord = info as Record<string, unknown>;
  const width = toFiniteNumber(getRecordEntry(infoRecord, "width"));
  const height = toFiniteNumber(getRecordEntry(infoRecord, "height"));
  const resolution = toFiniteNumber(getRecordEntry(infoRecord, "resolution"));
  const origin = getRecordEntry(infoRecord, "origin");
  const originRecord = origin && typeof origin === "object" ? (origin as Record<string, unknown>) : null;
  const position = originRecord ? getRecordEntry(originRecord, "position") : null;
  const positionRecord = position && typeof position === "object" ? (position as Record<string, unknown>) : null;
  const originX = positionRecord ? toFiniteNumber(getRecordEntry(positionRecord, "x")) : null;
  const originY = positionRecord ? toFiniteNumber(getRecordEntry(positionRecord, "y")) : null;
  const orientation = originRecord ? getRecordEntry(originRecord, "orientation") : null;
  const orientationRecord = orientation && typeof orientation === "object" ? (orientation as Record<string, unknown>) : null;
  const data = toNumericArray(getRecordEntry(message, "data"));
  const header = getRecordEntry(message, "header");
  const frameId =
    header && typeof header === "object" ? getRecordEntry(header as Record<string, unknown>, "frame_id") : null;

  if (width == null || height == null || resolution == null || originX == null || originY == null || data == null) {
    return null;
  }

  return {
    width,
    height,
    resolution,
    originX,
    originY,
    originYaw: quaternionToYawDegrees(orientationRecord),
    frameId: typeof frameId === "string" && frameId.length > 0 ? frameId : null,
    data,
  };
}

export function buildVacuumMapMetadata(grid: VacuumMapGrid | null, lastUpdateAt: number | null): VacuumMapMetadata {
  if (!grid || grid.width <= 0 || grid.height <= 0) {
    return {
      hasMap: false,
      width: 0,
      height: 0,
      resolution: 0,
      freeCells: 0,
      occupiedCells: 0,
      unknownCells: 0,
      knownCells: 0,
      totalCells: 0,
      freeRatio: 0,
      occupiedRatio: 0,
      unknownRatio: 1,
      knownRatio: 0,
      knownAreaSqM: 0,
      lastUpdateAt,
    };
  }

  let freeCells = 0;
  let occupiedCells = 0;
  let unknownCells = 0;
  const totalCells = Math.max(1, grid.width * grid.height);
  for (const cell of grid.data.slice(0, totalCells)) {
    if (cell < 0) {
      unknownCells += 1;
    } else if (cell <= 15) {
      freeCells += 1;
    } else {
      occupiedCells += 1;
    }
  }

  const knownCells = freeCells + occupiedCells;
  return {
    hasMap: true,
    width: grid.width,
    height: grid.height,
    resolution: grid.resolution,
    freeCells,
    occupiedCells,
    unknownCells,
    knownCells,
    totalCells,
    freeRatio: freeCells / totalCells,
    occupiedRatio: occupiedCells / totalCells,
    unknownRatio: unknownCells / totalCells,
    knownRatio: knownCells / totalCells,
    knownAreaSqM: knownCells * grid.resolution * grid.resolution,
    lastUpdateAt,
  };
}
