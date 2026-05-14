import type { VacuumMapGrid, VacuumPoseCoordinates } from "../../vacuum-adapter";
import type { CleanAreaRect } from "./MapCanvas";

export type CleanAreaCoverageCellState = "remaining" | "covered" | "occupied" | "unknown" | "out_of_bounds" | "too_small";

export type CleanAreaCoverageOverlayCell = {
  key: string;
  cellX: number;
  cellY: number;
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
  state: CleanAreaCoverageCellState;
};

export type CleanAreaCoverageTargetCell = {
  key: string;
  cellX: number;
  cellY: number;
  centerX: number;
  centerY: number;
};

export type CleanAreaCoverageRegion = {
  id: string;
  cellCount: number;
  areaSqM: number;
  tooSmall: boolean;
  bounds: CleanAreaRect;
  centroid: {
    x: number;
    y: number;
  };
  cells: CleanAreaCoverageTargetCell[];
};

export type CleanAreaCoverageTarget = {
  signature: string;
  cellAreaSqM: number;
  cleanableCells: CleanAreaCoverageTargetCell[];
  cleanableRegions: CleanAreaCoverageRegion[];
  skippedSmallRegionCells: CleanAreaCoverageOverlayCell[];
  skippedSmallRegionCount: number;
  occupiedCells: CleanAreaCoverageOverlayCell[];
  unknownCells: CleanAreaCoverageOverlayCell[];
  outOfBoundsCells: number;
  minimumUsefulCleanableRegionSqM: number;
};

export type CleanAreaCoverageSnapshot = {
  target: CleanAreaCoverageTarget;
  swathWidth: number;
  targetCells: number;
  coveredCells: number;
  remainingCells: number;
  occupiedCells: number;
  unknownCells: number;
  outOfBoundsCells: number;
  skippedSmallRegionCells: number;
  cleanableRegionCount: number;
  skippedSmallRegionCount: number;
  cleanableAreaSqM: number;
  coveredAreaSqM: number;
  remainingAreaSqM: number;
  skippedAreaSqM: number;
  progress: number;
  overlayCells: CleanAreaCoverageOverlayCell[];
};

const FREE_OCCUPANCY_MAX = 15;

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function cellKey(cellX: number, cellY: number): string {
  return `${cellX}:${cellY}`;
}

function getMapSignature(map: VacuumMapGrid): string {
  return [
    map.width,
    map.height,
    map.resolution,
    map.originX.toFixed(3),
    map.originY.toFixed(3),
    map.data.length,
  ].join(":");
}

function makeOverlayCell(
  cellX: number,
  cellY: number,
  map: VacuumMapGrid,
  state: CleanAreaCoverageCellState,
): CleanAreaCoverageOverlayCell {
  const minX = map.originX + cellX * map.resolution;
  const minY = map.originY + cellY * map.resolution;
  return {
    key: cellKey(cellX, cellY),
    cellX,
    cellY,
    minX,
    minY,
    maxX: minX + map.resolution,
    maxY: minY + map.resolution,
    state,
  };
}

function buildConnectedCleanableRegions(args: {
  cleanableCells: CleanAreaCoverageTargetCell[];
  cellAreaSqM: number;
  minimumUsefulCleanableRegionSqM: number;
}): CleanAreaCoverageRegion[] {
  const cellsByKey = new Map(args.cleanableCells.map((cell) => [cell.key, cell]));
  const visited = new Set<string>();
  const regions: CleanAreaCoverageRegion[] = [];

  for (const seed of args.cleanableCells) {
    if (visited.has(seed.key)) {
      continue;
    }

    const stack = [seed];
    const cells: CleanAreaCoverageTargetCell[] = [];
    visited.add(seed.key);

    while (stack.length > 0) {
      const cell = stack.pop()!;
      cells.push(cell);
      const neighbors = [
        cellKey(cell.cellX + 1, cell.cellY),
        cellKey(cell.cellX - 1, cell.cellY),
        cellKey(cell.cellX, cell.cellY + 1),
        cellKey(cell.cellX, cell.cellY - 1),
      ];

      for (const key of neighbors) {
        if (visited.has(key)) {
          continue;
        }
        const neighbor = cellsByKey.get(key);
        if (!neighbor) {
          continue;
        }
        visited.add(key);
        stack.push(neighbor);
      }
    }

    let minX = Number.POSITIVE_INFINITY;
    let minY = Number.POSITIVE_INFINITY;
    let maxX = Number.NEGATIVE_INFINITY;
    let maxY = Number.NEGATIVE_INFINITY;
    let sumX = 0;
    let sumY = 0;
    for (const cell of cells) {
      minX = Math.min(minX, cell.centerX);
      minY = Math.min(minY, cell.centerY);
      maxX = Math.max(maxX, cell.centerX);
      maxY = Math.max(maxY, cell.centerY);
      sumX += cell.centerX;
      sumY += cell.centerY;
    }

    const areaSqM = cells.length * args.cellAreaSqM;
    const cellSize = Math.sqrt(args.cellAreaSqM);
    regions.push({
      id: `region-${regions.length + 1}`,
      cellCount: cells.length,
      areaSqM,
      tooSmall: args.minimumUsefulCleanableRegionSqM > 0 && areaSqM < args.minimumUsefulCleanableRegionSqM,
      bounds: {
        minX: minX - cellSize / 2,
        minY: minY - cellSize / 2,
        maxX: maxX + cellSize / 2,
        maxY: maxY + cellSize / 2,
      },
      centroid: { x: sumX / cells.length, y: sumY / cells.length },
      cells,
    });
  }

  return regions.sort((a, b) => b.cellCount - a.cellCount);
}

function distanceToSegment(point: { x: number; y: number }, start: { x: number; y: number }, end: { x: number; y: number }): number {
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const lengthSq = dx * dx + dy * dy;
  if (lengthSq <= 0) {
    return Math.hypot(point.x - start.x, point.y - start.y);
  }
  const t = clamp(((point.x - start.x) * dx + (point.y - start.y) * dy) / lengthSq, 0, 1);
  return Math.hypot(point.x - (start.x + dx * t), point.y - (start.y + dy * t));
}

export function buildCleanAreaCoverageTarget(
  rect: CleanAreaRect | null,
  map: VacuumMapGrid | null,
  options?: {
    minimumUsefulCleanableRegionSqM?: number;
  },
): CleanAreaCoverageTarget | null {
  if (!rect || !map || map.width <= 0 || map.height <= 0 || map.resolution <= 0) {
    return null;
  }

  const minCellX = Math.floor((rect.minX - map.originX) / map.resolution);
  const maxCellX = Math.floor((rect.maxX - map.originX) / map.resolution);
  const minCellY = Math.floor((rect.minY - map.originY) / map.resolution);
  const maxCellY = Math.floor((rect.maxY - map.originY) / map.resolution);
  const candidateCleanableCells: CleanAreaCoverageTargetCell[] = [];
  const occupiedCells: CleanAreaCoverageOverlayCell[] = [];
  const unknownCells: CleanAreaCoverageOverlayCell[] = [];
  let outOfBoundsCells = 0;
  const minimumUsefulCleanableRegionSqM = Math.max(0, options?.minimumUsefulCleanableRegionSqM ?? 0);

  for (let cellY = minCellY; cellY <= maxCellY; cellY += 1) {
    for (let cellX = minCellX; cellX <= maxCellX; cellX += 1) {
      const centerX = map.originX + (cellX + 0.5) * map.resolution;
      const centerY = map.originY + (cellY + 0.5) * map.resolution;
      if (centerX < rect.minX || centerX > rect.maxX || centerY < rect.minY || centerY > rect.maxY) {
        continue;
      }

      if (cellX < 0 || cellX >= map.width || cellY < 0 || cellY >= map.height) {
        outOfBoundsCells += 1;
        continue;
      }

      const value = Number(map.data[cellX + cellY * map.width] ?? -1);
      if (value < 0) {
        unknownCells.push(makeOverlayCell(cellX, cellY, map, "unknown"));
      } else if (value <= FREE_OCCUPANCY_MAX) {
        const key = cellKey(cellX, cellY);
        candidateCleanableCells.push({ key, cellX, cellY, centerX, centerY });
      } else {
        occupiedCells.push(makeOverlayCell(cellX, cellY, map, "occupied"));
      }
    }
  }

  const allRegions = buildConnectedCleanableRegions({
    cleanableCells: candidateCleanableCells,
    cellAreaSqM: map.resolution * map.resolution,
    minimumUsefulCleanableRegionSqM,
  });
  const cleanableRegions = allRegions.filter((region) => !region.tooSmall);
  const skippedSmallRegions = allRegions.filter((region) => region.tooSmall);
  const cleanableCells = cleanableRegions.flatMap((region) => region.cells);
  const skippedSmallRegionCells = skippedSmallRegions
    .flatMap((region) => region.cells)
    .map((cell) => makeOverlayCell(cell.cellX, cell.cellY, map, "too_small"));

  return {
    signature: `${getMapSignature(map)}:${rect.minX.toFixed(3)}:${rect.minY.toFixed(3)}:${rect.maxX.toFixed(3)}:${rect.maxY.toFixed(3)}:${minimumUsefulCleanableRegionSqM.toFixed(3)}`,
    cellAreaSqM: map.resolution * map.resolution,
    cleanableCells,
    cleanableRegions,
    skippedSmallRegionCells,
    skippedSmallRegionCount: skippedSmallRegions.length,
    occupiedCells,
    unknownCells,
    outOfBoundsCells,
    minimumUsefulCleanableRegionSqM,
  };
}

export function markCleanAreaCoveredCells(args: {
  target: CleanAreaCoverageTarget;
  coveredCellKeys: Set<string>;
  previousPose: VacuumPoseCoordinates | null;
  currentPose: VacuumPoseCoordinates;
  swathWidth: number;
}): Set<string> {
  const radius = Math.max(0, args.swathWidth / 2);
  if (radius <= 0 || args.target.cleanableCells.length === 0) {
    return args.coveredCellKeys;
  }

  const start = args.previousPose ?? args.currentPose;
  const end = args.currentPose;
  let next: Set<string> | null = null;

  for (const cell of args.target.cleanableCells) {
    if (args.coveredCellKeys.has(cell.key)) {
      continue;
    }
    if (distanceToSegment({ x: cell.centerX, y: cell.centerY }, start, end) <= radius) {
      next ??= new Set(args.coveredCellKeys);
      next.add(cell.key);
    }
  }

  return next ?? args.coveredCellKeys;
}

export function buildCleanAreaCoverageSnapshot(args: {
  target: CleanAreaCoverageTarget | null;
  coveredCellKeys: Set<string>;
  swathWidth: number;
}): CleanAreaCoverageSnapshot | null {
  if (!args.target) {
    return null;
  }

  const target = args.target;
  const coveredCells = target.cleanableCells.reduce(
    (count, cell) => count + (args.coveredCellKeys.has(cell.key) ? 1 : 0),
    0,
  );
  const targetCells = target.cleanableCells.length;
  const remainingCells = Math.max(0, targetCells - coveredCells);
  const occupiedCells = target.occupiedCells.length;
  const unknownCells = target.unknownCells.length;
  const skippedSmallRegionCells = target.skippedSmallRegionCells.length;
  const skippedCells = occupiedCells + unknownCells + target.outOfBoundsCells + skippedSmallRegionCells;
  const cellAreaSqM = target.cellAreaSqM;
  const cellSize = Math.sqrt(cellAreaSqM);

  return {
    target,
    swathWidth: args.swathWidth,
    targetCells,
    coveredCells,
    remainingCells,
    occupiedCells,
    unknownCells,
    outOfBoundsCells: target.outOfBoundsCells,
    skippedSmallRegionCells,
    cleanableRegionCount: target.cleanableRegions.length,
    skippedSmallRegionCount: target.skippedSmallRegionCount,
    cleanableAreaSqM: targetCells * cellAreaSqM,
    coveredAreaSqM: coveredCells * cellAreaSqM,
    remainingAreaSqM: remainingCells * cellAreaSqM,
    skippedAreaSqM: skippedCells * cellAreaSqM,
    progress: targetCells > 0 ? coveredCells / targetCells : 0,
    overlayCells: [
      ...target.cleanableCells.map<CleanAreaCoverageOverlayCell>((cell) => ({
        key: cell.key,
        cellX: cell.cellX,
        cellY: cell.cellY,
        minX: cell.centerX - cellSize / 2,
        minY: cell.centerY - cellSize / 2,
        maxX: cell.centerX + cellSize / 2,
        maxY: cell.centerY + cellSize / 2,
        state: args.coveredCellKeys.has(cell.key) ? "covered" : "remaining",
      })),
      ...target.occupiedCells,
      ...target.unknownCells,
      ...target.skippedSmallRegionCells,
    ],
  };
}
