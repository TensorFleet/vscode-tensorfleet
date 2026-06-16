import type { CleanAreaRect } from "./MapCanvas";
import type { CleanAreaCoverageTarget, CleanAreaCoverageTargetCell } from "./cleanAreaCoverage";

export type CleanAreaWaypoint = {
  x: number;
  y: number;
  yaw: number;
};

type SweepOrientation = "horizontal" | "vertical";

type Segment = {
  start: number;
  end: number;
  boundaryStart: boolean;
  boundaryEnd: boolean;
};

const MIN_SEGMENT_LENGTH_M = 0.04;

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function nearlySamePoint(a: CleanAreaWaypoint, b: CleanAreaWaypoint): boolean {
  return Math.hypot(a.x - b.x, a.y - b.y) < 0.015;
}

function chooseSweepOrientation(rect: CleanAreaRect): SweepOrientation {
  const width = Math.max(0, rect.maxX - rect.minX);
  const height = Math.max(0, rect.maxY - rect.minY);
  return width >= height ? "horizontal" : "vertical";
}

function buildLaneCenters(min: number, max: number, swathWidth: number, spacing: number): number[] {
  const length = Math.max(0, max - min);
  if (length <= 0) {
    return [];
  }

  if (length <= Math.min(swathWidth, spacing)) {
    return [(min + max) / 2];
  }

  const count = Math.max(2, Math.ceil(length / spacing) + 1);
  return Array.from({ length: count }, (_, index) => {
    const t = count === 1 ? 0 : index / (count - 1);
    return min + length * t;
  });
}

function getCellCrossAxis(cell: CleanAreaCoverageTargetCell, orientation: SweepOrientation): number {
  return orientation === "horizontal" ? cell.centerY : cell.centerX;
}

function getCellPassAxis(cell: CleanAreaCoverageTargetCell, orientation: SweepOrientation): number {
  return orientation === "horizontal" ? cell.centerX : cell.centerY;
}

function buildSegmentsForLane(args: {
  rect: CleanAreaRect;
  target: CleanAreaCoverageTarget | null;
  orientation: SweepOrientation;
  lane: number;
  swathWidth: number;
  spacing: number;
}): Segment[] {
  const passMin = args.orientation === "horizontal" ? args.rect.minX : args.rect.minY;
  const passMax = args.orientation === "horizontal" ? args.rect.maxX : args.rect.maxY;
  const passLength = Math.max(0, passMax - passMin);
  if (passLength <= 0) {
    return [];
  }

  const fullSegment = {
    start: passMin,
    end: passMax,
    boundaryStart: true,
    boundaryEnd: true,
  };

  if (!args.target || args.target.cleanableCells.length === 0) {
    return [fullSegment];
  }

  const resolution = Math.sqrt(args.target.cellAreaSqM);
  const bandHalfWidth = Math.max(args.swathWidth / 2, args.spacing / 2, resolution / 2);
  const gapLimit = Math.max(resolution * 1.75, args.spacing * 0.75);
  const cells = args.target.cleanableCells
    .filter((cell) => Math.abs(getCellCrossAxis(cell, args.orientation) - args.lane) <= bandHalfWidth)
    .sort((a, b) => getCellPassAxis(a, args.orientation) - getCellPassAxis(b, args.orientation));

  if (cells.length === 0) {
    return [];
  }

  const segments: Segment[] = [];
  let segmentStart = getCellPassAxis(cells[0]!, args.orientation) - resolution / 2;
  let previous = getCellPassAxis(cells[0]!, args.orientation);

  for (let index = 1; index < cells.length; index += 1) {
    const current = getCellPassAxis(cells[index]!, args.orientation);
    if (current - previous > gapLimit) {
      segments.push({
        start: clamp(segmentStart, fullSegment.start, fullSegment.end),
        end: clamp(previous + resolution / 2, fullSegment.start, fullSegment.end),
        boundaryStart: segmentStart <= fullSegment.start + resolution / 2,
        boundaryEnd: previous + resolution / 2 >= fullSegment.end - resolution / 2,
      });
      segmentStart = current - resolution / 2;
    }
    previous = current;
  }

  segments.push({
    start: clamp(segmentStart, fullSegment.start, fullSegment.end),
    end: clamp(previous + resolution / 2, fullSegment.start, fullSegment.end),
    boundaryStart: segmentStart <= fullSegment.start + resolution / 2,
    boundaryEnd: previous + resolution / 2 >= fullSegment.end - resolution / 2,
  });

  return segments.filter((segment) => segment.end >= segment.start);
}

function makeWaypoint(
  orientation: SweepOrientation,
  lane: number,
  passAxis: number,
  yaw: number,
): CleanAreaWaypoint {
  return orientation === "horizontal"
    ? { x: passAxis, y: lane, yaw }
    : { x: lane, y: passAxis, yaw };
}

function compensateGoalTolerance(
  segment: Segment,
  passAxis: number,
  forward: boolean,
  compensation: number,
): number {
  if (compensation <= 0) {
    return passAxis;
  }
  if (forward) {
    if (passAxis === segment.start && segment.boundaryStart) {
      return passAxis - compensation;
    }
    if (passAxis === segment.end && segment.boundaryEnd) {
      return passAxis + compensation;
    }
  } else {
    if (passAxis === segment.end && segment.boundaryEnd) {
      return passAxis + compensation;
    }
    if (passAxis === segment.start && segment.boundaryStart) {
      return passAxis - compensation;
    }
  }
  return passAxis;
}

function appendWaypoint(waypoints: CleanAreaWaypoint[], waypoint: CleanAreaWaypoint): void {
  const previous = waypoints[waypoints.length - 1];
  if (previous && nearlySamePoint(previous, waypoint)) {
    return;
  }
  waypoints.push(waypoint);
}

export function buildLawnmowerWaypoints(args: {
  rect: CleanAreaRect;
  spacing: number;
  swathWidth: number;
  boundaryExtensionM?: number;
  goalCompletionTolerance?: number;
  target?: CleanAreaCoverageTarget | null;
}): CleanAreaWaypoint[] {
  const width = Math.max(0, args.rect.maxX - args.rect.minX);
  const height = Math.max(0, args.rect.maxY - args.rect.minY);
  if (width <= 0 || height <= 0 || args.spacing <= 0 || args.swathWidth <= 0) {
    return [];
  }

  const orientation = chooseSweepOrientation(args.rect);
  const lanes = orientation === "horizontal"
    ? buildLaneCenters(args.rect.minY, args.rect.maxY, args.swathWidth, args.spacing)
    : buildLaneCenters(args.rect.minX, args.rect.maxX, args.swathWidth, args.spacing);
  const waypoints: CleanAreaWaypoint[] = [];
  const boundaryExtensionM = Math.max(0, args.boundaryExtensionM ?? args.goalCompletionTolerance ?? 0);

  lanes.forEach((lane, laneIndex) => {
    const forward = laneIndex % 2 === 0;
    const yaw = orientation === "horizontal"
      ? forward ? 0 : 180
      : forward ? 90 : -90;
    const segments = buildSegmentsForLane({
      rect: args.rect,
      target: args.target ?? null,
      orientation,
      lane,
      swathWidth: args.swathWidth,
      spacing: args.spacing,
    });
    const orderedSegments = forward ? segments : [...segments].reverse();

    for (const segment of orderedSegments) {
      const start = forward ? segment.start : segment.end;
      const end = forward ? segment.end : segment.start;
      if (Math.abs(end - start) < MIN_SEGMENT_LENGTH_M) {
        appendWaypoint(waypoints, makeWaypoint(orientation, lane, (start + end) / 2, yaw));
      } else {
        appendWaypoint(
          waypoints,
          makeWaypoint(orientation, lane, compensateGoalTolerance(segment, start, forward, boundaryExtensionM), yaw),
        );
        appendWaypoint(
          waypoints,
          makeWaypoint(orientation, lane, compensateGoalTolerance(segment, end, forward, boundaryExtensionM), yaw),
        );
      }
    }
  });

  return waypoints.filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y));
}
