import { EntityCardData } from './EntityCardData';
import { getGazeboEntityName, getRuntimePoseEntityName } from './posePolicy';

export type PoseVector = { x: number; y: number; z: number };
export type PoseQuaternion = { x: number; y: number; z: number; w: number };
export type GazeboPose = {
  name: string;
  position: PoseVector;
  orientation: PoseQuaternion;
  id?: number;
  observedAtMs?: number;
};

export const unique = (items: Array<string | undefined>) => {
  return [...new Set(items.filter((value): value is string => Boolean(value && value.trim().length > 0)))];
};

const normalizeEntityAlias = (value: string): string | undefined => {
  const trimmed = value.trim();
  if (!trimmed) return undefined;
  const normalized = trimmed
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, '_')
    .replace(/^_+|_+$/g, '');
  return normalized.length > 0 ? normalized : undefined;
};

export const toPoseVector = (value: unknown): PoseVector | null => {
  if (!value || typeof value !== 'object') return null;
  const vector = value as Partial<PoseVector>;
  if (typeof vector.x !== 'number' || typeof vector.y !== 'number' || typeof vector.z !== 'number') {
    return null;
  }
  return { x: vector.x, y: vector.y, z: vector.z };
};

export const toPoseQuaternion = (value: unknown): PoseQuaternion | null => {
  if (!value || typeof value !== 'object') return null;
  const q = value as Partial<PoseQuaternion>;
  if (typeof q.x !== 'number' || typeof q.y !== 'number' || typeof q.z !== 'number' || typeof q.w !== 'number') {
    return null;
  }
  return { x: q.x, y: q.y, z: q.z, w: q.w };
};

export const addPoseVector = (a: PoseVector, b: PoseVector): PoseVector => ({
  x: a.x + b.x,
  y: a.y + b.y,
  z: a.z + b.z,
});

export const isFinitePoseVector = (value: PoseVector): boolean => {
  return Number.isFinite(value.x) && Number.isFinite(value.y) && Number.isFinite(value.z);
};

export const poseVectorMagnitude = (value: PoseVector): number => {
  return Math.sqrt((value.x * value.x) + (value.y * value.y) + (value.z * value.z));
};

export const roundPoseVector = (value: PoseVector, decimals = 4): PoseVector => {
  const factor = Math.pow(10, decimals);
  return {
    x: Math.round(value.x * factor) / factor,
    y: Math.round(value.y * factor) / factor,
    z: Math.round(value.z * factor) / factor,
  };
};

export const quaternionAngularDistanceRad = (
  observed: PoseQuaternion,
  expected: PoseQuaternion,
): number => {
  const dot = Math.abs(
    observed.x * expected.x +
    observed.y * expected.y +
    observed.z * expected.z +
    observed.w * expected.w,
  );
  const clampedDot = Math.min(1, Math.max(-1, dot));
  return 2 * Math.acos(clampedDot);
};

export const resolveVisualOffset = (
  offsets: Map<string, PoseVector>,
  world: string,
  poseName: string,
): PoseVector | null => {
  const unscopedName = poseName.includes('::')
    ? poseName.split('::').slice(1).join('::')
    : undefined;
  return (
    offsets.get(poseName) ??
    offsets.get(`${world}::${poseName}`) ??
    (unscopedName ? offsets.get(unscopedName) : undefined) ??
    null
  );
};

export const getEntityNameCandidates = (entity: EntityCardData): string[] => {
  const runtimeTarget = getRuntimePoseEntityName(entity);
  const mapped = getGazeboEntityName(entity);
  const runtimeBase = runtimeTarget.endsWith('_include')
    ? runtimeTarget.slice(0, -'_include'.length)
    : undefined;
  const includeBase = mapped.endsWith('_include')
    ? mapped.slice(0, -'_include'.length)
    : undefined;
  const targetBase = entity.target.endsWith('_include')
    ? entity.target.slice(0, -'_include'.length)
    : undefined;
  const runtimeNestedInclude = runtimeBase ? `${runtimeTarget}::${runtimeBase}` : undefined;
  const mappedNestedInclude = includeBase ? `${mapped}::${includeBase}` : undefined;
  const targetNestedInclude = targetBase ? `${entity.target}::${targetBase}` : undefined;
  const objectAliases = entity.type.toLowerCase() === 'object'
    ? unique([
      normalizeEntityAlias(entity.name),
    ])
    : [];
  return unique([
    ...objectAliases,
    runtimeTarget,
    runtimeBase,
    runtimeNestedInclude,
    mapped,
    includeBase,
    mappedNestedInclude,
    entity.target,
    targetBase,
    targetNestedInclude,
    entity.name,
  ]);
};

export const resolvePoseEntry = (
  poses: Map<string, GazeboPose>,
  requestedName: string,
): { poseName: string; pose: GazeboPose } | null => {
  const direct = poses.get(requestedName);
  if (direct) {
    return { poseName: requestedName, pose: direct };
  }

  const candidates: Array<{ poseName: string; pose: GazeboPose; score: number }> = [];

  for (const [poseName, pose] of poses.entries()) {
    let score = -1;
    if (poseName.endsWith(`::${requestedName}`)) {
      // world::model
      score = 30;
    } else if (poseName.startsWith(`${requestedName}::`)) {
      // model::link
      score = 20;
    } else if (poseName.includes(`::${requestedName}::`)) {
      // world::model::link
      score = 10;
    }
    if (score >= 0) {
      candidates.push({ poseName, pose, score });
    }
  }

  if (candidates.length === 0) return null;
  candidates.sort((a, b) => {
    if (a.score !== b.score) return b.score - a.score;
    return a.poseName.length - b.poseName.length;
  });
  return { poseName: candidates[0].poseName, pose: candidates[0].pose };
};

export const getUnscopedPoseName = (name: string): string | undefined => {
  if (!name.includes('::')) return undefined;
  return name.split('::').slice(1).join('::');
};

export const buildPoseNameAliases = (world: string, poseNames: string[]): string[] => {
  return unique([
    ...poseNames,
    ...poseNames.map((name) => (name.includes('::') ? undefined : `${world}::${name}`)),
    ...poseNames.map(getUnscopedPoseName),
  ]);
};

const scoreObservedPoseName = (
  poseName: string,
  aliases: Set<string>,
  preferredNames: Set<string>,
): number => {
  if (preferredNames.has(poseName)) return 1000;

  for (const alias of aliases) {
    if (poseName === alias) return 900;
    if (poseName.endsWith(`::${alias}`) || poseName.endsWith(`/${alias}`)) {
      return 800;
    }
    if (poseName.startsWith(`${alias}::`) || poseName.startsWith(`${alias}/`)) {
      return 500;
    }
    if (poseName.includes(`::${alias}::`) || poseName.includes(`/${alias}/`)) {
      return 400;
    }
  }

  return -1;
};

export const resolveObservedPoseEntry = (
  poses: Map<string, GazeboPose>,
  aliases: string[],
  preferredNames: string[] = [],
): { name: string; pose: GazeboPose } | null => {
  const aliasSet = new Set(aliases);
  const preferredSet = new Set(preferredNames);
  let best: { name: string; pose: GazeboPose; score: number } | null = null;

  for (const [poseName, pose] of poses.entries()) {
    const score = scoreObservedPoseName(poseName, aliasSet, preferredSet);
    if (score < 0) continue;
    if (
      !best ||
      score > best.score ||
      (score === best.score && poseName.length < best.name.length)
    ) {
      best = { name: poseName, pose, score };
    }
  }

  return best ? { name: best.name, pose: best.pose } : null;
};

export const capturePoseBaselines = (
  poses: Map<string, GazeboPose>,
  aliases: string[],
  preferredNames: string[] = [],
): Map<string, PoseVector> => {
  const baselineByName = new Map<string, PoseVector>();
  const preferred = resolveObservedPoseEntry(poses, aliases, preferredNames);
  if (preferred) {
    baselineByName.set(preferred.name, { ...preferred.pose.position });
  }
  for (const [poseName, pose] of poses.entries()) {
    if (!poseNameMatchesAliases(poseName, new Set(aliases))) continue;
    baselineByName.set(poseName, { ...pose.position });
  }
  return baselineByName;
};

export const isPoseWithinTolerance = (
  observed: PoseVector,
  expected: PoseVector,
  toleranceMeters: number,
): boolean => {
  const dx = observed.x - expected.x;
  const dy = observed.y - expected.y;
  const dz = observed.z - expected.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz) <= toleranceMeters;
};

const toOptionalNumber = (value: unknown): number | undefined => {
  if (typeof value === 'number' && Number.isFinite(value)) return value;
  if (typeof value === 'string') {
    const parsed = Number(value);
    if (Number.isFinite(parsed)) return parsed;
  }
  if (value && typeof value === 'object') {
    const maybeWithToNumber = value as { toNumber?: () => number };
    if (typeof maybeWithToNumber.toNumber === 'function') {
      const n = maybeWithToNumber.toNumber();
      if (Number.isFinite(n)) return n;
    }
    const asRecord = value as Record<string, unknown>;
    if (typeof asRecord.low === 'number') {
      return asRecord.low;
    }
  }
  return undefined;
};

export const poseNameMatchesAliases = (poseName: string, aliases: Set<string>): boolean => {
  if (aliases.has(poseName)) return true;
  for (const alias of aliases) {
    if (
      poseName.endsWith(`::${alias}`) ||
      poseName.startsWith(`${alias}::`) ||
      poseName.includes(`::${alias}::`) ||
      poseName.endsWith(`/${alias}`) ||
      poseName.startsWith(`${alias}/`) ||
      poseName.includes(`/${alias}/`)
    ) {
      return true;
    }
  }
  return false;
};

export const isExpectedPoseObserved = (
  poseEntries: Array<{ name?: string; position?: unknown; id?: unknown }>,
  poseAliases: Set<string>,
  expected: PoseVector,
  toleranceMeters: number,
  expectedPoseId?: number,
  expectedDelta?: PoseVector,
  baselineByName?: Map<string, PoseVector>,
): { matched: boolean; matchedName?: string; matchedBy?: 'absolute' | 'delta' } => {
  for (const poseEntry of poseEntries) {
    const observedId = toOptionalNumber(poseEntry.id);
    const idMatch =
      typeof expectedPoseId === 'number' &&
      typeof observedId === 'number' &&
      observedId === expectedPoseId;
    const nameMatch =
      typeof poseEntry.name === 'string' &&
      poseNameMatchesAliases(poseEntry.name, poseAliases);

    if (!idMatch && !nameMatch) continue;
    const observed = toPoseVector(poseEntry.position);
    if (!observed) continue;
    if (isPoseWithinTolerance(observed, expected, toleranceMeters)) {
      return { matched: true, matchedName: poseEntry.name, matchedBy: 'absolute' };
    }

    if (expectedDelta && poseEntry.name && baselineByName) {
      const baseline = baselineByName.get(poseEntry.name);
      if (baseline) {
        const observedDelta = {
          x: observed.x - baseline.x,
          y: observed.y - baseline.y,
          z: observed.z - baseline.z,
        };
        if (isPoseWithinTolerance(observedDelta, expectedDelta, toleranceMeters)) {
          return { matched: true, matchedName: poseEntry.name, matchedBy: 'delta' };
        }
      }
    }
  }
  return { matched: false };
};
