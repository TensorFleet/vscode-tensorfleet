import {
  buildPoseNameAliases,
  capturePoseBaselines,
  GazeboPose,
  getUnscopedPoseName,
  isExpectedPoseObserved,
  quaternionAngularDistanceRad,
  PoseQuaternion,
  PoseVector,
  resolvePoseEntry,
  toPoseQuaternion,
  toPoseVector,
  unique,
} from './moveControl';

export type ManipulationStateEvent = {
  dragging: boolean;
  entity: string;
};

export type ManipulationCommitEvent = {
  name: string;
  position: PoseVector;
  orientation: PoseQuaternion;
};

export type ManipulationDispatchEvent = ManipulationCommitEvent & {
  ok: boolean;
  requestId?: string;
  world: string;
  serviceName?: string;
  msgType?: string;
  error?: string;
};

export type ManipulationServiceReplyEvent = {
  requestId: string;
  name: string;
  world: string;
  serviceName?: string;
  requestType?: string;
  responseType?: string;
  ok: boolean | null;
  detail?: string;
};

export type ManipulationPoseBinding = {
  requestedName: string;
  poseName: string;
  poseId?: number;
  aliases: string[];
  preferredPoseNames: string[];
  baselinePose: GazeboPose | null;
  baselineByName: Map<string, PoseVector>;
};

const selectObservedPoseCandidates = (
  poses: Map<string, GazeboPose>,
  binding: ManipulationPoseBinding,
): Array<{ name: string; pose: GazeboPose }> => {
  const allEntries = [...poses.entries()].map(([name, pose]) => ({ name, pose }));
  if (typeof binding.poseId === 'number') {
    const idMatches = allEntries.filter((entry) => entry.pose.id === binding.poseId);
    if (idMatches.length > 0) {
      return idMatches;
    }
  }

  const preferredNameSet = new Set(binding.preferredPoseNames);
  const preferredMatches = allEntries.filter((entry) => preferredNameSet.has(entry.name));
  if (preferredMatches.length > 0) {
    return preferredMatches;
  }

  const aliasSet = new Set(binding.aliases);
  return allEntries.filter((entry) => aliasSet.has(entry.name));
};

export const parseManipulationStateEvent = (
  payload: unknown,
): ManipulationStateEvent => {
  const candidate = (payload ?? {}) as { dragging?: unknown; entity?: unknown };
  const entity = typeof candidate.entity === 'string' ? candidate.entity.trim() : '';
  return {
    dragging: Boolean(candidate.dragging),
    entity,
  };
};

export const parseManipulationCommitEvent = (
  payload: unknown,
): ManipulationCommitEvent | null => {
  const candidate = (payload ?? {}) as {
    name?: unknown;
    position?: unknown;
    orientation?: unknown;
  };
  const name = typeof candidate.name === 'string' ? candidate.name.trim() : '';
  const position = toPoseVector(candidate.position);
  const orientation = toPoseQuaternion(candidate.orientation);
  if (!name || !position || !orientation) return null;
  return { name, position, orientation };
};

export const parseManipulationDispatchEvent = (
  payload: unknown,
): ManipulationDispatchEvent | null => {
  const candidate = (payload ?? {}) as {
    ok?: unknown;
    requestId?: unknown;
    world?: unknown;
    serviceName?: unknown;
    msgType?: unknown;
    error?: unknown;
    name?: unknown;
    position?: unknown;
    orientation?: unknown;
  };
  const commit = parseManipulationCommitEvent(candidate);
  const world = typeof candidate.world === 'string' ? candidate.world.trim() : '';
  if (!commit || !world) return null;
  return {
    ...commit,
    ok: Boolean(candidate.ok),
    requestId: typeof candidate.requestId === 'string' ? candidate.requestId : undefined,
    world,
    serviceName: typeof candidate.serviceName === 'string' ? candidate.serviceName : undefined,
    msgType: typeof candidate.msgType === 'string' ? candidate.msgType : undefined,
    error: typeof candidate.error === 'string' ? candidate.error : undefined,
  };
};

export const parseManipulationServiceReplyEvent = (
  payload: unknown,
): ManipulationServiceReplyEvent | null => {
  const candidate = (payload ?? {}) as {
    requestId?: unknown;
    name?: unknown;
    world?: unknown;
    serviceName?: unknown;
    requestType?: unknown;
    responseType?: unknown;
    ok?: unknown;
    detail?: unknown;
  };
  const requestId = typeof candidate.requestId === 'string' ? candidate.requestId.trim() : '';
  const name = typeof candidate.name === 'string' ? candidate.name.trim() : '';
  const world = typeof candidate.world === 'string' ? candidate.world.trim() : '';
  if (!requestId || !name || !world) return null;
  return {
    requestId,
    name,
    world,
    serviceName: typeof candidate.serviceName === 'string' ? candidate.serviceName : undefined,
    requestType: typeof candidate.requestType === 'string' ? candidate.requestType : undefined,
    responseType: typeof candidate.responseType === 'string' ? candidate.responseType : undefined,
    ok:
      typeof candidate.ok === 'boolean'
        ? candidate.ok
        : candidate.ok === null
          ? null
          : null,
    detail: typeof candidate.detail === 'string' ? candidate.detail : undefined,
  };
};

export const buildManipulationPoseBinding = (options: {
  poses: Map<string, GazeboPose>;
  world: string;
  requestedName: string;
}): ManipulationPoseBinding => {
  const resolvedPoseEntry = resolvePoseEntry(options.poses, options.requestedName);
  const poseName = resolvedPoseEntry?.poseName ?? options.requestedName;
  const worldScopedPoseName = poseName.includes('::') ? poseName : `${options.world}::${poseName}`;
  const unscopedPoseName = getUnscopedPoseName(poseName);
  const preferredPoseNames = unique([
    poseName,
    worldScopedPoseName,
    unscopedPoseName,
    options.requestedName,
  ]);
  const aliases = buildPoseNameAliases(options.world, preferredPoseNames);
  return {
    requestedName: options.requestedName,
    poseName,
    poseId: resolvedPoseEntry?.pose.id,
    aliases,
    preferredPoseNames,
    baselinePose: resolvedPoseEntry?.pose ?? null,
    baselineByName: capturePoseBaselines(options.poses, aliases, preferredPoseNames),
  };
};

export const resolveManipulationObservation = (options: {
  poses: Map<string, GazeboPose>;
  binding: ManipulationPoseBinding;
  targetPose: GazeboPose;
  toleranceMeters: number;
  orientationToleranceRad?: number;
  minObservedAtMs?: number;
}): {
  matched: boolean;
  matchedBy?: 'absolute' | 'delta';
  observed: { name: string; pose: GazeboPose } | null;
  distance: number | null;
  angularDistanceRad: number | null;
} => {
  const expectedDelta = options.binding.baselinePose
    ? {
        x: options.targetPose.position.x - options.binding.baselinePose.position.x,
        y: options.targetPose.position.y - options.binding.baselinePose.position.y,
        z: options.targetPose.position.z - options.binding.baselinePose.position.z,
      }
    : undefined;
  const observedCandidates = selectObservedPoseCandidates(options.poses, options.binding);
  const observation = isExpectedPoseObserved(
    observedCandidates.map(({ name, pose }) => ({
      name,
      position: pose.position,
      id: pose.id,
    })),
    new Set(options.binding.aliases),
    options.targetPose.position,
    options.toleranceMeters,
    options.binding.poseId,
    expectedDelta,
    options.binding.baselineByName,
  );
  const observed =
    (observation.matchedName
      ? (() => {
          const pose = options.poses.get(observation.matchedName!);
          return pose ? { name: observation.matchedName!, pose } : null;
        })()
      : null) ??
    observedCandidates[0] ??
    null;

  let distance: number | null = null;
  let angularDistanceRad: number | null = null;
  if (observed) {
    if (observation.matchedBy === 'delta' && expectedDelta) {
      const baseline = options.binding.baselineByName.get(observed.name);
      if (baseline) {
        const dx =
          observed.pose.position.x - baseline.x - expectedDelta.x;
        const dy =
          observed.pose.position.y - baseline.y - expectedDelta.y;
        const dz =
          observed.pose.position.z - baseline.z - expectedDelta.z;
        distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
      }
    }
    if (distance === null) {
      const dx = observed.pose.position.x - options.targetPose.position.x;
      const dy = observed.pose.position.y - options.targetPose.position.y;
      const dz = observed.pose.position.z - options.targetPose.position.z;
      distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    angularDistanceRad = quaternionAngularDistanceRad(
      observed.pose.orientation,
      options.targetPose.orientation,
    );
  }

  const orientationToleranceRad = options.orientationToleranceRad ?? Number.POSITIVE_INFINITY;
  const orientationMatched =
    angularDistanceRad === null || angularDistanceRad <= orientationToleranceRad;

  return {
    matched:
      observation.matched &&
      orientationMatched &&
      (!options.minObservedAtMs ||
        ((observed?.pose.observedAtMs ?? 0) >= options.minObservedAtMs)),
    matchedBy: observation.matchedBy,
    observed,
    distance,
    angularDistanceRad,
  };
};

type PoseConfirmationOptions = {
  isDisposed: () => boolean;
  getObservation: () => {
    matched: boolean;
    observed: { name: string; pose: GazeboPose } | null;
    distance: number | null;
    angularDistanceRad: number | null;
  };
  timeoutMs: number;
  onConfirmed: (observed: {
    name: string;
    pose: GazeboPose;
    distance: number;
    angularDistanceRad: number | null;
  }) => void;
  onSettledMismatch: (observed: {
    name: string;
    pose: GazeboPose;
    distance: number;
    angularDistanceRad: number | null;
  }) => void;
  onTimeout: (observedPose: GazeboPose | null) => void;
  pollIntervalMs?: number;
  initialDelayMs?: number;
  stabilityWindowMs?: number;
  stabilityToleranceMeters?: number;
};

export const pollForPoseConfirmation = (
  options: PoseConfirmationOptions,
): (() => void) => {
  const pollIntervalMs = options.pollIntervalMs ?? 100;
  const initialDelayMs = options.initialDelayMs ?? 120;
  const stabilityWindowMs = options.stabilityWindowMs ?? 400;
  const stabilityToleranceMeters = options.stabilityToleranceMeters ?? 0.0025;
  const startedAtMs = Date.now();
  let cancelled = false;
  let timeoutHandle: number | null = null;
  let lastObservedPosition:
    | { x: number; y: number; z: number }
    | null = null;
  let stableSinceMs: number | null = null;

  const poll = () => {
    if (cancelled || options.isDisposed()) return;
    const now = Date.now();
    const observation = options.getObservation();
    const observed = observation.observed;
    const observedPose = observed?.pose ?? null;
    if (observedPose) {
      const currentPosition = observedPose.position;
      if (!lastObservedPosition) {
        lastObservedPosition = { ...currentPosition };
        stableSinceMs = now;
      } else {
        const dx = currentPosition.x - lastObservedPosition.x;
        const dy = currentPosition.y - lastObservedPosition.y;
        const dz = currentPosition.z - lastObservedPosition.z;
        const movement = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (movement > stabilityToleranceMeters) {
          lastObservedPosition = { ...currentPosition };
          stableSinceMs = now;
        }
      }
    } else {
      lastObservedPosition = null;
      stableSinceMs = null;
    }
    if (observation.matched && observed && observedPose) {
      const distance = observation.distance ?? 0;
      options.onConfirmed({
        name: observed.name,
        pose: observedPose,
        distance,
        angularDistanceRad: observation.angularDistanceRad,
      });
      return;
    }
    if (now - startedAtMs >= options.timeoutMs) {
      if (
        observed &&
        observedPose &&
        stableSinceMs !== null &&
        now - stableSinceMs >= stabilityWindowMs
      ) {
        options.onSettledMismatch({
          name: observed.name,
          pose: observedPose,
          distance: observation.distance ?? Number.POSITIVE_INFINITY,
          angularDistanceRad: observation.angularDistanceRad,
        });
        return;
      }
      options.onTimeout(observedPose);
      return;
    }
    timeoutHandle = window.setTimeout(poll, pollIntervalMs);
  };

  timeoutHandle = window.setTimeout(poll, initialDelayMs);
  return () => {
    cancelled = true;
    if (timeoutHandle !== null) {
      window.clearTimeout(timeoutHandle);
      timeoutHandle = null;
    }
  };
};
