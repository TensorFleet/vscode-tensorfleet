type GazeboEntityTarget = {
  target: string;
  params?: Record<string, unknown>;
};

export type PosePolicyEntity = GazeboEntityTarget & {
  type: string;
};

export type PoseEditAccess = {
  enabled: boolean;
  reason?: string;
};

const trimNonEmptyString = (value: unknown): string | undefined => {
  return typeof value === 'string' && value.trim().length > 0 ? value.trim() : undefined;
};

const uniqueStrings = (values: Array<string | undefined>): string[] => {
  return [...new Set(values.filter((value): value is string => Boolean(value && value.length > 0)))];
};

export const getGazeboEntityName = (entity: GazeboEntityTarget | null | undefined): string => {
  if (!entity) return '';
  const gazeboEntity = trimNonEmptyString(entity.params?.gazebo_entity);
  if (gazeboEntity) {
    return gazeboEntity;
  }
  const modelNames = entity.params?.model_names;
  if (Array.isArray(modelNames)) {
    const firstModelName = trimNonEmptyString(modelNames[0]);
    if (firstModelName) {
      return firstModelName;
    }
  }
  // Safety fallback for malformed cards missing model_names.
  return entity.target;
};

export const getRuntimePoseEntityName = (entity: GazeboEntityTarget | null | undefined): string => {
  if (!entity) return '';
  const runtimePoseEntity = trimNonEmptyString(entity.params?.runtime_pose_entity);
  if (runtimePoseEntity) {
    return runtimePoseEntity;
  }
  const poseEntity = trimNonEmptyString(entity.params?.pose_entity);
  if (poseEntity) {
    return poseEntity;
  }
  return getGazeboEntityName(entity);
};

export const getManipulationTargetName = (
  entity: GazeboEntityTarget | null | undefined,
): string => {
  return getRuntimePoseEntityName(entity);
};

export const getManipulationSelectionNames = (
  entity: GazeboEntityTarget | null | undefined,
): string[] => {
  if (!entity) return [];
  const modelNames = Array.isArray(entity.params?.model_names)
    ? entity.params?.model_names.map(trimNonEmptyString)
    : [];
  return uniqueStrings([
    getRuntimePoseEntityName(entity),
    getGazeboEntityName(entity),
    ...modelNames,
    trimNonEmptyString(entity.target),
  ]);
};

export const getPoseEditAccess = (entity: PosePolicyEntity | null): PoseEditAccess => {
  if (!entity) return { enabled: false };

  const editable = entity.params?.runtime_pose_editable;
  const policy = entity.params?.pose_edit_policy;
  const note = entity.params?.pose_edit_note;
  const gazeboEntity = getGazeboEntityName(entity);

  if (typeof editable === 'boolean') {
    return {
      enabled: editable,
      reason: !editable && typeof note === 'string' ? note : undefined,
    };
  }

  if (typeof policy === 'string' && policy.toLowerCase() === 'locked') {
    return {
      enabled: false,
      reason: typeof note === 'string' ? note : 'Pose edits are disabled for this entity.',
    };
  }

  // Fallback until every featured entity explicitly declares its pose policy.
  if (entity.type.toLowerCase() === 'arm' || gazeboEntity.startsWith('so101')) {
    return {
      enabled: false,
      reason: 'Arm base is fixed in this simulation, so runtime nudging is disabled.',
    };
  }

  return { enabled: true };
};
