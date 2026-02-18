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

export const getGazeboEntityName = (entity: GazeboEntityTarget | null | undefined): string => {
  if (!entity) return '';
  const mapped = entity.params?.gazebo_entity;
  if (typeof mapped === 'string' && mapped.trim().length > 0) {
    return mapped.trim();
  }
  return entity.target;
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
