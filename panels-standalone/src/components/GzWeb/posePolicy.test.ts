import { describe, expect, it } from 'bun:test';
import { getGazeboEntityName, getPoseEditAccess } from './posePolicy';

describe('posePolicy', () => {
  it('uses first model_names entry as gazebo entity name', () => {
    expect(
      getGazeboEntityName({
        target: 'fallback_target',
        params: { model_names: ['x500_0', 'x500_0::base_link'] },
      }),
    ).toBe('x500_0');
  });

  it('respects runtime_pose_editable false with note', () => {
    expect(
      getPoseEditAccess({
        type: 'drone',
        target: 'x500_0',
        params: {
          runtime_pose_editable: false,
          pose_edit_note: 'locked by runtime policy',
        },
      }),
    ).toEqual({
      enabled: false,
      reason: 'locked by runtime policy',
    });
  });

  it('locks entities when policy is locked', () => {
    expect(
      getPoseEditAccess({
        type: 'drone',
        target: 'x500_0',
        params: {
          pose_edit_policy: 'locked',
          pose_edit_note: 'fixed-base model',
        },
      }),
    ).toEqual({
      enabled: false,
      reason: 'fixed-base model',
    });
  });

  it('falls back to locking arm entities without explicit policy', () => {
    expect(
      getPoseEditAccess({
        type: 'arm',
        target: 'so101_robot',
        params: {},
      }),
    ).toEqual({
      enabled: false,
      reason: 'Arm base is fixed in this simulation, so runtime nudging is disabled.',
    });
  });

  it('allows move controls when no lock rule applies', () => {
    expect(
      getPoseEditAccess({
        type: 'drone',
        target: 'x500_0',
        params: {},
      }),
    ).toEqual({ enabled: true });
  });
});
