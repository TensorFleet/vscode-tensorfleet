import { describe, expect, it } from 'bun:test';
import {
  getGazeboEntityName,
  getManipulationSelectionNames,
  getManipulationTargetName,
  getPoseEditAccess,
  getRuntimePoseEntityName,
} from './posePolicy';

describe('posePolicy', () => {
  it('prefers canonical gazebo_entity over model_names aliases', () => {
    expect(
      getGazeboEntityName({
        target: 'fallback_target',
        params: {
          gazebo_entity: 'simple_bot_include',
          model_names: ['simple_bot', 'simple_bot_include'],
        },
      }),
    ).toBe('simple_bot_include');
  });

  it('uses first model_names entry as fallback gazebo entity name', () => {
    expect(
      getGazeboEntityName({
        target: 'fallback_target',
        params: { model_names: ['x500_0', 'x500_0::base_link'] },
      }),
    ).toBe('x500_0');
  });

  it('prefers explicit runtime pose entity over gazebo entity', () => {
    expect(
      getRuntimePoseEntityName({
        target: 'fallback_target',
        params: {
          runtime_pose_entity: 'mug',
          gazebo_entity: 'Room_Essentials_Mug_White_Yellow',
        },
      }),
    ).toBe('mug');
  });

  it('uses runtime pose entity as the primary manipulation target', () => {
    expect(
      getManipulationTargetName({
        target: 'Room_Essentials_Mug_White_Yellow',
        params: {
          runtime_pose_entity: 'mug',
          gazebo_entity: 'Room_Essentials_Mug_White_Yellow',
          model_names: ['Room_Essentials_Mug_White_Yellow'],
        },
      }),
    ).toBe('mug');
  });

  it('includes both runtime and asset aliases when resolving manipulation selection names', () => {
    expect(
      getManipulationSelectionNames({
        target: 'Room_Essentials_Mug_White_Yellow',
        params: {
          runtime_pose_entity: 'mug',
          gazebo_entity: 'Room_Essentials_Mug_White_Yellow',
          model_names: ['Room_Essentials_Mug_White_Yellow', 'room_essentials_mug_white_yellow'],
        },
      }),
    ).toEqual([
      'mug',
      'Room_Essentials_Mug_White_Yellow',
      'room_essentials_mug_white_yellow',
    ]);
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
