import { describe, expect, it } from 'bun:test';
import {
  getGazeboEntityName,
  getManipulationSelectionNames,
  getManipulationTargetName,
  getPoseEditAccess,
  getRuntimePoseEntityName,
} from './posePolicy';
import { humanizeEntityName } from 'tensorfleet-util/ros/fetchFeaturedEntities';

describe('posePolicy', () => {
  it('uses the first model_names entry as the canonical entity name', () => {
    expect(
      getGazeboEntityName({
        target: 'fallback_target',
        params: {
          model_names: ['simple_bot', 'simple_bot_include'],
        },
      }),
    ).toBe('simple_bot');
  });

  it('uses first model_names entry as fallback gazebo entity name', () => {
    expect(
      getGazeboEntityName({
        target: 'fallback_target',
        params: { model_names: ['x500_0', 'x500_0::base_link'] },
      }),
    ).toBe('x500_0');
  });

  it('falls back to target when model_names is absent', () => {
    expect(
      getRuntimePoseEntityName({
        target: 'fallback_target',
        params: {},
      }),
    ).toBe('fallback_target');
  });

  it('uses the canonical model name as the primary manipulation target', () => {
    expect(
      getManipulationTargetName({
        target: 'Room_Essentials_Mug_White_Yellow',
        params: {
          model_names: ['Room_Essentials_Mug_White_Yellow'],
        },
      }),
    ).toBe('Room_Essentials_Mug_White_Yellow');
  });

  it('includes canonical and alias model names when resolving manipulation selection names', () => {
    expect(
      getManipulationSelectionNames({
        target: 'Room_Essentials_Mug_White_Yellow',
        params: {
          model_names: ['Room_Essentials_Mug_White_Yellow', 'room_essentials_mug_white_yellow'],
        },
      }),
    ).toEqual([
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

  it('humanizes canonical runtime names for UI labels', () => {
    expect(humanizeEntityName('mug')).toBe('Mug');
    expect(humanizeEntityName('x500_0')).toBe('X500 0');
    expect(humanizeEntityName('so-arm101')).toBe('SO ARM101');
  });
});
