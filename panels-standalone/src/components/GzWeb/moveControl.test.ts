import { describe, expect, it } from 'bun:test';
import { EntityCardData } from './EntityCardData';
import {
  addPoseVector,
  buildPoseNameAliases,
  getEntityNameCandidates,
  isFinitePoseVector,
  isExpectedPoseObserved,
  poseVectorMagnitude,
  roundPoseVector,
  resolvePoseEntry,
} from './moveControl';

const sampleEntity: EntityCardData = {
  name: 'X500',
  type: 'drone',
  target: 'x500_0_include',
  params: {
    model_names: ['x500_0_include'],
  },
};

describe('moveControl', () => {
  it('builds candidate names including _include-stripped variants', () => {
    expect(getEntityNameCandidates(sampleEntity)).toEqual([
      'x500_0_include',
      'x500_0',
      'x500_0_include::x500_0',
      'X500',
    ]);
  });

  it('prefers world model match when resolving pose entries', () => {
    const poses = new Map([
      ['world::x500_0', { name: 'world::x500_0', position: { x: 0, y: 0, z: 1 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }],
      ['x500_0::link', { name: 'x500_0::link', position: { x: 0, y: 0, z: 1 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }],
    ]);
    const resolved = resolvePoseEntry(poses, 'x500_0');
    expect(resolved?.poseName).toBe('world::x500_0');
  });

  it('smoke: move request is observed once dynamic pose reaches expected position', () => {
    const initialPose = {
      name: 'world::x500_0',
      position: { x: 1, y: 2, z: 3 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    };
    const nextPosition = addPoseVector(initialPose.position, { x: 4, y: 0, z: 0 });
    const aliases = buildPoseNameAliases('world', ['world::x500_0', 'x500_0']);

    const observed = isExpectedPoseObserved(
      [{ name: 'world::x500_0', position: nextPosition }],
      new Set(aliases),
      nextPosition,
      0.25,
    );
    expect(observed).toEqual({ matched: true, matchedName: 'world::x500_0' });
  });

  it('accepts world/model/link pose names for the same alias', () => {
    const observed = isExpectedPoseObserved(
      [{ name: 'empty_world::x500_0::base_link', position: { x: 5, y: 1, z: 0 } }],
      new Set(['x500_0']),
      { x: 5, y: 1, z: 0 },
      0.25,
    );
    expect(observed).toEqual({ matched: true, matchedName: 'empty_world::x500_0::base_link' });
  });

  it('accepts slash-delimited pose names for the same alias', () => {
    const observed = isExpectedPoseObserved(
      [{ name: 'empty_world/x500_0/base_link', position: { x: 5, y: 1, z: 0 } }],
      new Set(['x500_0']),
      { x: 5, y: 1, z: 0 },
      0.25,
    );
    expect(observed).toEqual({ matched: true, matchedName: 'empty_world/x500_0/base_link' });
  });

  it('accepts pose id matches even when name differs', () => {
    const observed = isExpectedPoseObserved(
      [{ name: 'some_other_name', position: { x: 3, y: 0, z: 0 }, id: 76 }],
      new Set(['x500_0']),
      { x: 3, y: 0, z: 0 },
      0.25,
      76,
    );
    expect(observed).toEqual({ matched: true, matchedName: 'some_other_name' });
  });

  it('confirms by delta-from-baseline for link frames with shifted origins', () => {
    const observed = isExpectedPoseObserved(
      [{ name: 'empty_world::x500_0::base_link', position: { x: 10.5, y: 0.5, z: 1 } }],
      new Set(['x500_0']),
      { x: 12, y: 0, z: 1 },
      0.25,
      undefined,
      { x: 4, y: 0, z: 0 },
      new Map([
        ['empty_world::x500_0::base_link', { x: 6.5, y: 0.5, z: 1 }],
      ]),
    );
    expect(observed).toEqual({ matched: true, matchedName: 'empty_world::x500_0::base_link' });
  });

  it('does not confirm move when pose update is outside tolerance', () => {
    const observed = isExpectedPoseObserved(
      [{ name: 'world::x500_0', position: { x: 0, y: 0, z: 0 } }],
      new Set(['world::x500_0']),
      { x: 10, y: 0, z: 0 },
      0.25,
    );
    expect(observed.matched).toBe(false);
  });

  it('validates finite move vectors', () => {
    expect(isFinitePoseVector({ x: 1, y: 2, z: 3 })).toBe(true);
    expect(isFinitePoseVector({ x: Number.NaN, y: 2, z: 3 })).toBe(false);
    expect(isFinitePoseVector({ x: 1, y: Number.POSITIVE_INFINITY, z: 3 })).toBe(false);
  });

  it('computes vector magnitude', () => {
    expect(poseVectorMagnitude({ x: 3, y: 4, z: 0 })).toBe(5);
    expect(poseVectorMagnitude({ x: 0, y: 0, z: 0 })).toBe(0);
  });

  it('rounds vectors for stable dispatch payloads', () => {
    expect(roundPoseVector({ x: 1.23456, y: -2.34567, z: 0.00009 })).toEqual({
      x: 1.2346,
      y: -2.3457,
      z: 0.0001,
    });
  });
});
