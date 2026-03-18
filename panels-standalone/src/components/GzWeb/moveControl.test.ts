import { describe, expect, it } from 'bun:test';
import { EntityCardData } from './EntityCardData';
import {
  addPoseVector,
  buildPoseNameAliases,
  capturePoseBaselines,
  getEntityNameCandidates,
  getUnscopedPoseName,
  isFinitePoseVector,
  isExpectedPoseObserved,
  poseVectorMagnitude,
  quaternionAngularDistanceRad,
  roundPoseVector,
  resolveObservedPoseEntry,
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

const sampleObjectEntity: EntityCardData = {
  name: 'Mug',
  type: 'object',
  target: 'Room_Essentials_Mug_White_Yellow',
  params: {
    model_names: ['Room_Essentials_Mug_White_Yellow'],
  },
};

const sampleObjectWithAliasNames: EntityCardData = {
  name: 'Mug',
  type: 'object',
  target: 'Room_Essentials_Mug_White_Yellow',
  params: {
    model_names: ['Room_Essentials_Mug_White_Yellow', 'mug'],
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

  it('prefers normalized display aliases first for movable objects', () => {
    expect(getEntityNameCandidates(sampleObjectEntity)).toEqual([
      'mug',
      'Room_Essentials_Mug_White_Yellow',
      'Mug',
    ]);
  });

  it('accepts additional model-name aliases after the canonical model name', () => {
    expect(getEntityNameCandidates(sampleObjectWithAliasNames)).toEqual([
      'mug',
      'Room_Essentials_Mug_White_Yellow',
      'Mug',
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
    expect(observed).toEqual({ matched: true, matchedName: 'world::x500_0', matchedBy: 'absolute' });
  });

  it('accepts world/model/link pose names for the same alias', () => {
    const observed = isExpectedPoseObserved(
      [{ name: 'empty_world::x500_0::base_link', position: { x: 5, y: 1, z: 0 } }],
      new Set(['x500_0']),
      { x: 5, y: 1, z: 0 },
      0.25,
    );
    expect(observed).toEqual({ matched: true, matchedName: 'empty_world::x500_0::base_link', matchedBy: 'absolute' });
  });

  it('accepts slash-delimited pose names for the same alias', () => {
    const observed = isExpectedPoseObserved(
      [{ name: 'empty_world/x500_0/base_link', position: { x: 5, y: 1, z: 0 } }],
      new Set(['x500_0']),
      { x: 5, y: 1, z: 0 },
      0.25,
    );
    expect(observed).toEqual({ matched: true, matchedName: 'empty_world/x500_0/base_link', matchedBy: 'absolute' });
  });

  it('accepts pose id matches even when name differs', () => {
    const observed = isExpectedPoseObserved(
      [{ name: 'some_other_name', position: { x: 3, y: 0, z: 0 }, id: 76 }],
      new Set(['x500_0']),
      { x: 3, y: 0, z: 0 },
      0.25,
      76,
    );
    expect(observed).toEqual({ matched: true, matchedName: 'some_other_name', matchedBy: 'absolute' });
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
    expect(observed).toEqual({ matched: true, matchedName: 'empty_world::x500_0::base_link', matchedBy: 'delta' });
  });

  it('prefers canonical model poses over nested link aliases', () => {
    const poses = new Map([
      ['empty_world::plate::base_link', {
        name: 'empty_world::plate::base_link',
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      }],
      ['empty_world::plate', {
        name: 'empty_world::plate',
        position: { x: 4, y: 5, z: 6 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      }],
    ]);

    expect(resolveObservedPoseEntry(poses, ['plate', 'empty_world::plate'], ['empty_world::plate'])).toEqual({
      name: 'empty_world::plate',
      pose: {
        name: 'empty_world::plate',
        position: { x: 4, y: 5, z: 6 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
    });
  });

  it('captures baselines for both canonical and nested alias matches', () => {
    const poses = new Map([
      ['empty_world::plate', {
        name: 'empty_world::plate',
        position: { x: 1, y: 2, z: 3 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      }],
      ['empty_world::plate::base_link', {
        name: 'empty_world::plate::base_link',
        position: { x: 0.1, y: 0.2, z: 0.3 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      }],
    ]);

    expect([...capturePoseBaselines(poses, ['plate'], ['empty_world::plate']).entries()]).toEqual([
      ['empty_world::plate', { x: 1, y: 2, z: 3 }],
      ['empty_world::plate::base_link', { x: 0.1, y: 0.2, z: 0.3 }],
    ]);
  });

  it('extracts unscoped pose names from world-scoped entries', () => {
    expect(getUnscopedPoseName('empty_world::plate')).toBe('plate');
    expect(getUnscopedPoseName('plate')).toBeUndefined();
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

  it('computes quaternion angular distance independent of sign', () => {
    expect(
      quaternionAngularDistanceRad(
        { x: 0, y: 0, z: 0, w: 1 },
        { x: 0, y: 0, z: 0, w: -1 },
      ),
    ).toBe(0);

    expect(
      quaternionAngularDistanceRad(
        { x: 0, y: 0, z: 0, w: 1 },
        { x: 0, y: Math.sin(Math.PI / 4), z: 0, w: Math.cos(Math.PI / 4) },
      ),
    ).toBeCloseTo(Math.PI / 2, 6);
  });
});
