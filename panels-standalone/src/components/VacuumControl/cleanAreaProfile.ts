import type { MapCanvasMetadata } from "./MapCanvas";

export type CleanAreaCoverageProfile = {
  id: string;
  label: string;
  cleaningSwathWidthM: number;
  desiredOverlapRatio: number;
  navigationGoalToleranceM: number;
  boundarySafetyMarginM: number;
  safetyMarginNearBoundariesM: number;
  minimumUsefulCleanableRegionSqM: number;
  completionThreshold: number;
  obstacleClearancePreferenceM: number;
};

export type CleanAreaCoverageRuntimeConfig = CleanAreaCoverageProfile & {
  laneSpacingM: number;
  boundaryExtensionM: number;
};

export const DEFAULT_CLEAN_AREA_COVERAGE_PROFILE: CleanAreaCoverageProfile = {
  id: "adapter-default",
  label: "Adapter default",
  cleaningSwathWidthM: 0.3,
  desiredOverlapRatio: 0.6,
  navigationGoalToleranceM: 0.25,
  boundarySafetyMarginM: 0.03,
  safetyMarginNearBoundariesM: 0,
  minimumUsefulCleanableRegionSqM: 0.01,
  completionThreshold: 0.95,
  obstacleClearancePreferenceM: 0,
};

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

export function buildCleanAreaCoverageRuntimeConfig(args: {
  profile?: CleanAreaCoverageProfile;
  mapMetadata: MapCanvasMetadata | null;
}): CleanAreaCoverageRuntimeConfig {
  const profile = args.profile ?? DEFAULT_CLEAN_AREA_COVERAGE_PROFILE;
  const swathWidth = Math.max(0.01, profile.cleaningSwathWidthM);
  const overlapRatio = clamp(profile.desiredOverlapRatio, 0, 0.9);
  const overlapSpacing = swathWidth * (1 - overlapRatio);
  const resolution = args.mapMetadata?.resolution ?? 0;
  const cellAwareSpacing = resolution > 0 ? resolution * 1.5 : 0;

  return {
    ...profile,
    cleaningSwathWidthM: swathWidth,
    desiredOverlapRatio: overlapRatio,
    navigationGoalToleranceM: Math.max(0, profile.navigationGoalToleranceM),
    boundarySafetyMarginM: Math.max(0, profile.boundarySafetyMarginM),
    safetyMarginNearBoundariesM: Math.max(0, profile.safetyMarginNearBoundariesM),
    minimumUsefulCleanableRegionSqM: Math.max(0, profile.minimumUsefulCleanableRegionSqM),
    completionThreshold: clamp(profile.completionThreshold, 0.5, 1),
    obstacleClearancePreferenceM: Math.max(0, profile.obstacleClearancePreferenceM),
    laneSpacingM: clamp(Math.max(overlapSpacing, cellAwareSpacing), 0.08, swathWidth * 0.6),
    boundaryExtensionM: Math.max(0, profile.navigationGoalToleranceM + profile.boundarySafetyMarginM),
  };
}
