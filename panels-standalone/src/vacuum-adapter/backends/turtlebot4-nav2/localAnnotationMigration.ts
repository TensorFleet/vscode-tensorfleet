import type { VacuumMapAnnotation } from "../../state";

const LOCAL_MAP_ANNOTATION_KEY_PREFIX = "tensorfleet:vacuums:turtlebot4-nav2:map-annotations:";
const LOCAL_MAP_ANNOTATION_MIGRATION_KEY_PREFIX = "tensorfleet:vacuums:turtlebot4-nav2:map-annotations-migrated:";

function toFiniteNumber(value: unknown): number | null {
  const numeric = typeof value === "string" ? Number(value) : value;
  return typeof numeric === "number" && Number.isFinite(numeric) ? numeric : null;
}

function parseMapAnnotation(value: unknown): VacuumMapAnnotation | null {
  const record = value && typeof value === "object" ? (value as Partial<VacuumMapAnnotation>) : null;
  if (
    !record ||
    typeof record.id !== "string" ||
    (record.kind !== "room" && record.kind !== "zone") ||
    typeof record.name !== "string" ||
    !record.area ||
    typeof record.area !== "object"
  ) {
    return null;
  }
  return {
    id: record.id,
    kind: record.kind,
    name: record.name,
    area: record.area as VacuumMapAnnotation["area"],
    mapId: typeof record.mapId === "string" ? record.mapId : null,
    createdAt: toFiniteNumber(record.createdAt) ?? Date.now(),
    updatedAt: toFiniteNumber(record.updatedAt) ?? Date.now(),
  };
}

function localMapAnnotationStorageKey(mapId: string): string {
  return `${LOCAL_MAP_ANNOTATION_KEY_PREFIX}${mapId}`;
}

function localMapAnnotationMigrationKey(mapId: string): string {
  return `${LOCAL_MAP_ANNOTATION_MIGRATION_KEY_PREFIX}${mapId}`;
}

export function readLocalPrototypeMapAnnotations(mapId: string): VacuumMapAnnotation[] {
  if (typeof window === "undefined") {
    return [];
  }
  try {
    const raw = window.localStorage.getItem(localMapAnnotationStorageKey(mapId));
    if (!raw) {
      return [];
    }
    const parsed = JSON.parse(raw);
    const rawAnnotations = Array.isArray(parsed)
      ? parsed
      : parsed && typeof parsed === "object" && Array.isArray((parsed as { annotations?: unknown }).annotations)
        ? (parsed as { annotations: unknown[] }).annotations
        : null;
    if (!rawAnnotations) {
      return [];
    }
    const annotations = rawAnnotations.flatMap((entry) => {
      const annotation = parseMapAnnotation(entry);
      if (!annotation || (annotation.mapId != null && annotation.mapId !== mapId)) {
        return [];
      }
      return [{ ...annotation, mapId }];
    });
    annotations.sort((a, b) => a.name.localeCompare(b.name));
    return annotations;
  } catch {
    return [];
  }
}

export function hasMigratedLocalPrototypeMapAnnotations(mapId: string): boolean {
  if (typeof window === "undefined") {
    return true;
  }
  try {
    return window.localStorage.getItem(localMapAnnotationMigrationKey(mapId)) === "true";
  } catch {
    return true;
  }
}

export function markLocalPrototypeMapAnnotationsMigrated(mapId: string): void {
  if (typeof window === "undefined") {
    return;
  }
  try {
    window.localStorage.setItem(localMapAnnotationMigrationKey(mapId), "true");
  } catch {
    // Migration markers are best effort; runtime state remains authoritative.
  }
}
