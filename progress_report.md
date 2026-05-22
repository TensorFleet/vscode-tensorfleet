# Progress Report — Local Storage Migration / Fallback Cleanup

Current report date: May 22, 2026.

## 1. What changed

Existing prototype room/zone annotations stored in webview local storage now have a safe migration path into VM/runtime-owned annotation persistence.

When the TurtleBot4/Nav2 adapter hydrates annotations for the active map, it checks runtime state first. If the runtime annotation store for that map is empty, the adapter reads the old prototype local-storage key for the same map identity and uploads those annotations through the existing runtime `save_map_annotation` path. After that, `snapshot.map.annotations` is populated from runtime-owned state.

If runtime annotations already exist for the map, local prototype annotations are not imported or allowed to overwrite runtime state. The adapter records that the old local annotations for that map have been handled, so they do not reappear later after a runtime-owned delete.

## 2. Which mode this affects

- Mapping: migration is keyed to the current accepted/loaded/live map identity.
- Navigation: unchanged.
- Clean Area: unchanged.
- Rooms / Zones: previously saved prototype rooms/zones can appear after upgrading, but runtime persistence becomes authoritative after migration.
- Shared adapter/runtime architecture: preserves the existing adapter contract while treating webview storage as migration-only fallback.

## 3. Ownership check

- Is this still owned by React/webview state? Unsaved room/zone drafts, draft names, selected id, and preview remain webview presentation state.
- Is this now owned by the runtime/backend? Saved room/zone annotations remain VM/runtime-owned after Milestone 1; this pass only imports old local data into that runtime store when safe.
- What state is the UI only rendering? The UI renders saved annotations from `snapshot.map.annotations`.
- What command does the UI submit? The UI still submits `save_map_annotation` and `delete_map_annotation`; migration internally writes through the same runtime annotation service path.

This follows the rule:

Before Start:
UI may own draft and preview state.

After Start:
runtime/backend owns confirmed mission state.

Saved annotations are also treated as runtime/backend-owned product state before mission start.

## 4. Webview close/reopen behavior

- mapping: unchanged; mapping state hydrates from runtime status.
- navigation: unchanged; active navigation hydrates from runtime mission state.
- clean area: unchanged; active coverage mission state hydrates from runtime mission snapshots.
- room/zone editing: unsaved drafts may be lost.
- active room/zone cleaning: unchanged in this milestone; active mission hydration still comes from `snapshot.activeMission`.
- terminal room/zone result: unchanged in this milestone; recent summaries are still handled by the existing mission snapshot/recent fallback path.

After reopening, the adapter computes the current map identity and asks the VM annotation runtime for that map's annotations. If runtime annotations are empty and an old local prototype annotation set exists for the same map identity, the adapter imports it once into runtime persistence. Future reopens render runtime annotations only.

## 5. Real hardware compatibility check

- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this require Nav2 waypoint sequencing as a public concept? No.
- Can the same adapter shape be implemented by Valetudo later? Yes; Valetudo can keep using the same `snapshot.map.annotations`, `save_map_annotation`, and `delete_map_annotation` surfaces if/when room or zone persistence is implemented.
- What capability flags decide whether controls are shown/enabled? `map_annotations`, `room_semantics`, and `zone_semantics`.
- What operations are explicitly unsupported? Valetudo map annotations and room/zone semantics remain unsupported in the current stub; TurtleBot4/Nav2 annotations remain unsupported if the VM annotation services are not advertised.

## 6. Feature behavior changed

- Existing local prototype rooms/zones can be imported into VM-owned annotation persistence.
- Migration only runs for the active map identity.
- Runtime annotations win over local annotations when both exist.
- Old local annotations are marked handled after migration or conflict detection, so they do not become authoritative again.
- The product UI still renders only `snapshot.map.annotations`.

## 7. Files changed

- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/localAnnotationMigration.ts`
  - Adds the local prototype annotation reader and per-map migration marker helpers.
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/useTurtleBot4Nav2Adapter.ts`
  - Adds runtime-first annotation hydration with one-time safe import from old local storage only when the runtime map store is empty.
- `scripts/vacuum-adapter-regression.ts`
  - Adds regression coverage for old local annotation parsing, map identity filtering, legacy array shape support, and migration marker detection.
- `progress_report.md`
  - Records Milestone 2 behavior, ownership, validation, and remaining risks.

## 8. Tests / validation run

Automated checks run:

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Manual runtime checks not run in this pass:

```text
Opened updated build with old local prototype annotations and confirmed they migrate into runtime persistence.
Closed/reopened webview and confirmed migrated annotations hydrate from runtime-owned state.
Created runtime annotation A while local annotation B exists and confirmed B does not overwrite A.
Deleted migrated annotations, reopened webview, and confirmed stale local annotations do not reappear.
```

## 9. Remaining risks

- Recent terminal mission summaries are still not fully VM-owned durable history; that is Milestone 3.
- This milestone validates migration parsing and build behavior automatically, but live VM/webview migration validation is still needed.
- Runtime annotation persistence still depends on the active map identity string supplied by mapping status; weak or missing map identity can still collapse to `live-map`.
- Webview local storage remains present only as a migration/fallback source for old prototype annotations and current recent mission fallback.
- Valetudo remains explicitly unsupported for room/zone annotation persistence.

## 10. Next recommended step

Milestone 3: persist terminal mission summaries in the runtime and hydrate Recent Missions from `snapshot.missions.recent` without relying on webview-local recent history as the long-term source of truth.
