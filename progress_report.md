# Progress Report — Runtime-Owned Map Annotation Persistence

Current report date: May 21, 2026.

## 1. What changed

Saved room/zone annotations now use VM/runtime-owned persistence for the TurtleBot4/Nav2 path.

The operator still uses the same Rooms / Zones flow: draw a rectangular room or zone draft, name it, save it, select it, and delete it. The product-facing surface remains `snapshot.map.annotations`, with saves and deletes still submitted through `save_map_annotation` and `delete_map_annotation`.

The difference is ownership. The TurtleBot4/Nav2 adapter now hydrates saved annotations from VM annotation services keyed by the accepted, loaded, or live map identity instead of treating browser local storage as the annotation source of truth.

## 2. Which mode this affects

- Mapping: saved annotations are keyed to the current accepted/loaded/live map identity exposed by mapping status.
- Navigation: unchanged.
- Clean Area: unchanged.
- Rooms / Zones: saved room/zone list and map overlays now hydrate from runtime-owned annotation persistence.
- Shared adapter/runtime architecture: adds backend-neutral VM annotation services behind the existing vacuum adapter contract.

## 3. Ownership check

- Unsaved draft rectangle/name/kind: still owned by React/webview state.
- Saved room/zone annotations: now owned by the VM/runtime for TurtleBot4/Nav2.
- UI only renders: `snapshot.map.annotations` for saved annotations.
- Commands submitted: `save_map_annotation` and `delete_map_annotation`.

This follows the rule:

Before Start:
UI may own draft and preview state.

After Start:
runtime/backend owns confirmed mission state.

For this milestone, saved room/zone annotations also move to runtime/backend ownership before any cleaning mission starts.

## 4. Webview close/reopen behavior

- mapping: unchanged; mapping state hydrates from runtime status.
- navigation: unchanged; active navigation hydrates from runtime mission state.
- clean area: unchanged; active coverage mission state hydrates from runtime mission snapshots.
- room/zone editing: unsaved drafts may be lost.
- active room/zone cleaning: unchanged in this milestone; active mission hydration still comes from `snapshot.activeMission`.
- terminal room/zone result: unchanged in this milestone; recent summaries are still handled by the existing mission snapshot/recent fallback path.

After reopening, the adapter computes the current map identity, asks the VM annotation runtime for that map's annotations, and exposes them through `snapshot.map.annotations`.

## 5. Real hardware compatibility check

- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this require Nav2 waypoint sequencing as a public concept? No.
- Can the same adapter shape be implemented by Valetudo later? Yes; Valetudo can provide the same `snapshot.map.annotations`, `save_map_annotation`, and `delete_map_annotation` behavior from vendor segments/zones or adapter-owned annotations later.
- What capability flags decide whether controls are shown/enabled? `map_annotations`, `room_semantics`, and `zone_semantics`.
- What operations are explicitly unsupported? Valetudo map annotations and room/zone semantics remain unsupported in the current stub; TurtleBot4/Nav2 annotations are unsupported if the VM annotation services are not advertised.

## 6. Feature behavior changed

- Saved rooms/zones no longer depend on browser local storage as the source of truth.
- Saved rooms/zones hydrate from VM runtime state after webview close/reopen.
- Deleting a saved room/zone writes through runtime persistence, so deleted annotations stay deleted after webview close/reopen.
- Annotations are scoped by active accepted/loaded map identity, so annotations for one map do not intentionally appear on another map.
- The product UI still renders only `snapshot.map.annotations`.

## 7. Files changed

- `~/firecracker-vm/assets/opt/ros2_ws/src/turtlebot4_firecracker_bringup/scripts/vacuum_mission_node.py`
  - Adds VM-owned annotation persistence, save/delete/get services, validation, map-keyed storage, and atomic JSON writes.
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/capabilityMapper.ts`
  - Adds VM annotation service discovery and gates annotation/room/zone capabilities on those services.
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/useTurtleBot4Nav2Adapter.ts`
  - Replaces browser-local annotation hydration/save/delete with VM annotation service calls while preserving adapter-facing state and commands.
- `scripts/vacuum-adapter-regression.ts`
  - Updates the regression runtime fixture and capability checks for VM annotation services.
- `progress_report.md`
  - Records Milestone 1 behavior, ownership, validation, and remaining risks.

## 8. Tests / validation run

Automated checks run:

```sh
python3 -m py_compile /home/shane/firecracker-vm/assets/opt/ros2_ws/src/turtlebot4_firecracker_bringup/scripts/vacuum_mission_node.py
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
```

Manual runtime checks not run in this pass:

```text
Created room, saved it, reopened webview, confirmed runtime hydration.
Deleted room, reopened webview, confirmed deletion persisted.
Saved room on map A, loaded/accepted map B, confirmed map A room did not appear.
Returned to map A, confirmed map A room appeared again.
```

## 9. Remaining risks

- Existing prototype annotations in browser local storage are not migrated yet; that is Milestone 2.
- Recent terminal mission summaries are still not fully VM-owned durable history; that is Milestone 3.
- Runtime annotation persistence uses the active map identity string supplied by mapping status; weak or missing map identity can still collapse to `live-map`.
- This has automated validation only; live VM/webview close-reopen validation is still needed.
- Valetudo remains explicitly unsupported for room/zone annotation persistence.

## 10. Next recommended step

Milestone 2: add one-time migration from old webview-local annotations into the VM annotation store only when the runtime store for the current map is empty.
