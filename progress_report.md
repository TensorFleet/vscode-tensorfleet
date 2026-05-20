# Runtime-Owned Vacuum Mission Architecture

## Progress Report — Room / Zone Semantics Prototype, Milestone 1

### 1. What changed

Milestone 1 adds product-level manual room/zone annotation state.

The operator can now open `Vacuum Control`, switch to `Rooms / Zones`, draw a
rectangular draft on the map, choose `Room` or `Zone`, name it, save it, select
it later from the map or side panel, and delete it.

Saved rooms/zones are exposed through `snapshot.map.annotations` and persisted
by the TurtleBot4/Nav2 adapter in webview storage keyed to the active map
identity when one is available. This is intentionally prototype durability, not
final VM-owned map persistence.

This pass does not add room/zone cleaning execution.

### 2. Which mode this affects

- Mapping: unchanged except saved annotations are associated with the current
  accepted/loaded/live map identity.
- Navigation: unchanged.
- Clean Area: existing rectangle editor is reused as the draft editor for
  Rooms / Zones; Clean Area execution behavior is unchanged.
- Rooms / Zones: new mode for manual creation, naming, map overlay rendering,
  selection, and deletion of saved room/zone annotations.
- Shared adapter/runtime architecture: adds product-facing map annotation state,
  annotation capabilities, and save/delete annotation commands.

### 3. Ownership check

- Before start: UI owns draft room/zone rectangle, name input, kind selector,
  selected saved annotation id, and local validation.
- Saved annotation state: adapter snapshot owns `snapshot.map.annotations`.
- Runtime/backend-owned state: no mission execution changes in this pass.
- UI-rendered state: Rooms / Zones renders saved annotations from the adapter
  snapshot, not only local React state.
- Commands submitted: `save_map_annotation` and `delete_map_annotation`.

### 4. Webview close/reopen behavior

- Mapping: unchanged; mapping hydrates from adapter/runtime snapshots.
- Navigation: unchanged; active/terminal navigation hydrates from mission
  snapshots.
- Clean Area: unchanged; active coverage hydrates from `snapshot.activeMission`.
- Room/zone editing: unsaved draft rectangle/name is still local webview state
  and is not durable.
- Room/zone saved annotations: survive webview close/reopen through adapter
  storage and hydrate into `snapshot.map.annotations`.
- Room/zone cleaning: not implemented in this milestone.

### 5. Real hardware compatibility check

- Product UI does not branch on TurtleBot4/Nav2 or Valetudo backend identity.
- Nav2 waypoint sequencing is not exposed as a public product concept.
- The same adapter shape can be implemented by Valetudo later by mapping
  product annotations to vendor segments/zones or adapter-owned annotations.
- Controls are gated by `map_annotations`, `room_semantics`, and
  `zone_semantics`.
- Valetudo currently reports annotation persistence and room/zone semantics as
  unsupported until the Layer 6 integration runtime maps them explicitly.

### 6. Feature behavior changed

- User can enter `Rooms / Zones` mode.
- User can draw a rectangular room/zone draft using the existing map rectangle
  editor.
- User can name a draft and choose `Room` or `Zone`.
- User can save the draft as a product-level map annotation.
- User can see saved rooms/zones on the map.
- User can select saved rooms/zones from the map overlay or list.
- User can delete a selected saved room/zone.
- Saved room/zone state survives webview reload for the TurtleBot4/Nav2
  prototype path.

### 7. Files changed

- `panels-standalone/src/vacuum-adapter/state.ts`
  - Adds `VacuumMapAnnotation` state and `snapshot.map.annotations`.
- `panels-standalone/src/vacuum-adapter/capabilities.ts`
  - Adds `map_annotations`, `room_semantics`, and `zone_semantics`.
- `panels-standalone/src/vacuum-adapter/commands.ts`
  - Adds `save_map_annotation` and `delete_map_annotation`.
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/*`
  - Exposes annotation capabilities, persists saved annotations in prototype
    storage, and hydrates annotations into the normalized snapshot.
- `panels-standalone/src/vacuum-adapter/backends/valetudo/*`
  - Keeps annotation and room/zone semantics explicitly unsupported for the
    current stub.
- `panels-standalone/src/components/VacuumControl/*`
  - Adds the Rooms / Zones mode, annotation overlays, list selection, and draft
    creation UI using the existing rectangle editor.
- `scripts/vacuum-adapter-regression.ts`
  - Extends contract coverage for annotation capabilities, state, and command
    names.
- `VACUUM_STACK_PLAN.md`, `steps.md`, `extension.md`, and this file
  - Document Milestone 1 status and remaining scope.

### 8. Tests / validation run

Passed:

```sh
npm run build
npm run test:vacuum-adapter
git diff --check
```

Product-flow checks covered by implementation review:

- Create room/zone draft.
- Name room/zone draft.
- Save room/zone annotation.
- Select saved annotation from map overlay.
- Select saved annotation from side-panel list.
- Delete selected annotation.
- Confirm no room/zone cleaning execution was added.
- Confirm product UI still branches on capabilities rather than backend names.

### 9. Remaining risks

- Annotation persistence is still webview storage, not VM/runtime map-owned
  state.
- Unsaved draft rectangle/name is not durable.
- Geometry is rectangle-only.
- There is no room/zone cleaning execution yet.
- Webview visual flow still needs manual extension validation.
- Valetudo support remains an explicit unsupported stub.

### 10. Next recommended step

Milestone 2: allow selecting a saved room/zone as a cleaning target preview,
including cleanable/partially cleanable/invalid status, without starting an
active mission yet.
