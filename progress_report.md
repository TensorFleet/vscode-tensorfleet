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

## Progress Report — Room / Zone Semantics Prototype, Milestone 2

### 1. What changed

The operator can now select a saved room or zone and preview it as a cleaning
target before starting any mission.

The selected room/zone is converted into the existing Clean Area coverage target
model, shown on the map with route and per-cell cleanability overlays, and
classified as cleanable, partially cleanable, or invalid.

### 2. Which mode this affects

- Mapping: unchanged.
- Navigation: unchanged.
- Clean Area: unchanged execution path; its coverage target and route preview
  logic is reused.
- Rooms / Zones: selected saved annotations now show target preview and
  cleanability state.
- Shared adapter/runtime architecture: no new runtime execution command yet;
  preview uses product-facing `snapshot.map.annotations`.

### 3. Ownership check

- React/webview state owns only pre-start selected annotation id and preview
  presentation.
- Runtime/backend still owns no room/zone cleaning mission in this milestone.
- UI renders saved room/zone state from `snapshot.map.annotations`.
- UI submits no cleaning command for room/zone preview.

### 4. Webview close/reopen behavior

- Mapping, navigation, and Clean Area active missions hydrate as before.
- Room/zone editing drafts remain local and are not durable.
- Saved rooms/zones hydrate from adapter annotation state after reopen.
- Selected room/zone id is local presentation state; after reopen the operator
  can reselect any saved room/zone and the preview is recomputed from snapshot
  state.
- Room/zone cleaning is still not implemented.

### 5. Real hardware compatibility check

- Product UI does not expose TurtleBot4/Nav2 specifics.
- Nav2 waypoint sequencing is not a public concept; route preview is local
  pre-start presentation.
- Valetudo can implement the same annotation/target shape later through
  capabilities.
- Controls remain gated by `map_annotations`, `room_semantics`, and
  `zone_semantics`; execution will later require room/zone cleaning
  capabilities.
- Valetudo remains explicitly unsupported for current annotation semantics.

### 6. Feature behavior changed

- User can select a saved room/zone as a cleaning target preview.
- Selected target highlights on the map.
- Selected target shows route preview.
- Selected target shows cleanable/skipped cell overlay.
- Side panel reports cleanable, partially cleanable, or invalid status.

### 7. Files changed

- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
  - Converts selected saved annotations into Clean Area coverage targets and
    route previews.
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.css`
  - Adds target-status styling for the Rooms / Zones panel.
- `VACUUM_STACK_PLAN.md`, `steps.md`, `extension.md`, and this file
  - Record Milestone 2 status and remaining Layer 5 scope.

### 8. Tests / validation run

Passed:

```sh
npm run build:panels
npm run test:vacuum-adapter
git diff --check
```

Blocked by existing unrelated TypeScript errors:

```sh
npx tsc --noEmit
```

The repo-wide typecheck currently fails on unused variables in
`packages/tensorfleet-auth/src/oauth-core.ts`, outside the room/zone feature
path.

Implementation review checks:

- Select room
- Select zone
- Preview cleanable target
- Preview partially cleanable target
- Confirm no room/zone cleaning command is submitted
- Confirm no backend-name branching was added to product UI

### 9. Remaining risks

- Room/zone cleaning execution is still not implemented.
- Selected target id is not durable presentation state.
- Annotation durability is still prototype webview storage.
- Polygon target preview is not implemented.
- Preview still uses coverage as the temporary implementation strategy.

### 10. Next recommended step

Milestone 3: add product-facing room/zone cleaning intent commands and map them
to the existing runtime-owned coverage mission path.
