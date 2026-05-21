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

## Progress Report — Room / Zone Semantics Prototype, Milestone 3

### 1. What changed

The operator can now select a saved room or zone and start cleaning it from
Rooms / Zones mode.

Room/zone cleaning uses product-facing `start_room_cleaning` and
`start_zone_cleaning` commands. The TurtleBot4/Nav2 adapter translates those
commands into the existing VM-owned coverage mission path, while preserving the
selected annotation id, name, kind, and requested mission intent.

### 2. Which mode this affects

- Mapping: unchanged.
- Navigation: unchanged.
- Clean Area: unchanged execution machinery; its runtime coverage path is reused.
- Rooms / Zones: selected saved rooms/zones can now start, pause, and cancel
  cleaning.
- Shared adapter/runtime architecture: new room/zone intent commands and
  capabilities were added without exposing Nav2 concepts to product UI.

### 3. Ownership check

- React/webview state still owns only room/zone draft, selected annotation, and
  pre-start preview state.
- Runtime/backend owns active room/zone cleaning after start.
- UI renders active/terminal cleaning state from `snapshot.activeMission` and
  `snapshot.mission`.
- UI submits `start_room_cleaning` or `start_zone_cleaning`; pause/cancel use
  `pause_mission` and `cancel_mission`.

### 4. Webview close/reopen behavior

- Mapping hydrates from mapping runtime snapshots as before.
- Navigation hydrates from adapter/runtime mission snapshots as before.
- Clean Area hydrates from `snapshot.activeMission` as before.
- Room/zone editing drafts remain local and are not durable.
- Saved rooms/zones hydrate from adapter annotation state.
- Active room/zone cleaning hydrates from `snapshot.activeMission`; if the VM
  runtime reports the mission as coverage but preserves the requested command or
  annotation target, the UI still treats it as the Rooms / Zones workflow.

State still not durable: in-progress edit drafts and VM-owned annotation
persistence.

### 5. Real hardware compatibility check

- Product UI does not expose TurtleBot4/Nav2 specifics.
- Nav2 waypoint sequencing remains private to the TurtleBot4/Nav2 runtime.
- The same command shape can be implemented by Valetudo later through
  `room_cleaning` and `zone_cleaning` capabilities.
- Controls are gated by `room_semantics`, `zone_semantics`, `room_cleaning`,
  `zone_cleaning`, `mission_state`, `pause_mission`, and `cancel_mission`.
- Valetudo room/zone cleaning remains explicitly unsupported until Layer 6 maps
  vendor segments/zones or adapter-owned annotations.

### 6. Feature behavior changed

- User can clean a selected saved room.
- User can clean a selected saved zone.
- Room/zone cleaning shows active mission state in Rooms / Zones mode.
- Pause/cancel controls follow mission action capabilities.
- Reopened UI can restore active room/zone cleaning from runtime snapshots.

### 7. Files changed

- `panels-standalone/src/vacuum-adapter/commands.ts`
  - Adds product-facing room/zone cleaning commands.
- `panels-standalone/src/vacuum-adapter/capabilities.ts`
  - Adds `room_cleaning` alongside `zone_cleaning`.
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/*`
  - Maps room/zone cleaning capabilities and dispatches the new commands
    through the existing coverage mission runtime.
- `panels-standalone/src/vacuum-adapter/backends/valetudo/*`
  - Keeps room/zone cleaning explicitly unsupported for the stub backend.
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
  - Adds Clean Room/Zone actions and hydrates room/zone active missions.
- `scripts/vacuum-adapter-regression.ts`
  - Covers the new command and capability contract.
- `VACUUM_STACK_PLAN.md`, `steps.md`, `extension.md`, and this file
  - Record Milestone 3 status.

### 8. Tests / validation run

Passed:

```sh
npm run test:vacuum-adapter
npm run build:panels
```

Also run:

```sh
npx tsc --noEmit
npx tsc -p panels-standalone/tsconfig.json --noEmit
```

Both TypeScript checks are still blocked by existing unrelated workspace issues,
including unused variables in `packages/tensorfleet-auth/src/oauth-core.ts`,
missing `tensorfleet-ros` type resolution in standalone panel typecheck, and
older gzweb/SensorView3D type errors.

### 9. Remaining risks

- VM runtime support for preserving `room_cleaning` / `zone_cleaning` as the
  confirmed mission type may need hardening; UI currently tolerates runtime
  snapshots that report coverage with a room/zone requested command.
- `snapshot.missions.recent` still needs richer room/zone terminal summaries.
- Annotation durability is still prototype webview storage.
- Polygon editing/cleaning is not production-ready.
- Room/zone execution still intentionally uses coverage as the temporary
  implementation strategy.

### 10. Next recommended step

Milestone 4: harden hydration and recent mission summaries so terminal
room/zone results remain clearly labeled after webview reopen.

## Progress Report — Room / Zone Semantics Prototype, Milestone 4

### 1. What changed

The operator can now reopen the Vacuum Control webview after a room/zone clean
and still see the runtime-owned active mission or a recent terminal mission
summary with the room/zone label.

The TurtleBot4/Nav2 adapter now parses `snapshot.missions.recent` from VM
mission snapshots when available, keeps terminal mission summaries across
webview reloads for the prototype path, and exposes them through the normalized
adapter snapshot.

### 2. Which mode this affects

- Mapping: unchanged.
- Navigation: unchanged except shared recent mission hydration can preserve
  terminal mission entries.
- Clean Area: terminal coverage summaries can appear in recent missions.
- Rooms / Zones: active and terminal room/zone cleaning state is restored from
  adapter/runtime snapshots and recent mission history.
- Shared adapter/runtime architecture: `snapshot.missions.recent` is now used
  as the product-facing recent mission surface.

### 3. Ownership check

- React/webview state owns only room/zone draft/editing state, selected
  annotation id, and dismissal/presentation state.
- Runtime/backend owns active room/zone cleaning after start.
- UI renders active state from `snapshot.activeMission`.
- UI renders terminal/recent summaries from `snapshot.missions.recent`.
- UI submits `start_room_cleaning` or `start_zone_cleaning`; lifecycle actions
  still use `pause_mission`, `resume_mission`, and `cancel_mission`.

### 4. Webview close/reopen behavior

- Mapping hydrates from adapter/runtime mapping snapshots.
- Navigation hydrates from adapter/runtime mission snapshots.
- Clean Area active missions hydrate from `snapshot.activeMission`; terminal
  coverage entries can appear in recent missions.
- Room/zone editing drafts remain local and are not durable.
- Room/zone cleaning hydrates from `snapshot.activeMission` while active and
  from `snapshot.missions.recent` after terminal completion/failure/cancel.

Still not durable: unsaved room/zone drafts and final VM-owned persistent
mission history. The TurtleBot4/Nav2 prototype keeps terminal summaries in
webview storage when the runtime does not provide a durable recent list.

### 5. Real hardware compatibility check

- Product UI does not expose TurtleBot4/Nav2 specifics.
- Nav2 waypoint sequencing remains private to the TurtleBot4/Nav2 runtime.
- The same adapter shape can be implemented by Valetudo later by returning
  normalized active/recent mission snapshots.
- Controls remain gated by `room_semantics`, `zone_semantics`,
  `room_cleaning`, `zone_cleaning`, `mission_state`, `pause_mission`,
  `resume_mission`, and `cancel_mission`.
- Valetudo room/zone execution remains explicitly unsupported until Layer 6
  maps vendor segments/zones or adapter-owned annotations.

### 6. Feature behavior changed

- Active room/zone cleaning hydrates after reopening the webview.
- Terminal room/zone cleaning summaries appear in Recent Missions.
- Recent mission entries preserve room/zone annotation labels when provided by
  the runtime target payload.
- Terminal mission summaries are deduplicated and ordered newest-first.

### 7. Files changed

- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/useTurtleBot4Nav2Adapter.ts`
  - Parses active and recent mission snapshots, merges terminal history, and
    persists recent terminal summaries for prototype reload hydration.
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/stateMapper.ts`
  - Carries recent mission snapshots into `snapshot.missions.recent`.
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
  - Adds a Recent Missions card for terminal mission summaries.
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.css`
  - Styles recent mission summary rows.
- `scripts/vacuum-adapter-regression.ts`
  - Adds regression coverage for hydrated recent room-cleaning summaries.
- `VACUUM_STACK_PLAN.md`, `steps.md`, `extension.md`, and this file
  - Record Layer 5 completion status and remaining risks.

### 8. Tests / validation run

Passed:

```sh
npm run test:vacuum-adapter
npm run build:panels
git diff --check
```

Blocked by existing unrelated TypeScript errors:

```sh
npx tsc --noEmit
```

The repo-wide typecheck still fails on unused variables in
`packages/tensorfleet-auth/src/oauth-core.ts`, outside the room/zone feature
path.

Also checked:

```sh
npm run build
```

The root package has no `build` script; `build:panels` is the available panel
build command.

### 9. Remaining risks

- Recent mission persistence is prototype webview storage when the VM runtime
  does not provide durable `missions.recent`.
- Unsaved room/zone drafts are still frontend-owned and not durable.
- Annotation durability is still adapter/webview storage, not VM-owned map
  annotation persistence.
- Room/zone execution still intentionally uses coverage as the temporary
  implementation strategy.
- Valetudo support remains an explicit unsupported stub.

### 10. Next recommended step

Move the prototype durability boundary into the VM runtime: persist map
annotations and recent mission summaries with the accepted/loaded map instead
of relying on webview storage.
