# Manual Room / Zone MVP Answers

Date: May 21, 2026

This is a standalone implementation-backed explanation of the current Manual Room / Zone MVP. It avoids relying on source paths because the intended reader may not have codebase access.

## 1. Current-State Mapping

### Draw -> Name -> Save -> Select -> Preview -> Clean -> Progress -> Summary

The current Rooms / Zones MVP is implemented as a mode inside the Vacuum Control operator panel. It reuses the same rectangular map editor and coverage preview machinery used by Clean Area, then adds saved annotation state and product-level room/zone cleaning commands.

1. Draw room or zone

The operator enters Rooms / Zones mode and activates the drawing tool. Pointer interactions on the map create a rectangular draft in map coordinates. The rectangle can be drawn, moved, and resized. Validation is run against the currently normalized occupancy grid, so the UI can tell whether the rectangle intersects the map, whether it contains cleanable cells, and whether it includes unknown, occupied, out-of-bounds, or too-small regions.

State ownership:

- The draft rectangle is local webview state.
- The validation result is local webview state.
- The map grid used for validation comes from the adapter snapshot.
- The VM/runtime does not know about unsaved drafts.

Prototype-only parts:

- Geometry is rectangle-only in the UI.
- Unsaved drafts do not survive webview reload.
- Polygon annotations exist in the public type shape, but the current UI preview only supports rectangles.

2. Name and choose kind

The operator chooses `Room` or `Zone` and enters a name. The UI keeps this as local state until save. Room creation is gated by the `room_semantics` capability, and zone creation is gated by the `zone_semantics` capability. If the relevant capability is unsupported, the kind button is disabled and saving that kind is not allowed.

State ownership:

- Draft kind and draft name are local webview state.
- Capability support comes from the adapter snapshot.

Prototype-only parts:

- The default draft name is currently a hardcoded example-style value rather than a generated `Room 1` / `Zone 1` sequence.
- There is no durable in-progress edit session.

3. Save annotation

When the operator saves, the UI constructs a normalized map annotation:

```ts
{
  id: "room-<timestamp>" | "zone-<timestamp>",
  kind: "room" | "zone",
  name: "<trimmed name or fallback>",
  area: {
    shape: "rectangle",
    minX: number,
    minY: number,
    maxX: number,
    maxY: number
  },
  mapId: string | null
}
```

The `mapId` is chosen from the current map identity in this order:

1. active map name
2. loaded map path
3. saved map path
4. `"live-map"` if a map grid exists
5. `null` if no map identity is available

The UI sends:

```ts
{
  command: "save_map_annotation",
  annotation
}
```

The TurtleBot4/Nav2 adapter handles this command locally today. It verifies `map_annotations` support, fills timestamps, normalizes blank names to `Room` or `Zone`, stores the annotation in adapter state, sorts saved annotations by name, writes them to browser local storage, and exposes the resulting list through:

```ts
snapshot.map.annotations
```

State ownership:

- After save, the UI renders annotations from the adapter snapshot.
- Current durable-ish storage is browser local storage owned by the webview/adapter process.
- Final desired owner is the VM/runtime map persistence layer.

Prototype-only parts:

- Saved annotations are not VM-owned yet.
- Persistence is tied to local webview storage and the current computed map key.

4. Select saved annotation

Saved annotations appear both as list rows and as clickable rectangles on the map. Selecting one stores only the selected annotation id in local UI state. The actual annotation data is always looked up from `snapshot.map.annotations`.

State ownership:

- Saved annotation list is adapter snapshot state.
- Selected id is local presentation state.
- The selected id is cleared if the annotation disappears from the snapshot.

Prototype-only parts:

- Selection is not durable across reload.
- Selection is not currently stored per map.

5. Preview cleanability and route

When an annotation is selected, the UI converts its rectangle into the same coverage target model used by Clean Area. It evaluates occupancy-grid cells inside the rectangle:

- free cells become cleanable target cells
- occupied cells become skipped occupied cells
- unknown cells become skipped unknown cells
- cells outside map bounds are counted as out-of-bounds
- tiny disconnected free regions below the minimum useful area are skipped as too-small

The UI then derives:

- target status: `none`, `cleanable`, `partial`, or `invalid`
- cleanable area
- skipped area
- route pass count
- estimated route distance
- preview waypoints
- map overlays for cleanable, skipped, and route cells

Status rules:

- `cleanable`: selected rectangle has cleanable cells and no skipped/invalid cells.
- `partial`: selected rectangle has cleanable cells but also occupied, unknown, out-of-bounds, or too-small cells.
- `invalid`: selected rectangle is unsupported or has no cleanable target cells.
- `none`: no annotation selected.

State ownership:

- Annotation data comes from adapter snapshot.
- Map grid comes from adapter snapshot.
- Preview route and target status are derived local presentation state.
- Runtime does not own preview before start.

Prototype-only parts:

- Preview is rectangle-only.
- Preview is a local estimate; active execution route/progress should come from runtime snapshots after start.

6. Start room or zone cleaning

The Clean button is enabled only when:

- an annotation is selected
- the target is not invalid
- it has cleanable cells
- `room_cleaning` or `zone_cleaning` capability is supported
- adapter readiness is ready
- no conflicting navigation, mapping, or clean-area mission is active

The UI sends one of:

```ts
{
  command: "start_room_cleaning",
  annotation,
  coverage: {
    swathWidth,
    laneSpacing,
    completionThreshold,
    boundaryExtension
  }
}
```

```ts
{
  command: "start_zone_cleaning",
  annotation,
  coverage: {
    swathWidth,
    laneSpacing,
    completionThreshold,
    boundaryExtension
  }
}
```

The adapter validates:

- capability support
- adapter readiness
- room command must receive a `room` annotation
- zone command must receive a `zone` annotation

The TurtleBot4/Nav2 backend does not run a separate room-cleaning executor. Instead, it translates the room/zone intent into the existing runtime-owned coverage mission path. It writes a coverage request into the VM mission runtime parameter service with this payload:

```ts
{
  area: annotation.area,
  missionType: "room_cleaning" | "zone_cleaning",
  requestedCommand: "start_room_cleaning" | "start_zone_cleaning",
  annotation: {
    id: annotation.id,
    kind: annotation.kind,
    name: annotation.name,
    mapId: annotation.mapId
  },
  route?: optionalRoute,
  coverage?: {
    swathWidth,
    laneSpacing,
    completionThreshold,
    boundaryExtension
  }
}
```

Then it calls the VM runtime coverage-start service. That means the product-level command is room/zone cleaning, but execution is currently coverage execution.

State ownership:

- Before start, UI owns draft and preview.
- After start, runtime/backend owns confirmed mission state.
- The adapter may create a temporary optimistic mission snapshot while waiting for the VM runtime snapshot.

Prototype-only parts:

- Room/zone execution intentionally reuses coverage execution.
- Room/zone identity depends on runtime preserving `missionType`, `requestedCommand`, or `target.annotation` in mission snapshots.

7. Active mission progress

Active mission state hydrates from normalized mission snapshots:

```ts
snapshot.activeMission
snapshot.missions.active
```

The adapter receives runtime mission updates from a mission status topic and also polls a mission snapshot service when available. It parses active mission, recent missions, target payload, progress, available actions, result, and error into a backend-neutral mission snapshot.

The UI treats an active mission as a room/zone workflow when either:

- `activeMission.type` is `room_cleaning` or `zone_cleaning`, or
- `activeMission.requestedCommand` is `start_room_cleaning` or `start_zone_cleaning`

This is deliberate. The runtime may execute the mission through coverage while the UI still preserves the product intent of "clean this room/zone."

Progress display comes from:

- `activeMission.status`
- `activeMission.phase`
- `activeMission.progress.percent`
- `activeMission.progress.currentStep`
- `activeMission.progress.totalSteps`
- `activeMission.progress.areaCoveredSqM`
- `activeMission.progress.areaRemainingSqM`
- route and coverage details in `activeMission.target`
- `activeMission.availableActions`
- `activeMission.error`

8. Terminal summary

Terminal mission history is exposed through:

```ts
snapshot.missions.recent
```

The UI filters recent missions to terminal statuses:

- `completed`
- `failed`
- `canceled`
- `unsupported`

It displays up to four recent rows. The display name is chosen in this order:

1. `mission.target.annotation.name`
2. `Room cleaning` if type or requested command indicates room cleaning
3. `Zone cleaning` if type or requested command indicates zone cleaning
4. `Area cleaning` for coverage
5. `Navigation`
6. `Mapping`
7. the raw mission type formatted as words

Current persistence:

- If the VM runtime provides recent missions, the adapter hydrates from runtime.
- If runtime does not provide durable recent missions, the TurtleBot4/Nav2 adapter merges terminal missions into browser local storage and rehydrates them on webview reopen.

Prototype-only parts:

- Durable mission history is not VM-owned yet.
- Recent summary durability is local to the webview storage context.

### Saved Annotation Data Flow

The complete saved annotation path is:

1. Operator draws a rectangle.
2. UI stores rectangle/name/kind as local draft state.
3. Operator clicks Save.
4. UI constructs a normalized annotation with id, kind, name, rectangle area, and map identity.
5. UI dispatches `save_map_annotation`.
6. Adapter handles the command.
7. Adapter normalizes name/timestamps.
8. Adapter stores annotations in memory and browser local storage.
9. Adapter snapshot includes the list as `snapshot.map.annotations`.
10. UI renders only from `snapshot.map.annotations` for saved annotations.

Current storage key shape:

```text
tensorfleet:vacuums:turtlebot4-nav2:map-annotations:<map-identity>
```

The map identity suffix is:

```text
activeMapName ?? loadedMapPath ?? savedMapPath ?? "live-map"
```

What happens on webview reopen:

- The adapter recomputes the storage key from current mapping state.
- It reads local storage for that key.
- It exposes the annotations through `snapshot.map.annotations`.
- Unsaved drafts and selected annotation id are not restored.

What must change for VM/runtime-owned persistence:

- `save_map_annotation` and `delete_map_annotation` should call VM/runtime persistence services.
- The VM should store annotations with the accepted/loaded map identity.
- The runtime snapshot should return annotations for the active map.
- The adapter should expose those runtime annotations through `snapshot.map.annotations`.
- Local storage should become migration/fallback only, not the source of truth.

### Clean Living Room Execution Path

"Clean Living Room" is implemented as a saved room annotation translated into a coverage mission.

Full path:

1. The operator selects the `Living Room` saved annotation.
2. The UI derives preview status from the annotation rectangle and map grid.
3. The Clean button is enabled if the target is cleanable/partial, readiness is ready, no conflicting mission is active, and `room_cleaning` capability is supported.
4. The operator clicks `Clean Living Room`.
5. The UI dispatches:

```ts
{
  command: "start_room_cleaning",
  annotation: {
    id: "room-...",
    kind: "room",
    name: "Living Room",
    area: { shape: "rectangle", minX, minY, maxX, maxY },
    mapId,
    createdAt,
    updatedAt
  },
  coverage: {
    swathWidth,
    laneSpacing,
    completionThreshold,
    boundaryExtension
  }
}
```

6. The adapter checks `room_cleaning` support and readiness.
7. The adapter verifies the annotation kind is `room`.
8. The adapter writes a runtime `coverage_request` containing the annotation area, room-cleaning mission type, requested command, annotation label/id/map id, and coverage settings.
9. The adapter triggers the runtime coverage start service.
10. The VM runtime owns route generation, waypoint sequencing, lifecycle state, progress, result, and recovery actions.
11. The adapter creates an optimistic room-cleaning mission snapshot only until runtime hydration arrives.
12. The runtime publishes or serves mission snapshots.
13. The adapter exposes the mission as `snapshot.activeMission`.
14. The UI recognizes the workflow as room cleaning if either the mission type is `room_cleaning` or the requested command is `start_room_cleaning`.
15. The UI renders progress using the runtime mission snapshot.
16. When the mission becomes terminal, the runtime or adapter recent-history fallback exposes it through `snapshot.missions.recent`.
17. The Recent Missions card shows `Living Room` if the target annotation name is present.

## 2. Architecture Boundary Questions

### Boundary Audit

The product boundary is the `vacuum_adapter` contract. The Rooms / Zones UI mostly respects it. Product UI branches on capabilities and normalized snapshots, not on backend identity.

Findings:

1. Room/zone UI does not branch on TurtleBot4/Nav2 or Valetudo identity.

- Classification: aligned with architecture.
- The UI uses `snapshot.capabilities`, `snapshot.map.annotations`, `snapshot.activeMission`, and `snapshot.missions.recent`.

2. Room/zone UI does not expose raw ROS topics.

- Classification: aligned with architecture.
- Room/zone save/select/clean actions go through adapter commands and normalized state.

3. Room/zone execution does not expose Nav2 waypoints as a product concept.

- Classification: aligned with architecture.
- Preview waypoints exist only as local presentation. Runtime sequencing is private to the VM coverage runtime.

4. Map overlays use direct map/costmap/plan/sensor data for visualization.

- Classification: acceptable diagnostic/debug surface.
- This is not room/zone command authority. The product base map still uses normalized map state.

5. Manual teleop publishes directly to a ROS velocity topic.

- Classification: acceptable diagnostic/operator control surface, but not part of room/zone product semantics.
- Teleop is intentionally not routed as a one-shot adapter command.

6. The panel has a helper that publishes zero velocity directly when stopping manual motion around mapping actions.

- Classification: temporary prototype leak.
- It is not a room/zone workflow dependency, but it is product UI code touching a raw ROS topic.

7. Backend adapter uses raw ROS services and Nav2 action concepts internally.

- Classification: acceptable backend implementation detail.
- It is below the adapter boundary.

8. Capability metadata includes backend names like Nav2 action names and VM service names.

- Classification: acceptable diagnostic metadata.
- UI should not branch on these strings.

9. The hook that chooses the adapter currently only instantiates TurtleBot4/Nav2 and throws for Valetudo.

- Classification: temporary prototype limitation.
- The public contract and Valetudo mapping stubs exist, but runtime backend selection is not complete.

10. Annotation and recent-history persistence keys are TurtleBot4/Nav2-specific.

- Classification: temporary prototype leak inside backend adapter.
- This should be replaced by VM-owned persistence.

No architecture violation was found where the Rooms / Zones product UI depends on TurtleBot4-specific details, raw ROS topics, Nav2 waypoint concepts, backend names, or VM-specific assumptions for product room/zone behavior.

### Ownership Rule: Before Start vs After Start

Rule:

```text
Before Start: UI may own draft and local preview.
After Start: runtime/backend owns confirmed mission state.
```

Clean Area:

- Before start, UI owns rectangle draft, validation, confirm state, local preview waypoints, and local coverage preview.
- On start, UI sends `start_coverage`.
- After start, active execution should be owned by runtime and hydrated through `snapshot.activeMission`.
- Current code still has fallback local coverage progress state used when no active runtime coverage mission is present. This is prototype/fallback behavior and should not become mission authority.

Rooms/Zones:

- Before start, UI owns draft rectangle, draft name, draft kind, selected annotation id, and preview.
- Saved annotations are adapter snapshot state.
- On start, UI sends `start_room_cleaning` or `start_zone_cleaning`.
- After start, active room/zone cleaning is rendered from `snapshot.activeMission`.
- The adapter creates an optimistic mission snapshot after successful dispatch, then fetches runtime truth.

Active mission authority that still lives outside runtime:

- Optimistic mission snapshots in the adapter while waiting for runtime response.
- Local command error state in the UI.
- Local dismissed mission/navigation presentation state.
- Local fallback clean-area covered-cell tracking when no active runtime mission exists.

The major rule is preserved for room/zone cleaning, with the caveat that optimistic adapter state and fallback Clean Area progress are prototype bridges.

### Capability Flags Used by Rooms / Zones UI

Capability flags relevant to Rooms / Zones:

- `map_annotations`
- `room_semantics`
- `zone_semantics`
- `room_cleaning`
- `zone_cleaning`
- `mission_state`
- `pause_mission`
- `resume_mission`
- `cancel_mission`
- `retry_mission_step`
- `skip_mission_step`
- `return_to_dock`
- `dock_state`
- `battery`

Control behavior:

1. Room kind button

- Gated by `room_semantics`.
- Disabled when unsupported.

2. Zone kind button

- Gated by `zone_semantics`.
- Disabled when unsupported.

3. Save

- Effectively gated by valid draft plus `room_semantics` or `zone_semantics`.
- Adapter additionally checks `map_annotations`.
- If unsupported at adapter level, save returns an explicit unsupported error.

4. Clean selected room

- Gated by `room_cleaning`, readiness, cleanable target, and no conflicting active workflow.
- Disabled when unsupported.

5. Clean selected zone

- Gated by `zone_cleaning`, readiness, cleanable target, and no conflicting active workflow.
- Disabled when unsupported.

6. Pause

- Gated by `pause_mission` and `activeMission.availableActions` containing `pause_mission`.

7. Resume

- Gated by `resume_mission` and `activeMission.availableActions` containing `resume_mission`.

8. Cancel

- Gated by `cancel_mission` and `activeMission.availableActions` containing `cancel_mission`.

9. Retry and skip

- Adapter capabilities and commands exist.
- Clean Area exposes these controls.
- Rooms/Zones currently does not expose retry/skip controls even though it uses the same coverage runtime. This is a UI gap.

10. Mission lifecycle card

- Shows dock, return-to-dock, battery, and charging capability/status.
- `return_to_dock` button is disabled when unsupported.

Valetudo stub behavior:

- Annotation persistence is explicitly unsupported.
- Room semantics are explicitly unsupported.
- Zone semantics are explicitly unsupported.
- Room cleaning is explicitly unsupported.
- Zone cleaning is explicitly unsupported.
- Coverage missions are explicitly unsupported.
- Pause/cancel mission map only when the relevant Valetudo basic controls are available.
- Unsupported commands return explicit unsupported responses.

The UI branches on capabilities and normalized state rather than backend identity.

## 3. Durability and Reload Behavior

### All Non-Durable State

Acceptable local presentation state:

- active mode tab
- selected room/zone id
- drawing tool active flag
- map viewport zoom/pan/follow mode
- command error messages
- dismissed terminal navigation target
- dismissed terminal coverage mission
- local route preview
- recent card display limit

Risky state that should survive reload for operator continuity:

- selected room/zone id
- in-progress draft rectangle/name/kind
- whether the operator was in Rooms / Zones mode
- map viewport focused on the selected room/zone

State that should move to VM/runtime ownership:

- saved map annotations
- recent mission summaries
- active coverage progress fallback
- any mission result history needed after reconnect

State that should be tied to accepted/loaded map identity:

- saved annotations
- selected saved annotation
- room/zone recent mission summaries
- future annotation edit history
- any cached target preview metadata

### Minimal VM-Owned Persistence Model

The current adapter contract is already close to sufficient. The minimal persistence milestone should preserve these public surfaces:

```ts
snapshot.map.annotations
snapshot.missions.recent
save_map_annotation
delete_map_annotation
```

Storage location:

- Store with VM map data, not browser local storage.
- Use a per-map directory or per-map metadata file under the VM's map storage root.
- Each accepted/loaded map should have its own annotation document.
- Recent mission summaries can be stored per map or globally with map id references.

Suggested annotation schema:

```json
{
  "schemaVersion": 1,
  "mapId": "stable-map-id",
  "mapIdentity": {
    "activeMapName": "optional-name",
    "loadedMapPath": "optional-path",
    "savedMapPath": "optional-path",
    "mapChecksum": "optional-checksum"
  },
  "annotations": [
    {
      "id": "room-...",
      "kind": "room",
      "name": "Living Room",
      "area": {
        "shape": "rectangle",
        "minX": 0,
        "minY": 0,
        "maxX": 1,
        "maxY": 1
      },
      "mapId": "stable-map-id",
      "createdAt": 0,
      "updatedAt": 0
    }
  ]
}
```

Suggested recent mission schema:

```json
{
  "schemaVersion": 1,
  "missions": [
    {
      "id": "mission-id",
      "type": "room_cleaning",
      "status": "completed",
      "backendSource": "turtlebot4_nav2",
      "startedAt": 0,
      "updatedAt": 0,
      "requestedCommand": "start_room_cleaning",
      "phase": "completed",
      "progress": {
        "percent": 1,
        "currentStep": 4,
        "totalSteps": 4,
        "distanceRemaining": 0,
        "areaCoveredSqM": 3,
        "areaRemainingSqM": 0
      },
      "result": {
        "status": "completed",
        "completedAt": 0,
        "summary": "Living Room completed."
      },
      "error": null,
      "target": {
        "area": {
          "shape": "rectangle",
          "minX": 0,
          "minY": 0,
          "maxX": 1,
          "maxY": 1
        },
        "annotation": {
          "id": "room-...",
          "kind": "room",
          "name": "Living Room",
          "mapId": "stable-map-id"
        }
      }
    }
  ]
}
```

Map identity keying:

- Prefer a stable VM-generated map id.
- Include accepted/loaded map name/path as metadata, not as the only durable id.
- Include a checksum or generated UUID when accepting a map.
- Avoid using `"live-map"` as a durable identity except for temporary unsaved sessions.

Migration from webview storage:

1. On first connection to a runtime that supports VM annotation persistence, read local webview annotations for the matching old key.
2. Fetch VM annotations for the active map.
3. If VM annotations are empty, upload local annotations once.
4. If VM annotations are non-empty, do not overwrite them automatically.
5. Mark migration complete per map id in local storage.
6. Stop reading local storage as authoritative after migration.

Hydration flow on webview open:

1. Adapter connects to runtime.
2. Runtime reports active/loaded/accepted map identity.
3. Adapter fetches or receives map annotations for that map.
4. Adapter exposes annotations through `snapshot.map.annotations`.
5. Adapter fetches or receives active mission and recent mission summaries.
6. Adapter exposes active mission through `snapshot.activeMission`.
7. Adapter exposes terminal history through `snapshot.missions.recent`.
8. UI derives preview and display from snapshots.

Failure behavior:

- If annotation persistence read fails, expose an adapter fault/detail and an empty or stale-safe annotation list.
- If save fails, return an explicit command error instead of pretending the annotation was durably saved.
- If recent-history persistence fails, active mission should still hydrate, but terminal history should show a clear degraded behavior in diagnostics.
- If map identity is unknown, save can either be blocked or explicitly marked session-only; it should not silently persist under an ambiguous durable key.

Test plan:

- Save annotation, reload webview, annotation hydrates from VM.
- Delete annotation, reload webview, annotation remains deleted.
- Accept map A, save room, load map B, confirm map A annotation does not appear.
- Return to map A, annotation returns.
- Start room clean, complete mission, reload, summary persists.
- Runtime returns no recent history, adapter does not invent durable history.
- Migration uploads local annotations only when VM store is empty.
- Corrupt annotation file is handled without crashing snapshot hydration.
- Unsupported Valetudo stub continues to return explicit unsupported behavior.

### Exact Current Reload/Map-Change Behavior

1. User creates rooms

- Unsaved drafts live only in UI memory.
- Saved annotations are stored in browser local storage under the current map key.
- Saved annotations appear in `snapshot.map.annotations`.

2. User accepts or loads a different map

- The adapter recomputes the annotation storage key.
- If the new map identity differs, the annotation list changes to whatever is stored under the new key.
- If identity is missing and both maps fall back to `"live-map"`, unrelated live-map annotations can be mixed.

3. User closes/reopens the webview

- Saved annotations return only if browser local storage still contains the matching key and the same map identity is computed.
- Unsaved draft is lost.
- Selected annotation id is lost.
- Recent missions return only from runtime history or local storage fallback.

4. User starts a room clean

- Command is dispatched as `start_room_cleaning`.
- Backend translates it to runtime coverage start.
- UI identifies it as room cleaning if the runtime or optimistic adapter snapshot preserves room intent.

5. User reloads during the mission

- Active mission should rehydrate from runtime mission snapshot.
- The selected annotation id is gone, but the UI can reconstruct an annotation-like selected target from `activeMission.target.annotation` and `activeMission.target.area`.
- If runtime target lacks annotation metadata, room label can degrade to generic room/area text.

6. User reconnects after terminal completion

- If runtime provides recent mission history, Recent Missions hydrates correctly.
- If runtime does not provide recent history, only browser local storage fallback preserves it.
- Another browser/webview/client will not see the local fallback history.

## 4. Coverage Quality

### Current Coverage Route Generation

Room and zone cleaning inherit Clean Area coverage behavior.

How rectangle becomes cleanable cells:

1. The selected rectangle is expressed in map coordinates.
2. The occupancy grid's origin, resolution, width, and height are used to find candidate cells.
3. A cell is considered inside the rectangle if its center is inside the rectangle.
4. Occupancy value `< 0` is unknown.
5. Occupancy value `<= 15` is cleanable free space.
6. Occupancy value `> 15` is occupied.
7. Candidate cells outside grid bounds are counted as out-of-bounds.

How lanes are generated:

1. The planner chooses horizontal lanes when rectangle width is greater than or equal to height.
2. It chooses vertical lanes when height is greater.
3. Lane spacing comes from the coverage profile.
4. The default swath is `0.30 m`.
5. The default desired overlap is `0.6`, producing dense overlapping passes.
6. Lane centers are distributed across the shorter axis.
7. For each lane, nearby cleanable cells are projected onto the pass axis.
8. Gaps in cleanable cells split a lane into segments.
9. Segment endpoints become waypoints.
10. Endpoints at boundaries are extended by goal tolerance plus boundary safety margin.

How occupied/unknown/out-of-bounds cells are skipped:

- Occupied cells become skipped occupied overlay cells.
- Unknown cells become skipped unknown overlay cells.
- Out-of-bounds cells are counted and included in skipped area.
- The planner only uses cleanable free cells for segments.

How tiny disconnected regions are handled:

- Cleanable cells are decomposed into 4-connected regions.
- Regions below `minimumUsefulCleanableRegionSqM` are marked too-small.
- Too-small cells are excluded from route generation and rendered as skipped.

Where route generation is still lane-level:

- Connected regions are used for classification and skipping tiny regions.
- Route generation still creates lanes through available cleanable cells rather than planning each connected component independently.
- There is no component-level ordering, component-specific entry/exit planning, or component-level recovery policy.

Where uncovered cells can remain:

- Nav2 goal tolerance can mark a waypoint complete before the robot physically covers the endpoint.
- Edges/corners can remain uncovered.
- Obstacle-adjacent strips can remain uncovered.
- Small gaps between lanes and grid discretization can leave cells.
- The runtime can finish route steps even if coverage is below ideal.
- The UI treats `95%` coverage as the completion threshold.

### Top 5 Coverage Failure Modes

1. Edge and corner undercoverage

- User sees completed or nearly completed route while some edge/corner cells remain.
- Runtime may report completed if waypoints are done or threshold is met.
- Right panel partially communicates this when coverage is below threshold.
- Fix belongs in planner and runtime completion semantics.

2. Obstacle-adjacent missed cells

- User sees remaining or skipped cells near obstacles/unknown cells.
- Runtime may report completed because it only executed safe route segments.
- Right panel reports skipped area, but does not clearly say "obstacle-adjacent strip".
- Fix belongs first in planner, then UI explanation.

3. Disconnected cleanable islands

- User sees fragmented preview, skipped too-small cells, or cleanable islands not sensibly routed.
- Runtime may complete the main lane route while islands remain skipped or poorly covered.
- Right panel reports skipped area and route count, but not region-level reasons.
- Fix belongs in component-level planner.

4. Unknown or out-of-bounds room annotations

- User sees `Partially cleanable` or `Invalid target`.
- Runtime should not clean unknown/out-of-bounds cells.
- Right panel currently groups unknown, occupied, out-of-bounds, and too-small into one explanation.
- Fix belongs in UI feedback and map-quality workflow; planner should stay conservative.

5. Route finished but coverage remains below desired quality

- User sees terminal result plus remaining cells or a coverage warning.
- Runtime may report completed unless it has stronger incomplete/partial semantics.
- Right panel has some warning text for completed-with-gap.
- Fix belongs in runtime result semantics, planner recovery, and UI result language.

### Clean Area vs Room/Zone Coverage Paths

They are mostly unified already.

Shared behavior:

- same rectangle representation
- same occupancy-grid target classification
- same connected-region decomposition
- same lawnmower waypoint preview
- same coverage profile
- same map overlay renderer
- same runtime coverage mission path
- same normalized active mission hydration

Remaining duplication:

- Clean Area has its own draft/confirm/start local state.
- Rooms/Zones has draft/save/select/start state.
- Clean Area exposes retry/skip controls; Rooms/Zones currently does not.
- Both compute can-start, target status, and route metrics in separate UI paths.

Recommended abstraction:

```ts
type CoverageTarget =
  | {
      kind: "one_off_area";
      area: VacuumCoverageArea;
    }
  | {
      kind: "saved_annotation";
      annotation: VacuumMapAnnotation;
    };
```

Boundary recommendation:

- UI can own target creation and pre-start preview.
- Runtime owns active mission execution.
- Adapter owns backend-neutral commands/snapshots.
- Nav2 routes and waypoint sequencing remain private to the VM runtime.

## 5. Mission Lifecycle

### Room/Zone Lifecycle UI by Status

The UI maps normalized mission statuses into a coverage-style visual state.

`idle`

- No active room/zone mission.
- Saved annotation list, draw, save, delete, and clean controls are shown according to selection/capabilities.
- Operator can create/select/start if no other workflow blocks it.

`preparing`

- Mission is treated as starting/cleaning.
- UI shows active cleaning state.
- Cancel may be available if runtime advertises `cancel_mission`.
- Pause is available only if runtime advertises `pause_mission` in available actions.

`running`

- UI shows "Cleaning".
- Pause and Cancel are shown if available.
- Map shows route/progress overlays from runtime target/progress when present.

`paused`

- UI shows "Paused".
- Resume and Cancel are shown if available.
- Operator can recover by resuming or canceling.

`canceling`

- UI shows "Canceling".
- Controls are mostly disabled unless runtime still advertises actions.
- Operator waits for terminal state.

`completed`

- UI maps it to completed visuals.
- Terminal row appears in Recent Missions.
- Clean Area can keep a terminal area until dismissed; Rooms/Zones relies mainly on recent summaries and saved selection.

`failed`

- UI maps it to failed.
- Error message is shown when mission error exists.
- Recovery depends on available actions; Room/Zones currently lacks retry/skip UI.

`canceled`

- UI maps it to canceled.
- Recent Missions can show canceled result.
- Operator can select/start again after active mission clears.

`needs_assistance`

- UI maps it to failed for clean-area visual state.
- Error message can show from mission error.
- Runtime may advertise recoverable actions, but Rooms/Zones does not expose retry/skip yet.

`unsupported`

- UI maps it to failed.
- Recent summary can show unsupported.
- Operator cannot recover except by using a supported backend/capability path.

Distinguishability from Clean Area and Navigate:

- Active room/zone missions auto-select Rooms mode when mission type or requested command indicates room/zone.
- Navigate uses navigation-specific destination/progress state.
- Clean Area and Rooms/Zones share coverage visuals, so room/zone identity depends on selected annotation, active mission type/requested command, and labels.

### Pause, Resume, Cancel, Retry, Skip Wiring

Room/zone pause:

```text
Pause button
-> checks active room/zone mission exists
-> checks pause_mission capability
-> sends { command: "pause_mission" }
-> backend calls runtime pause service
-> runtime snapshot updates activeMission.status to paused
```

Room/zone resume:

```text
Resume button
-> checks active room/zone mission exists
-> checks resume_mission capability
-> sends { command: "resume_mission" }
-> backend calls runtime resume service
-> runtime snapshot updates activeMission.status
```

Room/zone cancel:

```text
Cancel button
-> checks active room/zone mission exists
-> checks cancel_mission capability
-> sends { command: "cancel_mission" }
-> backend calls runtime cancel service
-> runtime snapshot transitions through canceling to canceled or another terminal state
```

Retry step:

- Adapter command exists.
- Runtime service call exists.
- Clean Area UI exposes it.
- Rooms/Zones UI does not currently expose it.

Skip step:

- Adapter command exists.
- Runtime service call exists.
- Clean Area UI exposes it.
- Rooms/Zones UI does not currently expose it.

Terminal handling:

- Runtime terminal snapshots flow into recent missions.
- Adapter merges recent terminal missions, dedupes by id, sorts newest first, and caps the list.
- UI renders terminal summaries from `snapshot.missions.recent`.

### Legacy Mission State vs Normalized Mission State

Normalized contract:

```ts
snapshot.activeMission
snapshot.missions.active
snapshot.missions.recent
```

Legacy compatibility state:

```ts
snapshot.mission
```

Current usage:

- Active room/zone execution uses `snapshot.activeMission`.
- Recent summaries use `snapshot.missions.recent`.
- The generic Mission Lifecycle card still uses legacy `snapshot.mission`.
- Navigation UI still relies heavily on normalized navigation state as well as active mission bridging.

Migration recommendation:

- VM-owned durability does not need to wait on this.
- Before the next UI milestone, migrate the generic lifecycle card to read `snapshot.activeMission` and capability state directly.
- Keep `snapshot.mission` only as compatibility fallback.

## 6. UI / UX Review

### Operator Perspective

Mode clarity:

- The mode switcher separates Mapping, Navigate, Clean Area, and Rooms.
- Active room/zone missions force the UI back to Rooms mode.
- This is clear enough for the prototype.

Draft vs saved vs selected:

- Draft state is indicated by drawing/editing status and draft validation.
- Saved annotations appear in a list and as map rectangles.
- Selected annotation is highlighted.
- The distinction between "Done drawing" and "Saved" could be clearer.

Save and Clean availability:

- Save is disabled until a valid draft exists and the chosen kind is supported.
- Clean is disabled until a selected target has cleanable cells and the relevant cleaning capability is supported.
- The UI does not always explain every disabled condition directly.

Cleanable/partial/invalid states:

- The UI reports Cleanable, Partially cleanable, or Invalid target.
- Partial explanation is currently broad: occupied, unknown, out of bounds, or too small.
- Operators would benefit from separate counts and clearer cause labels.

Recent missions:

- Useful for terminal room/zone outcomes.
- Preserves annotation labels when runtime includes target annotation metadata.
- Not durable enough until VM-owned history exists.

Error states:

- Command errors are displayed in the card.
- Runtime errors can appear through mission error.
- Some errors are actionable, but partial target and lifecycle recovery states need clearer next actions.

Recovery from mistakes:

- Delete exists, but no confirmation/undo.
- Clear exists, but it can conflate clearing a draft and clearing selection.
- There is no explicit rename/edit saved annotation flow.

### UX Improvements for Room Creation Flow

Prioritized improvements:

1. Replace hardcoded default name with generated defaults:

```text
Room 1
Room 2
Zone 1
Zone 2
```

2. Make draft lifecycle explicit:

```text
No selection
Drawing draft
Unsaved draft
Saved selected
Cleaning selected room
```

3. Change "Done drawing" to a clearer label such as "Finish shape" and keep Save visually primary when an unsaved draft exists.

4. Show why Save is disabled:

- no rectangle
- invalid rectangle
- unsupported room semantics
- unsupported zone semantics

5. Show why Clean is disabled:

- no saved selection
- no cleanable cells
- map not ready
- mission already active
- capability unsupported

6. Split partial target feedback:

- occupied area
- unknown area
- out-of-bounds area
- too-small skipped area

7. Add safe destructive actions:

- confirm delete
- undo delete if easy
- distinguish Clear draft from Clear selection
- cancel active cleaning should remain visually destructive

8. Improve selected-room readability:

- stronger selected map outline
- show selected name in a stable header
- keep map centered or offer "Fit selected"

9. Add edit/rename flow:

- select saved annotation
- choose Edit
- modify geometry/name/kind
- save updates same id

10. Add room/zone retry and skip controls during active coverage-backed room/zone missions.

### Should Rooms/Zones and Clean Area Remain Separate?

Recommendation: keep Rooms/Zones as saved targets and Clean Area as one-off target.

This corresponds to option 3 from the question set.

Why:

- Clean Area is an ephemeral one-off rectangle.
- Rooms/Zones are named, saved map annotations.
- Current planning/rendering code is already shared enough.
- Keeping modes separate is clearer for operators because saved targets and one-off targets have different mental models.
- Fully merging into a generic "Targets" workflow would add UX complexity before persistence is fixed.
- A higher-level Clean mode with tabs may be useful later, but it is premature before VM-owned durability and history are done.

## 7. Testing and Validation

### Existing Tests Covering Room / Zone Prototype

The current regression harness covers these contracts:

1. Command enumeration

- Protects that all public command names are represented.
- Does not protect UI wiring or runtime behavior.
- Type: regression/unit.

2. TurtleBot4/Nav2 command dispatch

- Protects `start_navigation`, `start_coverage`, `start_room_cleaning`, lifecycle actions, and service-call routing.
- Confirms room cleaning maps to coverage runtime start.
- Does not validate live VM behavior.
- Type: unit/regression.

3. Unsupported TurtleBot4 commands

- Protects explicit unsupported responses for vacuum-native commands not supported by TurtleBot4/Nav2.
- Does not validate UI messaging.
- Type: unit/regression.

4. Capability mapping

- Protects support flags for navigation, mapping, coverage, annotations, room semantics, zone semantics, room cleaning, zone cleaning, and mission lifecycle.
- Protects Valetudo stub unsupported behavior.
- Does not validate rendered disabled controls.
- Type: unit/regression.

5. State mapping

- Protects normalized mission state for idle, navigation, coverage, mapping, annotations, and recent room-cleaning summaries.
- Does not test browser local storage, webview reload, or actual runtime snapshots.
- Type: unit/regression.

6. Map metadata parsing

- Protects occupancy grid metadata normalization.
- Does not test full visual rendering.
- Type: unit.

7. Coverage target and planning

- Protects free/occupied/unknown classification, connected-region decomposition, too-small region skipping, lane generation, waypoint density, and coverage marking.
- Does not prove production-quality coverage.
- Type: unit/regression.

8. Valetudo command stub

- Protects explicit unsupported behavior for annotations and room/zone cleaning.
- Does not implement Layer 6 runtime.
- Type: unit/regression.

9. Public contract boundary

- Protects that the public adapter contract does not import Nav2/runtime internals.
- Protects that the Vacuum Control UI does not branch on backend names or backend source.
- Does not catch every possible architecture leak.
- Type: static regression.

High-value missing tests:

- save/delete annotation persistence with storage mock
- annotation storage key changes across map identity changes
- runtime snapshot parsing when room/zone is reported as coverage plus requested command
- component-level disabled/enabled behavior for the room/zone card
- webview reload behavior
- selected annotation reconstruction during active mission
- Recent Missions label fallback behavior
- delete confirmation/undo behavior after UX work

### Manual Validation Checklist

Room creation:

- Enter Rooms mode.
- Activate drawing.
- Draw valid rectangle.
- Confirm draft validation updates.
- Enter room name.
- Save.
- Confirm room appears on map and in list.

Zone creation:

- Switch kind to Zone.
- Draw valid rectangle.
- Save.
- Confirm zone appears on map and in list.

Rename/edit behavior:

- Select saved annotation.
- Confirm name/kind fields populate from selection.
- Confirm current implementation does not provide true saved rename/edit-in-place.

Save/select/delete:

- Save a room.
- Select it from list.
- Select it from map overlay.
- Delete it.
- Confirm it disappears from list and map.

Webview reload:

- Save annotation.
- Close/reopen webview.
- Confirm saved annotation returns when map identity key matches.
- Confirm unsaved draft does not return.
- Confirm selected annotation id does not return.

Map reload/change:

- Save annotation on map A.
- Accept/load map B.
- Confirm annotation set changes.
- Return to map A.
- Confirm annotation returns only if same identity key is restored.

Cleanable target:

- Select fully free saved target.
- Confirm `Cleanable`.
- Confirm Clean button enables when ready.

Partially cleanable target:

- Select target including unknown/occupied/out-of-bounds/too-small cells.
- Confirm `Partially cleanable`.
- Confirm skipped area appears.

Invalid target:

- Select target with no cleanable cells or unsupported shape.
- Confirm `Invalid target`.
- Confirm Clean button disabled.

Start cleaning:

- Select saved room.
- Start cleaning.
- Confirm active mission appears in Rooms mode.
- Confirm progress/route overlays hydrate.

Pause/resume/cancel:

- Pause active room/zone mission if available.
- Resume.
- Cancel.
- Confirm status transitions match runtime snapshot.

Terminal summary:

- Complete, fail, or cancel a room/zone mission.
- Confirm Recent Missions row shows label and result.
- Reopen webview and confirm whether history persists.

Reconnect during active mission:

- Start room cleaning.
- Reload/reconnect.
- Confirm active mission hydrates.
- Confirm label is preserved when runtime target includes annotation metadata.

### Code Paths Only Validated Manually Today

Currently manual/visual:

- drawing, moving, resizing rectangle
- map annotation overlay click selection
- rendered disabled/enabled states
- map overlay readability
- mode switching under active missions
- webview reload behavior
- local storage annotation hydration
- local storage recent mission hydration
- delete/clear UX
- live VM runtime start/pause/resume/cancel behavior

Smallest regression tests to add:

1. Storage-key unit tests for map identity selection.
2. Save/delete annotation tests using a storage mock.
3. Recent mission merge/dedupe tests using terminal room/zone missions.
4. Runtime parser test for coverage mission carrying `requestedCommand: "start_room_cleaning"`.
5. Component test for Room/Zone Clean button disabled reasons.
6. Component test for partial/invalid target labels.
7. State mapper test for active room/zone mission reconstruction from target annotation.

## 8. Next-Focus Decision

### Ranked Next 5 Technical Priorities

Scoring: 1 low, 5 high. Effort score means ease: 5 is easier, 1 is harder.

| Priority | User-visible value | Architecture risk reduction | Effort ease | VM/runtime dependency | Blocks Layer 6? | Rationale |
|---|---:|---:|---:|---:|---:|---|
| VM-owned annotation durability | 5 | 5 | 3 | 5 | 4 | Saved rooms are core product state and currently live in webview storage. |
| VM-owned recent mission history | 4 | 5 | 3 | 5 | 3 | Reconnect/reload truth depends on durable runtime history. |
| Room/zone UX cleanup | 4 | 3 | 4 | 1 | 2 | Improves operator confidence once state is durable. |
| Coverage failure semantics | 4 | 4 | 2 | 3 | 2 | Needed so completed/partial/failed outcomes are truthful. |
| Component-level coverage planning | 3 | 4 | 2 | 3 | 2 | Important for quality, but less urgent than persistence. |

### Recommended Next Milestone

Recommendation: A. VM-owned annotation and mission-history durability.

Why:

- Saved annotations currently persist in browser local storage.
- Recent mission summaries also use browser local storage when runtime history is absent.
- Both are explicitly called out as remaining prototype durability gaps.
- The public adapter contract already has the right shape.
- Moving durability into the VM reduces architecture risk before Layer 6 Valetudo work.
- It also prevents UI polish from being built on the wrong ownership model.

Recommended sequence:

1. VM-owned annotation persistence.
2. VM-owned recent mission history.
3. UI cleanup for draft/save/select/clean clarity.
4. Coverage quality and stronger failure semantics.
5. Layer 6 Valetudo reachability/status/capability/one-command slice.

### What Not To Work On Next

Do not prioritize these yet:

- docking workflow UI
- simulated battery/charging behavior
- battery-aware return/resume
- consumables
- scheduling
- OpenClaw integration
- full Valetudo room/zone workflows before basic Layer 6 reachability
- polygon editing before rectangle annotations are durable
- broad "Targets" workflow redesign before persistence is fixed
- production-grade coverage planner rewrite before mission durability and history are runtime-owned
- any product UI branch on TurtleBot4/Nav2 or Valetudo identity

Tempting but premature:

- Making the Rooms/Zones UI highly polished before saved rooms are VM-owned.
- Adding complex room segmentation before manual annotations are durable.
- Adding real-hardware room cleaning before Valetudo reachability, status, capabilities, and one basic command are validated.
- Treating local webview storage as acceptable persistence because it survives simple reloads.
