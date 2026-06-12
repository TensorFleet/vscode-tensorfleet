# Progress Report - Valetudo Target Hover and Selection
Current report date: 2026-06-12.

## 1. What changed
- Added local `hoveredMapTargetKey` and `selectedMapTargetKey` state in the Vacuum Control panel.
- Added read-only target row hover/focus and click selection in the Map Targets card.
- Added normalized map highlight resolution for target geometry, `target.id -> layer.id`, `target.id -> layer.segmentId`, and `target.id -> entity.id`.
- Added selected target details in the Map Targets card with label, kind, availability, geometry summary, and optional normalized detail.
- Did not change adapter/runtime/vm-manager/firecracker-vm; the existing normalized target and preview links were sufficient.
- Updated the vacuum adapter regression harness for hover/selection state, renderer highlight plumbing, normalized ID/geometry usage, command non-enablement, sidebar preview deduplication, and unchanged MapCanvas gating.

## 2. Product behavior
- Hovering a normalized target row temporarily highlights matching geometry on the main static layered preview when a normalized link exists.
- Clicking a target row selects it, keeps its highlight visible, and shows selected target details; clicking it again or Clear deselects it.
- This remains read-only: no cleaning, go-to, drawing, editing, target execution, runtime route calls, ROS subscriptions, or SSE/live streaming were added.
- No map, segment, zone, room, go-to, or Clean Area commands were enabled; `capabilities.map.supported` remains false for Valetudo full interactive map support.
- If a target has no normalized geometry and no normalized preview ID/link, the row can still be selected and described, but no fake map highlight is drawn.

## 3. Still deferred
- Segment cleaning command.
- Zone cleaning command.
- Room cleaning command.
- Go-to command.
- User-created zone drawing.
- Map SSE/live streaming.
- Hardware validation.

## 4. Validation

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Result:

- `bun run test:vacuum-adapter`: passed - `vacuum_adapter regression harness passed`.
- `bun run --cwd panels-standalone build`: passed; Vite emitted existing browser-externalization, eval, and chunk-size warnings.
- `git diff --check`: passed.

---

# Progress Report - Valetudo Map Preview Performance and Sidebar Dedup
Current report date: 2026-06-12.

## 1. What changed
- Added an explicit sidebar deduplication gate: when the main canvas layered Valetudo preview is visible, the compact sidebar `MapPreviewCard` is not mounted.
- Kept `MapPreviewCard` as a fallback for the case where normalized preview data exists but the main layered preview branch is absent.
- Memoized expensive layered-preview render data with `useMemo`, including content bounds, `viewBox`, aspect ratio, layer ordering, converted run paths, marker sizing, and legend items.
- Reduced SVG node count by converting each layer's run rectangles into one compound SVG path per layer, while preserving optional point polylines and entity markers.
- Kept the sidebar `MapTargetsCard` visible and unchanged apart from sharing the existing Operate group spacing.
- Did not change the adapter, runtime, vm-manager, or `/home/shane/firecracker-vm/tensorfleet-mgr`; this was a UI/test/report-only pass.
- Updated the adapter regression harness to assert sidebar preview deduplication, shared memoized renderer use, run-to-path conversion, normalized UI boundaries, disabled command gates, and unchanged `MapCanvas` capability gating.

## 2. Product behavior
- The main canvas static Valetudo preview remains visible when normalized `snapshot.map.layeredPreview` exists and the normal map-capable `MapCanvas` path is unavailable.
- The duplicate sidebar Map Preview no longer renders when the main layered preview is visible, so the full SVG renderer is mounted once by default.
- Map Targets remains available in the sidebar from normalized `snapshot.map.targets`.
- The preview remains read-only: no target hover, selection, drawing, editing, runtime route calls, ROS subscriptions, SSE streaming, or command controls were added.
- Commands/capabilities remain disabled for Valetudo interactive map work: segment cleaning, zone cleaning, room cleaning, go-to, Clean Area, and full map support were not enabled.
- Missing, malformed, stale, or unreachable preview data still omits `snapshot.map.layeredPreview`, which falls back to the existing no-map placeholder behavior; malformed/unavailable targets continue to render only when normalized targets exist.

## 3. Still deferred
- Target hover interaction.
- Target selection.
- Segment cleaning command.
- Zone cleaning command.
- Room cleaning command.
- Go-to command.
- User-created zone drawing.
- Map SSE/live streaming.
- Hardware validation.

## 4. Validation

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Result:

- `bun run test:vacuum-adapter`: passed - `vacuum_adapter regression harness passed`.
- `bun run --cwd panels-standalone build`: passed; Vite emitted existing browser-externalization, eval, and chunk-size warnings.
- `git diff --check`: passed.

---

# Progress Report - Valetudo Realistic Saved Map Fixture
Current report date: 2026-06-12.

## 1. What changed
* Inspected `Hypfer/lovelace-valetudo-map-card` only as a visual checklist for common map concepts: floor, walls, segments, robot, charger, paths, zones, no-go/no-mop areas, virtual walls, and obstacles.
* Replaced the fixed mock Valetudo map fixture in `/home/shane/firecracker-vm/tensorfleet-mgr` with a TensorFleet-owned `Home Floor 1` saved-map fixture.
* Added saved-map metadata, four named room segments (`Kitchen`, `Living Room`, `Bedroom`, `Hallway`), three zone-like targets (`Dining Area`, `Entryway`, `Around Sofa`), a charger, robot pose, cleaning path, predicted path, obstacle, no-go area, no-mop area, and virtual wall.
* Extended runtime normalization with read-only product-owned preview entity kinds for `no_go_area`, `no_mop_area`, and `virtual_wall`; malformed optional overlays are omitted without failing the whole map.
* Updated adapter contract/types and preview normalization so the new overlay kinds stay normalized and stale/unreachable/malformed top-level map data still fails closed.
* Updated the sidebar Map Preview renderer and CSS to draw no-go/no-mop polygons, virtual walls, obstacles, paths, zones, rooms, charger, and robot as static read-only preview content.
* Updated Map Targets wording to stay generic and normalized rather than exposing backend payload language.
* No vm-manager proxy change was required; the existing generic proxy forwards the expanded runtime JSON. Firecracker runtime changes were required in `tensorfleet-mgr`.
* Updated Go runtime tests and the `vacuum_adapter` regression harness for named rooms, named zones, overlay normalization/omission, UI boundary checks, and unchanged disabled command capabilities.

## 2. Product behavior
* The sidebar Map Preview now shows a realistic saved vacuum map with floor, walls, room blocks, recent/predicted paths, robot, charger, three zones, no-go/no-mop restrictions, a virtual wall, and an obstacle when those normalized records are present.
* Map Targets now shows useful room/segment labels and zone labels without exposing raw coordinates by default.
* The preview remains read-only: no buttons, drawing, target selection, dragging, or interactive map behavior was added.
* No map, segment, zone, room, go-to, or Clean Area commands were enabled; `capabilities.map.supported` remains false for Valetudo full interactive map support.
* Missing, malformed, stale, and unreachable top-level map data still suppress preview/targets. Malformed optional overlay rows are omitted while valid map content remains available.

## 3. Still deferred
* Main canvas Valetudo map preview.
* Full interactive map rendering.
* Target hover/selection.
* Segment cleaning command.
* Zone cleaning command.
* Go-to command.
* User-created zone drawing.
* Map SSE/live streaming.
* Hardware validation.

## 4. Validation

```sh
cd /home/shane/firecracker-vm/tensorfleet-mgr && go test ./...
cd /home/shane/firecracker-vm/tensorfleet-mgr && git diff --check
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Result:

* `cd /home/shane/firecracker-vm/tensorfleet-mgr && go test ./...`: passed.
* `cd /home/shane/firecracker-vm/tensorfleet-mgr && git diff --check`: passed.
* `bun run test:vacuum-adapter`: passed - `vacuum_adapter regression harness passed`.
* `bun run --cwd panels-standalone build`: passed; Vite emitted existing browser-externalization, eval, and chunk-size warnings.
* `git diff --check`: passed.

---

# Progress Report - Valetudo Static Map Preview Visual Validation
Current report date: 2026-06-12.

## 1. What changed
- Improved the sidebar `MapPreviewCard` SVG renderer so it derives meaningful content bounds from normalized preview layers/entities, fits those bounds into the compact preview frame, and preserves aspect ratio with an explicit `viewBox` transform.
- Added contained coordinate helpers for normalized run-to-map-pixel conversion, point bounds, content bounds, layer ordering, marker sizing, and legend generation inside the preview renderer.
- Rendered floor, segment, path, and wall layers in a deliberate visual order so walls stay legible over segment/floor fills.
- Strengthened robot and charger markers with scale-aware sizing and higher contrast; zone polygons and path-style entities remain rendered when normalized data exists.
- Added a compact read-only legend for present map concepts without adding any controls.
- Updated preview CSS so floor, walls, segment tones, robot, charger, zone, and path all use distinct styling instead of reading as one gray block.
- Confirmed the real fixed mock runtime fixture in `/home/shane/firecracker-vm/tensorfleet-mgr` already has floor, wall, three segment layers, robot, charger, and zone geometry; no vm-manager/firecracker-vm changes were required.
- Updated the adapter regression fixture and assertions so normalized preview coverage includes floor, wall, three segment layers, robot, charger, and zone polygon data.
- Added/updated regression checks for unreachable preview suppression, all-invalid preview suppression, UI consumption of only normalized `snapshot.map.layeredPreview`, explicit renderer transforms, and visual style coverage.

## 2. Product behavior
- The sidebar Map Preview now makes floor, walls, segment areas, robot, charger, zone polygons, and path-style data visually distinguishable when those normalized records are present.
- The preview remains read-only: no buttons, target selection, drawing, dragging, runtime route calls, ROS subscriptions, or map commands were added.
- The preview still lives in the sidebar and the full `MapCanvas` Valetudo path remains untouched.
- Valetudo map commands remain disabled: `capabilities.map.supported` is still false, and segment cleaning, zone cleaning, room cleaning, go-to, and Clean Area remain unsupported.
- When preview data is missing, malformed, stale, or unreachable, `snapshot.map.layeredPreview` remains absent and the card is omitted.

## 3. Still deferred
- Main canvas Valetudo map preview.
- Full interactive map rendering.
- Segment selection.
- Segment cleaning command.
- Zone cleaning command.
- Go-to command.
- User-created zone drawing.
- Map SSE/live streaming.
- Hardware validation.

## 4. Validation

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Result:

- `bun run test:vacuum-adapter`: passed - `vacuum_adapter regression harness passed`.
- `bun run --cwd panels-standalone build`: passed; Vite emitted existing browser-externalization, eval, and chunk-size warnings.
- `git diff --check`: passed.

---

# Progress Report - Valetudo Static Map Preview Foundation

Current report date: 2026-06-12.

## 1. What changed

* Extended the Valetudo runtime map normalization in `/home/shane/firecracker-vm/tensorfleet-mgr` with a conservative `map.preview` section containing product-owned layer/entity kinds, normalized RLE-style runs, normalized points, robot/charger/zone entities, and map diagnostics for omitted malformed rows.
* Fixed mock preview data now exposes renderable floor, wall, three segment layers, robot, charger, and one zone-like polygon from the embedded map fixture.
* Added adapter fields `snapshot.map.layeredPreview` plus typed normalized map layers, entities, and runs; `snapshot.map.grid` remains the ROS/Nav2 occupancy-grid surface and stays `null` for Valetudo.
* Added a compact read-only `MapPreviewCard` in the no-map/basic robot sidebar, before the existing `MapTargetsCard`, rendering only from normalized adapter preview data.
* Capabilities did not enable interactive map behavior: `capabilities.map.supported` remains `false`, and no segment, room, zone, go-to, or Clean Area commands were enabled.
* No vm-manager proxy change was required; the expanded runtime snapshot JSON remains forwarded through the existing flow. Firecracker runtime changes were required in `tensorfleet-mgr`.
* Added/updated tests for fixed mock preview normalization, malformed preview row omission, stale/unreachable preview suppression, invalid metadata preview suppression, adapter preview mapping, UI boundary checks, and existing Valetudo command disablement.
* No live VM deployment was performed.

## 2. Product behavior

* When fresh normalized Valetudo layered preview data exists, the basic/no-map sidebar shows a static **Map Preview** card with floor, wall, segment, robot, charger, path/zone-style geometry if present.
* The preview is read-only: no buttons, target selection, drawing, dragging, route calls, Valetudo HTTP calls, or ROS subscriptions were added.
* Existing TurtleBot4/Nav2 simulation `MapCanvas` behavior is unchanged; `MapCanvas` still depends on the existing normalized map capability/grid path.
* If preview data is missing, metadata-only, malformed, stale, or unreachable, the preview is omitted and the existing no-map placeholder behavior remains.
* Segment cleaning, zone cleaning, room cleaning, go-to, and Clean Area remain unsupported for Valetudo.

## 3. Still deferred

* Full interactive map rendering.
* MapCanvas Valetudo support.
* Segment selection.
* Segment cleaning command.
* Zone cleaning command.
* Go-to command.
* User-created zone drawing.
* Map SSE/live streaming.
* Hardware validation.

## 4. Validation

```sh
cd /home/shane/firecracker-vm/tensorfleet-mgr && go test ./...
cd /home/shane/firecracker-vm/tensorfleet-mgr && git diff --check
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Result:

* `cd /home/shane/firecracker-vm/tensorfleet-mgr && go test ./...`: passed.
* `cd /home/shane/firecracker-vm/tensorfleet-mgr && git diff --check`: passed.
* `bun run test:vacuum-adapter`: passed - `vacuum_adapter regression harness passed`.
* `bun run --cwd panels-standalone build`: passed; Vite emitted existing browser-externalization/eval/chunk-size warnings.
* `git diff --check`: passed.

---

# Progress Report - Valetudo Map Target Inventory UI
Current report date: 2026-06-12.

## 1. What changed
- Added a compact read-only `MapTargetsCard` to the Vacuum Control no-map/basic robot sidebar.
- Placed the card in the Operate group after Current Statistics and before Battery/Dock readiness content, matching the requested priority near operation context without implying command support.
- Gated the card only on normalized `snapshot.map.targets` presence: it renders when either `segments` or `zones` has rows and renders nothing when target lists are absent or empty.
- Rendered target groups with product-owned labels: **Map Targets**, **Segments / Rooms**, and **Zones**.
- Rows show normalized label, kind, availability, optional detail, and a geometry summary such as `Polygon`, `Rectangle`, or `Area available`; raw coordinates and raw Valetudo pixel payloads are not exposed.
- Adapter/runtime/vm-manager/firecracker-vm code was not changed in this UI pass.
- Updated `scripts/vacuum-adapter-regression.ts` with UI boundary checks proving the card consumes normalized targets, keeps generic labels, and does not render raw geometry fields.

## 2. Product behavior
- Normalized segment targets appear in the **Segments / Rooms** section when `snapshot.map.targets.segments` exists.
- Normalized zone-like targets appear in the **Zones** section when `snapshot.map.targets.zones` exists.
- The card is read-only and has no buttons, unsupported command CTA, or map rendering behavior.
- Product map rendering remains unavailable for Valetudo; the existing no-map placeholder remains the map surface when `capabilities.map.supported` is false.
- No segment, room, zone, go-to, Clean Area, or map commands were enabled.
- If no normalized targets exist, including metadata-only layered maps, the card does not render.

## 3. Still deferred
- Full map rendering.
- MapCanvas Valetudo support.
- Segment cleaning command.
- Zone cleaning command.
- Go-to command.
- User-created zone drawing.
- Map SSE/live streaming.
- Hardware validation.

## 4. Validation

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Result:

- `bun run test:vacuum-adapter`: passed - `vacuum_adapter regression harness passed`.
- `bun run --cwd panels-standalone build`: passed; Vite emitted existing browser-externalization/eval/chunk-size warnings.
- `git diff --check`: passed.

---

# Progress Report - Valetudo Map Data Foundation
Current report date: 2026-06-12.

## 1. What changed
- Inspected the local Valetudo checkout at `/home/shane/Valetudo` and confirmed `GET /api/v2/robot/state/map` returns `robot.state.map` from `RobotRouter`.
- Documented the discovered Valetudo map shape in `review.md`: `ValetudoMap` metadata, `size`, `pixelSize`, `layers[]` with RLE `compressedPixels`, and `entities[]` for robot, charger, paths, and zone-like polygons.
- Added a TensorFleet-owned embedded fixed mock map fixture at `firecracker-vm/tensorfleet-mgr/fixtures/valetudo/maps/fixed_mock_map.json`.
- Added VM runtime map normalization in `firecracker-vm/tensorfleet-mgr`: fixed mock fixture loading, HTTP map endpoint polling, conservative metadata extraction, segment target extraction, zone-like polygon extraction, and map diagnostics.
- Added optional normalized `map` to the Valetudo runtime snapshot with `available`, `source`, `metadata`, `targets`, `detail`, and diagnostics.
- Added optional adapter fields `snapshot.map.layeredMetadata` and `snapshot.map.targets`; existing `snapshot.map.grid` remains the ROS/Nav2 occupancy-grid surface and is still `null` for Valetudo.
- No product UI changes were made.
- No vm-manager code change was required after inspection; `/vms/self/tensorfleet/...` is a generic reverse proxy to `tensorfleet-mgr` and forwards expanded JSON without typed Valetudo schema changes.
- Added/updated tests for fixed mock map data, HTTP map data, missing fixture, malformed fixture, stale/unreachable source omission, unsupported map endpoint, malformed map payload, metadata-only map payload, and adapter stale-target suppression.
- No live VM deployment was performed.

## 2. Product behavior
- Product UI behavior is unchanged.
- Valetudo product map rendering remains unavailable: `snapshot.map.grid` stays `null`, `snapshot.map.readiness` stays `unavailable`, and `capabilities.map.supported` stays `false`.
- Internally, fresh trusted Valetudo runtime snapshots can now carry normalized layered map metadata and target inventory for segments and zone-like entities.
- Missing, malformed, stale, unreachable, and unsupported map data do not crash the runtime or adapter and do not expose stale targets as current product truth.
- If map data exists but has no segments or zones, metadata is exposed internally while target lists remain empty and target/cleaning capabilities stay unsupported.
- No segment, zone, room, go-to, Clean Area, or map rendering commands were enabled.

## 3. Still deferred
- Full map rendering.
- MapCanvas Valetudo support.
- Segment/room/zone UI.
- Segment cleaning command.
- Zone cleaning command.
- Go-to command.
- User-created zone drawing.
- Map SSE/live streaming.
- Hardware validation.

## 4. Validation

```sh
cd /home/shane/firecracker-vm/tensorfleet-mgr && go test ./...
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
cd /home/shane/firecracker-vm/tensorfleet-mgr && git diff --check
git diff --check
```

Result:

- `cd /home/shane/firecracker-vm/tensorfleet-mgr && go test ./...`: passed.
- `bun run test:vacuum-adapter`: passed — `vacuum_adapter regression harness passed`.
- `bun run --cwd panels-standalone build`: passed; Vite emitted existing browser-externalization/eval/chunk-size warnings.
- `cd /home/shane/firecracker-vm/tensorfleet-mgr && git diff --check`: passed.
- `git diff --check`: passed.

---

# Progress Report - Valetudo No-Map Sidebar Organization Implementation

Current report date: 2026-06-12.

We are reorganizing the Valetudo/no-map Vacuum Control sidebar using existing normalized surfaces. Current Statistics has moved from its previous lower position to the Operate group immediately after Basic Cleaning Controls. This pass adds TensorFleet-owned grouping and conservative attention indicators without copying Valetudo UI/UX and without adding backend/runtime contract changes.

## 1. What changed

- Moved `CurrentStatisticsCard` from position 7 (after `CleaningSettingsCard`) to position 3 (after `BasicControlsCard`) in the Valetudo/no-map sidebar. All other relative card positions are preserved.
- Added five non-interactive visual group sections to the no-map sidebar using the existing `vacuum-card-group` CSS infrastructure (already styled but previously unused in the basic robot profile branch):
  - **Operate**: Robot Overview, Basic Cleaning Controls, Current Statistics
  - **Readiness**: Battery and Dock, Attachments, Dock Components
  - **Configure**: Cleaning Settings (rendered only when supported)
  - **Maintain**: Maintenance / Consumables (rendered only when supported)
  - **Context**: Source / Health (rendered only when supported)
- Group wrappers for Configure, Maintain, and Context are conditionally rendered: each wrapper is only emitted when its contained card would render, so no floating empty group headers appear.
- Added conservative per-card attention badge (`vacuum-state-badge vacuum-state-badge--warning` with count) to `AttachmentsCard`, `DockComponentsCard`, and `MaintenanceCard` card headers when attention-worthy items exist:
  - Attachments: `missing` on any kind, `error` on any kind, `full` on dustbin, `empty` on water_tank, `low` on water_tank or detergent.
  - Dock Components: `missing` on any kind, `error` on any kind, `full` on wastewater or dustbag, `empty` on freshwater, `low` on freshwater or detergent.
  - Maintenance: `warning`, `replace_soon`, or `replace_now` consumable status.
  - Severity is derived from explicit (kind, status) pair logic, not status alone.
- Added compact Robot Summary secondary attention line via new `peripheralAttentionLine` prop on `RobotOverviewCard`. The line is derived from normalized attachment, dock component, and consumable data. It only renders when no primary robot/source fault is active (i.e., `compactDetail` is null), to avoid duplicating fault context. Items beyond three are collapsed with `+N more`. A new `.vacuum-robot-overview__peripheral-attention` CSS rule styles it in warning color.
- Added five new helper functions (`isAttachmentAttentionWorthy`, `isDockComponentAttentionWorthy`, `isConsumableAttentionWorthy`, `formatAttachmentAttentionLabel`, `formatDockComponentAttentionLabel`) and one derivation function (`derivePeripheralAttentionLine`). All derive only from existing normalized `VacuumAttachmentState`, `VacuumDockComponentState`, and `VacuumMaintenanceState` fields.
- Pre-computed sidebar visibility predicates (`sidebarShow*` constants) before the JSX return in `VacuumControlPanelContent` to gate group wrappers without duplicating card-internal logic.
- No vm-manager or firecracker-vm changes were required. The implementation consumes only existing normalized `vacuum_adapter` surfaces.
- Collapse behavior remains deferred. Cards are all expanded (no collapse/expand state was added).
- No new adapter fields, capabilities, runtime fields, or backend-specific payloads were introduced.
- Tests updated: existing adapter regression harness continues to pass. No UI layout tests existed to update for card order.

## 2. Product behavior

- The Valetudo/no-map sidebar now renders cards in the following order: Robot Overview → Basic Cleaning Controls → Current Statistics → Battery and Dock → Attachments → Dock Components → Cleaning Settings → Maintenance → Source / Health.
- Current Statistics now appears immediately after Basic Cleaning Controls in the Operate group, making active-run duration and area immediately visible without scrolling.
- Cards are visually grouped under labeled section headers (Operate, Readiness, Configure, Maintain, Context). Headers are non-interactive static labels using `vacuum-card-group__head--static`.
- Attachments, Dock Components, and Maintenance card headers show a warning badge with a count of attention-worthy items when any exist. A count of zero suppresses the badge entirely.
- When no primary robot/source fault is active, the Robot Overview card shows a compact secondary line (e.g., "Dustbin full · Freshwater empty") derived from normalized attachment, dock component, and consumable attention conditions. When a fault is active, the line is suppressed to avoid duplication.
- Optional cards (Current Statistics, Attachments, Dock Components, Cleaning Settings, Maintenance, Source/Health) still gate on normalized capabilities and snapshot presence. No placeholder cards or empty-state rows render for unsupported capabilities.
- Group wrappers for Configure, Maintain, and Context are also conditionally rendered and do not appear when their contained card is absent.
- The full product flow is unchanged: Valetudo source or fixed mock → VM-managed Valetudo integration runtime → vm-manager proxy path if applicable → Valetudo backend adapter → normalized vacuum_adapter snapshot/capabilities/commands → VacuumControlPanel UI. This is a UI-only pass.

## 3. Still deferred

- Full collapse/expand behavior. Cards are all expanded. Collapse state is explicitly deferred for a future pass; when implemented, it must be local UI state only and must not require adapter/runtime changes.
- Explicit `severity` field on attachment and dock component items. Severity inference currently uses (kind, status) pair logic in `isAttachmentAttentionWorthy` and `isDockComponentAttentionWorthy`; a future adapter-side `severity: "ok" | "warning" | "error"` field would replace this inference.
- Dock action commands.
- Total statistics.
- Operation mode.
- Consumable reset commands.
- Map rendering and segment/room/zone/go-to behavior.
- Hardware validation.

## 4. Validation

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Result:

- `bun run test:vacuum-adapter`: passed — `vacuum_adapter regression harness passed`
- `bun run --cwd panels-standalone build`: passed — built in ~10.5s, zero errors, zero type errors
- `git diff --check`: passed — no whitespace errors

---

# Progress Report - Attachments and Dock Components Normalization
Current report date: 2026-06-12.

We are adding read-only normalized Attachments and Dock Components surfaces for the Valetudo path. Attachments represent robot-side installed/present equipment and material state, while Dock Components represent dock-side material readiness. The UI must only consume normalized vacuum_adapter state and capabilities, never raw Valetudo capability names, HTTP routes, MQTT topics, source URLs, SSE/cache internals, or backend-specific payloads.

## 1. What changed
- Extended the VM-managed Valetudo runtime snapshot with optional `attachments.items[]` and `dock.components[]`.
- Added fixed mock default data for dustbin, water tank, mop, detergent, freshwater, wastewater, dock detergent, and dustbag.
- Added an attention fixed mock scenario for full dustbin, low water tank, missing mop, low detergent, empty freshwater, full wastewater, and missing dustbag.
- Added conservative HTTP-source normalization for explicit attachment/component state attributes only; raw capability names and model names do not imply support.
- Added malformed-row handling: rows without meaningful identity are dropped, unknown kinds/statuses normalize to `unknown`, and invalid percentages are omitted.
- Extended the normalized `vacuum_adapter` contract with `snapshot.attachments` and `snapshot.dock.components`.
- Added read-only `attachments` and `dock_components` capabilities with empty command lists and kind attributes when trusted rows exist.
- Added compact read-only Attachments and Dock Components cards in the Valetudo/no-map Vacuum Control sidebar.
- Updated `firecracker-vm/tensorfleet-mgr` runtime structs, fixed mock construction, HTTP source normalization, and handler tests.
- No `vm-manager` code change was required after inspection; `/vms/self/tensorfleet/...` uses a generic reverse proxy to the guest runtime and forwards expanded JSON without typed schema changes.
- Synced the updated `tensorfleet-mgr` binary to the live VM at `172.16.0.11`, backed up the previous binary as `/usr/local/bin/tensorfleet-mgr.backup-20260612T033851Z`, and restarted `tensorfleet-mgr`.
- Verified the live service is active on port `9090`; the current `valetudo_mock_http` source does not provide trusted attachment/component rows, so the live service correctly omits populated readiness rows.
- Verified the deployed binary on the VM with a temporary fixed-mock smoke on port `19090`; it returned 4 attachment rows and 4 dock component rows, then the temporary process was stopped.
- Added/updated regression tests for fixed mock data, capability support, malformed data, stale/unreachable omission, and existing adapter behavior.

## 2. Product behavior
- Operators on the Valetudo/no-map path can now see compact Attachments and Dock Components readiness cards when fresh normalized data exists.
- Attachments show robot-side installed/present/material state such as dustbin, water tank, mop, and detergent.
- Dock Components show dock-side readiness such as freshwater, wastewater, detergent, and dustbag.
- Cards appear only when normalized capabilities are supported and normalized rows exist.
- Missing, stale, unreachable, or offline data omits the new surfaces and leaves capabilities unsupported/unavailable rather than showing stale readiness as current truth.
- Malformed rows do not crash snapshot mapping and do not expose misleading rows.
- The feature is read-only and backend-neutral; no dock action, reset, wash, dry, refill, or empty commands were added.

## 3. Still deferred
- Dock action commands.
- Auto-empty command implementation.
- Mop wash/dry command implementation.
- Water refill command implementation.
- Consumable reset commands.
- Total statistics.
- Statistics history/charts/export.
- Map rendering and segment/room/zone/go-to behavior.
- Valetudo Clean Area behavior.
- Hardware validation.

## 4. Validation

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
cd /home/shane/firecracker-vm/tensorfleet-mgr && go test ./...
cd /home/shane/firecracker-vm/tensorfleet-mgr && git diff --check
curl -fsS -H 'Authorization: Bearer default-tensorfleet-token' http://172.16.0.11:9090/api/v1/valetudo/snapshot
PORT=19090 VALETUDO_RUNTIME_SOURCE_MODE=fixed_mock /usr/local/bin/tensorfleet-mgr
curl -fsS -H 'Authorization: Bearer default-tensorfleet-token' http://172.16.0.11:19090/api/v1/valetudo/snapshot
```
