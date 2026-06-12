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
