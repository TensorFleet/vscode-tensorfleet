Goal: Implement Valetudo Segment / Zone Target Inventory UI

Why:
We now have a Valetudo map data foundation. The VM-managed Valetudo runtime can fetch or load Valetudo-like map data and normalize map metadata plus segment/zone-like targets into the adapter path. The next useful product slice is to display this normalized target inventory in the UI without enabling map rendering or cleaning commands.

Context:
Valetudo map data is not ROS/Nav2 OccupancyGrid data. The existing simulation map path uses grid/pose/Nav2-style data, while Valetudo exposes a layer/entity map model with size, pixelSize, layers, compressedPixels, and entities. Do not force Valetudo map data into snapshot.map.grid. Keep grid as the ROS/Nav2 surface. Use the newly normalized Valetudo map metadata/target fields.

Relevant docs:
- VACUUM_STACK_PLAN.md
- extension.md
- review.md
- progress_report.md
- Valetudo Map Data Foundation Review
- Valetudo Map Data Foundation progress report

Current completed foundation:
- Runtime inspected Valetudo endpoint GET /api/v2/robot/state/map.
- Runtime added fixed mock map fixture.
- Runtime added optional normalized map section with metadata and targets.
- Adapter added optional snapshot.map.layeredMetadata and snapshot.map.targets.
- Product map rendering remains unavailable.
- snapshot.map.grid remains null for Valetudo.
- capabilities.map.supported remains false for Valetudo.
- Segment/zone/go-to commands remain unsupported.

Scope:
Implement a read-only UI surface for normalized Valetudo map target inventory.

Primary target:
Add a compact read-only card or panel that displays normalized map targets when they exist:
- Segments
- Zone-like targets

Suggested UI label:
Map Targets

Suggested sections:
- Segments / Rooms
- Zones

Suggested row fields:
- label
- kind
- availability/status if present
- optional detail
- optional geometry summary, such as “Polygon”, “Rectangle”, or “Area available”, but do not expose raw coordinates by default

Non-goals:
- Do not implement full map rendering.
- Do not modify MapCanvas for Valetudo rendering yet.
- Do not enable capabilities.map.supported.
- Do not enable segment_cleaning.
- Do not enable zone_cleaning.
- Do not enable room_cleaning.
- Do not enable go_to_location.
- Do not add Clean Area behavior.
- Do not add user-created zone drawing.
- Do not add Map SSE/live streaming.
- Do not expose raw Valetudo payloads, compressedPixels, raw entity arrays, HTTP routes, source URLs, SSE/cache internals, or backend-specific payload shapes in product UI.
- Do not claim hardware support.

Architecture constraints:
Product UI must consume only normalized vacuum_adapter state and capabilities.
The product flow remains:
Valetudo source or fixed mock
-> VM-managed Valetudo integration runtime
-> vm-manager proxy path if applicable
-> Valetudo backend adapter
-> normalized vacuum_adapter snapshot/capabilities/commands
-> VacuumControlPanel UI

vm-manager / firecracker-vm:
This should be a UI/adapter consumption pass only if snapshot.map.targets already exists in the adapter.
No firecracker-vm/tensorfleet-mgr change is expected unless the target data is incomplete or incorrectly shaped.
No vm-manager change is expected because the generic proxy already forwards expanded JSON.
If you discover that runtime or adapter fields are missing, stop and report the gap instead of inventing raw UI access.

Implementation steps:
1. Inspect the current adapter types for snapshot.map.layeredMetadata and snapshot.map.targets.
2. Confirm the exact target shape used by the adapter.
3. Add a read-only Map Targets card/component in the Vacuum Control UI.
4. Gate it only on normalized target presence, not raw Valetudo capability names.
5. Render segment targets separately from zone-like targets.
6. Keep labels product-owned and generic:
   - “Segments / Rooms”
   - “Zones”
   - “Map Targets”
7. If no targets exist, do not render the card.
8. If layered map metadata exists but no targets exist, do not render fake target rows.
9. Preserve existing no-map behavior and no-map placeholder.
10. Do not change command availability.
11. Add or update regression coverage if existing tests cover adapter target mapping or UI gating.
12. Update progress_report.md.

UI placement recommendation:
Place the Map Targets card in the Valetudo/no-map sidebar after Current Statistics or after Battery/Dock.

Preferred order:
1. Robot Overview
2. Basic Cleaning Controls
3. Current Statistics
4. Map Targets
5. Battery and Dock
6. Attachments
7. Dock Components
8. Cleaning Settings
9. Maintenance
10. Source / Health

Reason:
Map Targets are related to future operation selection, but they are read-only for now. Placing them near Operate makes them discoverable without implying commands are available.

Card behavior:
- Read-only.
- Compact.
- No buttons.
- No raw geometry dump.
- No unsupported command CTA.
- Optional detail line: “Map targets discovered from normalized Valetudo map data. Cleaning commands are not enabled yet.”
- Zone-like entities should be described carefully as “zone-like targets” or “zones from map data” unless persistence semantics are proven.

Validation:
Run:
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check

If firecracker-vm/tensorfleet-mgr is touched unexpectedly, also run:
cd /home/shane/firecracker-vm/tensorfleet-mgr && go test ./...
cd /home/shane/firecracker-vm/tensorfleet-mgr && git diff --check

Done criteria:
- UI displays normalized segment targets when present.
- UI displays normalized zone-like targets when present.
- UI does not render the card when no normalized targets exist.
- UI does not use raw Valetudo payloads.
- Product map rendering remains unavailable.
- No map/segment/zone/go-to commands are enabled.
- Existing simulation map behavior remains unchanged.
- Validation passes.
- progress_report.md is updated.

Progress report format:

# Progress Report - Valetudo Map Target Inventory UI
Current report date: 2026-06-12.

## 1. What changed
- Describe UI/component changes.
- Describe card placement.
- Describe target gating behavior.
- Describe whether adapter/runtime/vm-manager/firecracker-vm changed.
- Describe tests added/updated.

## 2. Product behavior
- Explain how normalized map targets appear.
- Explain that this is read-only.
- Explain that map rendering remains unavailable.
- Explain that no segment/zone/go-to commands were enabled.
- Explain behavior when targets are missing.

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
<commands run>
