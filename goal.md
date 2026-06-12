Goal: Implement Valetudo Target Hover and Selection

Why:
We now have normalized Valetudo map targets and a main-canvas static map preview. The next step is to connect the sidebar target inventory with the map preview so operators can hover or select rooms/segments/zones and see the corresponding map area highlighted. This should remain read-only and should not enable cleaning commands yet.

Context:
Current completed work:
- Runtime normalizes Valetudo map metadata, preview layers/entities, and targets.
- Adapter exposes snapshot.map.layeredPreview and snapshot.map.targets.
- Main canvas shows a read-only Valetudo layered map preview.
- Sidebar shows Map Targets from normalized target data.
- Sidebar duplicate preview was removed when main preview is visible.
- capabilities.map.supported remains false for Valetudo full interactive map support.
- Segment cleaning, zone cleaning, room cleaning, go-to, and Clean Area remain unsupported.

Important architecture rule:
Product UI must consume only normalized vacuum_adapter state and capabilities.
Do not use raw Valetudo payloads, raw compressedPixels, raw HTTP routes, source URLs, MQTT topics, SSE/cache internals, ROS subscriptions, vm-manager routes, or backend-specific payloads in product UI.

Product flow remains:
Valetudo source or fixed mock
-> VM-managed Valetudo integration runtime
-> vm-manager proxy path if applicable
-> Valetudo backend adapter
-> normalized vacuum_adapter snapshot/capabilities/commands
-> VacuumControlPanel UI

Goal:
Add read-only target hover and selection for normalized Valetudo map targets.

Scope:
1. Add local UI state for hovered target and selected target.
2. Allow hovering a target row in Map Targets to highlight its corresponding segment/zone on the main map preview.
3. Allow clicking a target row to select it.
4. Show selected target details in the sidebar or within the Map Targets card.
5. Highlight the selected target on the main map preview.
6. Keep hover and selection read-only.
7. Keep all map/cleaning commands disabled.
8. Update tests and progress_report.md.

Non-goals:
- Do not enable segment cleaning.
- Do not enable zone cleaning.
- Do not enable room cleaning.
- Do not enable go-to.
- Do not enable Clean Area.
- Do not enable target execution.
- Do not add drawing/editing.
- Do not add user-created zones.
- Do not add Map SSE/live streaming.
- Do not enable capabilities.map.supported if it means full interactive map support.
- Do not alter TurtleBot4/Nav2 simulation MapCanvas behavior.
- Do not claim hardware support.

Data model expectations:
Use existing normalized data:
- snapshot.map.targets.segments
- snapshot.map.targets.zones
- snapshot.map.layeredPreview.layers
- snapshot.map.layeredPreview.entities

If target geometry is already present on target records, use it.
If target geometry is not present but segmentId links to a preview layer, highlight by matching normalized target id/segmentId to the corresponding preview layer.
If no reliable link exists between a target and preview geometry, do not fake highlighting. Show selection/details only and document the missing link.

Important:
Before implementing, inspect the actual normalized target and preview shapes.
Decide how targets map to preview geometry:
- target.id -> layer.segmentId
- target.id -> entity.id
- target.geometry directly
- another normalized field already present

If the existing normalized contract cannot reliably connect targets to preview geometry, stop and report the gap instead of using raw Valetudo fields in UI.

UI behavior:
- Hover target row: temporary map highlight.
- Click target row: persistent selected highlight.
- Click selected target again or clear button: optional deselect.
- Selected target details may show:
  - label
  - kind
  - availability
  - detail
  - geometry type summary
- Do not show raw coordinates by default.
- Do not show command buttons yet.
- Use labels like “Selected target” or “Read-only target preview”.
- Make it clear cleaning commands are not enabled yet if needed.

Map preview behavior:
- Existing layers render normally.
- Hovered target uses a subtle highlight.
- Selected target uses a stronger highlight.
- If both hover and selection exist, hover may temporarily override or complement selection.
- Robot, charger, restrictions, paths, and walls should remain visible.
- Preserve renderer memoization and reduced SVG node count.
- Avoid re-rendering heavy geometry unnecessarily.

Performance:
- Keep selection state lightweight.
- Memoize derived highlighted layer/entity sets.
- Do not remount the whole preview if only hover state changes unless unavoidable.
- Do not reintroduce duplicate sidebar map preview rendering.

Capability behavior:
- Keep capabilities.map.supported=false if that means full interactive map support.
- Do not enable segment_cleaning, zone_cleaning, room_cleaning, go_to_location, or Clean Area.
- This is selection/preview only.

vm-manager / firecracker-vm:
This should be UI/adapter-only if the target-to-geometry link already exists.
No vm-manager or firecracker-vm changes are expected unless normalized target geometry/linking is missing.
If runtime/adapter data is insufficient, report the gap and propose the smallest normalized field addition.

Testing:
Update or add regression checks for:
- Map Targets rows support hover/selection state.
- Hovered/selected target is passed to the normalized preview renderer.
- Highlighting uses normalized IDs/geometry only.
- No raw Valetudo fields are consumed by UI.
- Commands remain disabled.
- Main preview remains read-only.
- Sidebar duplicate preview does not return.
- Existing simulation MapCanvas behavior remains unchanged.

Validation:
Run:
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check

If firecracker-vm/tensorfleet-mgr is touched unexpectedly, also run:
cd /home/shane/firecracker-vm/tensorfleet-mgr && go test ./...
cd /home/shane/firecracker-vm/tensorfleet-mgr && git diff --check

Done criteria:
- Hovering a segment/room/zone target highlights the corresponding map area when a normalized link exists.
- Clicking a target selects it and keeps the highlight visible.
- Selected target details are visible.
- No command buttons are added.
- No map/segment/zone/go-to/Clean Area commands are enabled.
- No raw Valetudo payloads are used in UI.
- Existing simulation behavior is unchanged.
- Validation passes.
- progress_report.md is updated.

Progress report format:

# Progress Report - Valetudo Target Hover and Selection
Current report date: 2026-06-12.

## 1. What changed
- Describe target hover state.
- Describe target selection state.
- Describe map highlight behavior.
- Describe selected target details.
- Describe whether adapter/runtime/vm-manager/firecracker-vm changed.
- Describe tests added/updated.

## 2. Product behavior
- Explain how hovering targets works.
- Explain how selecting targets works.
- Explain that this is read-only.
- Explain that no cleaning commands were enabled.
- Explain behavior when target geometry/linking is missing.

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
<commands run>