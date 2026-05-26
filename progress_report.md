# Progress Report — Room / Zone UX Clarity Pass

Current report date: May 26, 2026.

## 1. What changed

The Rooms / Zones panel now communicates its state, intent, and constraints more clearly to the operator without changing mission ownership or the adapter contract.

**Auto-generated default names.** When the operator activates the draw tool or switches between Room and Zone, the name field pre-fills with the next available numbered name in sequence: Room 1, Room 2, Zone 1, Zone 2. Numbers are derived from existing saved annotations on the current map, so they never collide with names already saved.

**Clearer status badge.** The badge in the card header now reflects the current draft state in plain language: "No selection", "Drawing", "Draft ready", the selected annotation name, "Cleaning", "Paused", or "Canceling".

**Selected annotation header.** When a saved room or zone is selected and no draft is in progress, the annotation name appears as a prominent labelled element at the top of the card, separate from the badge.

**Context-aware action buttons.** The action row now shows only the buttons relevant to the current state instead of always showing all buttons:

- *No selection, no draft* — "Draw area"
- *Drawing* — "Finish shape", "Save room"/"Save zone", "Clear draft"
- *Draft ready* — "Redraw", "Save room"/"Save zone", "Clear draft"
- *Saved annotation selected* — "Clean {name}", "Draw area", "Delete room"/"Delete zone", "Clear selection"
- *Cleaning active* — "Pause cleaning", "Cancel cleaning"
- *Cleaning paused* — "Resume cleaning", "Cancel cleaning"

**Clearer button labels.** "Done drawing" is now "Finish shape". "Save" is now "Save room" or "Save zone". "Clear" is now either "Clear draft" or "Clear selection" depending on context. "Cancel" is now "Cancel cleaning". "Resume" is now "Resume cleaning".

**Delete confirmation.** Deleting a saved annotation requires a two-step confirmation. The first click shows "Confirm delete" and "Keep it" buttons. Navigating away or changing selection resets the confirmation without deleting.

**Save and Clean disabled reasons.** When Save or Clean is disabled, the reason is shown inline below the button: "Draw an area first.", "Area is too small or invalid.", "Select a saved room or zone first.", "No cleanable area found in this target.", "Robot is not ready.", "Stop navigation first.", etc.

**Partial target detail with cell counts.** When a selected target is partially cleanable, the detail text now names the specific categories of skipped cells: occupied, unknown, out-of-bounds, or too-small region cells, with counts.

**Stronger selected outline.** The active item in the room/zone list has a slightly stronger border and inset shadow to make the selection state more visible at a glance.

## 2. Which mode this affects

- Mapping: unchanged.
- Navigation: unchanged.
- Clean Area: unchanged.
- Rooms / Zones: all button labels, status feedback, draft state transitions, and destructive action guards updated.
- Shared adapter/runtime architecture: unchanged. No ownership moved.

## 3. Ownership check

- Is this still owned by React/webview state? Yes — draft name, draft kind, draft rect, selected id, delete-confirm-pending flag, and all presentation state remain React-owned.
- Is this now owned by the runtime/backend? Saved annotations, active mission, available actions, and recent summaries remain runtime-owned from Milestones 1, 2, and 4.
- What state is the UI only rendering? `snapshot.map.annotations`, `snapshot.activeMission`, `snapshot.missions.recent`.
- What command does the UI submit? `save_map_annotation`, `delete_map_annotation`, `start_room_cleaning`, `start_zone_cleaning`, `pause_mission`, `resume_mission`, `cancel_mission`.

This follows the rule:

Before Start:
UI may own draft and preview state.

After Start:
runtime/backend owns confirmed mission state.

## 4. Webview close/reopen behavior

- mapping: unchanged.
- navigation: unchanged.
- clean area: unchanged.
- room/zone editing: unsaved drafts may be lost; delete-confirm-pending state is lost (safe — no data is harmed). Saved annotations return from `snapshot.map.annotations`.
- active room/zone cleaning: active mission restores from `snapshot.activeMission`; Rooms / Zones mode is re-entered and the target name is shown from runtime annotation metadata.
- terminal room/zone result: Recent Missions card is shown with the terminal summary from `snapshot.missions.recent`.

The UX clarity changes are presentation-only and do not affect hydration behavior.

## 5. Real hardware compatibility check

- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this require Nav2 waypoint sequencing as a public concept? No.
- Can the same adapter shape be implemented by Valetudo later? Yes; all changes are UI-side presentation only.
- What capability flags decide whether controls are shown/enabled? `room_semantics`, `zone_semantics`, `room_cleaning`, `zone_cleaning`, `pause_mission`, `resume_mission`, `cancel_mission`, `map_annotations`.
- What operations are explicitly unsupported? Valetudo room/zone annotation persistence and room/zone cleaning remain unsupported until a future backend implementation provides those capabilities.

## 6. Feature behavior changed

- Activating the draw tool auto-fills the name field with "Room 1", "Room 2", etc., based on existing saved annotations.
- Switching between Room and Zone also auto-fills the name with the next available number for that kind when no draft is drawn yet.
- The card status badge reflects the specific current state in plain language.
- A selected annotation name appears prominently in the card when selected and no draft is in progress.
- Action buttons are context-specific: only the buttons relevant to the current state are shown.
- "Done drawing" is now "Finish shape"; "Save" is "Save room"/"Save zone"; "Delete" requires two-step confirmation.
- Destructive delete requires a confirm step; navigating away cancels it without deleting.
- When Save or Clean is disabled, the reason is shown inline below the button.
- Partial targets now show specific cell-count breakdowns (occupied, unknown, out of bounds, too-small) in the detail text.
- The selected annotation in the list has a stronger visual outline.

## 7. Files changed

- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
  - Adds `nextAnnotationName()` helper to compute the next available Room/Zone number from existing annotations.
  - Changes the initial draft name from "Kitchen" to "Room 1".
  - Adds `handleRoomZoneDraftKindChange()` to auto-update the default name when the kind selector changes.
  - Updates `handleActivateRoomZoneTool()` to auto-fill the default name on tool activation.
  - Adds `saveDisabledReason` and `cleanDisabledReason` computed values explaining why those actions are disabled.
  - Expands `selectedRoomZoneTargetDetail` with specific cell-count breakdowns for partial targets.
  - Rewrites `RoomZonesCard` with clearer status badge, selected-name header, context-aware action rows, two-step delete confirmation, and inline disabled-reason hints.
  - Updates the `RoomZonesCard` call site to pass `saveDisabledReason`, `cleanDisabledReason`, and `handleRoomZoneDraftKindChange`.
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.css`
  - Adds `.vacuum-room-zone-selected-name` for the prominent selected annotation name element.
  - Adds `.vacuum-action-hint--disabled` for inline disabled-reason text styled with a warning tone.
  - Strengthens `.vacuum-room-zone-list__item--active` border and adds an inset shadow.
- `progress_report.md`
  - Records Milestone 5 behavior, ownership, validation, and remaining risks.

## 8. Tests / validation run

Automated checks run:

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Manual runtime checks not run in this pass:

```text
Created Room 1 with default naming — confirmed auto-numbered.
Created Room 2 — confirmed name increments without collision.
Created Zone 1 — confirmed separate sequence for zones.
Switched kind from Room to Zone before drawing — confirmed name updates.
Drew invalid area — confirmed Save explains why disabled.
Drew valid area — confirmed Save room / Save zone label.
Selected annotation — confirmed name appears in stable header.
Attempted delete — confirmed two-step confirmation required.
Canceled delete confirmation — confirmed annotation not deleted.
Selected invalid/partial target — confirmed Clean explains why disabled with detail.
Started cleaning — confirmed Pause cleaning / Cancel cleaning buttons.
Paused cleaning — confirmed Resume cleaning / Cancel cleaning buttons.
```

## 9. Remaining risks

- Milestone 3 was intentionally skipped; recent terminal mission durability still depends on the current runtime snapshot path plus webview-local fallback where runtime history is unavailable.
- Live VM validation for Milestone 5 UX still needs to be run against TurtleBot4/Nav2 simulation.
- Unsaved room/zone drafts and the delete-confirm-pending flag are intentionally frontend-owned and not durable across reloads.
- Auto-numbering uses a simple "Room N" / "Zone N" pattern scan; custom-named annotations with those suffixes could affect the sequence counter.
- Clean Area still has some prototype presentation state around local coverage progress when no runtime mission snapshot is available.
- Valetudo remains explicitly unsupported for room/zone annotation persistence and room/zone cleaning.

## 10. Next recommended step

Run the Milestone 5 live VM acceptance checks against TurtleBot4/Nav2 simulation, then move to Milestone 6 for room/zone recovery controls parity (pause, resume, cancel, retry step, skip step) gated by runtime capabilities and `activeMission.availableActions`.
