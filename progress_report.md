# Progress Report — Reload / Reconnect Hardening

Current report date: May 26, 2026.

## 1. What changed

Rooms / Zones now recovers clearer state after the webview closes or reloads during and after room/zone cleaning.

When a runtime snapshot reports an active room or zone cleaning mission, the panel returns to Rooms / Zones mode, reconstructs the selected target from `snapshot.activeMission.target.annotation`, and keeps the active target outlined on the map. Paused room/zone missions hydrate as paused from runtime state, with resume/cancel controls gated by runtime capabilities and `activeMission.availableActions`.

When the webview reloads after a terminal room/zone result and the runtime exposes only a recent terminal summary, the panel performs a one-time return to Rooms / Zones mode so Recent Missions is visible. Recent terminal room/zone summaries keep using `snapshot.missions.recent`; the UI does not invent terminal truth from React state.

Terminal room/zone missions are no longer treated as actively cleaning for the Rooms / Zones action row. The saved or recovered target remains visible, while active controls are reserved for non-terminal runtime states.

## 2. Which mode this affects

- Mapping: unchanged.
- Navigation: unchanged, except Rooms / Zones terminal recovery no longer leaves the operator stranded in Navigate mode after reload.
- Clean Area: unchanged for command behavior; shared coverage mission hydration remains the source for active cleaning state.
- Rooms / Zones: active, paused, and terminal room/zone reload behavior is hardened.
- Shared adapter/runtime architecture: regression coverage now verifies paused room-cleaning hydration, target annotation preservation, and terminal room-cleaning summaries through normalized snapshots.

## 3. Ownership check

- Is this still owned by React/webview state? Unsaved room/zone drafts, draft names, selected id, dismissed local messages, and the one-time presentation choice to return to Rooms / Zones after reload remain webview state.
- Is this now owned by the runtime/backend? Saved annotations are runtime-owned from Milestones 1-2. Active room/zone mission state, paused state, available actions, target annotation metadata, and terminal summaries are read from runtime snapshots.
- What state is the UI only rendering? The UI renders saved annotations from `snapshot.map.annotations`, active mission context from `snapshot.activeMission`, and terminal summaries from `snapshot.missions.recent`.
- What command does the UI submit? The UI submits normalized commands only: `start_room_cleaning`, `start_zone_cleaning`, `pause_mission`, `resume_mission`, `cancel_mission`, `save_map_annotation`, and `delete_map_annotation`.

This follows the rule:

Before Start:
UI may own draft and preview state.

After Start:
runtime/backend owns confirmed mission state.

## 4. Webview close/reopen behavior

- mapping: mapping status still hydrates from runtime mapping snapshots.
- navigation: active navigation still hydrates from `snapshot.activeMission` or normalized navigation state.
- clean area: active coverage missions still hydrate from runtime mission snapshots.
- room/zone editing: unsaved drafts may be lost; saved annotations return from `snapshot.map.annotations`.
- active room/zone cleaning: the panel returns to Rooms / Zones mode, reconstructs the selected target from runtime mission metadata, and renders progress/status from `snapshot.activeMission`.
- terminal room/zone result: Recent Missions is visible after reload when the runtime exposes the terminal summary through `snapshot.missions.recent`.

After reopening, the UI hydrates from adapter snapshots. It does not reconstruct active room/zone mission authority from prior React state.

## 5. Real hardware compatibility check

- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this require Nav2 waypoint sequencing as a public concept? No.
- Can the same adapter shape be implemented by Valetudo later? Yes; the UI reads normalized missions, annotations, capabilities, and available actions.
- What capability flags decide whether controls are shown/enabled? `room_cleaning`, `zone_cleaning`, `pause_mission`, `resume_mission`, `cancel_mission`, `room_semantics`, `zone_semantics`, and `map_annotations`.
- What operations are explicitly unsupported? Valetudo room/zone annotation persistence and room/zone cleaning remain unsupported in the current stub until a future backend implementation provides those capabilities.

## 6. Feature behavior changed

- Reload during active room/zone cleaning returns the operator to Rooms / Zones mode.
- Reload during paused room/zone cleaning preserves the paused status and recovery controls from runtime state.
- Active room/zone target outlines can recover from `snapshot.activeMission.target.annotation`.
- Reload after a terminal room/zone result can show Recent Missions without requiring the operator to manually switch modes.
- Completed, canceled, failed, or unsupported room/zone missions are no longer shown as actively cleaning in the Rooms / Zones action row.

## 7. Files changed

- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
  - Hardens Rooms / Zones mode recovery from active and recent runtime mission snapshots.
  - Uses runtime annotation metadata to recover selected target presentation after reload.
  - Separates non-terminal active cleaning controls from terminal room/zone results.
- `scripts/vacuum-adapter-regression.ts`
  - Adds regression checks for paused room-cleaning hydration, annotation metadata preservation, and terminal room-cleaning recent summaries.
- `progress_report.md`
  - Records Milestone 4 behavior, ownership, validation, and remaining risks.

## 8. Tests / validation run

Automated checks run:

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Manual runtime checks not run in this pass:

```text
Started Clean Living Room, reloaded webview during active mission, and confirmed Rooms / Zones restored active context.
Paused room cleaning, reloaded webview, and confirmed paused state hydrated correctly.
Reloaded after terminal state and confirmed Recent Missions showed the terminal summary.
Restarted/reconnected the panel after terminal state and confirmed the same recent summary hydrated.
```

## 9. Remaining risks

- Milestone 3 was intentionally skipped, so recent terminal mission durability still depends on the current runtime snapshot path plus existing webview-local fallback where runtime history is unavailable.
- Live VM reload/reconnect validation still needs to be run against TurtleBot4/Nav2 simulation.
- Unsaved room/zone drafts remain intentionally frontend-owned and are not durable.
- Clean Area still has some prototype presentation state around local coverage progress when no runtime mission snapshot is available.
- Valetudo remains explicitly unsupported for room/zone annotation persistence and room/zone cleaning.

## 10. Next recommended step

Run the Milestone 4 live VM acceptance checks against TurtleBot4/Nav2 simulation, then move to Milestone 5 for the Rooms / Zones UX clarity pass.
