# Progress Report — Room / Zone Recovery Controls and Result Semantics

Current report date: May 27, 2026.

## 1. What changed

Room/zone cleaning now has recovery-control parity with Clean Area. When the runtime reports the actions as available, the Rooms / Zones panel can show and dispatch pause, resume, cancel, retry step, and skip step for the active room or zone mission.

Room/zone cleaning results now preserve product-level target identity and coverage outcome details from the VM runtime. Runtime snapshots keep room/zone mission type, requested command, annotation id/kind/name/mapId, cleaned area, remaining area, skipped area, skipped reasons, route completion, and coverage-threshold status.

Recent Missions now renders completed coverage-style results as "Cleaned" or "Partially cleaned" when runtime coverage metadata says remaining/skipped area exists or the completion threshold was not reached.

## 2. Which mode this affects

- Mapping: unchanged.
- Navigation: unchanged.
- Clean Area: unchanged except shared mission result parsing can now preserve result details.
- Rooms / Zones: active room/zone missions now expose retry/skip recovery controls when runtime action gates allow them; terminal labels can distinguish cleaned from partially cleaned.
- Shared adapter/runtime architecture: extended normalized mission result details while keeping backend-specific execution inside the TurtleBot4/Nav2 runtime.

## 3. Ownership check

- Is this still owned by React/webview state? Room/zone draft drawing, draft name, selected id, and presentation state remain React-owned.
- Is this now owned by the runtime/backend? Active mission state, available lifecycle/recovery actions, target annotation metadata, terminal result wording, and coverage result details are runtime-owned.
- What state is the UI only rendering? `snapshot.activeMission`, `snapshot.missions.recent`, `activeMission.availableActions`, `mission.result`, and `mission.target.coverage`.
- What command does the UI submit? `pause_mission`, `resume_mission`, `cancel_mission`, `retry_mission_step`, and `skip_mission_step`.

This follows the rule:

Before Start:
UI may own draft and preview state.

After Start:
runtime/backend owns confirmed mission state.

## 4. Webview close/reopen behavior

- mapping: unchanged.
- navigation: unchanged.
- clean area: active and terminal state still hydrate from runtime snapshots.
- room/zone editing: unsaved drafts may still be lost; saved annotations still hydrate from `snapshot.map.annotations`.
- active room/zone cleaning: active mission context, available actions, selected annotation metadata, and recovery controls hydrate from `snapshot.activeMission`.
- terminal room/zone result: Recent Missions renders runtime-provided result summary and coverage detail from `snapshot.missions.recent`.

After reopening, the UI does not reconstruct mission authority from React state. It renders the runtime snapshot and only keeps local presentation state such as selected id and mode.

## 5. Real hardware compatibility check

- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this require Nav2 waypoint sequencing as a public concept? No; retry/skip remain normalized mission-step actions.
- Can the same adapter shape be implemented by Valetudo later? Yes; Valetudo can provide the same normalized capabilities, available actions, and mission result fields later.
- What capability flags decide whether controls are shown/enabled? `pause_mission`, `resume_mission`, `cancel_mission`, `retry_mission_step`, `skip_mission_step`, `room_cleaning`, and `zone_cleaning`.
- What operations are explicitly unsupported? Valetudo room/zone annotation persistence and room/zone cleaning remain unsupported until that backend maps vendor rooms/zones into the adapter contract.

## 6. Feature behavior changed

- Room/zone active cleaning controls are action-gated by `activeMission.availableActions`.
- Retry step and Skip step now appear for room/zone missions when the runtime reports a recoverable step issue.
- Pause, Resume, and Cancel handlers now refuse to dispatch unless the current runtime action is available.
- Room/zone mission snapshots preserve annotation target metadata from the original product command.
- Terminal room/zone summaries can say cleaned, partially cleaned, canceled, failed, needs assistance, or unsupported.
- Coverage result summaries include cleaned area, remaining area, skipped area, skipped reason counts, route-completed status, and coverage-threshold status.

## 7. Files changed

- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
  - Adds room/zone retry/skip controls and action-gated handlers.
  - Renders "Needs assistance" for recoverable room/zone step failures.
  - Labels terminal coverage-style missions as Cleaned or Partially cleaned from runtime coverage details.
- `panels-standalone/src/vacuum-adapter/state.ts`
  - Allows normalized mission results to carry structured result details.
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/useTurtleBot4Nav2Adapter.ts`
  - Preserves runtime result details when parsing mission snapshots.
- `scripts/vacuum-adapter-regression.ts`
  - Adds regression coverage for room/zone retry/skip available actions and partial-clean result details.
- `/home/shane/firecracker-vm/assets/opt/ros2_ws/src/turtlebot4_firecracker_bringup/scripts/vacuum_mission_node.py`
  - Keeps room/zone mission identity and annotation metadata in runtime snapshots.
  - Allows room/zone missions to use the same pause/resume/retry/skip/cancel runtime path as coverage.
  - Adds runtime-owned coverage result details and operator-readable terminal summaries.
- `progress_report.md`
  - Records Milestone 6 and Milestone 7 behavior, ownership, validation, and remaining risks.

## 8. Tests / validation run

Automated checks run:

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
python3 -m py_compile /home/shane/firecracker-vm/assets/opt/ros2_ws/src/turtlebot4_firecracker_bringup/scripts/vacuum_mission_node.py
git diff --check
git -C /home/shane/firecracker-vm diff --check
```

Additional check attempted:

```sh
npx tsc --noEmit
```

This still fails on existing project-wide type issues outside this pass, including missing declarations for `tensorfleet-ros`, gzweb package typing issues, and RawMessages/SensorView3D strictness errors. The local `replaceAll` and terminal-result typing issues surfaced by this command were fixed, and the production build passes.

Manual runtime checks not run in this pass:

```text
Started room cleaning, forced recoverable step failure, confirmed Retry / Skip appear only when available.
Used Retry step and Skip step from Rooms / Zones and confirmed runtime state transitions.
Canceled room cleaning and confirmed terminal result appears in Recent Missions.
Cleaned a fully cleanable room and confirmed result says cleaned.
Cleaned a partially cleanable room and confirmed result says partially cleaned with skipped/remaining detail.
Forced failed route and confirmed actionable runtime error appears.
```

## 9. Remaining risks

- Milestone 3 was intentionally skipped; durable recent mission history still depends on the current runtime snapshot path plus webview-local fallback where runtime history is unavailable.
- Live VM validation for retry/skip controls and coverage result wording still needs to be run against TurtleBot4/Nav2 simulation.
- Runtime skipped-reason counts currently cover occupied, unknown, and out-of-bounds cells from the VM coverage grid; too-small region accounting remains frontend preview-oriented.
- Coverage result quality still depends on the current coverage planner and pose-based covered-cell tracking.
- Valetudo remains explicitly unsupported for room/zone annotation persistence and room/zone cleaning.

## 10. Next recommended step

Run the Milestone 6 and 7 live VM acceptance checks against TurtleBot4/Nav2 simulation, then decide whether to backfill Milestone 3 runtime-owned recent mission history or start the next small coverage-planner improvement.
