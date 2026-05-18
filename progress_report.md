# Runtime-Owned Vacuum Mission Architecture

## Progress Report — May 18, 2026

### Current status

The TurtleBot4/Nav2 simulation path now has runtime-owned mission execution for
mapping, navigation, and Clean Area coverage.

- Mapping auto-exploration is VM-owned and hydrates through adapter/runtime
  snapshots.
- Navigation starts through `start_navigation`; the VM mission runtime owns the
  Nav2 goal and publishes mission state.
- Clean Area starts through `start_coverage`; the VM mission runtime owns route
  generation, Nav2 waypoint sequencing, lifecycle actions, progress, and
  terminal snapshots.
- The webview owns draft UI state before start: selected mode, target draft,
  clean-area rectangle, and local preview.
- After start, the webview renders `snapshot.activeMission` and
  `snapshot.missions`; it does not own active mission authority.

### Latest documentation-relevant change

The latest UI regression pass fixed two operator-facing issues without changing
the adapter contract.

- Clean Area route preview is visually prominent on the map.
- Clean Area route overlays render above coverage cells during preview and
  runtime coverage display.
- Terminal navigation snapshots no longer force the panel back into Navigate
  mode after a run is completed, canceled, or failed.

### Ownership check

- Local UI state: clean-area draft, preview visualization, selected tab/mode,
  and dismissed terminal navigation presentation.
- Runtime/backend state: active mapping, navigation, and coverage execution.
- Adapter-rendered state: active mission, mission history, destination/progress
  snapshots, route/progress overlays, and terminal mission results.
- Runtime commands: `start_navigation`, `start_coverage`, `pause_mission`,
  `resume_mission`, `cancel_mission`, `retry_mission_step`, and
  `skip_mission_step`.

### Webview close/reopen behavior

- Mapping continues in the VM runtime and hydrates from adapter/runtime state.
- Active navigation continues in the VM runtime and hydrates as Navigate.
- Terminal navigation remains visible as context in Navigate mode but no longer
  blocks switching to Mapping or Clean Area.
- Pre-start Clean Area preview is local webview state and is not durable.
- Started Clean Area missions continue in the VM runtime and hydrate from
  `snapshot.activeMission`.

### Real hardware compatibility check

- TurtleBot4/Nav2 specifics remain private to the TurtleBot4 backend adapter.
- Nav2 waypoint sequencing is not a public product concept.
- Valetudo can implement the same adapter command/state shape later.
- Capability flags continue to decide visible/enabled controls:
  `start_coverage`, `coverage_mission`, `pause_mission`, `resume_mission`,
  `cancel_mission`, `retry_mission_step`, and `skip_mission_step`.
- Unsupported operations still flow through adapter capabilities and command
  errors.

### Files changed in the latest pass

- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
  added an active-navigation-only auto-switch check so terminal navigation
  snapshots do not trap the mode switcher.
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.css`
  raised the plan overlay above coverage cells and increased Clean Area route
  stroke visibility.
- `VACUUM_STACK_PLAN.md`, `steps.md`, `extension.md`, and this file now reflect
  the current runtime-owned mission boundary.

### Validation

Passed in the latest code pass:

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Known unrelated typecheck failure:

```sh
bun run typecheck
```

```text
packages/tensorfleet-auth/src/oauth-core.ts(178,13): error TS6133: 'callbackBaseUrl' is declared but its value is never read.
packages/tensorfleet-auth/src/oauth-core.ts(202,13): error TS6133: 'actualPort' is declared but its value is never read.
```

### Remaining risks

- Clean Area preview/execution route visibility still needs a live webview
  visual retest.
- Clean Area pause, cancel, retry, skip, failure handling, and mode locking need
  live VM validation.
- Runtime coverage remains first-pass footprint-history accounting, not
  production-complete coverage.
- Dock/undock and battery-aware return/resume remain future Layer 4 work.

### Next recommended validation

Retest the two recently reported flows in the webview:

1. `Preview Path` should show a visible Clean Area route.
2. After canceling a Navigate run, Mapping and Clean Area should remain
   selectable.
