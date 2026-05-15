# Runtime-Owned Vacuum Mission Architecture

## Progress Report — Runtime-Owned Mission Architecture

### 1. What changed

Fixed two runtime-test regressions in the Vacuum Control webview.

- Clean Area route preview is now visually prominent on the map.
- The Clean Area route overlay now renders above coverage cells instead of being hidden under them.
- A canceled/terminal navigation mission no longer forces the panel back into Navigate mode after the run is no longer active.

### 2. Which mode this affects

- Mapping: mode switching back to Mapping is no longer overridden by a terminal navigation snapshot.
- Navigation: active navigation still auto-selects Navigate; canceled/completed/failed navigation does not trap the operator in Navigate.
- Clean Area: preview route visualization is visible while drawing/reviewing an area and during runtime coverage display.
- Shared adapter/runtime architecture: unchanged command boundary; this pass only changes UI rendering and mode-selection rules.

### 3. Ownership check

- Is this still owned by React/webview state?
  - Clean Area draft and preview visualization are still local UI state before start.
  - Mode selection is UI presentation state.
- Is this now owned by the runtime/backend?
  - Active navigation and coverage execution remain runtime/backend-owned.
- What state is the UI only rendering?
  - Clean Area draft route preview before `start_coverage`.
  - Runtime `snapshot.activeMission` route/progress once a coverage mission is active.
  - Terminal navigation snapshots without treating them as active blockers.
- What command does the UI submit?
  - `Preview Path` submits no runtime command.
  - `Start Cleaning` submits `start_coverage`.
  - Navigation start/cancel behavior is unchanged.

### 4. Webview close/reopen behavior

- mapping
  - Mapping continues in the VM runtime and hydrates from adapter/runtime state.
- navigation
  - Active navigation continues in the VM runtime and hydrates as Navigate.
  - Terminal navigation remains visible when in Navigate, but it no longer prevents switching modes.
- clean area
  - Pre-start preview is local webview state.
  - Started Clean Area missions continue in the VM runtime and hydrate from `snapshot.activeMission`.
  - Runtime route/progress visualization remains UI-rendered from the adapter snapshot.

### 5. Real hardware compatibility check

- Does this expose TurtleBot4/Nav2 specifics to product UI?
  - No.
- Does this require Nav2 waypoint sequencing as a public concept?
  - No.
- Can the same adapter shape be implemented by Valetudo later?
  - Yes; adapter command/state shapes did not change in this pass.
- What capability flags decide whether controls are shown/enabled?
  - Existing flags remain: `start_coverage`, `coverage_mission`, `pause_mission`, `resume_mission`, `cancel_mission`, `retry_mission_step`, and `skip_mission_step`.
- What operations are explicitly unsupported?
  - Unchanged; unsupported operations still flow through adapter capabilities and command errors.

### 6. Files changed

- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
  - Added an active-navigation-only mode auto-switch check so terminal navigation snapshots do not trap the mode switcher.
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.css`
  - Raised the plan overlay above coverage cells.
  - Increased Clean Area path stroke widths and dash spacing so route preview is visible.
- `progress_report.md`
  - Updated this progress report.

### 7. Tests / validation run

Passed:

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Failed due to existing unrelated repository errors:

```sh
bun run typecheck
```

```text
packages/tensorfleet-auth/src/oauth-core.ts(178,13): error TS6133: 'callbackBaseUrl' is declared but its value is never read.
packages/tensorfleet-auth/src/oauth-core.ts(202,13): error TS6133: 'actualPort' is declared but its value is never read.
```

### 8. Remaining risks

- I did not perform a live webview visual check, so the exact route stroke weight may need one more polish pass after seeing it in the map.
- Terminal navigation still remains available in Navigate mode for operator context, but it should no longer auto-force the mode.
- Pre-start Clean Area preview remains non-durable by design; durability starts after `start_coverage`.

### 9. Next recommended step

Retest the two reported flows in the webview: `Preview Path` should show a visible Clean Area route, and after canceling a Navigate run the Mapping and Clean Area tabs should stay selectable.
