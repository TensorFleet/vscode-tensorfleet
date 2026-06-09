# Progress Report - Valetudo Milestone 3B: Basic Operator Surface Clarity Pass
Current report date: 2026-06-09.

## 1. What changed

This milestone polished the existing supported no-map Valetudo operator surface. It was UI-only presentation work: no adapter contract fields, runtime behavior, backend behavior, commands, advanced Valetudo capabilities, or diagnostics drawer were added.

### No-map status strip

- **System group narrowed to system health signals**: The no-map/basic status strip still shows connection, runtime, and source chips only. It does not add dock or battery to the system group.
- **Task group deduplicated**: The no-map/basic task group now renders one primary activity chip plus one dock/battery summary chip. It no longer renders separate activity, battery, and dock chips that can repeat the same dock state.
- **Returning copy clarified**: `returning` activity is shown as `Returning to dock`. When activity and dock state communicate the same dock/returning meaning, the summary chip omits the repeated dock label and keeps the battery value.
- **Simulation strip preserved**: The TurtleBot4/Nav2 map-supported strip still uses the existing `Connected`, `Map Live`, `Localized`, `Ready`, and `Target Selected` layout.

### No-map canvas placeholder

- **Sparse hierarchy**: The placeholder now presents `MAP UNAVAILABLE`, robot identity, one concise status line such as `Docked · 100%`, and the two operator-facing guidance lines.
- **Duplicate placeholder chips removed**: The previous availability/source/dock/battery chip row was removed from the placeholder because those signals are already covered by the top strip and sidebar.
- **Returning summary clarified**: If normalized activity reports returning, the placeholder status line prefers `Returning to dock · {battery}` over a nearby dock-state duplicate.

### Sidebar

- **Robot Status card unchanged in scope**: The sidebar keeps connection, runtime, source, activity, dock, battery, and fault rows from normalized fields.
- **Basic Cleaning card unchanged in behavior**: Operator action labels remain `Start cleaning`, `Pause`, `Stop`, and `Return to dock`. Disabled reasons remain visible and human-readable. The removed `Runtime` badge was not reintroduced.
- **Unavailable workflows remain secondary**: The compact secondary card still says advanced workflows are hidden until the backend reports support. Unsupported advanced features do not dominate the UI.

## 2. Which modes are affected or unchanged

- **Valetudo no-map/basic backend**: Affected by the top-strip deduplication and placeholder hierarchy cleanup.
- **Fixed mock Valetudo runtime**: Runtime and adapter behavior unchanged; UI polish applies after the normalized snapshot reaches the panel.
- **HTTP/MQTT Valetudo mock sources**: Runtime and adapter behavior unchanged; UI polish applies through the same normalized no-map snapshot.
- **TurtleBot4/Nav2 simulation**: Unchanged. The simulation layout, MapCanvas, mode switcher, status strip, map/navigation/mapping/Clean Area/Rooms surfaces, and simulation-specific controls were preserved.

## 3. Ownership boundaries

- UI owns only presentation formatting for no-map chips, placeholder hierarchy, and operator-facing copy.
- Adapter still owns normalized `identity`, `availability`, `source`, `activity`, `dock`, `battery`, `health`, `fault`, and `capabilities`.
- VM runtime still owns source reachability, state cache, and command routing.
- No product behavior branches on backend id, raw Valetudo capability names, diagnostics, ROS/Nav2 topics/actions, MQTT topics, HTTP endpoint names, runtime ID/version, source URLs, or saved map paths.
- No raw diagnostics, backend IDs, raw Valetudo capability names, runtime IDs, MQTT details, or HTTP endpoint names were added to operator-facing UI.

## 4. Webview close/reopen behavior

No change. The panel hydrates from `adapter.snapshot` when opened. The new labels are derived from the current normalized snapshot and introduce no local state that must persist across close/reopen.

## 5. Real hardware compatibility

No real hardware behavior changed. This pass is compatible with future real Valetudo-compatible hardware because it only formats existing normalized no-map fields and leaves runtime/source/command handling untouched.

## 6. Feature behavior changed

- No backend behavior changed.
- No adapter/runtime behavior changed.
- No command names or runtime commands were added.
- No advanced Valetudo capabilities were added.
- No diagnostics drawer was added.
- No Valetudo map rendering, go-to/navigation, mapping, Clean Area, rooms/zones/segments, fan speed, water usage, consumables, OpenClaw, MQTT production hardening, or real hardware support was added.

## 7. Files changed

- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
  - Added shared formatting helpers for battery level, no-map activity copy, and dock/battery summaries.
  - Collapsed no-map task chips to activity plus one dock/battery summary.
  - Simplified the no-map placeholder and removed its duplicate chip row.
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.css`
  - Removed unused no-map placeholder chip styles.
- `scripts/vacuum-adapter-regression.ts`
  - Added narrow source-boundary checks for sparse no-map placeholder hierarchy, collapsed dock/battery task summary, and preserved simulation status labels.
- `progress_report.md`
  - Updated this report for Valetudo Milestone 3B.

## 8. Tests / validation run

Passed on 2026-06-09:

```sh
bun run test:vacuum-adapter              # passed
bun run --cwd panels-standalone build    # passed, existing warnings only
git diff --check                         # passed
```

The panel build emitted only existing Vite/browser externalization, eval, and bundle-size warnings.

Manual live webview validation was not run.
Real hardware validation has not been run.
TurtleBot4/Nav2 live simulation validation has not been run.
