# Progress Report - Valetudo Milestone 3: Basic Operator Surface Forward Pass
Current report date: 2026-06-09.

## 1. What changed

This milestone improved the product/operator UX for the existing supported no-map Valetudo path. No new Valetudo capabilities, no diagnostics drawer, and no advanced robot features were added. All behavior still derives from normalized adapter fields and normalized commands.

### No-map canvas placeholder

- **Tonal chip states**: The four status chips at the bottom of the placeholder (availability, source, dock, battery) now carry visual tone — green for healthy/ready states, orange for stale/warning states, red for unreachable/danger states. Previously all chips were the same muted grey regardless of health.
- **Source chip label**: The source chip now shows human-readable context: "Source live", "Source stale", "Source offline", or "Source: {status}" instead of just the raw status value. This makes the source reachability state clear without requiring hover.
- **Model line removed when not available**: The placeholder no longer shows "Model not reported" when the backend provides no model string. The model line only appears when a model is actually populated in `identity.model`.
- **CSS tonal classes added**: `.vacuum-no-map-chip--ready`, `.vacuum-no-map-chip--warning`, `.vacuum-no-map-chip--danger` added to the CSS, used by the updated placeholder chips.

### No-map status strip

- **Activity chip label**: The task chip for activity previously said `Activity: Idle` (with a redundant label prefix). It now shows the activity state value directly — e.g., `Idle`, `Cleaning`, `Docked` — letting the chip's icon and position in the "Task" group provide context.
- **Dock chip label**: The dock chip previously said `Dock: Docked` (with a redundant label prefix). It now shows the dock state value directly — e.g., `Docked`, `Returning`, `Unknown`.
- The existing simulation status strip (Map Live, Localized, Ready, Target Selected) is unchanged for map/pose/navigation-capable backends.

### Basic Cleaning controls card

- **Removed "Runtime" badge**: The `BasicControlsCard` header previously included a "Runtime" badge beside the "Basic Cleaning" eyebrow. "Runtime" is not operator-meaningful vocabulary on a controls card. The badge was removed. The "Basic Cleaning" eyebrow remains and is sufficient to identify the card.
- All four controls (start_cleaning, pause, stop, return_to_dock) still gate on normalized capability descriptors. Disabled reasons remain visible under disabled buttons in human-readable form.

### Unavailable workflows summary

- "Rooms / zones" → "Rooms / Zones": Capitalization made consistent with other workflow labels in the card.
- The card structure, copy, and compactness are otherwise unchanged.

## 2. Which modes are affected or unchanged

- **Valetudo no-map backend**: Affected by all operator surface changes above.
- **TurtleBot4/Nav2 simulation**: Unchanged. The simulation layout, MapCanvas, mode switcher (Mapping/Navigate/Clean Area/Rooms), status strip (Map Live, Localized, Ready, Target Selected), and all simulation-specific cards are unchanged. The `isBasicRobotProfile` gate ensures no-map changes do not touch the simulation path.
- **Fixed mock Valetudo runtime**: Unaffected at the runtime or adapter level. Operator surface changes apply when the normalized snapshot reaches the panel.
- **HTTP/MQTT Valetudo mock sources**: Unaffected at the runtime or adapter level. Same as fixed mock.

## 3. Ownership boundaries

- UI owns presentation of chip tone, chip labels, and card header badges. No adapter contract field was added or changed.
- Adapter still owns all normalized descriptor fields: `capabilities`, `activity`, `health`, `source`, `dock`, `battery`, `fault`, `identity`.
- VM runtime owns source reachability, state cache, and command routing. Unchanged.
- No raw Valetudo capability names, raw reason codes, or backend-specific identifiers are exposed in the product UI.
- Product behavior continues to branch on normalized capability descriptors and normalized state, not on backend id.

## 4. Webview close/reopen behavior

No change. The panel hydrates from `adapter.snapshot` on mount. Normalized state from the adapter is the source of truth on re-open. No new local React state was introduced that would need to persist across closes.

## 5. Real hardware compatibility

No hardware paths were added or changed. This milestone is UI-only polish for the normalized no-map path. Real hardware compatibility (Layer 6) remains unchanged and depends only on the adapter contract, which was not modified.

## 6. Feature behavior changed

- No behavioral changes. All changes are presentation-only: chip tone, chip labels, badge removal, label capitalization.
- No new commands were added.
- No new capability fields were added.
- No advanced Valetudo capabilities were added (map rendering, go-to/navigation, Clean Area, rooms/zones/segments, fan speed, water usage, consumables, OpenClaw, MQTT hardening, real hardware).
- No diagnostics drawer was added.

## 7. Files changed

- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
  - Added `noMapChipClass()` helper function for tonal chip class mapping.
  - Rewrote `NoMapCanvasPlaceholder`: removed "Model not reported" fallback line, added tonal chip classes using `getSourceTone` and `getDockTone`, added context-aware source chip label ("Source live", "Source stale", "Source offline"), updated battery label to "Battery unknown" when percentage is null.
  - `taskChips` for no-map case: removed `Activity: ` and `Dock: ` label prefixes; chips now show the state value directly.
  - `BasicControlsCard`: removed the "Runtime" badge from the card head.
  - `UnavailableWorkflowsCard`: "Rooms / zones" → "Rooms / Zones".
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.css`
  - Added `.vacuum-no-map-chip--ready`, `.vacuum-no-map-chip--warning`, `.vacuum-no-map-chip--danger` tonal modifier classes for the no-map placeholder chips.

No runtime Go code changed. No adapter TypeScript code changed. No regression test changed.

## 8. Tests / validation run

```sh
bun run test:vacuum-adapter      # passed
bun run --cwd panels-standalone build   # passed, existing warnings only
git diff --check                  # passed
```

All three commands passed on 2026-06-09. The panel build emitted only existing Vite/browser externalization, eval, and bundle-size warnings.

No regression test changes were required. The existing tests continue to verify: no backend-name branching, `NoMapCanvasPlaceholder` gated by `mapSurfaceAvailable`, normalized `health/source/activity/dock/battery/fault` props passed to the basic profile sidebar, supported controls filtering, disabled state and visible reasons, human-readable reason copy, `isBasicRobotProfile` gate with `UnavailableWorkflowsCard`, and `mapSurfaceAvailable = mapSupported`.

Manual live webview validation was not run.
Real hardware validation was not run.
TurtleBot4/Nav2 live simulation validation was not run.
