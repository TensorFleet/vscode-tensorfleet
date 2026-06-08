# Progress Report - Valetudo Final Boundary Lock
Current report date: 2026-06-09.

## 1. What changed

This was the final command/regression/diagnostics-boundary wrap-up pass for the Valetudo mock/backend hardening thread.

- Audited the Milestone 2A runtime and adapter tests before adding coverage.
- Added one focused adapter regression for the already-supported `malformed_command_response` runtime alias mapping to shared `malformed_backend_response`.
- Updated `review.md` to mark the Milestone 1 inventory, Milestone 2A command hardening, and final boundary checks closed for now.
- No product behavior changed.
- No new product features were added.
- No diagnostics drawer or broad fixture matrix was added.

## 2. Which mode this affects

- Valetudo backend: Affected only by one regression check and docs. Runtime command/result behavior is unchanged from Milestone 2A.
- Fixed mock Valetudo runtime: Unchanged.
- HTTP Valetudo mock source: Unchanged.
- MQTT Valetudo mock source: Unchanged.
- Mapping: Unchanged. Valetudo map rendering remains unsupported.
- Navigation/go-to: Unchanged. Detected Valetudo go-to remains diagnostics-only/detected-not-ready.
- Clean Area: Unchanged. Coverage remains unsupported.
- Rooms / Zones: Unchanged. Segment/zone detection remains deferred and not usable controls.
- Fan/water/consumables: Unchanged. These remain unsupported or detected-not-ready only.
- TurtleBot4/Nav2 simulation: Unchanged.

## 3. Ownership check

- VM runtime owns source reachability, source freshness, fixed mock state transitions, HTTP/MQTT command routing, malformed request handling, and command audit diagnostics.
- Backend adapter owns translation from runtime command result shapes into shared `VacuumCommandResult` / `VacuumCommandError` values.
- Shared `vacuum_adapter` continues to own the public command error union. Raw runtime aliases do not become public shared error codes.
- UI owns presentation of command errors and disabled reasons. It does not use diagnostics to decide whether controls render or enable.
- Raw Valetudo capability names remain diagnostics-only. Product behavior still gates on normalized descriptors and state.

Diagnostics remain intentionally passive and private to diagnostics surfaces. This pass did not add diagnostics UI.

## 4. Coverage confirmed

Existing VM runtime tests already cover:

- stale source blocks command dispatch
- invalid JSON and missing command return invalid-request-shaped failures
- source unreachable returns unavailable command results
- fixed mock invalid-state command results for unsafe basic commands
- missing `BasicControlCapability` returns unsupported/capability-unavailable behavior
- HTTP mock source command routing and source command failure
- MQTT disconnected and stale command unavailability
- raw Valetudo capability diagnostics and capability tiers
- last-command audit diagnostics

Existing adapter/UI regression already covers:

- Valetudo capability mapping and command mapping
- unsupported/capability-unavailable mapping to `unsupported`
- invalid-state mapping to `invalid_state`
- invalid request alias mapping to `invalid_request`
- source command failure mapping to `backend_error`
- state-aware command availability and stale-source gating
- runtime unavailable/malformed snapshot fallback behavior
- readable disabled reasons instead of raw machine codes
- raw Valetudo capability names staying out of public contract and product UI behavior

Small gap closed in this pass:

- `malformed_command_response` now has explicit adapter regression coverage mapping to `malformed_backend_response`.

## 5. Feature behavior changed

None in this pass.

Milestone 2A remains the behavior-changing hardening milestone: fixed mock invalid-state handling, stale MQTT command blocking, adapter error alias normalization, and readable disabled reason presentation.

Deferred capabilities remain deferred: map rendering, go-to/navigation, Clean Area, rooms/zones/segments, fan speed, water usage, consumables, OpenClaw, MQTT production hardening, real hardware support, and diagnostics drawer work.

## 6. Files changed

- `/home/shane/vscode-tensorfleet/scripts/vacuum-adapter-regression.ts`
  Adds the missing malformed backend command-response alias regression.
- `/home/shane/vscode-tensorfleet/review.md`
  Marks the inventory and hardening thread closed, records final coverage, and points the next milestone toward product/UI work.
- `/home/shane/vscode-tensorfleet/progress_report.md`
  Replaces the Milestone 2A implementation report with this final boundary-lock wrap-up report.

No runtime Go code changed in this pass.
No product UI code changed in this pass.

## 7. Next product direction

Recommended next milestone: `Valetudo Milestone 3 - Basic Operator Surface Forward Pass`.

Suggested focus:

1. Improve product/operator UX for the existing supported no-map Valetudo path.
2. Keep the simulation layout intact.
3. Use only normalized adapter fields.
4. Do not add a diagnostics drawer unless explicitly requested later.
5. Do not add advanced controls until the runtime normalizes the required state, presets, and targets.

## 8. Tests / validation run

Required because one adapter regression test changed:

```sh
bun run test:vacuum-adapter
go test ./...                         # in /home/shane/firecracker-vm/tensorfleet-mgr
bun run --cwd panels-standalone build
git diff --check                      # in /home/shane/vscode-tensorfleet
git diff --check                      # in /home/shane/firecracker-vm/tensorfleet-mgr
```

All commands passed on 2026-06-09. The panel build emitted existing Vite/browser externalization, eval, and bundle-size warnings only.

Manual live webview validation was not run; it was not required for this boundary wrap-up.
Real hardware validation was not run.
TurtleBot4/Nav2 live simulation validation was not run.
