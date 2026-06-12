# Progress Report - Attachments and Dock Components Normalization
Current report date: 2026-06-12.

We are adding read-only normalized Attachments and Dock Components surfaces for the Valetudo path. Attachments represent robot-side installed/present equipment and material state, while Dock Components represent dock-side material readiness. The UI must only consume normalized vacuum_adapter state and capabilities, never raw Valetudo capability names, HTTP routes, MQTT topics, source URLs, SSE/cache internals, or backend-specific payloads.

## 1. What changed
- Extended the VM-managed Valetudo runtime snapshot with optional `attachments.items[]` and `dock.components[]`.
- Added fixed mock default data for dustbin, water tank, mop, detergent, freshwater, wastewater, dock detergent, and dustbag.
- Added an attention fixed mock scenario for full dustbin, low water tank, missing mop, low detergent, empty freshwater, full wastewater, and missing dustbag.
- Added conservative HTTP-source normalization for explicit attachment/component state attributes only; raw capability names and model names do not imply support.
- Added malformed-row handling: rows without meaningful identity are dropped, unknown kinds/statuses normalize to `unknown`, and invalid percentages are omitted.
- Extended the normalized `vacuum_adapter` contract with `snapshot.attachments` and `snapshot.dock.components`.
- Added read-only `attachments` and `dock_components` capabilities with empty command lists and kind attributes when trusted rows exist.
- Added compact read-only Attachments and Dock Components cards in the Valetudo/no-map Vacuum Control sidebar.
- Updated `firecracker-vm/tensorfleet-mgr` runtime structs, fixed mock construction, HTTP source normalization, and handler tests.
- No `vm-manager` code change was required after inspection; `/vms/self/tensorfleet/...` uses a generic reverse proxy to the guest runtime and forwards expanded JSON without typed schema changes.
- Synced the updated `tensorfleet-mgr` binary to the live VM at `172.16.0.11`, backed up the previous binary as `/usr/local/bin/tensorfleet-mgr.backup-20260612T033851Z`, and restarted `tensorfleet-mgr`.
- Verified the live service is active on port `9090`; the current `valetudo_mock_http` source does not provide trusted attachment/component rows, so the live service correctly omits populated readiness rows.
- Verified the deployed binary on the VM with a temporary fixed-mock smoke on port `19090`; it returned 4 attachment rows and 4 dock component rows, then the temporary process was stopped.
- Added/updated regression tests for fixed mock data, capability support, malformed data, stale/unreachable omission, and existing adapter behavior.

## 2. Product behavior
- Operators on the Valetudo/no-map path can now see compact Attachments and Dock Components readiness cards when fresh normalized data exists.
- Attachments show robot-side installed/present/material state such as dustbin, water tank, mop, and detergent.
- Dock Components show dock-side readiness such as freshwater, wastewater, detergent, and dustbag.
- Cards appear only when normalized capabilities are supported and normalized rows exist.
- Missing, stale, unreachable, or offline data omits the new surfaces and leaves capabilities unsupported/unavailable rather than showing stale readiness as current truth.
- Malformed rows do not crash snapshot mapping and do not expose misleading rows.
- The feature is read-only and backend-neutral; no dock action, reset, wash, dry, refill, or empty commands were added.

## 3. Still deferred
- Dock action commands.
- Auto-empty command implementation.
- Mop wash/dry command implementation.
- Water refill command implementation.
- Consumable reset commands.
- Total statistics.
- Statistics history/charts/export.
- Map rendering and segment/room/zone/go-to behavior.
- Valetudo Clean Area behavior.
- Hardware validation.

## 4. Validation

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
cd /home/shane/firecracker-vm/tensorfleet-mgr && go test ./...
cd /home/shane/firecracker-vm/tensorfleet-mgr && git diff --check
curl -fsS -H 'Authorization: Bearer default-tensorfleet-token' http://172.16.0.11:9090/api/v1/valetudo/snapshot
PORT=19090 VALETUDO_RUNTIME_SOURCE_MODE=fixed_mock /usr/local/bin/tensorfleet-mgr
curl -fsS -H 'Authorization: Bearer default-tensorfleet-token' http://172.16.0.11:19090/api/v1/valetudo/snapshot
```
