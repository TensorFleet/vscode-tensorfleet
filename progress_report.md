# Progress Report - Valetudo Mock UI State and Layout Pass
Current report date: 2026-06-10.

## 1. What changed

- Added a normalized primary robot state helper that derives one UI state from adapter snapshots: offline, unavailable, idle, docked, charging, cleaning, paused, returning to dock, or error.
- Removed the no-map robot overview dashboard from the map area so the map stage stays visually quiet when no product map exists.
- Collapsed Robot Status into one compact card with the primary robot state and battery bar.
- Removed inline Basic Cleaning disabled reason text such as invalid-state explanations from the visible control list.
- Removed the unavailable-workflows summary from the basic no-map Valetudo rail.
- Kept basic controls, fan/water settings, maintenance, and unavailable workflows behind normalized adapter state/capabilities.
- Added fixed mock runtime scenarios for docked idle, cleaning, paused, returning to dock, charging, stale/unreachable source, and fault/maintenance warning.
- Updated adapter/runtime regression coverage for primary state derivation, no-map map gating, capability-driven UI behavior, normalized display surfaces, and fixed mock scenarios.

## 2. Product behavior

- Operators now see one clear main robot state in a compact Robot card instead of a stack of status rows.
- The map area no longer displays robot overview content when no map exists.
- Basic Cleaning buttons still disable from normalized availability, but no longer show inline invalid-state reason sentences.
- Fan speed, water usage, maintenance/consumables, and dock/battery remain normalized product displays in the right rail.
- No new advanced state-changing controls were added.

## 3. Still deferred

- Real hardware behavior and production MQTT.
- Valetudo map rendering, segment cleaning, zone cleaning, Clean Area, go-to/navigation, room semantics, and robot pose visualization.
- Consumable reset, dock clean/dry/empty actions, robot options, map reset/export, scheduling, updater/log/system configuration UI, and diagnostics drawer.
- Normalized attachments, dock components, and current/total statistics fields beyond the display surfaces already backed by adapter state.

## 4. Validation

```sh
bun run test:vacuum-adapter
go test ./... # from /home/shane/firecracker-vm/tensorfleet-mgr
bun run --cwd panels-standalone build
git diff --check
git -C /home/shane/firecracker-vm diff --check
```

Result:

* `bun run test:vacuum-adapter` passed.
* `go test ./...` from `/home/shane/firecracker-vm/tensorfleet-mgr` passed.
* `bun run --cwd panels-standalone build` passed with existing Vite browser-externalization, eval, and chunk-size warnings.
* `git diff --check` passed.
* `git -C /home/shane/firecracker-vm diff --check` passed.
