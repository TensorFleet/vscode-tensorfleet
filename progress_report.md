# Progress Report - Valetudo Readiness Source Mode Corrections
Current report date: 2026-06-10.

## 1. What changed

- Preserved `valetudo_mock` in normalized adapter source state instead of collapsing mock runtime snapshots into `valetudo_http`.
- Kept actual `valetudo_http` runtime snapshots mapped to normalized `valetudo_http`.
- Removed production-sounding `valetudo_mqtt` support from the runtime source-mode parser and source-kind contracts.
- Made explicit `VALETUDO_RUNTIME_SOURCE_MODE=valetudo_http` override `VALETUDO_MQTT_ENABLED=true`, so missing `VALETUDO_SOURCE_URL` stays on the HTTP missing-config path.
- Kept intentional mock MQTT selection through `VALETUDO_RUNTIME_SOURCE_MODE=valetudo_mock_mqtt` and legacy `VALETUDO_MQTT_ENABLED=true` only when no explicit source mode is set.
- Updated adapter and runtime tests for mock/HTTP source-kind preservation, rejected `valetudo_mqtt`, HTTP-over-MQTT env precedence, and capability-driven product behavior.

## 2. Product behavior

- Operators still see the existing no-map Valetudo surface driven by normalized capabilities and state.
- Mock HTTP/MQTT runtime sources report mock-safe `valetudo_mock` source state, not future HTTP hardware source state.
- Explicit HTTP readiness mode reports `valetudo_http` and fails safely when HTTP config is missing instead of falling through to MQTT cache data.
- Raw transport details and Valetudo capability names remain diagnostics-only; product controls remain capability-driven.

## 3. Still deferred

- Production MQTT support.
- Real hardware validation against a Valetudo-compatible robot.
- Segment target normalization, map rendering, pose/go-to/navigation, Clean Area, zone cleaning, room semantics/editor, consumable reset commands, OpenClaw, scheduling, and a diagnostics drawer.

## 4. Validation

```sh
bun run test:vacuum-adapter
go test ./...
go test ./... # from /home/shane/firecracker-vm/tensorfleet-mgr
bun run --cwd panels-standalone build
git diff --check
git -C /home/shane/firecracker-vm diff --check
```

Result:

* `bun run test:vacuum-adapter` passed.
* `go test ./...` from `/home/shane/vscode-tensorfleet` failed with the known caveat: no Go module exists at the repo root.
* `go test ./...` from `/home/shane/firecracker-vm/tensorfleet-mgr` passed.
* `bun run --cwd panels-standalone build` passed with existing Vite browser-externalization, eval, and chunk-size warnings.
* `git diff --check` passed.
* `git -C /home/shane/firecracker-vm diff --check` passed.
