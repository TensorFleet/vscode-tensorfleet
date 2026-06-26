# Progress Report - Vacuum shared-core extension closeout
Current report date: 2026-06-26.

## 1. What changed

- Closed the VS Code side of the vacuum shared-core refactor: pure vacuum adapter modules are compatibility shims to `tensorfleet-util/vacuum`.
- Kept extension-owned behavior local: React hooks, browser runtime clients, auth injection, polling, local annotation/UI state, rendering, and VS Code lifecycle.
- Confirmed OpenClaw/tools do not depend on VS Code extension code for product-level vacuum control.
- Preserved shared room/zone target parity in the vendored util copy used by the extension.

## 2. Product behavior

- Extension UI behavior is unchanged by this closeout pass.
- OpenClaw/agents can list normalized map/room/zone targets, preflight room/zone cleaning, and start room/zone cleaning for simulation only through `tensorfleet-tools` plus `tensorfleet-util/vacuum`.
- Simulation room/zone starts use shared target readiness and normalized commands.
- No live robot support, real-vacuum room/zone writes, or map editing behavior is claimed from the extension refactor.

## 3. Still deferred

- Real-vacuum room/zone writes.
- Map annotation mutation/editing.
- Live robot validation.
- Arbitrary waypoint/raw backend tools.
- MCP as the primary vacuum control path.
- New extension UI controls/workflows for room/zone starts.

## 4. Validation

- `bun run --cwd /home/shane/vscode-tensorfleet/panels-standalone prepare:tensorfleet-util` - passed.
- `bun run --cwd /home/shane/vscode-tensorfleet test:vacuum-shared-parity` - passed.
- `bun run --cwd /home/shane/vscode-tensorfleet test:vacuum-shared-boundary` - passed.
- `bun run --cwd /home/shane/vscode-tensorfleet/panels-standalone build` - passed; non-blocking warnings: existing Vite browser-externalization, protobufjs `eval`, and chunk-size warnings.
- `bun run --cwd /home/shane/vscode-tensorfleet compile` - passed; non-blocking warnings: same panel build warnings plus existing Vite CJS Node API deprecation warning.
- `bun run --cwd /home/shane/vscode-tensorfleet build:extension` - passed; non-blocking warning: existing Vite CJS Node API deprecation warning.
- `git -C /home/shane/vscode-tensorfleet diff --check` - passed.

The vacuum shared-core refactor is complete. Future work should be treated as product/tool feature work, with product-level semantics starting in tensorfleet-util/vacuum, OpenClaw policy in tensorfleet-tools, and UI lifecycle in vscode-tensorfleet.
