# Progress Report - Vacuum room zone target shared parity
Current report date: 2026-06-26.

## 1. What changed

- Mirrored the shared `tensorfleet-util/vacuum` target contract into the extension vendored util copy: normalized target metadata, readiness statuses, geometry validation, runtime target mapping, and annotation-to-target mapping.
- Added the pure extension shim `panels-standalone/src/vacuum-adapter/targets.ts` and exported it from the local adapter barrel.
- Extended `scripts/vacuum-shared-parity.ts` to compare shared/local target mapping and readiness behavior.
- Extended `scripts/vacuum-shared-boundary.test.ts` so the target shim remains a pure shared re-export.

## 2. Product behavior

- Extension UI lifecycle, rendering, hooks, local annotation editing, localStorage, runtime clients, auth injection, and polling stayed local.
- Browser panel and extension host source still do not import `tensorfleet-util/vacuum/node-runtime`.
- Target inventory/readiness semantics now come from the same shared pure module used by OpenClaw/tools.
- No new UI controls, room/zone cleaning starts, or map editing behavior were added in the extension.

## 3. Still deferred

- Extension UI changes for room/zone target inventory beyond shared pure-module parity.
- `start-room-cleaning` and `start-zone-cleaning`.
- OpenClaw map annotation mutation/editing.
- Real-vacuum room/zone write/control behavior.
- MCP vacuum tools as a primary integration path.
- Live robot validation was not performed or claimed.

## 4. Validation

- `bun run --cwd /home/shane/vscode-tensorfleet/panels-standalone prepare:tensorfleet-util` - passed.
- `bun run --cwd /home/shane/vscode-tensorfleet test:vacuum-shared-parity` - passed with target parity and no known drift.
- `bun run --cwd /home/shane/vscode-tensorfleet test:vacuum-shared-boundary` - passed.
- `bun run --cwd /home/shane/vscode-tensorfleet/panels-standalone build` - passed; Vite still emitted existing browser-externalization, eval, and chunk-size warnings.
- `bun run --cwd /home/shane/vscode-tensorfleet compile` - passed; Vite still emitted existing panel warnings and the existing CJS Node API deprecation warning during extension build.
- `bun run --cwd /home/shane/vscode-tensorfleet build:extension` - passed; Vite still emitted the existing CJS Node API deprecation warning.
- `git -C /home/shane/vscode-tensorfleet diff --check` - passed.
