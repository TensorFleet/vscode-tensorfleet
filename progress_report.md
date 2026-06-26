# Progress Report - Vacuum shared-core boundary closeout
Current report date: 2026-06-26.

## 1. What changed

- Finalized the architecture boundary in docs: `tensorfleet-util/vacuum` is the shared vacuum control foundation for tools, agents, and UI clients; `vscode-tensorfleet` is one UI client; `tensorfleet-vacuum` is the OpenClaw plugin tool facade over `tensorfleet-tools` and `tensorfleet-util`.
- Added `scripts/vacuum-shared-boundary.test.ts` and `test:vacuum-shared-boundary` to guard that extension source does not import `tensorfleet-util/vacuum/node-runtime` or OpenClaw/tool packages, that pure shim files stay shared re-exports, and that extension-local hooks/runtime clients remain local.
- Updated shared vacuum architecture references under `/home/shane/docs/vacuum` to describe the final shared-core boundary.
- Verified Step 6C remains complete: extension pure modules are shared re-export shims to `tensorfleet-util/vacuum`.

## 2. Product behavior

- Product behavior is unchanged; this pass added boundary guardrails and documentation only.
- The extension adapter was not removed. It is thinner through shared shims, while `useVacuumAdapter.ts`, TurtleBot4/Nav2 hooks, Valetudo hooks/runtime client, local annotation migration, TurtleBot4 state mapper, and TurtleBot4 command dispatcher remain extension-local.
- Extension panel and extension host source do not import `tensorfleet-util/vacuum/node-runtime`.
- OpenClaw and other agents can use `tensorfleet-util/vacuum` through `tensorfleet-tools` without relying on `vscode-tensorfleet`.

## 3. Still deferred

- No new vacuum actions, room/zone tools, real-vacuum writes, MCP changes, or OpenClaw config changes were added.
- No React hooks, browser runtime clients, polling, localStorage behavior, VS Code SecretStorage/auth behavior, or rendering/presentation code was moved.
- No local adapter entrypoints were deleted.
- No live robot/runtime validation was performed or claimed.
- Future product-level vacuum behavior should start in `tensorfleet-util/vacuum`; OpenClaw-specific response shape or tool policy belongs in `tensorfleet-tools`; UI lifecycle/presentation stays in `vscode-tensorfleet`.

## 4. Validation

- `bun run --cwd /home/shane/vscode-tensorfleet/panels-standalone prepare:tensorfleet-util` - passed.
- `bun run --cwd /home/shane/vscode-tensorfleet test:vacuum-shared-parity` - passed with all shared parity checks and no known drift.
- `bun run --cwd /home/shane/vscode-tensorfleet test:vacuum-shared-boundary` - passed.
- `bun run --cwd /home/shane/vscode-tensorfleet/panels-standalone build` - passed; Vite still emitted existing browser-externalization, eval, and chunk-size warnings.
- `bun run --cwd /home/shane/vscode-tensorfleet compile` - passed; Vite still emitted the same panel warnings and the existing CJS Node API deprecation warning during extension build.
- `bun run --cwd /home/shane/vscode-tensorfleet build:extension` - passed; Vite still emitted the existing CJS Node API deprecation warning.
- `git -C /home/shane/vscode-tensorfleet diff --check` - passed.
- `bun run --filter tensorfleet-tools test:vacuum-discovery` - passed.
- `bun run --filter tensorfleet-tools test:vacuum-read-preflight` - passed.
- `bun run --filter tensorfleet-tools test:vacuum-write-actions` - passed.
- `bun run --filter tensorfleet-tools test:vacuum-boundary` - passed.
- `bun run --filter tensorfleet-openclaw-plugin test:discovery-smoke` - passed.
- `bun run --filter tensorfleet-openclaw-plugin build` - passed; tsup still emitted existing direct-`eval` bundler warnings from `tensorfleet-tools/dist/index.mjs`.
- `bunx tsc -p packages/tensorfleet-openclaw-plugin/tsconfig.json --noEmit` - passed.
- `git -C /home/shane/tensorfleet-claw-interface diff --check` - passed.
