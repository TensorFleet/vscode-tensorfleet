# Progress Report - Step 6C vacuum shared imports
Current report date: 2026-06-26.

## 1. What changed

- Replaced these extension-local pure vacuum modules with compatibility re-export shims to `tensorfleet-util/vacuum`: `capabilities.ts`, `commands.ts`, `errors.ts`, `state.ts`, `mapGrid.ts`, Valetudo `capabilityMapper.ts`, `commandMapper.ts`, `runtimeCommandMapper.ts`, `runtimeContract.ts`, `stateMapper.ts`, `types.ts`, and TurtleBot4/Nav2 `capabilityMapper.ts`.
- Added `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/runtimeTypes.ts` as a local compatibility shim to shared TurtleBot4/Nav2 runtime types.
- Updated `scripts/vacuum-shared-parity.ts` so it verifies extension-facing local entrypoints are shared re-export shims and still exercises generated shared util parity for pure contracts/mappers.
- Fixed `packages/tensorfleet-openclaw-plugin/scripts/discovery-smoke.test.mjs` in the OpenClaw repo to normalize string, direct object, and OpenClaw text-content wrapper results before asserting the structured vacuum result.
- The smoke test now validates backend selection, normalized adapter, gated read/write actions, `canMoveVacuumNow`, missing-runtime/auth status, invalid navigation input, and absence of secret config values.

## 2. Product behavior

- Product behavior is unchanged; this pass only changed pure module import ownership and a stale test expectation.
- Vacuum UI/hooks and runtime clients continue to consume local adapter entrypoints.
- These extension-specific files remain local: `useVacuumAdapter.ts`, TurtleBot4/Nav2 `useTurtleBot4Nav2Adapter.ts`, `localAnnotationMigration.ts`, `stateMapper.ts`, `commandDispatcher.ts`, Valetudo `useValetudoAdapter.ts`, and Valetudo `runtimeClient.ts`.
- `node-runtime` remains excluded from panel and extension source imports; `rg` found no `tensorfleet-util/vacuum/node-runtime` or `/vacuum/node-runtime` imports under `/home/shane/vscode-tensorfleet/panels-standalone/src` or `/home/shane/vscode-tensorfleet/src`.

## 3. Still deferred

- No React hooks, browser runtime clients, local storage behavior, VS Code SecretStorage/auth behavior, Valetudo runtime client behavior, or TurtleBot4 runtime subscriptions/polling were refactored.
- No local adapter files were deleted aggressively; local entrypoints remain as shims.
- No OpenClaw vacuum actions, room/zone tools, MCP changes, or OpenClaw config changes were added.
- Parity coverage remains fixture-based for pure shared-core contracts/mappers; it is not live robot/runtime integration coverage.
- No known pure shared-core drift remains after the shim replacement.

## 4. Validation

- `bun run --cwd /home/shane/vscode-tensorfleet/panels-standalone prepare:tensorfleet-util` - passed.
- `bun run --cwd /home/shane/vscode-tensorfleet test:vacuum-shared-parity` - passed with the requested PASS lines through shared re-export shims and `DONE shared parity with no known drift.`
- `bun run --cwd /home/shane/vscode-tensorfleet/panels-standalone build` - passed; Vite still emitted existing browser-externalization/eval/chunk-size warnings.
- `bun run --cwd /home/shane/vscode-tensorfleet compile` - passed; this repo script runs panel and extension builds.
- `bun run --cwd /home/shane/vscode-tensorfleet build:extension` - passed; Vite still emitted the existing CJS Node API deprecation warning.
- `git -C /home/shane/vscode-tensorfleet diff --check` - passed.
- `bun run --filter tensorfleet-tools test:vacuum-discovery` - passed.
- `bun run --filter tensorfleet-tools test:vacuum-read-preflight` - passed.
- `bun run --filter tensorfleet-tools test:vacuum-write-actions` - passed.
- `bun run --filter tensorfleet-openclaw-plugin test:discovery-smoke` - passed.
- `bun run --filter tensorfleet-openclaw-plugin build` - passed with existing direct-`eval` bundler warnings from `tensorfleet-tools/dist/index.mjs`.
- `bunx tsc -p packages/tensorfleet-openclaw-plugin/tsconfig.json --noEmit` - passed.
- `git -C /home/shane/tensorfleet-claw-interface diff --check` - passed.
