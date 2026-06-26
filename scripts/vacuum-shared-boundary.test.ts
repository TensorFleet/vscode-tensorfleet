#!/usr/bin/env bun

import assert from "node:assert/strict";
import { existsSync, readdirSync, readFileSync, statSync } from "node:fs";
import { relative, resolve } from "node:path";

const repoRoot = resolve(import.meta.dir, "..");

const sourceRoots = [
  "src",
  "panels-standalone/src",
] as const;

const sharedShims = new Map([
  ["panels-standalone/src/vacuum-adapter/capabilities.ts", "tensorfleet-util/vacuum/capabilities"],
  ["panels-standalone/src/vacuum-adapter/commands.ts", "tensorfleet-util/vacuum/commands"],
  ["panels-standalone/src/vacuum-adapter/errors.ts", "tensorfleet-util/vacuum/errors"],
  ["panels-standalone/src/vacuum-adapter/state.ts", "tensorfleet-util/vacuum/state"],
  ["panels-standalone/src/vacuum-adapter/mapGrid.ts", "tensorfleet-util/vacuum/mapGrid"],
  [
    "panels-standalone/src/vacuum-adapter/backends/valetudo/capabilityMapper.ts",
    "tensorfleet-util/vacuum/backends/valetudo/capabilityMapper",
  ],
  [
    "panels-standalone/src/vacuum-adapter/backends/valetudo/commandMapper.ts",
    "tensorfleet-util/vacuum/backends/valetudo/commandMapper",
  ],
  [
    "panels-standalone/src/vacuum-adapter/backends/valetudo/runtimeCommandMapper.ts",
    "tensorfleet-util/vacuum/backends/valetudo/runtimeCommandMapper",
  ],
  [
    "panels-standalone/src/vacuum-adapter/backends/valetudo/runtimeContract.ts",
    "tensorfleet-util/vacuum/backends/valetudo/runtimeContract",
  ],
  [
    "panels-standalone/src/vacuum-adapter/backends/valetudo/stateMapper.ts",
    "tensorfleet-util/vacuum/backends/valetudo/stateMapper",
  ],
  ["panels-standalone/src/vacuum-adapter/backends/valetudo/types.ts", "tensorfleet-util/vacuum/backends/valetudo/types"],
  [
    "panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/capabilityMapper.ts",
    "tensorfleet-util/vacuum/backends/turtlebot4-nav2/capabilityMapper",
  ],
  [
    "panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/runtimeTypes.ts",
    "tensorfleet-util/vacuum/backends/turtlebot4-nav2/runtimeTypes",
  ],
]);

const extensionLocalFiles = [
  "panels-standalone/src/vacuum-adapter/useVacuumAdapter.ts",
  "panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/useTurtleBot4Nav2Adapter.ts",
  "panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/localAnnotationMigration.ts",
  "panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/stateMapper.ts",
  "panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/commandDispatcher.ts",
  "panels-standalone/src/vacuum-adapter/backends/valetudo/useValetudoAdapter.ts",
  "panels-standalone/src/vacuum-adapter/backends/valetudo/runtimeClient.ts",
] as const;

function main() {
  const sources = sourceRoots.flatMap((root) => listSourceFiles(resolve(repoRoot, root)));

  assertNoMatches(sources, /tensorfleet-util\/vacuum\/node-runtime|\/vacuum\/node-runtime/, "extension source must not import vacuum node-runtime");
  assertNoMatches(
    sources,
    /(?:from|import)\s*\(?["'][^"']*(?:tensorfleet-openclaw-plugin|tensorfleet-tools)|require\(["'][^"']*(?:tensorfleet-openclaw-plugin|tensorfleet-tools)/,
    "extension source must not import OpenClaw/tool packages",
  );

  for (const [path, target] of sharedShims) {
    const source = readRepoFile(path).trim();
    assert.equal(source, `export * from "${target}";`, `${path} must remain a shared util re-export shim`);
  }

  for (const path of extensionLocalFiles) {
    assert.ok(existsSync(resolve(repoRoot, path)), `${path} must remain extension-local`);
    assert.doesNotMatch(readRepoFile(path), /tensorfleet-util\/vacuum\/node-runtime|\/vacuum\/node-runtime/, `${path} must not use node-runtime`);
  }

  console.log("vacuum shared boundary checks passed");
}

function listSourceFiles(dir: string): string[] {
  const entries = readdirSync(dir);
  const files: string[] = [];
  for (const entry of entries) {
    const path = resolve(dir, entry);
    const stat = statSync(path);
    if (stat.isDirectory()) {
      files.push(...listSourceFiles(path));
      continue;
    }
    if (/\.(ts|tsx|js|jsx|mjs|cjs)$/.test(entry)) {
      files.push(path);
    }
  }
  return files;
}

function assertNoMatches(files: string[], pattern: RegExp, message: string): void {
  const offenders = files
    .filter((file) => pattern.test(readFileSync(file, "utf8")))
    .map((file) => relative(repoRoot, file));
  assert.deepEqual(offenders, [], `${message}: ${offenders.join(", ")}`);
}

function readRepoFile(path: string): string {
  return readFileSync(resolve(repoRoot, path), "utf8");
}

main();
