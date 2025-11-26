#!/usr/bin/env bun
/**
 * Get the full version with auto-incremented patch number
 * 
 * Uses MAJOR.MINOR from package.json and auto-generates PATCH from:
 * - GitHub Actions: Uses run_number
 * - Local: Uses git commit count since last MAJOR.MINOR tag
 */

import { readFileSync } from 'fs';
import { join } from 'path';
import { execSync } from 'child_process';

const packageJsonPath = join(process.cwd(), 'package.json');

function getBaseVersion(): string {
  const packageJson = JSON.parse(readFileSync(packageJsonPath, 'utf-8'));
  return packageJson.version;
}

function parseVersion(version: string): [number, number, number?] {
  const parts = version.split('.').map(Number);
  if (parts.length < 2 || parts.length > 3) {
    throw new Error(`Invalid version format: ${version}. Expected MAJOR.MINOR or MAJOR.MINOR.PATCH`);
  }
  return [parts[0], parts[1], parts[2]];
}

function formatVersion([major, minor, patch]: [number, number, number?]): string {
  if (patch !== undefined) {
    return `${major}.${minor}.${patch}`;
  }
  return `${major}.${minor}`;
}

function getGitCommitCount(baseVersion: string): number {
  try {
    const [major, minor] = parseVersion(baseVersion);
    const tagPattern = `v${major}.${minor}.*`;
    
    // Get the latest tag matching MAJOR.MINOR pattern
    let lastTag: string | null = null;
    try {
      const tags = execSync(`git tag --list '${tagPattern}' --sort=-version:refname`, { encoding: 'utf-8' }).trim();
      if (tags) {
        lastTag = tags.split('\n')[0];
      }
    } catch {
      // No tags found, that's okay
    }
    
    // Count commits since last tag (or since beginning if no tag)
    let commitCount: number;
    if (lastTag) {
      commitCount = parseInt(
        execSync(`git rev-list --count ${lastTag}..HEAD`, { encoding: 'utf-8' }).trim(),
        10
      ) || 0;
    } else {
      // Count all commits
      commitCount = parseInt(
        execSync('git rev-list --count HEAD', { encoding: 'utf-8' }).trim(),
        10
      ) || 0;
    }
    
    return commitCount;
  } catch (error) {
    console.warn('Warning: Could not determine git commit count, using 0');
    return 0;
  }
}

// Main
try {
  const baseVersion = getBaseVersion();
  const [major, minor, existingPatch] = parseVersion(baseVersion);
  
  // If patch is already specified, use it
  if (existingPatch !== undefined) {
    console.log(formatVersion([major, minor, existingPatch]));
    process.exit(0);
  }
  
  // Auto-generate patch version
  let patch: number;
  
  // Check if we're in GitHub Actions
  if (process.env.GITHUB_RUN_NUMBER) {
    // Use GitHub Actions run number as patch version
    patch = parseInt(process.env.GITHUB_RUN_NUMBER, 10) || 0;
  } else {
    // Use git commit count for local builds
    patch = getGitCommitCount(baseVersion);
  }
  
  const fullVersion = formatVersion([major, minor, patch]);
  console.log(fullVersion);
} catch (error) {
  console.error('Error:', error instanceof Error ? error.message : error);
  process.exit(1);
}

