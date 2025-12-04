/**
 * TensorFleet Environment Configuration
 * 
 * Central module for development/production mode detection and feature flags.
 * Initialize once during extension activation, then import anywhere.
 * 
 * TWO LEVELS OF DEV/PROD:
 * 
 * 1. BUILD-TIME (__DEV__, __PROD__):
 *    - Set when compiling: `bun run compile:dev` vs `bun run compile:prod`
 *    - Dead code elimination: `if (__DEV__) { ... }` removed entirely in prod builds
 *    - Use for: code that should NEVER ship to marketplace
 * 
 * 2. RUNTIME (isDev(), isProd()):
 *    - Detected from vscode.ExtensionMode when extension activates
 *    - Same binary can behave differently based on how it's launched
 *    - Use for: features that exist in prod but are enhanced in dev
 * 
 * Usage:
 *   // Build-time (stripped from prod package):
 *   if (__DEV__) {
 *     registerDebugCommand();  // This entire block removed in prod build
 *   }
 * 
 *   // Runtime (same binary, different behavior):
 *   if (isDev()) {
 *     showExtraDebugInfo();  // Code exists but only runs in dev mode
 *   }
 */

import * as vscode from 'vscode';

// ============================================================================
// State
// ============================================================================

let _extensionMode: vscode.ExtensionMode = vscode.ExtensionMode.Production;
let _initialized = false;

// ============================================================================
// Build-Time Constants (from vite.config.ts define)
// ============================================================================

/**
 * Check if this is a DEVELOPMENT BUILD
 * Code inside `if (__DEV__)` blocks is completely removed in production builds.
 */
export const BUILD_DEV = __DEV__;

/**
 * Check if this is a PRODUCTION BUILD
 */
export const BUILD_PROD = __PROD__;

/**
 * Build mode string
 */
export const BUILD_MODE = __BUILD_MODE__;

/**
 * Build timestamp (ISO string)
 */
export const BUILD_TIME = __BUILD_TIME__;

// ============================================================================
// Initialization
// ============================================================================

/**
 * Initialize the environment module with extension context.
 * Must be called once during extension activation (before using any other functions).
 */
export function initializeEnv(context: vscode.ExtensionContext): void {
  _extensionMode = context.extensionMode;
  _initialized = true;
  
  if (__DEV__) {
    console.log('[TensorFleet:Build] Development build');
    console.log('[TensorFleet:Build] Built at:', __BUILD_TIME__);
  }
}

/**
 * Check if the environment module has been initialized
 */
export function isInitialized(): boolean {
  return _initialized;
}

// ============================================================================
// Runtime Mode Detection
// ============================================================================

/**
 * Check if running in development mode (F5 debugging or test)
 * This is RUNTIME detection - the code exists but behavior changes.
 * For build-time exclusion, use `if (__DEV__)` directly.
 */
export function isDev(): boolean {
  return _extensionMode !== vscode.ExtensionMode.Production;
}

/**
 * Check if running in production mode (installed from marketplace)
 */
export function isProd(): boolean {
  return _extensionMode === vscode.ExtensionMode.Production;
}

/**
 * Check if running in test mode
 */
export function isTest(): boolean {
  return _extensionMode === vscode.ExtensionMode.Test;
}

/**
 * Get the current extension mode as a string
 */
export function getMode(): 'development' | 'production' | 'test' {
  switch (_extensionMode) {
    case vscode.ExtensionMode.Development:
      return 'development';
    case vscode.ExtensionMode.Test:
      return 'test';
    default:
      return 'production';
  }
}

/**
 * Get the raw extension mode enum value
 */
export function getExtensionMode(): vscode.ExtensionMode {
  return _extensionMode;
}

// ============================================================================
// Conditional Execution
// ============================================================================

/**
 * Execute a function only in development mode
 * Returns the function's result in dev mode, undefined in production
 */
export function devOnly<T>(fn: () => T): T | undefined {
  if (isDev()) {
    return fn();
  }
  return undefined;
}

/**
 * Execute a function only in production mode
 * Returns the function's result in production, undefined in dev
 */
export function prodOnly<T>(fn: () => T): T | undefined {
  if (isProd()) {
    return fn();
  }
  return undefined;
}

/**
 * Return different values based on environment
 */
export function envSwitch<T>(options: { dev: T; prod: T; test?: T }): T {
  if (isTest() && options.test !== undefined) {
    return options.test;
  }
  return isDev() ? options.dev : options.prod;
}

// ============================================================================
// Feature Flags
// ============================================================================

/**
 * Feature flags - controls what features are available
 * 
 * These combine BUILD-TIME and RUNTIME checks:
 * - Build features: Only exist in dev builds (code stripped in prod)
 * - Runtime features: Exist in all builds but behavior differs
 */
export interface FeatureFlags {
  /** Show local development server option in region selector (runtime) */
  localRegion: boolean;
  /** Enable verbose console logging (runtime) */
  verboseLogging: boolean;
  /** Show debug commands in command palette (build-time) */
  debugCommands: boolean;
  /** Enable experimental features (build-time) */
  experimental: boolean;
}

/**
 * Get feature flags based on current environment
 * 
 * Note: Some flags use __DEV__ (build-time), others use isDev() (runtime).
 * - __DEV__ features are completely stripped from prod builds
 * - isDev() features exist but are disabled at runtime in prod
 */
export function getFeatureFlags(): FeatureFlags {
  return {
    // Runtime flags (code exists in prod, just disabled)
    localRegion: isDev(),
    verboseLogging: isDev(),
    
    // Build-time flags (code stripped from prod builds)
    debugCommands: __DEV__,
    experimental: __DEV__,
  };
}

/**
 * Check if a specific feature is enabled
 */
export function isFeatureEnabled(feature: keyof FeatureFlags): boolean {
  return getFeatureFlags()[feature];
}

/**
 * Check if a feature should be INCLUDED in the build
 * Use this to guard entire code blocks that should be stripped:
 * 
 * if (shouldIncludeFeature('experimental')) {
 *   // This entire block removed in prod build
 * }
 */
export function shouldIncludeFeature(feature: 'debugCommands' | 'experimental'): boolean {
  // These are build-time only
  return __DEV__;
}

// ============================================================================
// Development Utilities
// ============================================================================

/**
 * Environment-aware utilities object
 * 
 * Uses __DEV__ for build-time elimination where possible.
 * In production builds, these become no-ops with minimal overhead.
 */
export const env = {
  /**
   * Console log only in development builds
   * Entire function body stripped in prod builds
   */
  log: (...args: unknown[]): void => {
    if (__DEV__) {
      console.log('[TensorFleet:Dev]', ...args);
    }
  },

  /**
   * Console warn only in development builds
   */
  warn: (...args: unknown[]): void => {
    if (__DEV__) {
      console.warn('[TensorFleet:Dev]', ...args);
    }
  },

  /**
   * Console error (always logs, but prefixed in dev)
   */
  error: (...args: unknown[]): void => {
    if (__DEV__) {
      console.error('[TensorFleet:Dev]', ...args);
    } else {
      console.error('[TensorFleet]', ...args);
    }
  },

  /**
   * Assert condition only in development (no-op in production builds)
   */
  assert: (condition: boolean, message: string): void => {
    if (__DEV__ && !condition) {
      console.error('[TensorFleet:Assert]', message);
    }
  },

  /**
   * Time a function execution (only logs in dev builds)
   */
  time: async <T>(label: string, fn: () => Promise<T>): Promise<T> => {
    if (!__DEV__) {
      return fn();
    }
    const start = performance.now();
    try {
      return await fn();
    } finally {
      const duration = performance.now() - start;
      console.log(`[TensorFleet:Time] ${label}: ${duration.toFixed(2)}ms`);
    }
  },

  /**
   * Get mode string for telemetry/logging
   */
  mode: getMode,
  
  /**
   * Get build info (useful for debugging)
   */
  buildInfo: () => ({
    buildMode: __BUILD_MODE__,
    buildTime: __BUILD_TIME__,
    runtimeMode: getMode(),
  }),
};

// ============================================================================
// VS Code Integration
// ============================================================================

/**
 * Show a notification only in development builds
 * In production builds, this is a no-op
 */
export function devNotification(message: string, type: 'info' | 'warn' | 'error' = 'info'): void {
  if (!__DEV__) return;
  
  const prefixed = `[Dev] ${message}`;
  switch (type) {
    case 'warn':
      vscode.window.showWarningMessage(prefixed);
      break;
    case 'error':
      vscode.window.showErrorMessage(prefixed);
      break;
    default:
      vscode.window.showInformationMessage(prefixed);
  }
}

/**
 * Register a command only in DEVELOPMENT BUILDS
 * In production builds, returns a no-op disposable (command doesn't exist)
 * 
 * Use this for debug/internal commands that shouldn't ship to marketplace.
 */
export function registerDevCommand(
  commandId: string,
  handler: (...args: unknown[]) => unknown
): vscode.Disposable {
  if (!__DEV__) {
    return new vscode.Disposable(() => {});
  }
  return vscode.commands.registerCommand(commandId, handler);
}

/**
 * Register a command that exists in all builds but behaves differently
 * Use isDev() inside the handler for runtime behavior changes.
 */
export function registerCommand(
  commandId: string,
  handler: (...args: unknown[]) => unknown
): vscode.Disposable {
  return vscode.commands.registerCommand(commandId, handler);
}

