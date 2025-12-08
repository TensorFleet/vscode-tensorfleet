/**
 * Build-time constants injected by Vite
 * 
 * These are replaced at compile time, enabling dead code elimination.
 * In production builds, `if (__DEV__) { ... }` blocks are completely removed.
 * 
 * Build commands:
 *   bun run compile:dev   - Development build (__DEV__ = true)
 *   bun run compile:prod  - Production build (__DEV__ = false)
 *   bun run compile       - Defaults to production
 */

/** True when built with TENSORFLEET_BUILD=development */
declare const __DEV__: boolean;

/** True when built with TENSORFLEET_BUILD=production (default) */
declare const __PROD__: boolean;

/** Build mode string: 'development' | 'production' */
declare const __BUILD_MODE__: 'development' | 'production';

/** ISO timestamp of when the build was created */
declare const __BUILD_TIME__: string;

