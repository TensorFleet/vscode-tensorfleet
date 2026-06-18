/**
 * TensorFleet Auth - Platform-Agnostic OAuth Module
 * 
 * This module provides OAuth authentication logic that can be used
 * across different platforms (VSCode, web, CLI, etc.).
 * 
 * It does NOT handle session storage - the caller is responsible
 * for storing and retrieving tokens.
 */

export * from './oauth-core.js';
export * from './types.js';
export * from './regions.js';
export * from './vm-config.js';
export {
  buildVmSnapshot,
  fetchVmIdFromManager,
  isVmAuthError,
  isVmNotFoundError,
} from './vm-manager.js';
export type {
  VmHttpError,
} from './vm-manager.js';
export * from './vm-manager-client.js';
export * from './global-auth.js';
export * from './config-store.js';
