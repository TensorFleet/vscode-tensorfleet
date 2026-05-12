/**
 * VSCode Plugin Authentication Module
 * 
 * This module handles OAuth authentication for TensorFleet VSCode extension.
 * It uses the platform-agnostic tensorfleet-auth package for core OAuth logic
 * and provides VSCode-specific implementations for:
 * - Token storage (using VSCode SecretStorage)
 * - Browser opening
 * - UI notifications
 * 
 * Based on vscode-plugin-auth/auth.js
 */

import * as vscode from 'vscode';
import * as http from 'http';
import type { AddressInfo } from 'net';
import { getBackendUrl as getRegionBackendUrl } from './regions';
import {
  initiateAuth,
  createCallbackHandler,
  isValidJwtShape,
  verifyToken,
  extractUserProfile,
  type OAuthConfig,
  type OAuthCallbacks,
  type UserProfile,
} from 'tensorfleet-auth';

/**
 * Get backend URL from the selected region configuration
 * URLs are derived from region selection - users cannot set explicit domains
 */
function getBackendUrl(): string {
  return getRegionBackendUrl().replace(/\/+$/, '');
}

// Single-flight login guard to prevent concurrent authenticate() calls
let loginInProgress = false;

// Active login state for cancellation support
let activeLoginState: {
  server: http.Server;
  timeoutId: NodeJS.Timeout;
  reject: (error: Error) => void;
} | null = null;

/**
 * Cancel any in-progress login attempt
 * Called when user logs out or explicitly cancels
 */
export function cancelLogin(): void {
  if (activeLoginState) {
    console.log('[Auth] Cancelling in-progress login');
    clearTimeout(activeLoginState.timeoutId);
    activeLoginState.server.close();
    activeLoginState.reject(new Error('Login cancelled'));
    activeLoginState = null;
  }
  loginInProgress = false;
}

/**
 * Check if login is currently in progress
 */
export function isLoginInProgress(): boolean {
  return loginInProgress;
}

/**
 * Main authentication function
 * Opens browser for login and waits for callback
 */
export async function authenticate(context: vscode.ExtensionContext): Promise<string> {
  // Single-flight guard: prevent concurrent login attempts
  if (loginInProgress) {
    vscode.window.showWarningMessage('Login already in progress. Please complete the current login first.');
    throw new Error('Login already in progress');
  }

  loginInProgress = true;

  try {
    const backendUrl = getBackendUrl();
    const config: OAuthConfig = { backendUrl };

    // Step 1: Initiate OAuth flow using core module
    const { state, authUrl } = await initiateAuth(config);

    vscode.window.showInformationMessage('Opening browser for authentication...');

    // Step 2: Start local server with ephemeral port to receive callback
    const callbacks: OAuthCallbacks = {
      openBrowser: async (url: string) => {
        vscode.env.openExternal(vscode.Uri.parse(url));
      },
      onTokenReceived: async (token: string) => {
        // Validate JWT shape before storing
        if (!isValidJwtShape(token)) {
          throw new Error('Invalid token format - not a valid JWT');
        }
      },
      onError: (error: Error) => {
        console.error('[Auth] OAuth error:', error);
      }
    };

    // Create and start callback server
    const server = http.createServer();
    let timeoutId: NodeJS.Timeout;

    // Store for cancellation
    activeLoginState = { server, timeoutId: null as any, reject: () => {} };

    const token = await new Promise<string>((resolve, reject) => {
      // Update reject function
      activeLoginState!.reject = reject;

      server.on('listening', () => {
        const addressInfo = server.address() as AddressInfo;
        const port = addressInfo.port;
        const callbackBaseUrl = `http://127.0.0.1:${port}/callback`;

        // Create callback handler using core module
        const handler = createCallbackHandler({
          expectedState: state,
          callbacks: {
            openBrowser: callbacks.openBrowser,
            onTokenReceived: async (token: string) => {
              clearTimeout(timeoutId);
              server.close();
              await callbacks.onTokenReceived(token);
              resolve(token);
            },
            onError: (error: Error) => {
              clearTimeout(timeoutId);
              server.close();
              callbacks.onError(error);
              reject(error);
            }
          }
        });

        // Attach request handler
        server.on('request', handler);

        // Step 3: Open browser with callbackBaseUrl appended
        const finalAuthUrl = `${authUrl}&callbackBaseUrl=${encodeURIComponent(callbackBaseUrl)}`;
        vscode.env.openExternal(vscode.Uri.parse(finalAuthUrl));
      });

      server.on('error', (err: Error) => {
        activeLoginState = null;
        reject(new Error(`Failed to start callback server: ${err.message}`));
      });

      // Listen on ephemeral port (0) bound to localhost
      server.listen(0, '127.0.0.1');

      // Timeout after 5 minutes
      timeoutId = setTimeout(() => {
        activeLoginState = null;
        server.close();
        reject(new Error('Authentication timeout'));
      }, 5 * 60 * 1000);

      // Update stored reference with actual timeoutId
      if (activeLoginState) {
        activeLoginState.timeoutId = timeoutId;
      }
    });

    // Step 4: Store token securely
    await storeToken(context, token);

    vscode.window.showInformationMessage('Successfully authenticated!');

    return token;
  } catch (error) {
    console.error('[Auth] authenticate() error:', error);
    // Don't show error message for cancellation
    if (error instanceof Error && error.message !== 'Login cancelled') {
      vscode.window.showErrorMessage(`Authentication failed: ${error.message}`);
    }
    throw error;
  } finally {
    loginInProgress = false;
    activeLoginState = null;
  }
}

// ============================================================================
// Token Storage Functions (VSCode-specific)
// ============================================================================

/**
 * Store token securely in VSCode secret storage
 * Validates JWT shape before storing to prevent malformed tokens
 */
export async function storeToken(context: vscode.ExtensionContext, token: string): Promise<void> {
  if (!isValidJwtShape(token)) {
    console.error('[Auth] Refusing to store invalid JWT-shaped token');
    throw new Error('Invalid token format - not a valid JWT');
  }
  await context.secrets.store('tensorfleet-auth-token', token);
  // Clear verification cache so next isAuthenticated() call will verify with backend
  verificationCache = null;
}

/**
 * Get stored token
 */
export async function getToken(context: vscode.ExtensionContext): Promise<string | undefined> {
  const token = await context.secrets.get('tensorfleet-auth-token');
  return token;
}

/**
 * Clear stored token (logout)
 * Also cancels any in-progress login attempt
 */
export async function clearToken(context: vscode.ExtensionContext): Promise<void> {
  // Cancel any in-progress login when user logs out
  cancelLogin();
  await context.secrets.delete('tensorfleet-auth-token');
  verificationCache = null; // Clear cache on logout
}

/**
 * Get user profile from stored token
 */
export async function getUserProfile(context: vscode.ExtensionContext): Promise<UserProfile | null> {
  const token = await getToken(context);
  if (!token) {
    return null;
  }
  return extractUserProfile(token);
}

// Cache for token verification to avoid excessive API calls
let verificationCache: { valid: boolean; timestamp: number } | null = null;
const VERIFICATION_CACHE_TTL = 30000; // 30 seconds

/**
 * Check if user is authenticated
 * Verifies token with backend and caches result
 * 
 * HTTP status handling:
 * - 401: Definitive invalidation → clear token
 * - 200: Parse JSON and use data.valid
 * - 5xx/other: Fall back to JWT expiry check, do NOT clear token
 */
export async function isAuthenticated(context: vscode.ExtensionContext): Promise<boolean> {
  const token = await getToken(context);
  if (!token) {
    return false;
  }

  // Check cache first
  if (verificationCache && Date.now() - verificationCache.timestamp < VERIFICATION_CACHE_TTL) {
    return verificationCache.valid;
  }

  // Verify token with backend using core module
  try {
    const config: OAuthConfig = { backendUrl: getBackendUrl() };
    const valid = await verifyToken(config, token);

    verificationCache = { valid, timestamp: Date.now() };

    if (!valid) {
      // Backend says invalid with 200 OK - trust it
      await clearToken(context);
    }

    return valid;
  } catch (error) {
    console.error('[Auth] Token verification error:', error);

    // Fallback to basic JWT validation if backend is unreachable or returned error
    try {
      const payload = JSON.parse(Buffer.from(token.split('.')[1], 'base64').toString());
      const now = Math.floor(Date.now() / 1000);

      if (payload.exp && payload.exp < now) {
        // Token actually expired - clear it
        console.log('[Auth] JWT expired, clearing token');
        await clearToken(context);
        return false;
      }

      // Token format is valid and not expired, but couldn't verify with backend
      // Return true but don't cache (so we retry next time)
      return true;
    } catch (parseError) {
      // Invalid token format - this shouldn't happen with JWT validation on store
      console.error('[Auth] Invalid token format:', parseError);
      await clearToken(context);
      return false;
    }
  }
}

/**
 * Make authenticated API request
 */
export async function authenticatedFetch(
  context: vscode.ExtensionContext,
  url: string,
  options: RequestInit = {}
): Promise<Response> {
  const token = await getToken(context);

  if (!token) {
    console.error('[Auth] authenticatedFetch() called without token - not authenticated');
    throw new Error('Not authenticated');
  }

  const headers = {
    ...options.headers,
    'Authorization': `Bearer ${token}`,
  };

  const response = await fetch(url, {
    ...options,
    headers,
  });

  // If unauthorized, token might be expired
  if (response.status === 401) {
    await clearToken(context);
    throw new Error('Token expired - please login again');
  }

  return response;
}