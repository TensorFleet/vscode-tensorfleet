/**
 * VSCode Plugin Authentication Module
 * 
 * This module handles OAuth authentication for TensorFleet VSCode extension.
 * Integrates with backend OAuth flow via browser-based login.
 * 
 * Based on vscode-plugin-auth/auth.js
 */

import * as vscode from 'vscode';
import * as http from 'http';
import { URL } from 'url';

// Configuration
const CALLBACK_PORT = 3456; // Local server port for OAuth callback

// Type definitions for API responses
interface InitiateAuthResponse {
    state: string;
    authUrl: string;
}

interface PollAuthResponse {
    authenticated: boolean;
    token?: string;
}

interface VerifyTokenResponse {
    valid: boolean;
}

/**
 * Get backend URL from configuration or environment
 */
function getBackendUrl(): string {
    const defaultUrl = 'https://app.tensorfleet.net';
    const configuredUrl = vscode.workspace.getConfiguration('tensorfleet').get<string>('backendUrl');
    
    // Trim trailing slashes to keep request paths consistent
    return (configuredUrl?.trim() || defaultUrl).replace(/\/+$/, '');
}

/**
 * Main authentication function
 * Opens browser for login and waits for callback
 */
export async function authenticate(context: vscode.ExtensionContext): Promise<string> {
    try {
        // Step 1: Initiate OAuth flow
        const { state, authUrl } = await initiateAuth();
        
        vscode.window.showInformationMessage(
            'Opening browser for authentication...'
        );

        // Step 2: Start local server to receive callback
        const token = await new Promise<string>((resolve, reject) => {
            const server = createCallbackServer(state, resolve, reject);

            // Step 3: Open browser
            vscode.env.openExternal(vscode.Uri.parse(authUrl));

            // Timeout after 5 minutes
            setTimeout(() => {
                server.close();
                reject(new Error('Authentication timeout'));
            }, 5 * 60 * 1000);  
        });

        // Step 4: Store token securely
        await storeToken(context, token);

        vscode.window.showInformationMessage('Successfully authenticated!');

        return token;
    } catch (error) {
        console.error('[Auth] authenticate() error:', error);
        vscode.window.showErrorMessage(`Authentication failed: ${error instanceof Error ? error.message : String(error)}`);
        throw error;
    }
}

/**
 * Alternative: Poll-based authentication
 * Use this if callback server has issues
 */
export async function authenticateWithPolling(context: vscode.ExtensionContext): Promise<string> {
    try {
        // Step 1: Initiate OAuth flow
        const { state, authUrl } = await initiateAuth();
        
        vscode.window.showInformationMessage(
            'Opening browser for authentication...'
        );

        // Step 2: Open browser
        vscode.env.openExternal(vscode.Uri.parse(authUrl));

        // Step 3: Poll for authentication status
        const token = await pollAuthStatus(state);

        // Step 4: Store token
        await storeToken(context, token);

        vscode.window.showInformationMessage('Successfully authenticated!');

        return token;
    } catch (error) {
        console.error('[Auth] authenticateWithPolling() error:', error);
        vscode.window.showErrorMessage(`Authentication failed: ${error instanceof Error ? error.message : String(error)}`);
        throw error;
    }
}

/**
 * Initiate OAuth flow with backend
 */
async function initiateAuth(): Promise<{ state: string; authUrl: string }> {
    const BACKEND_URL = getBackendUrl();
    
    const response = await fetch(`${BACKEND_URL}/api/auth/vscode/initiate`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
    });

    if (!response.ok) {
        console.error('[Auth] initiateAuth() failed, non-OK status:', response.status);
        throw new Error('Failed to initiate authentication');
    }

    const data = await response.json() as InitiateAuthResponse;
    return {
        state: data.state,
        authUrl: data.authUrl,
    };
}

/**
 * Create local HTTP server to receive OAuth callback
 */
function createCallbackServer(
    expectedState: string,
    resolve: (token: string) => void,
    reject: (error: Error) => void
): http.Server {
    const server = http.createServer((req, res) => {
        const url = new URL(req.url || '/', `http://localhost:${CALLBACK_PORT}`);

        if (url.pathname === '/callback') {
            const token = url.searchParams.get('token');
            const state = url.searchParams.get('state');
            const error = url.searchParams.get('error');

            if (error) {
                res.writeHead(200, { 'Content-Type': 'text/html' });
                res.end(`
                    <html>
                        <body style="font-family: Arial; text-align: center; padding: 50px;">
                            <h1>❌ Authentication Failed</h1>
                            <p>${error}</p>
                            <p>You can close this window.</p>
                        </body>
                    </html>
                `);
                server.close();
                console.error('[Auth] Callback error from query parameter:', error);
                reject(new Error(error));
                return;
            }

            if (!token || state !== expectedState) {
                console.error('[Auth] Invalid callback - token missing or state mismatch:', {
                    hasToken: !!token,
                    stateMatches: state === expectedState,
                });
                res.writeHead(400, { 'Content-Type': 'text/html' });
                res.end(`
                    <html>
                        <body style="font-family: Arial; text-align: center; padding: 50px;">
                            <h1>❌ Invalid Callback</h1>
                            <p>You can close this window.</p>
                        </body>
                    </html>
                `);
                server.close();
                reject(new Error('Invalid callback'));
                return;
            }

            // Success!
            res.writeHead(200, { 'Content-Type': 'text/html' });
            res.end(`
<!DOCTYPE html>
<html>
  <head>
    <meta charset="UTF-8" />
    <title>TensorFleet Authentication</title>
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
        display: flex;
        align-items: center;
        justify-content: center;
        height: 100vh;
        margin: 0;
        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      }
      .container {
        background: white;
        padding: 2rem;
        border-radius: 8px;
        box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        text-align: center;
        max-width: 420px;
        width: 100%;
      }
      .success-icon {
        width: 64px;
        height: 64px;
        background: #10b981;
        border-radius: 50%;
        display: flex;
        align-items: center;
        justify-content: center;
        margin: 0 auto 1rem;
      }
      h1 {
        color: #1f2937;
        margin: 0 0 0.5rem;
      }
      p {
        color: #6b7280;
        margin: 0.25rem 0;
      }
      .hint {
        margin-top: 1rem;
        font-size: 0.9rem;
        color: #4b5563;
      }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="success-icon">
        <svg width="32" height="32" fill="white" viewBox="0 0 20 20">
          <path fill-rule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clip-rule="evenodd" />
        </svg>
      </div>
      <h1>Authentication Successful</h1>
      <p>You're now signed in to TensorFleet.</p>
      <p class="hint">You can close this window and return to VS Code.</p>
    </div>
  </body>
</html>
            `);

            server.close();
            resolve(token);
        } else {
            res.writeHead(404);
            res.end('Not found');
        }
    });

    server.listen(CALLBACK_PORT);

    return server;
}

/**
 * Poll backend for authentication status
 * Alternative to callback server approach
 */
async function pollAuthStatus(
    state: string,
    maxAttempts: number = 60,
    interval: number = 2000
): Promise<string> {
    const BACKEND_URL = getBackendUrl();
    
    for (let i = 0; i < maxAttempts; i++) {
        try {
            const response = await fetch(
                `${BACKEND_URL}/api/auth/vscode/poll/${state}`
            );

            if (!response.ok) {
                console.error('[Auth] pollAuthStatus() non-OK status:', response.status);
                throw new Error('Failed to poll authentication status');
            }

            const data = await response.json() as PollAuthResponse;

            if (data.authenticated && data.token) {
                return data.token;
            }

            // Wait before next poll
            await new Promise(resolve => setTimeout(resolve, interval));
        } catch (error) {
            console.error('Poll error:', error);
            // Continue polling even on errors
        }
    }

    throw new Error('Authentication timeout - please try again');
}


// ============================================================================
// Token Storage Functions
// ============================================================================

/**
 * Store token securely in VSCode secret storage
 */
export async function storeToken(context: vscode.ExtensionContext, token: string): Promise<void> {
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
 */
export async function clearToken(context: vscode.ExtensionContext): Promise<void> {
    await context.secrets.delete('tensorfleet-auth-token');
    verificationCache = null; // Clear cache on logout
}

// Cache for token verification to avoid excessive API calls
let verificationCache: { valid: boolean; timestamp: number } | null = null;
const VERIFICATION_CACHE_TTL = 30000; // 30 seconds

/**
 * Check if user is authenticated
 * Verifies token with backend and caches result
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

    // Verify token with backend
    try {
        const BACKEND_URL = getBackendUrl();
        
        const response = await fetch(`${BACKEND_URL}/api/auth/verify`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ token }),
        });

        const data = await response.json() as VerifyTokenResponse;
        const valid = data.valid === true;

        // Update cache
        verificationCache = {
            valid,
            timestamp: Date.now()
        };

        if (!valid) {
            await clearToken(context);
        }

        return valid;
    } catch (error) {
        console.error('[Auth] Token verification error:', error);
        
        // Fallback to basic JWT validation if backend is unreachable
        try {
            const payload = JSON.parse(Buffer.from(token.split('.')[1], 'base64').toString());
            const now = Math.floor(Date.now() / 1000);

            if (payload.exp && payload.exp < now) {
                await clearToken(context);
                return false;
            }

            // Token format is valid but couldn't verify with backend
            // Return true but don't cache (so we retry next time)
            return true;
        } catch (parseError) {
            // Invalid token format
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
