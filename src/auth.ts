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
    console.log('[Auth] getBackendUrl() called');
    const defaultUrl = 'https://app.tensorfleet.net';
    console.log('[Auth] Using fixed backend URL:', defaultUrl);
    return defaultUrl;
}

/**
 * Main authentication function
 * Opens browser for login and waits for callback
 */
export async function authenticate(context: vscode.ExtensionContext): Promise<string> {
    console.log('[Auth] authenticate() called (callback method)');
    try {
        // Step 1: Initiate OAuth flow
        console.log('[Auth] authenticate() initiating OAuth flow via initiateAuth()');
        const { state, authUrl } = await initiateAuth();
        console.log('[Auth] authenticate() got initiateAuth response:', {
            stateLength: state?.length ?? 0,
            hasAuthUrl: !!authUrl,
        });
        
        vscode.window.showInformationMessage(
            'Opening browser for authentication...'
        );

        // Step 2: Start local server to receive callback
        const token = await new Promise<string>((resolve, reject) => {
            console.log('[Auth] authenticate() creating callback server...');
            const server = createCallbackServer(state, resolve, reject);

            // Step 3: Open browser
            console.log('[Auth] authenticate() opening external browser with auth URL');
            vscode.env.openExternal(vscode.Uri.parse(authUrl));

            // Timeout after 5 minutes
            setTimeout(() => {
                console.log('[Auth] authenticate() timeout reached, closing callback server');
                server.close();
                reject(new Error('Authentication timeout'));
            }, 5 * 60 * 1000);  
        });

        // Step 4: Store token securely
        await storeToken(context, token);

        vscode.window.showInformationMessage('Successfully authenticated!');

        console.log('[Auth] authenticate() completed successfully, token stored');
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
    console.log('[Auth] authenticateWithPolling() called (polling method)');
    try {
        // Step 1: Initiate OAuth flow
        console.log('[Auth] authenticateWithPolling() initiating OAuth flow via initiateAuth()');
        const { state, authUrl } = await initiateAuth();
        console.log('[Auth] authenticateWithPolling() got initiateAuth response:', {
            stateLength: state?.length ?? 0,
            hasAuthUrl: !!authUrl,
        });
        
        vscode.window.showInformationMessage(
            'Opening browser for authentication...'
        );

        // Step 2: Open browser
        console.log('[Auth] authenticateWithPolling() opening external browser with auth URL');
        vscode.env.openExternal(vscode.Uri.parse(authUrl));

        // Step 3: Poll for authentication status
        console.log('[Auth] authenticateWithPolling() starting pollAuthStatus()');
        const token = await pollAuthStatus(state);

        // Step 4: Store token
        await storeToken(context, token);

        vscode.window.showInformationMessage('Successfully authenticated!');

        console.log('[Auth] authenticateWithPolling() completed successfully, token stored');
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
    console.log('[Auth] initiateAuth() called - POST to', `${BACKEND_URL}/api/auth/vscode/initiate`);
    
    const response = await fetch(`${BACKEND_URL}/api/auth/vscode/initiate`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
    });

    console.log('[Auth] initiateAuth() response status:', response.status);

    if (!response.ok) {
        console.error('[Auth] initiateAuth() failed, non-OK status:', response.status);
        throw new Error('Failed to initiate authentication');
    }

    const data = await response.json() as InitiateAuthResponse;
    console.log('[Auth] initiateAuth() parsed response:', {
        hasState: !!data.state,
        stateLength: data.state?.length ?? 0,
        hasAuthUrl: !!data.authUrl,
    });
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
    console.log('[Auth] createCallbackServer() called, expectedState length:', expectedState?.length ?? 0);
    const server = http.createServer((req, res) => {
        console.log('[Auth] Callback server received request:', {
            method: req.method,
            url: req.url,
        });
        const url = new URL(req.url || '/', `http://localhost:${CALLBACK_PORT}`);

        if (url.pathname === '/callback') {
            const token = url.searchParams.get('token');
            const state = url.searchParams.get('state');
            const error = url.searchParams.get('error');

            console.log('[Auth] /callback hit with params:', {
                hasToken: !!token,
                tokenLength: token?.length ?? 0,
                stateLength: state?.length ?? 0,
                error,
            });

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
            console.log('[Auth] Authentication callback successful, resolving token');
            res.writeHead(200, { 'Content-Type': 'text/html' });
            res.end(`
                <html>
                    <body style="font-family: Arial; text-align: center; padding: 50px;">
                        <h1>✅ Authentication Successful!</h1>
                        <p>You can close this window and return to VSCode.</p>
                        <script>setTimeout(() => window.close(), 2000);</script>
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

    server.listen(CALLBACK_PORT, () => {
        console.log(`[Auth] Callback server listening on port ${CALLBACK_PORT}`);
    });

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
    console.log('[Auth] pollAuthStatus() called with:', {
        stateLength: state?.length ?? 0,
        maxAttempts,
        interval,
        endpoint: `${BACKEND_URL}/api/auth/vscode/poll/${state}`,
    });
    
    for (let i = 0; i < maxAttempts; i++) {
        try {
            console.log('[Auth] pollAuthStatus() attempt', i + 1, 'of', maxAttempts);
            const response = await fetch(
                `${BACKEND_URL}/api/auth/vscode/poll/${state}`
            );

            console.log('[Auth] pollAuthStatus() response status:', response.status);

            if (!response.ok) {
                console.error('[Auth] pollAuthStatus() non-OK status:', response.status);
                throw new Error('Failed to poll authentication status');
            }

            const data = await response.json() as PollAuthResponse;
            console.log('[Auth] pollAuthStatus() parsed response:', data);

            if (data.authenticated && data.token) {
                console.log('[Auth] pollAuthStatus() authenticated, token length:', data.token.length);
                return data.token;
            }

            // Wait before next poll
            console.log('[Auth] pollAuthStatus() not yet authenticated, waiting', interval, 'ms before next attempt');
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
    console.log('[Auth] storeToken() called, token length:', token?.length ?? 0);
    console.log('[Auth] Storing token...');
    await context.secrets.store('tensorfleet-auth-token', token);
    // Clear verification cache so next isAuthenticated() call will verify with backend
    verificationCache = null;
    console.log('[Auth] Token stored successfully');
}

/**
 * Get stored token
 */
export async function getToken(context: vscode.ExtensionContext): Promise<string | undefined> {
    console.log('[Auth] getToken() called');
    const token = await context.secrets.get('tensorfleet-auth-token');
    console.log('[Auth] getToken() result:', {
        hasToken: !!token,
        tokenLength: token?.length ?? 0,
    });
    return token;
}

/**
 * Clear stored token (logout)
 */
export async function clearToken(context: vscode.ExtensionContext): Promise<void> {
    console.log('[Auth] clearToken() called - deleting stored token and clearing cache');
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
    console.log('[Auth] isAuthenticated() called - checking authentication status...');
    const token = await getToken(context);
    if (!token) {
        console.log('[Auth] No token found');
        return false;
    }

    console.log('[Auth] Token found, length:', token.length);

    // Check cache first
    if (verificationCache && Date.now() - verificationCache.timestamp < VERIFICATION_CACHE_TTL) {
        console.log('[Auth] Using cached verification result:', verificationCache.valid);
        return verificationCache.valid;
    }

    // Verify token with backend
    try {
        const BACKEND_URL = getBackendUrl();
        console.log('[Auth] Verifying token with backend:', BACKEND_URL);
        
        const response = await fetch(`${BACKEND_URL}/api/auth/verify`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ token }),
        });

        console.log('[Auth] Backend response status:', response.status);

        const data = await response.json() as VerifyTokenResponse;
        const valid = data.valid === true;

        console.log('[Auth] Token verification result:', valid);

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
            console.log('[Auth] Attempting fallback JWT validation...');
            const payload = JSON.parse(Buffer.from(token.split('.')[1], 'base64').toString());
            const now = Math.floor(Date.now() / 1000);

            console.log('[Auth] JWT payload exp:', payload.exp, 'now:', now);

            if (payload.exp && payload.exp < now) {
                console.log('[Auth] Token expired');
                await clearToken(context);
                return false;
            }

            // Token format is valid but couldn't verify with backend
            // Return true but don't cache (so we retry next time)
            console.log('[Auth] Token appears valid (fallback validation)');
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
    console.log('[Auth] authenticatedFetch() called for URL:', url);
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

    console.log('[Auth] authenticatedFetch() response status:', response.status);

    // If unauthorized, token might be expired
    if (response.status === 401) {
        console.warn('[Auth] authenticatedFetch() received 401, clearing token and prompting re-login');
        await clearToken(context);
        throw new Error('Token expired - please login again');
    }

    return response;
}
