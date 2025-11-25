/**
 * VSCode Plugin Authentication Module (Mock Version)
 * 
 * This module handles authentication for TensorFleet VSCode extension.
 * Currently using a mock flow with simple token input.
 * 
 * TODO: Replace with real Clerk OAuth flow when backend is ready.
 * 
 * Based on vscode-plugin-auth/auth.js
 */

import * as vscode from 'vscode';
import * as http from 'http';
import { URL } from 'url';

// Configuration
const CALLBACK_PORT = 3456; // Local server port for token callback

// MOCK AUTH: Simple token input flow
// TODO: Replace with real OAuth flow when backend is ready
// Real flow will use: BACKEND_URL = 'http://localhost:3000' or 'https://app.tensorfleet.net'

/**
 * Main authentication function (Mock Version)
 * Opens a simple HTML page for token input and waits for callback
 */
export async function authenticate(context: vscode.ExtensionContext): Promise<string> {
    try {
        vscode.window.showInformationMessage(
            'Opening browser for token input...'
        );

        // Step 1: Start local server to receive token callback
        const token = await new Promise<string>((resolve, reject) => {
            const server = createCallbackServer(resolve, reject);
            
            // Step 2: Open browser to token input page
            // MOCK: Simple HTML page with token input
            // TODO: Replace with real OAuth URL: vscode.env.openExternal(vscode.Uri.parse(authUrl));
            const tokenInputUrl = `http://localhost:${CALLBACK_PORT}/token-input`;
            vscode.env.openExternal(vscode.Uri.parse(tokenInputUrl));

            // Timeout after 5 minutes
            setTimeout(() => {
                server.close();
                reject(new Error('Authentication timeout'));
            }, 5 * 60 * 1000);
        });

        // Step 3: Store token securely
        await storeToken(context, token);

        vscode.window.showInformationMessage('Successfully authenticated!');
        
        return token;
    } catch (error) {
        console.error('Authentication error:', error);
        vscode.window.showErrorMessage(`Authentication failed: ${error instanceof Error ? error.message : String(error)}`);
        throw error;
    }
}

/**
 * Create local HTTP server to receive token callback
 * MOCK: Serves a simple HTML page for token input
 * TODO: Replace with real OAuth callback handler when backend is ready
 */
function createCallbackServer(resolve: (token: string) => void, reject: (error: Error) => void): http.Server {
    const server = http.createServer((req, res) => {
        const url = new URL(req.url || '/', `http://localhost:${CALLBACK_PORT}`);
        
        // Serve token input page
        if (url.pathname === '/token-input') {
            res.writeHead(200, { 'Content-Type': 'text/html' });
            res.end(getTokenInputHtml());
            return;
        }
        
        // Handle token submission callback
        if (url.pathname === '/callback') {
            const token = url.searchParams.get('token');
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
                reject(new Error(error));
                return;
            }

            if (!token) {
                res.writeHead(400, { 'Content-Type': 'text/html' });
                res.end(`
                    <html>
                        <body style="font-family: Arial; text-align: center; padding: 50px;">
                            <h1>❌ Invalid Callback</h1>
                            <p>No token provided</p>
                            <p>You can close this window.</p>
                        </body>
                    </html>
                `);
                server.close();
                reject(new Error('Invalid callback - no token'));
                return;
            }

            // Success!
            res.writeHead(200, { 'Content-Type': 'text/html' });
            res.end(`
                <html>
                    <body style="font-family: Arial; text-align: center; padding: 50px;">
                        <h1>Authentication Successful</h1>
                        <p>Token received. You can close this window and return to VSCode.</p>
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
        console.log(`[Auth] Token input server listening on port ${CALLBACK_PORT}`);
    });
    
    return server;
}

/**
 * Get HTML for token input page
 * MOCK: Simple form for token input
 * TODO: Remove when real OAuth flow is implemented
 */
function getTokenInputHtml(): string {
    return `
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>TensorFleet Authentication</title>
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
            margin: 0;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 20px;
        }
        .container {
            background: white;
            border-radius: 12px;
            padding: 40px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
            max-width: 500px;
            width: 100%;
        }
        h1 {
            margin: 0 0 10px 0;
            color: #333;
            font-size: 24px;
        }
        .subtitle {
            color: #666;
            margin-bottom: 30px;
            font-size: 14px;
        }
        .form-group {
            margin-bottom: 20px;
        }
        label {
            display: block;
            margin-bottom: 8px;
            color: #333;
            font-weight: 500;
            font-size: 14px;
        }
        textarea {
            width: 100%;
            padding: 12px;
            border: 2px solid #e0e0e0;
            border-radius: 6px;
            font-family: 'Courier New', monospace;
            font-size: 12px;
            resize: vertical;
            min-height: 100px;
            box-sizing: border-box;
        }
        textarea:focus {
            outline: none;
            border-color: #667eea;
        }
        button {
            width: 100%;
            padding: 12px;
            background: #667eea;
            color: white;
            border: none;
            border-radius: 6px;
            font-size: 16px;
            font-weight: 600;
            cursor: pointer;
            transition: background 0.2s;
        }
        button:hover {
            background: #5568d3;
        }
        button:disabled {
            background: #ccc;
            cursor: not-allowed;
        }
        .info {
            background: #f5f5f5;
            padding: 15px;
            border-radius: 6px;
            margin-top: 20px;
            font-size: 12px;
            color: #666;
        }
        .info strong {
            color: #333;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>TensorFleet Authentication</h1>
        <p class="subtitle">Enter your authentication token</p>
        
        <form id="tokenForm">
            <div class="form-group">
                <label for="token">Authentication Token:</label>
                <textarea id="token" name="token" placeholder="Paste your JWT token here..." required></textarea>
            </div>
            <button type="submit" id="submitBtn">Authenticate</button>
        </form>
        
        <div class="info">
            <strong>Note:</strong> This is a mock authentication flow for development.
            The token will be stored securely in VSCode and used for VM Manager API calls.
        </div>
    </div>
    
    <script>
        const form = document.getElementById('tokenForm');
        const tokenInput = document.getElementById('token');
        const submitBtn = document.getElementById('submitBtn');
        
        form.addEventListener('submit', (e) => {
            e.preventDefault();
            const token = tokenInput.value.trim();
            
            if (!token) {
                alert('Please enter a token');
                return;
            }
            
            submitBtn.disabled = true;
            submitBtn.textContent = 'Authenticating...';
            
            // Redirect to callback with token
            const callbackUrl = 'http://localhost:' + '${CALLBACK_PORT}' + '/callback?token=' + encodeURIComponent(token);
            window.location.href = callbackUrl;
        });
    </script>
</body>
</html>
    `.replace(/\$\{CALLBACK_PORT\}/g, String(CALLBACK_PORT));
}

// ============================================================================
// Real OAuth flow functions (commented out - will be used when backend is ready)
// ============================================================================

/**
 * Initiate OAuth flow with backend
 * TODO: Uncomment when backend is ready
 */
/*
async function initiateAuth() {
    const BACKEND_URL = 'http://localhost:3000'; // or 'https://app.tensorfleet.net'
    const response = await fetch(`${BACKEND_URL}/api/auth/vscode/initiate`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
    });

    if (!response.ok) {
        throw new Error('Failed to initiate authentication');
    }

    const data = await response.json();
    return {
        state: data.state,
        authUrl: data.authUrl,
    };
}
*/

/**
 * Poll backend for authentication status
 * Alternative to callback server approach
 * TODO: Uncomment when backend is ready
 */
/*
async function pollAuthStatus(state: string, maxAttempts = 60, interval = 2000): Promise<string> {
    const BACKEND_URL = 'http://localhost:3000';
    for (let i = 0; i < maxAttempts; i++) {
        try {
            const response = await fetch(
                `${BACKEND_URL}/api/auth/vscode/poll/${state}`
            );

            if (!response.ok) {
                throw new Error('Failed to poll authentication status');
            }

            const data = await response.json();

            if (data.authenticated) {
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
*/

// ============================================================================
// Token Storage Functions
// ============================================================================

/**
 * Store token securely in VSCode secret storage
 */
export async function storeToken(context: vscode.ExtensionContext, token: string): Promise<void> {
    await context.secrets.store('tensorfleet-auth-token', token);
}

/**
 * Get stored token
 */
export async function getToken(context: vscode.ExtensionContext): Promise<string | undefined> {
    return await context.secrets.get('tensorfleet-auth-token');
}

/**
 * Clear stored token (logout)
 */
export async function clearToken(context: vscode.ExtensionContext): Promise<void> {
    await context.secrets.delete('tensorfleet-auth-token');
}

/**
 * Check if user is authenticated
 * Performs basic JWT validation (check expiration without calling backend)
 * TODO: Add actual token verification endpoint call when backend is ready
 * TODO: Cache result for 30 seconds to avoid excessive API calls
 */
export async function isAuthenticated(context: vscode.ExtensionContext): Promise<boolean> {
    const token = await getToken(context);
    if (!token) {
        return false;
    }

    // Basic JWT validation (check expiration without calling backend)
    try {
        const payload = JSON.parse(Buffer.from(token.split('.')[1], 'base64').toString());
        const now = Math.floor(Date.now() / 1000);
        
        if (payload.exp && payload.exp < now) {
            await clearToken(context); // Auto-clear expired token
            return false;
        }
        
        return true;
    } catch (error) {
        // Invalid token format
        await clearToken(context);
        return false;
    }

    // TODO: Better fix (when backend ready):
    // Add actual token verification endpoint call
    // Cache result for 30 seconds to avoid excessive API calls
    /*
    try {
        const BACKEND_URL = 'http://localhost:3000';
        const response = await fetch(`${BACKEND_URL}/api/auth/verify`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ token }),
        });

        const data = await response.json();
        return data.valid === true;
    } catch (error) {
        console.error('Token verification error:', error);
        return false;
    }
    */
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
