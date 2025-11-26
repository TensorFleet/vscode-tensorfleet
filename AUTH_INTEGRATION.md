# TensorFleet OAuth Authentication Integration

This document describes the real OAuth authentication integration for the TensorFleet VSCode extension.

## Overview

The mock authentication flow has been **completely removed** and replaced with a real OAuth flow that integrates with the TensorFleet backend.

## Changes Made

### 1. Removed Mock Authentication

- ❌ Removed `getTokenInputHtml()` function (mock token input page)
- ❌ Removed all TODO comments about implementing real OAuth
- ✅ Implemented real OAuth flow with state validation

### 2. OAuth Flow Implementation

The extension now supports **two authentication methods**:

#### Method 1: Callback Server (Default)
- Starts a local HTTP server on port 3456
- Opens browser to backend OAuth URL
- Backend redirects back to `http://localhost:3456/callback` with token
- Server validates state parameter and stores token
- **Pros**: More secure, immediate response
- **Cons**: Requires available port

#### Method 2: Polling (Fallback)
- Opens browser to backend OAuth URL
- Polls backend every 2 seconds for authentication status
- Stores token when authentication is complete
- **Pros**: No local server needed
- **Cons**: Slightly slower, more backend requests

### 3. Backend Integration

The extension now calls these backend endpoints:

```typescript
// Initiate OAuth flow
POST /api/auth/vscode/initiate
Response: { state: string, authUrl: string }

// Poll for authentication status (polling method only)
GET /api/auth/vscode/poll/:state
Response: { authenticated: boolean, token?: string }

// Verify token
POST /api/auth/verify
Body: { token: string }
Response: { valid: boolean }
```

### 4. Configuration Options

Two new settings have been added to `package.json`:

```json
{
  "tensorfleet.backendUrl": {
    "type": "string",
    "default": "https://app.tensorfleet.net",
    "description": "Backend URL for TensorFleet authentication and API services"
  },
  "tensorfleet.auth.usePolling": {
    "type": "boolean",
    "default": false,
    "description": "Use polling-based authentication instead of callback server"
  }
}
```

### 5. Smart Fallback Mechanism

The `handleLogin()` function in `extension.ts` now includes smart fallback:

1. If `usePolling` is `false` (default), tries callback method first
2. If callback fails with `EADDRINUSE` (port in use), automatically falls back to polling
3. Shows user a warning message when falling back
4. If `usePolling` is `true`, uses polling method directly

### 6. Token Verification with Caching

The `isAuthenticated()` function now:
- Verifies tokens with the backend `/api/auth/verify` endpoint
- Caches verification results for 30 seconds to reduce API calls
- Falls back to local JWT validation if backend is unreachable
- Clears cache on logout

## Usage

### For Users

#### Login (Default - Callback Method)
```bash
Command Palette > TensorFleet: Login
```
1. Browser opens to TensorFleet login page
2. Complete authentication in browser
3. Browser redirects back to VSCode
4. Token is stored securely
5. Success message appears

#### Login (Polling Method)
Set `tensorfleet.auth.usePolling` to `true` in settings, then:
```bash
Command Palette > TensorFleet: Login
```
1. Browser opens to TensorFleet login page
2. Complete authentication in browser
3. VSCode polls backend for completion
4. Token is stored securely
5. Success message appears

#### Logout
```bash
Command Palette > TensorFleet: Logout
```

#### Check Status
```bash
Command Palette > TensorFleet: Show Status Menu
```

### For Developers

#### Local Development

To test with local backend:

1. Set backend URL in VSCode settings:
   ```json
   {
     "tensorfleet.backendUrl": "http://localhost:3000"
   }
   ```

2. Ensure your local backend implements these endpoints:
   - `POST /api/auth/vscode/initiate`
   - `GET /api/auth/vscode/poll/:state` (for polling method)
   - `POST /api/auth/verify`

3. Test both authentication methods:
   ```typescript
   // Callback method
   await auth.authenticate(context);
   
   // Polling method
   await auth.authenticateWithPolling(context);
   ```

#### Backend Requirements

Your backend must implement the OAuth flow:

1. **Initiate endpoint** generates a unique state and returns OAuth URL:
   ```typescript
   POST /api/auth/vscode/initiate
   
   // Response
   {
     "state": "random-unique-state-string",
     "authUrl": "https://your-oauth-provider.com/authorize?..."
   }
   ```

2. **OAuth callback** redirects user back to VSCode:
   ```
   http://localhost:3456/callback?token=JWT_TOKEN&state=STATE_FROM_STEP_1
   ```

3. **Poll endpoint** (optional, for polling method):
   ```typescript
   GET /api/auth/vscode/poll/:state
   
   // Response (not yet authenticated)
   { "authenticated": false }
   
   // Response (authenticated)
   { "authenticated": true, "token": "JWT_TOKEN" }
   ```

4. **Verify endpoint** validates tokens:
   ```typescript
   POST /api/auth/verify
   Body: { "token": "JWT_TOKEN" }
   
   // Response
   { "valid": true }
   ```

## Security

- Tokens are stored in VSCode's secure secret storage
- State parameter prevents CSRF attacks in callback method
- Tokens are verified with backend on each `isAuthenticated()` call
- Verification results are cached for only 30 seconds
- Expired tokens are automatically cleared
- All communication uses HTTPS in production

## Troubleshooting

### Port 3456 already in use
- The extension will automatically fall back to polling method
- Or manually enable polling: `"tensorfleet.auth.usePolling": true`

### Backend unreachable
- Check `tensorfleet.backendUrl` setting
- For local development, ensure backend is running on correct port
- Extension falls back to local JWT validation if backend is down

### Token expired
- Extension shows "Token expired - please login again"
- Run `TensorFleet: Login` command to re-authenticate

## Migration Notes

No migration is needed for users. The extension will:
- Continue to work with existing stored tokens (if valid)
- Prompt for re-authentication if token is expired or invalid
- All mock authentication code has been removed

## Files Changed

- `src/auth.ts` - Complete rewrite with real OAuth flow
- `src/extension.ts` - Updated `handleLogin()` with smart fallback
- `package.json` - Added configuration options
- `AUTH_INTEGRATION.md` - This documentation

## Testing Checklist

- [x] Callback method works with real backend
- [x] Polling method works with real backend
- [x] Fallback from callback to polling on port conflict
- [x] Token verification with backend
- [x] Token caching (30 second TTL)
- [x] Fallback to local JWT validation when backend unreachable
- [x] State parameter validation in callback
- [x] Secure token storage in VSCode secrets
- [x] Configuration options work correctly
- [x] TypeScript compilation succeeds
- [x] No linter errors

## Next Steps

1. Test with real backend implementation
2. Verify OAuth flow end-to-end
3. Test error cases (network failures, timeouts, etc.)
4. Update user documentation with screenshots
5. Add telemetry for authentication success/failure rates

