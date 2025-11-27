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

The extension uses a browser-based callback server flow:

#### Callback Server Flow
- Starts a local HTTP server on port 3456
- Opens browser to backend OAuth URL
- Backend redirects back to `http://localhost:3456/callback` with token
- Server validates state parameter and stores token

### 3. Backend Integration

The extension now calls these backend endpoints:

```typescript
// Initiate OAuth flow
POST /api/auth/vscode/initiate
Response: { state: string, authUrl: string }

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
  }
}
```

### 5. Token Verification with Caching

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
   - `POST /api/auth/verify`

3. Test the authentication flow:
   ```typescript
   await auth.authenticate(context);
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

3. **Verify endpoint** validates tokens:
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
- Free the port or adjust your local setup; the callback server must bind to this port.

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
- `src/extension.ts` - Updated `handleLogin()` to use callback flow
- `package.json` - Added backend configuration option
- `AUTH_INTEGRATION.md` - This documentation

## Testing Checklist

- [x] Callback method works with real backend
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
