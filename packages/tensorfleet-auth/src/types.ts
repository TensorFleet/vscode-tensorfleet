/**
 * Type definitions for TensorFleet Auth module
 */

/**
 * User profile information extracted from JWT
 */
export interface UserProfile {
  name?: string;
  email?: string;
  picture?: string;
}

/**
 * Response from initiating OAuth flow
 */
export interface InitiateAuthResponse {
  state: string;
  authUrl: string;
}

/**
 * Response from verifying token
 */
export interface VerifyTokenResponse {
  valid: boolean;
}

/**
 * Configuration for OAuth flow
 */
export interface OAuthConfig {
  /** Backend URL for OAuth API calls */
  backendUrl: string;
}

/**
 * Callbacks provided by the platform for OAuth flow
 */
export interface OAuthCallbacks {
  /** Called when browser needs to be opened for authentication */
  openBrowser: (url: string) => Promise<void>;
  
  /** Called when token is successfully received from callback */
  onTokenReceived: (token: string) => Promise<void>;
  
  /** Called when an error occurs during OAuth flow */
  onError: (error: Error) => void;
  
  /** Called when login is cancelled by user */
  onCancelled?: () => void;
}

/**
 * Active login state for tracking in-progress authentication
 */
export interface ActiveLoginState {
  /** Function to reject the login promise */
  reject: (error: Error) => void;
  
  /** Timeout ID for login timeout */
  timeoutId: NodeJS.Timeout;
  
  /** Function to close the callback server */
  closeServer: () => void;
}

/**
 * Options for creating a callback server
 */
export interface CallbackServerOptions {
  /** Expected OAuth state parameter for validation */
  expectedState: string;
  
  /** Callbacks to invoke during the flow */
  callbacks: OAuthCallbacks;
  
  /** Port to listen on (0 for ephemeral) */
  port?: number;
  
  /** Host to bind to (default: 127.0.0.1) */
  host?: string;
  
  /** Timeout in milliseconds (default: 5 minutes) */
  timeout?: number;
}

/**
 * Result of starting a callback server
 */
export interface CallbackServerResult {
  /** The actual port the server is listening on */
  port: number;
  
  /** The callback base URL */
  callbackBaseUrl: string;
  
  /** Function to cancel the login */
  cancel: () => void;
  
  /** Promise that resolves when token is received */
  tokenPromise: Promise<string>;
}