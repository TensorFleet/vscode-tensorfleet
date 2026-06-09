import { extractUserProfile, isTokenExpired, isValidJwtShape } from "./oauth-core";
import type { UserProfile } from "./types.js";

export const TENSORFLEET_AUTH_GLOBAL_KEY = Symbol.for("tensorfleet.auth");

export interface TensorfleetGlobalAuthInfo {
  token: string;
  userProfile: UserProfile | null;
  isValidJwtShape: boolean;
  isExpired: boolean;
  source: "oauth" | "process-env";
  updatedAt: string;
}

type TensorfleetGlobal = typeof globalThis & {
  [TENSORFLEET_AUTH_GLOBAL_KEY]?: TensorfleetGlobalAuthInfo;
};

export function storeAuthTokenOnGlobal(token: string, source: TensorfleetGlobalAuthInfo["source"] = "oauth"): TensorfleetGlobalAuthInfo {
  const authInfo = createAuthInfo(token, source);
  setGlobalAuthInfo(authInfo);
  return authInfo;
}

export function getGlobalAuthInfo(): TensorfleetGlobalAuthInfo | undefined {
  return (globalThis as TensorfleetGlobal)[TENSORFLEET_AUTH_GLOBAL_KEY];
}

export function clearGlobalAuthInfo(): void {
  delete (globalThis as TensorfleetGlobal)[TENSORFLEET_AUTH_GLOBAL_KEY];
}

export async function storeProjectAuthInfoOnGlobal(projectPath: string): Promise<void> {
  void projectPath;
  const authInfo = resolveProcessAuthInfo();
  if (!authInfo) {
    return;
  }

  setGlobalAuthInfo(authInfo);
}

function resolveProcessAuthInfo(): TensorfleetGlobalAuthInfo | undefined {
  const token = pickString(process.env.TENSORFLEET_JWT);
  return token ? createAuthInfo(token, "process-env") : undefined;
}

function createAuthInfo(token: string, source: TensorfleetGlobalAuthInfo["source"]): TensorfleetGlobalAuthInfo {
  const validJwtShape = isValidJwtShape(token);

  return {
    token,
    userProfile: validJwtShape ? extractUserProfile(token) : null,
    isValidJwtShape: validJwtShape,
    isExpired: validJwtShape ? isTokenExpired(token) : true,
    source,
    updatedAt: new Date().toISOString(),
  };
}

function setGlobalAuthInfo(authInfo: TensorfleetGlobalAuthInfo): void {
  const target = globalThis as TensorfleetGlobal;

  if (!Object.prototype.hasOwnProperty.call(target, TENSORFLEET_AUTH_GLOBAL_KEY)) {
    Object.defineProperty(target, TENSORFLEET_AUTH_GLOBAL_KEY, {
      configurable: true,
      enumerable: false,
      writable: true,
      value: authInfo,
    });
    return;
  }

  target[TENSORFLEET_AUTH_GLOBAL_KEY] = authInfo;
}

function pickString(value: unknown): string | undefined {
  return typeof value === "string" && value.length > 0 ? value : undefined;
}
