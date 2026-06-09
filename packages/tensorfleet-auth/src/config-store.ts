const configStore = new Map<string, unknown>();
const globalWindow = globalThis as typeof globalThis & { window?: Record<string, unknown> };

export function setConfig<T = unknown>(key: string, value: T): void {
  configStore.set(key, value);
}

export function getConfig<T = unknown>(key: string): T | undefined {
  // First check the config store
  if (configStore.has(key)) {
    return configStore.get(key) as T | undefined;
  }
  // Fall back to window globals (set by extension)
  const windowObject = globalWindow.window;
  if (windowObject && key in windowObject) {
    return windowObject[key] as T | undefined;
  }
  return undefined;
}

export function clearConfig(): void {
  configStore.clear();
}
