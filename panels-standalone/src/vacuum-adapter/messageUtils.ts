function snakeToCamel(value: string): string {
  return value.replace(/_([a-z])/g, (_, letter: string) => letter.toUpperCase());
}

export function getRecordEntry(record: Record<string, unknown>, key: string): unknown {
  if (key in record) {
    return record[key];
  }
  const camelKey = snakeToCamel(key);
  if (camelKey in record) {
    return record[camelKey];
  }
  return null;
}

export function normalizeRosMessage(message: unknown): Record<string, unknown> | null {
  if (!message || typeof message !== "object") {
    return null;
  }
  const record = message as Record<string, unknown>;
  if (record.msg && typeof record.msg === "object") {
    return record.msg as Record<string, unknown>;
  }
  return record;
}

export function formatDuration(value: unknown): string {
  if (typeof value === "number" && Number.isFinite(value)) {
    if (value < 60) {
      return `${value.toFixed(1)}s`;
    }
    const minutes = Math.floor(value / 60);
    const seconds = value % 60;
    return `${minutes}m ${seconds.toFixed(0)}s`;
  }
  if (!value || typeof value !== "object") {
    return "n/a";
  }
  const record = value as Record<string, unknown>;
  const sec = Number(getRecordEntry(record, "sec") ?? getRecordEntry(record, "secs") ?? 0);
  const nanosec = Number(getRecordEntry(record, "nanosec") ?? getRecordEntry(record, "nsecs") ?? 0);
  if (!Number.isFinite(sec) || !Number.isFinite(nanosec)) {
    return "n/a";
  }
  const totalSeconds = sec + nanosec / 1_000_000_000;
  if (totalSeconds < 60) {
    return `${totalSeconds.toFixed(1)}s`;
  }
  const minutes = Math.floor(totalSeconds / 60);
  const seconds = totalSeconds % 60;
  return `${minutes}m ${seconds.toFixed(0)}s`;
}
