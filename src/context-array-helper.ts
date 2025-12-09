import * as vscode from 'vscode';

// Helpers assume JSON-serializable values in array
export function getLastItem<T>(
    context: vscode.ExtensionContext,
    key: string
): T | undefined {
    const arr = context.globalState.get<T[]>(key);
    return arr?.length ? arr[arr.length - 1] : undefined;
}

export async function appendItem<T>(
    context: vscode.ExtensionContext,
    key: string,
    item: T
): Promise<void> {
    const arr = context.globalState.get<T[]>(key) ?? [];
    arr.push(item);
    await context.globalState.update(key, arr);
}

export async function popLastItem<T>(
    context: vscode.ExtensionContext,
    key: string
): Promise<T | undefined> {
    const arr = context.globalState.get<T[]>(key) ?? [];

    if (!arr.length) {
        return undefined;
    }

    const popped = arr.pop();
    await context.globalState.update(key, arr);
    return popped;
}
