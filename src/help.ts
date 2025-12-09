import * as vscode from 'vscode';

export type OnboardingProgress = {
  lastCompletedStep: 'none' | 'account' | 'project' | 'panels' | 'end';
};

const ONBOARDING_KEY = 'tensorfleet.onboarding.lastCompletedStep';
const ONBOARDING_DEFAULT: OnboardingProgress = { lastCompletedStep: 'account' };

/**
 * Read from VS Code extension storage (no filesystem)
 */
export function loadOnboardingProgress(
  ctx: vscode.ExtensionContext
): OnboardingProgress {
  return ctx.globalState.get<OnboardingProgress>(ONBOARDING_KEY, ONBOARDING_DEFAULT);
}

/**
 * Persist to VS Code extension storage (no filesystem)
 */
export function saveOnboardingProgress(
  ctx: vscode.ExtensionContext,
  progress: OnboardingProgress
): Thenable<void> {
  return ctx.globalState.update(ONBOARDING_KEY, progress);
}


export async function ensureOnboardingProgressInitialized(
  ctx: vscode.ExtensionContext
): Promise<OnboardingProgress> {
  const current = ctx.globalState.get<OnboardingProgress | undefined>(ONBOARDING_KEY);

  if (!current || !current.lastCompletedStep) {
    await saveOnboardingProgress(ctx, ONBOARDING_DEFAULT);
    return ONBOARDING_DEFAULT;
  }

  return current;
}

export function resetOnboardingProgress(ctx: vscode.ExtensionContext): Thenable<void> {
  return ctx.globalState.update(ONBOARDING_KEY, {
    lastCompletedStep: 'none'
  });
}