import * as vscode from 'vscode';
import * as Sentry from '@sentry/node';
import { TELEMETRY_CONFIG } from './generated/telemetry-config';

export type TelemetryProperties = Record<string, string>;
export type TelemetryMeasurements = Record<string, number>;

export class TelemetryService implements vscode.Disposable {
  private sentryEnabled = false;
  private unhandledRejectionHandler: ((reason: unknown) => void) | null = null;

  constructor(context: vscode.ExtensionContext) {
    const pkg = context.extension.packageJSON as { name?: string; version?: string };
    const extensionVersion = pkg.version ?? '0.0.0';

    const sentryDsn = (TELEMETRY_CONFIG.sentryDsn ?? '').trim();
    if (sentryDsn) {
      Sentry.init({
        dsn: sentryDsn,
        release: extensionVersion,
        environment: vscode.env.appHost ?? TELEMETRY_CONFIG.sentryEnvironment ?? 'development', // desktop, cursor, windsurf, etc.
        tracesSampleRate: 0.1,
        // IMPORTANT: remove integrations that access navigator
        integrations: (integrations) =>
          integrations.filter((i) => i.name !== "ProcessSession"),

        // Also remove SessionFlusher if present (some builds include it)
        beforeSend: (event) => {
          if (event.logger === "sentry.sessions") return null;
          return event;
        },
      });
      this.sentryEnabled = true;
      Sentry.metrics.count('app_started', 1);
    }

    // Note: We do NOT register uncaughtException handler because:
    // 1. It affects the entire VS Code process, not just this extension
    // 2. VS Code has its own error handling mechanisms
    // 3. Calling process.exit(1) would kill the entire VS Code process
    // Instead, we rely on explicit error handling in our code paths.

    // For unhandledRejection, we log but don't interfere with VS Code's handling
    // This is less intrusive but still allows us to track errors
    this.unhandledRejectionHandler = (reason: unknown) => {
      this.captureError(reason, { source: 'unhandledRejection' });
      // Don't re-throw - let VS Code handle it
    };

    if (this.unhandledRejectionHandler) {
      process.on('unhandledRejection', this.unhandledRejectionHandler);
    }
  }

  trackEvent(eventName: string, properties?: TelemetryProperties, measurements?: TelemetryMeasurements) {
    if (this.sentryEnabled) {
      Sentry.addBreadcrumb({
        category: 'event',
        message: eventName,
        data: properties,
        level: 'info'
      });
    }
  }

  trackError(eventName: string, properties?: TelemetryProperties) {
    if (this.sentryEnabled) {
      Sentry.addBreadcrumb({
        category: 'error',
        message: eventName,
        data: properties,
        level: 'error'
      });
    }
  }

  captureError(error: unknown, properties?: TelemetryProperties) {
    const errorDetails = error instanceof Error ? error : new Error(String(error));
    this.trackError('exception', {
      message: errorDetails.message,
      name: errorDetails.name,
      ...properties
    });

    if (this.sentryEnabled) {
      Sentry.captureException(errorDetails, {
        tags: properties
      });
    } else {
      console.error('[TensorFleet] Untracked exception', errorDetails, properties);
    }
  }

  async flush(timeoutMs = 2000) {
    if (this.sentryEnabled) {
      await Sentry.close(timeoutMs / 1000);
    }
  }

  dispose() {
    if (this.unhandledRejectionHandler) {
      process.off('unhandledRejection', this.unhandledRejectionHandler);
      this.unhandledRejectionHandler = null;
    }

    if (this.sentryEnabled) {
      void Sentry.close(2);
      this.sentryEnabled = false;
    }
  }
}
