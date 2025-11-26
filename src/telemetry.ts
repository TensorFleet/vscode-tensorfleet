import * as vscode from 'vscode';
import * as Sentry from '@sentry/node';
import { TELEMETRY_CONFIG } from './generated/telemetry-config';

export type TelemetryProperties = Record<string, string>;
export type TelemetryMeasurements = Record<string, number>;

export class TelemetryService implements vscode.Disposable {
  private sentryEnabled = false;
  private unhandledRejectionHandler: (reason: unknown) => void;
  private uncaughtExceptionHandler: (error: Error) => void;

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

    this.unhandledRejectionHandler = (reason: unknown) => {
      this.captureError(reason, { source: 'unhandledRejection' });
    };

    this.uncaughtExceptionHandler = (error: Error) => {
      this.captureError(error, { source: 'uncaughtException' });
    };

    process.on('unhandledRejection', this.unhandledRejectionHandler);
    process.on('uncaughtException', this.uncaughtExceptionHandler);
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
    process.off('unhandledRejection', this.unhandledRejectionHandler);
    process.off('uncaughtException', this.uncaughtExceptionHandler);

    if (this.sentryEnabled) {
      void Sentry.close(2);
      this.sentryEnabled = false;
    }
  }
}
