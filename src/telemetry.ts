import * as vscode from 'vscode';
import { TelemetryReporter } from '@vscode/extension-telemetry';
import * as Sentry from '@sentry/node';
import { TELEMETRY_CONFIG } from './generated/telemetry-config';

export type TelemetryProperties = Record<string, string>;
export type TelemetryMeasurements = Record<string, number>;

export class TelemetryService implements vscode.Disposable {
  private reporter?: TelemetryReporter;
  private sentryEnabled = false;
  private unhandledRejectionHandler: (reason: unknown) => void;
  private uncaughtExceptionHandler: (error: Error) => void;

  constructor(context: vscode.ExtensionContext) {
    const pkg = context.extension.packageJSON as { name?: string; version?: string };
    const extensionId = pkg.name ?? 'tensorfleet-drone';
    const extensionVersion = pkg.version ?? '0.0.0';

    const azureMonitorKey = (TELEMETRY_CONFIG.azureMonitorKey ?? '').trim();
    if (azureMonitorKey) {
      this.reporter = new TelemetryReporter(azureMonitorKey, undefined, {
        additionalCommonProperties: {
          extensionId,
          extensionVersion
        }
      });
      context.subscriptions.push(this.reporter);
    }

    const sentryDsn = (TELEMETRY_CONFIG.sentryDsn ?? '').trim();
    if (sentryDsn) {
      Sentry.init({
        dsn: sentryDsn,
        release: extensionVersion,
        environment: vscode.env.appHost ?? TELEMETRY_CONFIG.sentryEnvironment ?? 'development', // desktop, cursor, windsurf, etc.
        tracesSampleRate: 0.1
      });
      this.sentryEnabled = true;
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
    if (this.reporter) {
      this.reporter.sendTelemetryEvent(eventName, properties, measurements);
    }

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
    if (this.reporter) {
      this.reporter.sendTelemetryErrorEvent(eventName, properties);
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

    if (this.reporter) {
      void this.reporter.dispose();
      this.reporter = undefined;
    }

    if (this.sentryEnabled) {
      void Sentry.close(2);
      this.sentryEnabled = false;
    }
  }
}
