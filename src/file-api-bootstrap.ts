import * as vscode from 'vscode';

/**
 * Bootstrap script that gets injected into every TensorFleet webview
 * to provide the generic workspace file API.
 */
export function getFsBootstrapScript(): string {
  console.log('[Tensorfleet][FileAPI] Generating bootstrap script for webview');
  
  return `
(function() {
  'use strict';

  console.log('[Tensorfleet][FileAPI] Initializing file API client in webview');

  // Check if already injected
  if (window.tensorfleet && window.tensorfleet.vscode && window.tensorfleet.vscode.fs) {
    console.log('[Tensorfleet][FileAPI] File API already injected, skipping');
    return;
  }

  // Initialize global namespace
  window.tensorfleet = window.tensorfleet || {};
  window.tensorfleet.vscode = window.tensorfleet.vscode || {};

  const vscode = acquireVsCodeApi();
  const pendingRequests = new Map();
  const eventListeners = new Map();
  let nextRequestId = 1;

  // Default timeout for requests
  const DEFAULT_TIMEOUT = 30000;

  // Create error types
  function createError(code, message, details) {
    const error = new Error(message);
    error.code = code;
    error.details = details;
    return error;
  }

  // RPC client implementation
  function request(op, args = {}) {
    return new Promise((resolve, reject) => {
      const id = 'req_' + (nextRequestId++);
      const timeout = setTimeout(() => {
        pendingRequests.delete(id);
        reject(createError('TIMEOUT', 'Request timed out'));
      }, DEFAULT_TIMEOUT);

      pendingRequests.set(id, { resolve, reject, timeout });

      const message = {
        channel: 'tensorfleet.fs',
        type: 'request',
        id,
        op,
        args,
        protocolVersion: 1
      };

      vscode.postMessage(message);
    });
  }

  // Handle incoming messages
  window.addEventListener('message', (event) => {
    const message = event.data;

    if (!message || message.channel !== 'tensorfleet.fs') {
      return;
    }

    if (message.type === 'response') {
      const pending = pendingRequests.get(message.id);
      if (!pending) {
        return;
      }

      clearTimeout(pending.timeout);
      pendingRequests.delete(message.id);

      if (message.ok) {
        pending.resolve(message.result);
      } else {
        const error = createError(
          message.error?.code || 'INTERNAL_ERROR',
          message.error?.message || 'Unknown error',
          message.error?.details
        );
        pending.reject(error);
      }
    } else if (message.type === 'event') {
      const listeners = eventListeners.get(message.event) || [];
      listeners.forEach(listener => {
        try {
          listener(message);
        } catch (error) {
          console.warn('[TensorFleet File API] Event listener error:', error);
        }
      });
    }
  });

  // Convenience wrappers for common operations
  const fs = {
    // Core file operations
    readFile: (args) => request('readFile', args),
    writeFile: (args) => request('writeFile', args),
    mkdir: (args) => request('mkdir', args),
    readdir: (args) => request('readdir', args),
    delete: (args) => request('delete', args),
    stat: (args) => request('stat', args),
    rename: (args) => request('rename', args),
    copy: (args) => request('copy', args),

    // Search operations
    searchText: (args) => request('searchText', args),

    // Edit operations
    applyEdits: (args) => request('applyEdits', args),

    // Watch operations
    watch: (args) => request('watch', args),
    unwatch: (args) => request('unwatch', args),

    // Utility operations
    listWorkspaceRoots: () => request('listWorkspaceRoots'),

    // Event handling
    on: (eventName, listener) => {
      if (typeof listener !== 'function') {
        throw createError('INVALID_REQUEST', 'Listener must be a function');
      }

      const listeners = eventListeners.get(eventName) || [];
      listeners.push(listener);
      eventListeners.set(eventName, listeners);

      return () => {
        const index = listeners.indexOf(listener);
        if (index > -1) {
          listeners.splice(index, 1);
        }
      };
    },

    // Generic request method for extensibility
    request: request
  };

  // Expose the API
  window.tensorfleet.vscode.fs = fs;

  // Make it available globally for debugging
  if (typeof window !== 'undefined') {
    window.__tensorfleetFs = fs;
  }
})();
`;
}

/**
 * Inject the file API bootstrap script into HTML content
 */
export function injectFsApi(html: string, webview: vscode.Webview): string {
  console.log('[Tensorfleet][FileAPI] Injecting file API into webview HTML');
  
  const bootstrapScript = getFsBootstrapScript();
  
  // Check if bootstrap is already injected
  if (html.includes('window.tensorfleet.vscode.fs')) {
    console.log('[Tensorfleet][FileAPI] File API already present in HTML, skipping injection');
    return html;
  }

  // Inject before </body> if it exists, otherwise at the end
  if (html.includes('</body>')) {
    console.log('[Tensorfleet][FileAPI] Injecting script before </body> tag');
    return html.replace('</body>', `<script>${bootstrapScript}</script></body>`);
  } else {
    console.log('[Tensorfleet][FileAPI] Appending script to end of HTML');
    return html + `<script>${bootstrapScript}</script>`;
  }
}

/**
 * Enhanced inject function that also handles CSP updates
 */
export function injectFsApiWithCSP(html: string, webview: vscode.Webview): string {
  console.log('[Tensorfleet][FileAPI] Injecting file API with CSP handling');
  
  const bootstrapScript = getFsBootstrapScript();
  
  // Check if bootstrap is already injected
  if (html.includes('window.tensorfleet.vscode.fs')) {
    console.log('[Tensorfleet][FileAPI] File API already present in HTML, skipping injection');
    return html;
  }

  // Update CSP if present
  let updatedHtml = html;
  
  // Handle CSP meta tag
  const cspPattern = /<meta[^>]+Content-Security-Policy[^>]*>/i;
  const cspMatch = updatedHtml.match(cspPattern);
  
  if (cspMatch) {
    console.log('[Tensorfleet][FileAPI] Found CSP meta tag, updating for unsafe-inline');
    const cspTag = cspMatch[0];
    const cspContent = cspTag.match(/content="([^"]+)"/i);
    
    if (cspContent) {
      let cspValue = cspContent[1];
      
      // Add 'unsafe-inline' for script-src if not present
      if (!cspValue.includes("script-src") || !cspValue.includes("'unsafe-inline'")) {
        if (cspValue.includes("script-src")) {
          cspValue = cspValue.replace(/script-src([^;]*)/, "script-src$1 'unsafe-inline'");
        } else {
          cspValue += " script-src 'unsafe-inline'";
        }
      }
      
      const newCspTag = cspTag.replace(/content="[^"]+"/i, `content="${cspValue}"`);
      updatedHtml = updatedHtml.replace(cspPattern, newCspTag);
      console.log('[Tensorfleet][FileAPI] Updated CSP meta tag');
    }
  }

  // Inject script before </body> or at end
  if (updatedHtml.includes('</body>')) {
    console.log('[Tensorfleet][FileAPI] Injecting script before </body> tag with CSP');
    updatedHtml = updatedHtml.replace('</body>', `<script>${bootstrapScript}</script></body>`);
  } else {
    console.log('[Tensorfleet][FileAPI] Appending script to end of HTML with CSP');
    updatedHtml = updatedHtml + `<script>${bootstrapScript}</script>`;
  }

  return updatedHtml;
}
