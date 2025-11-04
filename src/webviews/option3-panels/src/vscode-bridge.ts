/**
 * VS Code Webview API bridge
 * Handles message passing between React components and VS Code extension
 */

// VS Code API type (provided by webview runtime)
type VSCodeAPI = {
  postMessage(message: any): void;
  getState(): any;
  setState(state: any): void;
};

declare function acquireVsCodeApi(): VSCodeAPI;

let vscode: VSCodeAPI | null = null;

try {
  vscode = acquireVsCodeApi();
} catch (e) {
  console.warn('Not running in VS Code webview, using mock API');
  vscode = {
    postMessage: (msg) => console.log('Mock postMessage:', msg),
    getState: () => ({}),
    setState: (state) => console.log('Mock setState:', state)
  };
}

export const vscodeBridge = {
  /**
   * Send message to extension host
   */
  postMessage: (message: any) => {
    vscode?.postMessage(message);
  },

  /**
   * Listen for messages from extension host
   */
  onMessage: (handler: (message: any) => void) => {
    const listener = (event: MessageEvent) => {
      handler(event.data);
    };
    window.addEventListener('message', listener);
    return () => window.removeEventListener('message', listener);
  },

  /**
   * Get persisted state
   */
  getState: () => {
    return vscode?.getState() ?? {};
  },

  /**
   * Persist state
   */
  setState: (state: any) => {
    vscode?.setState(state);
  }
};

