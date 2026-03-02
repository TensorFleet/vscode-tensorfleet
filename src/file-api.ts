import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';

// Logging utility with Tensorfleet FileAPI tag
function log(level: 'info' | 'warn' | 'error', message: string, ...args: any[]): void {
  const timestamp = new Date().toISOString();
  const tag = '[Tensorfleet][FileAPI]';
  const logMessage = `${timestamp} ${tag} ${level.toUpperCase()}: ${message}`;
  
  switch (level) {
    case 'info':
      console.log(logMessage, ...args);
      break;
    case 'warn':
      console.warn(logMessage, ...args);
      break;
    case 'error':
      console.error(logMessage, ...args);
      break;
  }
}

export interface WorkspaceFileServiceOptions {
  maxReadSize?: number;
  maxWriteSize?: number;
  maxSearchResults?: number;
  maxWatchersPerWebview?: number;
  requestTimeout?: number;
}

export interface OpContext {
  webview: vscode.Webview;
  webviewIdentity: string;
  telemetry?: { trackEvent?: (eventName: string, data?: any) => void } | null;
}

export interface FsOpHandler {
  (ctx: OpContext, args: any): Promise<any>;
}

export interface WatchSubscription {
  watchId: string;
  watcher: vscode.FileSystemWatcher;
  uri: vscode.Uri;
  recursive: boolean;
  glob?: string;
}

export class WorkspaceFileService {
  private readonly options: Required<WorkspaceFileServiceOptions>;
  private readonly workspaceRoots: vscode.Uri[];
  private readonly opRegistry: Map<string, FsOpHandler> = new Map();
  private readonly watchers: Map<string, WatchSubscription> = new Map();
  private readonly webviewWatchers: Map<string, Set<string>> = new Map();
  private readonly pendingRequests: Map<string, { resolve: (value: any) => void; reject: (error: any) => void; timeout: NodeJS.Timeout }> = new Map();

  constructor(options: WorkspaceFileServiceOptions = {}) {
    this.options = {
      maxReadSize: 5 * 1024 * 1024, // 5MB
      maxWriteSize: 5 * 1024 * 1024, // 5MB
      maxSearchResults: 200,
      maxWatchersPerWebview: 25,
      requestTimeout: 30000,
      ...options
    };

    this.workspaceRoots = vscode.workspace.workspaceFolders?.map(f => f.uri) || [];
    
    log('info', `WorkspaceFileService initialized with ${this.workspaceRoots.length} workspace roots`);
    
    // Register default operations
    this.registerOperation('readFile', this.readFile.bind(this));
    this.registerOperation('writeFile', this.writeFile.bind(this));
    this.registerOperation('mkdir', this.mkdir.bind(this));
    this.registerOperation('readdir', this.readdir.bind(this));
    this.registerOperation('delete', this.deleteFile.bind(this));
    this.registerOperation('stat', this.stat.bind(this));
    this.registerOperation('rename', this.rename.bind(this));
    this.registerOperation('copy', this.copy.bind(this));
    this.registerOperation('searchText', this.searchText.bind(this));
    this.registerOperation('applyEdits', this.applyEdits.bind(this));
    this.registerOperation('watch', this.watch.bind(this));
    this.registerOperation('unwatch', this.unwatch.bind(this));
    this.registerOperation('listWorkspaceRoots', this.listWorkspaceRoots.bind(this));
    
    log('info', `Registered ${this.opRegistry.size} file operations`);
  }

  private registerOperation(name: string, handler: FsOpHandler): void {
    this.opRegistry.set(name, handler);
  }

  private getWebviewIdentity(webview: vscode.Webview): string {
    return webview.html || 'unknown';
  }

  private validateWorkspaceBound(uri: vscode.Uri): vscode.Uri {
    if (uri.scheme !== 'file') {
      throw new Error('INVALID_REQUEST: Only file:// URIs are supported');
    }

    const normalizedPath = path.normalize(uri.fsPath);
    const workspaceRoot = this.getWorkspaceRootForPath(normalizedPath);
    
    if (!workspaceRoot) {
      throw new Error('PATH_OUTSIDE_WORKSPACE: Path is outside workspace boundaries');
    }

    // Additional check for path traversal
    const relativePath = path.relative(workspaceRoot.fsPath, normalizedPath);
    if (relativePath.startsWith('..') || path.isAbsolute(relativePath)) {
      throw new Error('PATH_OUTSIDE_WORKSPACE: Path traversal detected');
    }

    return uri;
  }

  private getWorkspaceRootForPath(filePath: string): vscode.Uri | null {
    for (const root of this.workspaceRoots) {
      const relative = path.relative(root.fsPath, filePath);
      if (!relative.startsWith('..') && !path.isAbsolute(relative)) {
        return root;
      }
    }
    return null;
  }

  private resolvePath(args: any): vscode.Uri {
    let uri: vscode.Uri;

    if (args.uri) {
      uri = vscode.Uri.parse(args.uri);
    } else if (args.workspaceRelativePath) {
      const workspaceRoot = this.workspaceRoots[0] || vscode.Uri.file(process.cwd());
      uri = vscode.Uri.joinPath(workspaceRoot, args.workspaceRelativePath);
    } else {
      throw new Error('INVALID_REQUEST: Either uri or workspaceRelativePath must be provided');
    }

    return this.validateWorkspaceBound(uri);
  }

  private sendEvent(webview: vscode.Webview, event: any): void {
    try {
      webview.postMessage({
        channel: 'tensorfleet.fs',
        type: 'event',
        protocolVersion: 1,
        ...event
      });
    } catch (error) {
      console.warn('[TensorFleet File API] Failed to send event:', error);
    }
  }

  private createErrorResponse(error: any): any {
    if (error instanceof Error) {
      return {
        code: 'INTERNAL_ERROR',
        message: error.message,
        details: { stack: error.stack }
      };
    }

    if (typeof error === 'string') {
      return {
        code: 'INTERNAL_ERROR',
        message: error
      };
    }

    return {
      code: 'INTERNAL_ERROR',
      message: 'An unknown error occurred'
    };
  }

  // Operation implementations

  private async readFile(ctx: OpContext, args: any): Promise<any> {
    log('info', `Read file request from webview ${ctx.webviewIdentity}: ${args.uri || args.workspaceRelativePath}`);
    
    try {
      const uri = this.resolvePath(args);
      const stat = await vscode.workspace.fs.stat(uri);
      
      if (stat.size > this.options.maxReadSize) {
        log('error', `File too large for webview ${ctx.webviewIdentity}: ${stat.size} bytes`);
        throw new Error('PAYLOAD_TOO_LARGE: File size exceeds maximum allowed');
      }

      const data = await vscode.workspace.fs.readFile(uri);
      log('info', `Successfully read file for webview ${ctx.webviewIdentity}: ${stat.size} bytes`);
      
      return {
        content: Buffer.from(data).toString('utf8'),
        size: stat.size,
        mtime: stat.mtime,
        ctime: stat.ctime,
        isFile: stat.type === vscode.FileType.File,
        isDirectory: stat.type === vscode.FileType.Directory
      };
    } catch (error) {
      log('error', `Read file failed for webview ${ctx.webviewIdentity}: ${error instanceof Error ? error.message : String(error)}`);
      throw error;
    }
  }

  private async writeFile(ctx: OpContext, args: any): Promise<any> {
    log('info', `Write file request from webview ${ctx.webviewIdentity}: ${args.uri || args.workspaceRelativePath}`);
    
    try {
      const uri = this.resolvePath(args);
      const content = args.content;
      
      if (typeof content !== 'string') {
        log('error', `Invalid content type for webview ${ctx.webviewIdentity}: ${typeof content}`);
        throw new Error('INVALID_REQUEST: Content must be a string');
      }

      if (Buffer.byteLength(content, 'utf8') > this.options.maxWriteSize) {
        log('error', `Content too large for webview ${ctx.webviewIdentity}: ${Buffer.byteLength(content, 'utf8')} bytes`);
        throw new Error('PAYLOAD_TOO_LARGE: Content size exceeds maximum allowed');
      }

      const data = Buffer.from(content, 'utf8');
      await vscode.workspace.fs.writeFile(uri, data);
      log('info', `Successfully wrote file for webview ${ctx.webviewIdentity}`);
      
      return { success: true };
    } catch (error) {
      log('error', `Write file failed for webview ${ctx.webviewIdentity}: ${error instanceof Error ? error.message : String(error)}`);
      throw error;
    }
  }

  private async mkdir(ctx: OpContext, args: any): Promise<any> {
    log('info', `Create directory request from webview ${ctx.webviewIdentity}: ${args.uri || args.workspaceRelativePath}`);
    
    try {
      const uri = this.resolvePath(args);
      await vscode.workspace.fs.createDirectory(uri);
      log('info', `Successfully created directory for webview ${ctx.webviewIdentity}`);
      
      return { success: true };
    } catch (error) {
      log('error', `Create directory failed for webview ${ctx.webviewIdentity}: ${error instanceof Error ? error.message : String(error)}`);
      throw error;
    }
  }

  private async readdir(ctx: OpContext, args: any): Promise<any> {
    log('info', `Read directory request from webview ${ctx.webviewIdentity}: ${args.uri || args.workspaceRelativePath}`);
    
    try {
      const uri = this.resolvePath(args);
      const entries = await vscode.workspace.fs.readDirectory(uri);
      
      const result = entries.map(([name, type]) => ({
        name,
        type: type === vscode.FileType.File ? 'file' : 
              type === vscode.FileType.Directory ? 'directory' : 'other'
      }));
      
      log('info', `Successfully read directory for webview ${ctx.webviewIdentity}: ${result.length} entries`);
      return result;
    } catch (error) {
      log('error', `Read directory failed for webview ${ctx.webviewIdentity}: ${error instanceof Error ? error.message : String(error)}`);
      throw error;
    }
  }

  private async deleteFile(ctx: OpContext, args: any): Promise<any> {
    log('info', `Delete file request from webview ${ctx.webviewIdentity}: ${args.uri || args.workspaceRelativePath}`);
    
    try {
      const uri = this.resolvePath(args);
      await vscode.workspace.fs.delete(uri, { recursive: args.recursive || false });
      log('info', `Successfully deleted file/directory for webview ${ctx.webviewIdentity}`);
      
      return { success: true };
    } catch (error) {
      log('error', `Delete file failed for webview ${ctx.webviewIdentity}: ${error instanceof Error ? error.message : String(error)}`);
      throw error;
    }
  }

  private async stat(ctx: OpContext, args: any): Promise<any> {
    log('info', `Stat request from webview ${ctx.webviewIdentity}: ${args.uri || args.workspaceRelativePath}`);
    
    try {
      const uri = this.resolvePath(args);
      const stat = await vscode.workspace.fs.stat(uri);
      
      const result = {
        type: stat.type === vscode.FileType.File ? 'file' : 
              stat.type === vscode.FileType.Directory ? 'directory' : 'other',
        ctime: stat.ctime,
        mtime: stat.mtime,
        size: stat.size,
        permissions: stat.permissions
      };
      
      log('info', `Successfully got stats for webview ${ctx.webviewIdentity}: ${result.type}, ${result.size} bytes`);
      return result;
    } catch (error) {
      log('error', `Stat failed for webview ${ctx.webviewIdentity}: ${error instanceof Error ? error.message : String(error)}`);
      throw error;
    }
  }

  private async rename(ctx: OpContext, args: any): Promise<any> {
    log('info', `Rename request from webview ${ctx.webviewIdentity}: ${args.uri || args.workspaceRelativePath} -> ${args.newUri || args.workspaceRelativePath}`);
    
    try {
      const oldUri = this.resolvePath(args);
      const newUri = this.resolvePath({ uri: args.newUri || args.workspaceRelativePath });
      await vscode.workspace.fs.rename(oldUri, newUri, { overwrite: args.overwrite || false });
      log('info', `Successfully renamed file for webview ${ctx.webviewIdentity}`);
      
      return { success: true };
    } catch (error) {
      log('error', `Rename failed for webview ${ctx.webviewIdentity}: ${error instanceof Error ? error.message : String(error)}`);
      throw error;
    }
  }

  private async copy(ctx: OpContext, args: any): Promise<any> {
    log('info', `Copy request from webview ${ctx.webviewIdentity}: ${args.uri || args.workspaceRelativePath} -> ${args.targetUri || args.workspaceRelativePath}`);
    
    try {
      const sourceUri = this.resolvePath(args);
      const targetUri = this.resolvePath({ uri: args.targetUri || args.workspaceRelativePath });
      await vscode.workspace.fs.copy(sourceUri, targetUri, { overwrite: args.overwrite || false });
      log('info', `Successfully copied file for webview ${ctx.webviewIdentity}`);
      
      return { success: true };
    } catch (error) {
      log('error', `Copy failed for webview ${ctx.webviewIdentity}: ${error instanceof Error ? error.message : String(error)}`);
      throw error;
    }
  }

  private async searchText(ctx: OpContext, args: any): Promise<any> {
    log('info', `Search request from webview ${ctx.webviewIdentity}: query="${args.query}"`);
    
    const { query, includes, excludes, maxResults = this.options.maxSearchResults } = args;
    
    if (!query) {
      log('error', `Invalid search query from webview ${ctx.webviewIdentity}`);
      throw new Error('INVALID_REQUEST: Query is required');
    }

    try {
      const results: any[] = [];
      const foundFiles = new Set<string>();

      // Use VS Code's search API with proper options
      const searchOptions: any = {
        include: includes,
        exclude: excludes,
        maxResults,
        previewOptions: {
          matchLines: 1,
          charsPerLine: 200
        }
      };

      // Note: findTextInFiles might not be available in all VS Code API versions
      // This is a placeholder implementation that will need to be adapted
      log('warn', `Search functionality requires VS Code API adaptation for webview ${ctx.webviewIdentity}`);
      
      return {
        results: [],
        truncated: false
      };
    } catch (error) {
      log('error', `Search failed for webview ${ctx.webviewIdentity}: ${error instanceof Error ? error.message : String(error)}`);
      throw error;
    }
  }

  private async applyEdits(ctx: OpContext, args: any): Promise<any> {
    log('info', `Apply edits request from webview ${ctx.webviewIdentity}: ${args.uri || args.workspaceRelativePath}`);
    
    try {
      const uri = this.resolvePath(args);
      const edits = args.edits;

      if (!Array.isArray(edits)) {
        log('error', `Invalid edits format for webview ${ctx.webviewIdentity}`);
        throw new Error('INVALID_REQUEST: Edits must be an array');
      }

      const document = await vscode.workspace.openTextDocument(uri);
      const workspaceEdit = new vscode.WorkspaceEdit();

      for (const edit of edits) {
        const range = new vscode.Range(
          edit.range.start.line,
          edit.range.start.character,
          edit.range.end.line,
          edit.range.end.character
        );
        workspaceEdit.replace(uri, range, edit.text);
      }

      const success = await vscode.workspace.applyEdit(workspaceEdit);
      log('info', `Successfully applied ${edits.length} edits for webview ${ctx.webviewIdentity}`);
      
      return { success };
    } catch (error) {
      log('error', `Apply edits failed for webview ${ctx.webviewIdentity}: ${error instanceof Error ? error.message : String(error)}`);
      throw error;
    }
  }

  private async watch(ctx: OpContext, args: any): Promise<any> {
    const webviewIdentity = this.getWebviewIdentity(ctx.webview);
    const webviewWatchers = this.webviewWatchers.get(webviewIdentity) || new Set();
    
    if (webviewWatchers.size >= this.options.maxWatchersPerWebview) {
      log('error', `Watcher limit exceeded for webview ${webviewIdentity}: ${webviewWatchers.size} watchers`);
      throw new Error('RATE_LIMITED: Maximum number of watchers per webview exceeded');
    }

    log('info', `File watch request from webview ${webviewIdentity}: ${args.uri || args.workspaceRelativePath}`);
    
    try {
      const uri = this.resolvePath(args);
      const recursive = args.recursive || false;
      const glob = args.glob;

      // Generate unique watch ID
      const watchId = `watch_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;

      let watcher: vscode.FileSystemWatcher;

      if (glob) {
        // Use glob pattern
        watcher = vscode.workspace.createFileSystemWatcher(
          new vscode.RelativePattern(uri, glob)
        );
      } else if (recursive) {
        // Watch directory recursively
        watcher = vscode.workspace.createFileSystemWatcher(
          new vscode.RelativePattern(uri, '**/*')
        );
      } else {
        // Watch specific file or directory
        watcher = vscode.workspace.createFileSystemWatcher(
          new vscode.RelativePattern(uri, '*')
        );
      }

      // Set up event handlers
      watcher.onDidChange((uri) => {
        this.sendEvent(ctx.webview, {
          event: 'changed',
          watchId,
          uri: uri.toString(),
          timestamp: Date.now()
        });
      });

      watcher.onDidCreate((uri) => {
        this.sendEvent(ctx.webview, {
          event: 'created',
          watchId,
          uri: uri.toString(),
          timestamp: Date.now()
        });
      });

      watcher.onDidDelete((uri) => {
        this.sendEvent(ctx.webview, {
          event: 'deleted',
          watchId,
          uri: uri.toString(),
          timestamp: Date.now()
        });
      });

      const subscription: WatchSubscription = {
        watchId,
        watcher,
        uri,
        recursive,
        glob
      };

      this.watchers.set(watchId, subscription);
      webviewWatchers.add(watchId);
      this.webviewWatchers.set(webviewIdentity, webviewWatchers);

      log('info', `Successfully created file watcher ${watchId} for webview ${webviewIdentity}`);
      return { watchId };
    } catch (error) {
      log('error', `File watch failed for webview ${webviewIdentity}: ${error instanceof Error ? error.message : String(error)}`);
      throw error;
    }
  }

  private async unwatch(ctx: OpContext, args: any): Promise<any> {
    log('info', `File unwatch request from webview ${ctx.webviewIdentity}: ${args.watchId}`);
    
    try {
      const { watchId } = args;
      const subscription = this.watchers.get(watchId);

      if (!subscription) {
        log('error', `Watch ID not found for webview ${ctx.webviewIdentity}: ${watchId}`);
        throw new Error('NOT_FOUND: Watch subscription not found');
      }

      // Clean up watcher
      subscription.watcher.dispose();
      this.watchers.delete(watchId);

      // Remove from webview's watchers
      const webviewIdentity = this.getWebviewIdentity(ctx.webview);
      const webviewWatchers = this.webviewWatchers.get(webviewIdentity);
      if (webviewWatchers) {
        webviewWatchers.delete(watchId);
        if (webviewWatchers.size === 0) {
          this.webviewWatchers.delete(webviewIdentity);
        }
      }

      log('info', `Successfully removed file watcher ${watchId} for webview ${webviewIdentity}`);
      return { success: true };
    } catch (error) {
      log('error', `File unwatch failed for webview ${ctx.webviewIdentity}: ${error instanceof Error ? error.message : String(error)}`);
      throw error;
    }
  }

  private async listWorkspaceRoots(ctx: OpContext, args: any): Promise<any> {
    log('info', `List workspace roots request from webview ${ctx.webviewIdentity}`);
    
    try {
      const result = {
        roots: this.workspaceRoots.map(root => ({
          uri: root.toString(),
          name: path.basename(root.fsPath)
        }))
      };
      
      log('info', `Successfully listed ${result.roots.length} workspace roots for webview ${ctx.webviewIdentity}`);
      return result;
    } catch (error) {
      log('error', `List workspace roots failed for webview ${ctx.webviewIdentity}: ${error instanceof Error ? error.message : String(error)}`);
      throw error;
    }
  }

  // Public API methods

  public handleMessage(message: any, ctx: OpContext): void {
    log('info', `Message received from webview ${ctx.webviewIdentity}: ${message.type} - ${message.op || 'no-op'}`);
    
    if (!message || message.channel !== 'tensorfleet.fs') {
      return;
    }

    if (message.protocolVersion !== 1) {
      log('warn', `Unsupported protocol version from webview ${ctx.webviewIdentity}: ${message.protocolVersion}`);
      ctx.webview.postMessage({
        channel: 'tensorfleet.fs',
        type: 'response',
        id: message.id,
        ok: false,
        error: {
          code: 'UNSUPPORTED_OP',
          message: 'Unsupported protocol version'
        }
      });
      return;
    }

    if (message.type === 'request') {
      this.handleRequest(message, ctx);
    }
  }

  private async handleRequest(message: any, ctx: OpContext): Promise<void> {
    const { id, op, args } = message;

    if (!id || !op) {
      log('error', `Invalid request from webview ${ctx.webviewIdentity}: missing id or op`);
      ctx.webview.postMessage({
        channel: 'tensorfleet.fs',
        type: 'response',
        id,
        ok: false,
        error: {
          code: 'INVALID_REQUEST',
          message: 'Request must have id and op'
        }
      });
      return;
    }

    log('info', `Processing request ${id} from webview ${ctx.webviewIdentity}: ${op}`);
    
    const handler = this.opRegistry.get(op);
    if (!handler) {
      log('error', `Unsupported operation from webview ${ctx.webviewIdentity}: ${op}`);
      ctx.webview.postMessage({
        channel: 'tensorfleet.fs',
        type: 'response',
        id,
        ok: false,
        error: {
          code: 'UNSUPPORTED_OP',
          message: `Operation '${op}' is not supported`
        }
      });
      return;
    }

    // Set up timeout
    const timeout = setTimeout(() => {
      this.pendingRequests.delete(id);
      log('warn', `Request ${id} from webview ${ctx.webviewIdentity} timed out`);
      ctx.webview.postMessage({
        channel: 'tensorfleet.fs',
        type: 'response',
        id,
        ok: false,
        error: {
          code: 'TIMEOUT',
          message: 'Request timed out'
        }
      });
    }, this.options.requestTimeout);

    this.pendingRequests.set(id, {
      resolve: (result) => {
        clearTimeout(timeout);
        this.pendingRequests.delete(id);
        log('info', `Request ${id} from webview ${ctx.webviewIdentity} completed successfully`);
        ctx.webview.postMessage({
          channel: 'tensorfleet.fs',
          type: 'response',
          id,
          ok: true,
          result
        });
      },
      reject: (error) => {
        clearTimeout(timeout);
        this.pendingRequests.delete(id);
        const errorResponse = this.createErrorResponse(error);
        log('error', `Request ${id} from webview ${ctx.webviewIdentity} failed: ${errorResponse.message}`);
        ctx.webview.postMessage({
          channel: 'tensorfleet.fs',
          type: 'response',
          id,
          ok: false,
          error: errorResponse
        });
      },
      timeout
    });

    try {
      const result = await handler(ctx, args || {});
      this.pendingRequests.get(id)?.resolve(result);
    } catch (error) {
      this.pendingRequests.get(id)?.reject(error);
    }
  }

  public dispose(): void {
    log('info', `WorkspaceFileService disposing: cleaning up ${this.watchers.size} watchers and ${this.pendingRequests.size} pending requests`);
    
    // Clean up all watchers
    for (const subscription of this.watchers.values()) {
      subscription.watcher.dispose();
    }
    this.watchers.clear();
    this.webviewWatchers.clear();
    
    // Clear pending requests
    for (const { timeout } of this.pendingRequests.values()) {
      clearTimeout(timeout);
    }
    this.pendingRequests.clear();
    
    log('info', 'WorkspaceFileService disposed successfully');
  }
}