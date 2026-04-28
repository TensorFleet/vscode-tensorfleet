import { builtinModules } from 'node:module';
import path from 'node:path';
import { defineConfig } from 'vite';

// Build mode: 'development' or 'production'
// Set via: TENSORFLEET_BUILD=development bun run compile
// Or use scripts: bun run compile:dev / bun run compile:prod
const buildMode = process.env.TENSORFLEET_BUILD || 'production';
const isDev = buildMode === 'development';

console.log(`[TensorFleet Build] Mode: ${buildMode}`);

// Keep VS Code API and Node built-ins external; bundle everything else used by the extension.
const external = [
  'vscode',
  ...builtinModules,
  ...builtinModules.map((mod) => `node:${mod}`)
];

// Preserve the shebang on the MCP server CLI entry.
const shebangPlugin = () => ({
  name: 'preserve-shebang',
  renderChunk(code: string, chunk: { name: string }) {
    if (chunk.name === 'mcp-server') {
      const sanitized = code.startsWith('#!')
        ? code.replace(/^#!.*\n/, '')
        : code;
      return {
        code: `#!/usr/bin/env node\n${sanitized}`,
        map: null
      };
    }
    return null;
  }
});

export default defineConfig({
  resolve: {
    preserveSymlinks: true
  },
  // Build-time constants - these are replaced at compile time
  // Usage: if (__DEV__) { ... } - dead code eliminated in prod builds
  define: {
    __DEV__: JSON.stringify(isDev),
    __PROD__: JSON.stringify(!isDev),
    __BUILD_MODE__: JSON.stringify(buildMode),
    __BUILD_TIME__: JSON.stringify(new Date().toISOString()),
  },
  build: {
    lib: {
      entry: {
        extension: path.resolve(__dirname, 'src/extension.ts'),
        'mcp-server': path.resolve(__dirname, 'src/mcp-server.ts')
      },
      formats: ['cjs'],
      fileName: (format, entryName) => `${entryName}.js`
    },
    outDir: 'dist',
    target: 'node16',
    sourcemap: true,
    emptyOutDir: true,
    // Minification in production helps tree-shake dead code from __DEV__ checks
    minify: !isDev,
    rollupOptions: {
      external,
      output: {
        entryFileNames: '[name].js',
        chunkFileNames: 'chunks/[name]-[hash].js',
        assetFileNames: 'assets/[name]-[hash][extname]'
      },
      plugins: [shebangPlugin()]
    }
  },
  ssr: {
    external: ['vscode']
  }
});
