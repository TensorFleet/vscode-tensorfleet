import { builtinModules } from 'node:module';
import path from 'node:path';
import { defineConfig } from 'vite';

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
