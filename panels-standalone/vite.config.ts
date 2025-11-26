// vite.config.ts
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { resolve } from 'path'
import url from '@rollup/plugin-url'

const rootDir = __dirname
const packagesDir = resolve(rootDir, './packages')
const typesDir = resolve(rootDir, './packages/@types')

export default defineConfig({
  plugins: [react()],

  worker: {
    plugins: () => [],
    format: "es"
  },

  define: {
    // Make bare `global` in dependencies resolve to `globalThis`
    global: "globalThis",
    __filename: JSON.stringify("browser"),
    __dirname: JSON.stringify("/"),
  },

  // stop Vite from trying to resolve tsconfig "extends" in workspace packages
  esbuild: {
    tsconfigRaw: '{}',
  },

  build: {
    outDir: 'dist',
    emptyOutDir: true,
    target: 'esnext',
    rollupOptions: {
      input: {
        main: resolve(rootDir, 'index.html'),
        image: resolve(rootDir, 'image.html'),
        teleops: resolve(rootDir, 'teleops.html'),
        map: resolve(rootDir, 'mission_control.html'),
        raw_messages: resolve(rootDir, 'raw_messages.html'),
      },
      plugins: [
        url({
          include: ['**/*.wasm'],
          limit: 0,
          fileName: 'assets/[name]-[hash][extname]',
        }),
      ],
    },
  },

  resolve: {
    alias: [
      { find: '@', replacement: resolve(rootDir, './src') },

      // @lichtblick/* packages that live in packages/<name>/src/*
      {
        find:
          /^@lichtblick\/(suite-base|log|suite|hooks|mcap-support|theme|message-path|typescript-transformers|comlink-transfer-handlers)(\/.*)?$/,
        replacement: `${packagesDir}/$1/src$2`,
      },

      // @lichtblick/den/* lives directly under packages/den/*
      {
        find: /^@lichtblick\/den(\/.*)?$/,
        replacement: `${packagesDir}/den$1`,
      },

      // Local @types/* packages under packages/@types/*
      {
        find: /^@types\/([^/]+)/,
        replacement: `${typesDir}/$1`,
      },
    ],
  },

  optimizeDeps: {
    exclude: [
      '@lichtblick/wasm-bz2',
      '@lichtblick/wasm-zstd',
      '@lichtblick/wasm-lz4',
      '@foxglove/wasm-bz2',
      '@foxglove/wasm-zstd',
      '@foxglove/wasm-lz4',
    ],
  },

  assetsInclude: ['**/*.wasm'],
})
