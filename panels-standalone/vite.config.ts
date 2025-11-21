// vite.config.ts
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { resolve } from 'path'
import url from '@rollup/plugin-url'

export default defineConfig({
  plugins: [react()],

  // (Optional; you can omit this whole worker block if you don't use workers)
  worker: {
    // Vite 5 requires this to be a function if provided
    plugins: () => [],
  },

  build: {
    outDir: 'dist',
    emptyOutDir: true,
    target: 'esnext',
    rollupOptions: {
      input: {
        main: resolve(__dirname, 'index.html'),
        image: resolve(__dirname, 'image.html'),
        teleops: resolve(__dirname, 'teleops.html'),
        map: resolve(__dirname, 'mission_control.html'),
        gzweb: resolve(__dirname, 'gzweb.html'),
        raw_messages: resolve(__dirname, 'raw_messages.html'),
      },
      // Treat *.wasm in deps as assets (URLs), not ESM modules
      plugins: [
        url({
          include: ['**/*.wasm'],
          limit: 0,                        // always emit a file
          fileName: 'assets/[name]-[hash][extname]',
          // publicPath is unnecessary for most Vite setups; the default works
        }),
      ],
    },
  },

  resolve: {
    alias: [
      { find: '@', replacement: resolve(__dirname, './src') },
      {
        find: /^@lichtblick\/suite-base\/(.*)$/,
        replacement: resolve(__dirname, './src/lichtblick/suite-base/$1'),
      },
      {
        find: '@lichtblick/suite-base',
        replacement: resolve(__dirname, './src/lichtblick/suite-base'),
      },
      {
        find: /^@lichtblick\/mcap-support\/(.*)$/,
        replacement: resolve(__dirname, './src/lichtblick/mcap-support/$1'),
      },
      {
        find: '@lichtblick/mcap-support',
        replacement: resolve(__dirname, './src/lichtblick/mcap-support'),
      },
      // (Optional) keep these if you want Foxglove builds specifically
      // { find: '@lichtblick/wasm-lz4', replacement: '@foxglove/wasm-lz4' },
      // { find: '@lichtblick/wasm-zstd', replacement: '@foxglove/wasm-zstd' },
      // { find: '@lichtblick/wasm-bz2', replacement: '@foxglove/wasm-bz2' },
    ],
  },

  // Avoid prebundling these so their CJS + require('./*.wasm') pattern survives
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

  // Helps Vite treat wasm as an asset type too (harmless redundancy)
  assetsInclude: ['**/*.wasm'],
})
