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
    alias: {
      '@': resolve(__dirname, './src'),
      // (Optional) keep these if you want Foxglove builds specifically
      // '@lichtblick/wasm-lz4': '@foxglove/wasm-lz4',
      // '@lichtblick/wasm-zstd': '@foxglove/wasm-zstd',
      // '@lichtblick/wasm-bz2': '@foxglove/wasm-bz2',
    },
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
