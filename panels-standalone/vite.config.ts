import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { resolve } from "path";
import url from "@rollup/plugin-url";

const rootDir = __dirname;
const packagesDir = resolve(rootDir, "./packages");
const typesDir = resolve(rootDir, "./packages/@types");

// ✅ tensorfleet-util resolves to SOURCE in dev/build (like your @lichtblick packages)
const tensorfleetUtilDir = resolve(rootDir, "./packages/tensorfleet-util/src");

const vmProxyTarget =
  process.env.VM_MANAGER_PROXY ?? process.env.VITE_VM_MANAGER_PROXY ?? "http://localhost:8080";
const wsProxyTarget = vmProxyTarget.replace(/^http/, "ws");

export default defineConfig({
  plugins: [
    react(),
    {
      name: "inject-react-import",
      enforce: "pre",
      transform(code, id) {
        if (!id.endsWith(".tsx") && !id.endsWith(".jsx")) return null;
        if (id.includes("node_modules")) return null;
        if (!code.includes("React.")) return null;

        if (
          /\bimport\s+React\s+from\s+["']react["']/.test(code) ||
          /\bimport\s+\*\s+as\s+React\s+from\s+["']react["']/.test(code) ||
          /\b(var|let|const)\s+React\b/.test(code) ||
          /\bfunction\s+React\b/.test(code) ||
          /\bclass\s+React\b/.test(code) ||
          /\bReact\s*=/.test(code)
        ) {
          return null;
        }

        return {
          code: `import * as React from "react";\n${code}`,
          map: null,
        };
      },
    },
  ],

  worker: {
    plugins: () => [],
    format: "es",
  },

  define: {
    global: "globalThis",
    __filename: JSON.stringify("browser"),
    __dirname: JSON.stringify("/"),
    ReactNull: "null",
  },

  esbuild: {
    tsconfigRaw: "{}",
  },

  build: {
    outDir: "dist",
    emptyOutDir: true,
    target: "esnext",
    rollupOptions: {
      input: {
        main: resolve(rootDir, "index.html"),
        image: resolve(rootDir, "image.html"),
        teleops: resolve(rootDir, "teleops.html"),
        map: resolve(rootDir, "mission_control.html"),
        vacuum_control: resolve(rootDir, "vacuum_control.html"),
        nav2: resolve(rootDir, "nav2.html"),
        gzweb: resolve(rootDir, "gzweb.html"),
        raw_messages: resolve(rootDir, "raw_messages.html"),
        sensor_3d: resolve(rootDir, "sensor_view_3d.html"),
        featured_entities: resolve(rootDir, "featured_entities.html"),
      },
      output: {
        assetFileNames: "assets/[name]-[hash][extname]",
      },
    },
  },

  resolve: {
    alias: [
      { find: "@", replacement: resolve(rootDir, "./src") },

      // ✅ tensorfleet-util deep imports:
      // import "... from 'tensorfleet-util/ros-util/ros-types'"
      // -> packages/tensorfleet-util/src/ros-util/ros-types.ts
      {
        find: /^tensorfleet-util(\/.*)?$/,
        replacement: `${tensorfleetUtilDir}$1`,
      },

      {
        find:
          /^@lichtblick\/(suite-base|log|suite|hooks|mcap-support|theme|message-path|typescript-transformers|comlink-transfer-handlers)(\/.*)?$/,
        replacement: `${packagesDir}/$1/src$2`,
      },
      {
        find: /^@lichtblick\/den(\/.*)?$/,
        replacement: `${packagesDir}/den$1`,
      },
      {
        find: /^@types\/([^/]+)/,
        replacement: `${typesDir}/$1`,
      },
      {
        find: /^gzweb(\/.*)?$/,
        replacement: `${packagesDir}/gzweb/src$1`,
      },
    ],
  },

  optimizeDeps: {
    exclude: [
      // ✅ keep this excluded so Vite doesn't prebundle it weirdly
      "tensorfleet-util",

      "@lichtblick/wasm-bz2",
      "@lichtblick/wasm-zstd",
      "@lichtblick/wasm-lz4",
      "@foxglove/wasm-bz2",
      "@foxglove/wasm-zstd",
      "@foxglove/wasm-lz4",
    ],
  },

  assetsInclude: ["**/*.wasm"],

  server: {
    proxy: vmProxyTarget
      ? {
          "/vms": {
            target: vmProxyTarget,
            changeOrigin: true,
          },
          "/ws": {
            target: wsProxyTarget,
            ws: true,
            changeOrigin: true,
          },
        }
      : undefined,
  },
});
