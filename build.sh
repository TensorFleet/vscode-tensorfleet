#!/bin/bash
# TensorFleet Build Script
# Builds React panels (including the static gzweb panel) and the VS Code extension

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
PANELS_DIR="$ROOT_DIR/panels-standalone"
GZWEB_BUNDLE="$PANELS_DIR/src/static/gzweb/index.html"

echo "🔨 Building TensorFleet Extension..."
echo ""

# Ensure gzweb panel exists so the panel ships with the extension.
if [ ! -f "$GZWEB_BUNDLE" ]; then
  echo "❌ Missing gzweb panel at $GZWEB_BUNDLE"
  echo "   Add the gzweb HTML into src/static/gzweb/ before running this script."
  exit 1
fi

# 1. Build React panels (includes gzweb static page via Vite publicDir)
echo "📦 Step 1/2: Building React panels..."
cd "$PANELS_DIR"
bun install
bun run build
cd "$ROOT_DIR"
echo ""

# 2. Build extension
echo "🔧 Step 2/2: Building extension..."
bun install
bun run compile

echo ""
echo "✅ Build complete!"
echo ""
echo "🚀 To launch:"
echo "   Press F5 in VS Code"
echo "   or run: code --extensionDevelopmentPath=$ROOT_DIR"
