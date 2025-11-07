#!/bin/bash
# TensorFleet Build Script
# Builds React panels and extension

set -e  # Exit on error

echo "🔨 Building TensorFleet Extension..."
echo ""

# 1. Build React panels
echo "📦 Step 1/2: Building React panels..."
cd panels-standalone
bun install
bun run build
cd ../
echo ""

# 2. Build extension
echo "🔧 Step 2/2: Building extension..."
bun run compile

echo ""
echo "✅ Build complete!"
echo ""
echo "🚀 To launch:"
echo "   Press F5 in VS Code"
echo "   or run: code --extensionDevelopmentPath=$(pwd)"

