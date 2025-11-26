# Building and Publishing Extension

This repository includes GitHub Actions workflows to automatically build VSIX packages for VS Code and open marketplaces like Windsurf and OpenVSX.

## Automatic Builds

The workflow automatically runs on:
- **Push to main/master/build-github branches** - Builds VSIX packages and uploads as artifacts
- **Push of version tags** (e.g., `v1.0.0`) - Builds packages with version from tag
- **Pull requests** - Builds packages for testing (artifacts available for review)

## Manual Builds

You can manually trigger builds from the GitHub Actions tab:
1. Go to **Actions** → **Build and Package Extension**
2. Click **Run workflow**
3. Optionally enable publishing (requires secrets - see below)

## Downloading Built VSIX Files

1. Go to the **Actions** tab in your GitHub repository
2. Click on the latest workflow run
3. Scroll down to the **Artifacts** section
4. Download the `vsix-packages` artifact
5. Extract the ZIP file to get:
   - `tensorfleet-drone-<version>.vsix` - For VS Code Marketplace
   - `tensorfleet-drone-openvsx-<version>.vsix` - For OpenVSX/Windsurf
   - `RELEASE_NOTES.md` - Installation instructions

## Installing from VSIX

### VS Code
1. Open VS Code
2. Go to Extensions view (Ctrl+Shift+X / Cmd+Shift+X)
3. Click the `...` menu → **Install from VSIX...**
4. Select the downloaded `.vsix` file

### Windsurf / OpenVSX Compatible Editors
Use the `-openvsx` version of the VSIX file with your editor's "Install from VSIX" option.

## Publishing to Marketplaces

### Prerequisites

To publish, you need to set up authentication tokens as GitHub Secrets:

1. **VS Code Marketplace** (`VSCE_PAT`):
   - Go to https://dev.azure.com → User Settings → Personal Access Tokens
   - Create a token with "Marketplace (Manage)" scope
   - Add as secret: `VSCE_PAT`

2. **OpenVSX** (`OVSX_PAT`):
   - Go to https://open-vsx.org/user-settings/keys
   - Create a Personal Access Token
   - Add as secret: `OVSX_PAT`

### Publishing

1. Push a version tag or commit to main/master
2. Go to **Actions** → **Build and Package Extension**
3. Click **Run workflow**
4. Check the **Publish to marketplace** option
5. The workflow will build and publish to both marketplaces (if secrets are configured)

## Local Building

To build locally:

```bash
# Build React panels
cd panels-standalone
bun install
bun run build
cd ..

# Compile extension
bun install
bun run compile

# Package VSIX
bunx vsce package
```

The VSIX file will be created in the root directory as `tensorfleet-drone-<version>.vsix`.

## Troubleshooting

### Build Fails
- Check that all dependencies are in `package.json` (not just `package-lock.json`)
- Ensure `panels-standalone` builds successfully
- Check that `out/` directory contains compiled JavaScript

### VSIX Too Large
- Check `.vscodeignore` excludes unnecessary files
- Ensure `node_modules` are excluded (they should be bundled if needed)
- Review what's being included in the package

### Publishing Fails
- Verify secrets are correctly set in GitHub repository settings
- Check that the PAT tokens have the correct scopes/permissions
- Ensure you're publishing from a tag or main/master branch

