Goal: Implement Valetudo Main Canvas Static Map Preview

Why:
The Valetudo map fixture and sidebar preview now exist, but the sidebar preview is too small to validate realistic saved-map data. The main canvas is currently mostly empty, so the next step is to render the normalized Valetudo layeredPreview there as a large read-only map preview. This should prove that rooms, zones, robot, charger, paths, and restrictions are visually understandable before adding selection or commands.

Scope:
- Add a Valetudo main-canvas preview path when snapshot.map.layeredPreview exists.
- Reuse/extract the existing MapPreviewCard SVG renderer core.
- Keep the preview read-only.
- Preserve aspect ratio and fit the map to the main canvas.
- Keep existing TurtleBot4/Nav2 MapCanvas behavior unchanged.
- Keep Map Targets in the sidebar.
- Decide whether the small sidebar Map Preview should stay, collapse, or be removed after main preview exists.

Non-goals:
- No segment selection.
- No zone selection.
- No target hover interaction.
- No segment cleaning command.
- No zone cleaning command.
- No go-to command.
- No Clean Area.
- No user-created zone drawing.
- No Map SSE/live streaming.
- No hardware support claim.

Important:
Do not use raw Valetudo payloads in UI. Use only normalized snapshot.map.layeredPreview and snapshot.map.targets.

Preferred architecture:
- Extract shared renderer:
  ValetudoLayeredMapSvg
- Use it in:
  MapPreviewCard for compact sidebar preview
  ValetudoMainMapPreview for large center preview

Capability behavior:
- Keep capabilities.map.supported=false if that flag still means full interactive map support.
- The main preview can be gated directly by normalized layeredPreview presence.
- Do not enable map/segment/zone/go-to capabilities.

Validation:
- bun run test:vacuum-adapter
- bun run --cwd panels-standalone build
- git diff --check

Progress report:
Update progress_report.md with:
- main canvas preview changes
- renderer extraction/reuse
- sidebar preview decision
- unchanged commands/capabilities
- validation results