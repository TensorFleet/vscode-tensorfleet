# Valetudo Map Data Foundation Review

Current review date: 2026-06-12.

## Implementation notes

We are beginning the Valetudo map foundation track using real or realistic
Valetudo map data. Simulation maps and Valetudo maps have different backend
representations, so this pass must not force Valetudo data directly into ROS
OccupancyGrid assumptions. The goal is to inspect Valetudo map formats, add
fixture support, add conservative runtime normalization, and prepare the adapter
contract for future map/segment/zone features.

Product flow remains:

```text
Valetudo source or fixed mock
  -> VM-managed Valetudo integration runtime
  -> vm-manager proxy path if applicable
  -> Valetudo backend adapter
  -> normalized vacuum_adapter snapshot/capabilities/commands
  -> VacuumControlPanel UI
```

Product UI must continue to consume only normalized `vacuum_adapter` state and
capabilities. Raw Valetudo map payloads, raw capability names, raw HTTP routes,
MQTT topics, source URLs, SSE/cache internals, vm-manager proxy details, and
backend-specific payload shapes remain runtime/backend diagnostics only.

## Valetudo map shape discovered

Local source inspected:

- `/home/shane/Valetudo` at commit
  `4a56bfe876c1f9b298fc5a1b3c8b213ad749ffa1`.
- `backend/lib/webserver/RobotRouter.js`.
- `backend/lib/entities/map/ValetudoMap.js`.
- `backend/lib/entities/map/MapLayer.js`.
- `backend/lib/entities/map/entities/*.js`.
- Representative Valetudo parser fixtures under
  `backend/test/lib/robots/*/res/map/*.json`.

Discovered endpoint:

```text
GET /api/v2/robot/state/map
```

`RobotRouter` returns `this.robot.state.map` directly. It also exposes
`/api/v2/robot/state/map/sse`, but live map SSE is out of scope for this pass.

Discovered normalized Valetudo map payload:

```ts
type ValetudoMapLike = {
  __class?: "ValetudoMap";
  metaData?: {
    version?: 2;
    nonce?: string;
    vendorMapId?: string | number;
    totalLayerArea?: number;
  };
  size: { x: number; y: number };
  pixelSize: number;
  layers: Array<{
    __class?: "MapLayer";
    type: "floor" | "wall" | "segment" | string;
    compressedPixels?: number[]; // [xStart, y, count, ...]
    dimensions?: {
      x?: { min?: number; max?: number; mid?: number; avg?: number };
      y?: { min?: number; max?: number; mid?: number; avg?: number };
      pixelCount?: number;
    };
    metaData?: {
      segmentId?: string | number;
      name?: string;
      active?: boolean;
      material?: string;
      area?: number;
    };
  }>;
  entities: Array<{
    __class?: "PointMapEntity" | "PathMapEntity" | "PolygonMapEntity" | "LineMapEntity" | string;
    type:
      | "charger_location"
      | "robot_position"
      | "go_to_target"
      | "obstacle"
      | "path"
      | "predicted_path"
      | "active_zone"
      | "no_go_area"
      | "no_mop_area"
      | "carpet"
      | "virtual_wall"
      | string;
    points: number[];
    metaData?: Record<string, unknown>;
  }>;
};
```

Important shape notes:

- Valetudo abstracts vendor maps into a layer/entity representation, not a ROS
  `nav_msgs/OccupancyGrid`.
- `size` and `pixelSize` are map-level dimensions; layer pixels use Valetudo
  compressed run-length tuples.
- Segment/room inventory is represented by `layers[]` with `type="segment"` and
  `metaData.segmentId`.
- Robot pose and charger position are point entities such as
  `robot_position` and `charger_location`.
- Zone-like entities can appear as polygon entities such as `active_zone`, but
  they should not be treated as persistent user-created zone inventory without
  additional source semantics.

## TensorFleet contract direction

Do not map Valetudo map data into `snapshot.map.grid`. That field remains the
ROS/Nav2 occupancy-grid surface for TurtleBot4 simulation behavior.

The recommended direction is to evolve `snapshot.map` with optional layered map
metadata and normalized target lists:

```ts
snapshot.map = {
  readiness,
  receiving,
  detail,
  grid,          // existing ROS occupancy grid surface
  metadata,      // existing occupancy-grid-derived metadata
  annotations,   // existing adapter/user annotations
  layeredMetadata?: {
    id?: string;
    width?: number;
    height?: number;
    pixelSize?: number;
    coordinateSystem?: "valetudo_pixel" | "unknown";
    layerCount?: number;
    entityCount?: number;
    segmentCount?: number;
    zoneCount?: number;
    updatedAt?: number | string;
    source?: "fixed_mock" | "valetudo_mock" | "valetudo_http" | "real_robot" | "unknown";
  };
  targets?: {
    segments?: VacuumMapTarget[];
    zones?: VacuumMapTarget[];
  };
};
```

Current implementation intentionally keeps:

- `snapshot.map.grid = null` for Valetudo.
- `snapshot.map.readiness = "unavailable"` for Valetudo product rendering.
- `capabilities.map.supported = false`.
- `segment_cleaning`, `zone_cleaning`, `room_cleaning`, `go_to_location`, and
  `start_coverage` unsupported or detected-not-ready.

This lets future work use realistic metadata and target fixtures without
creating UI dependencies on raw Valetudo payloads or prematurely enabling
commands.

## Runtime behavior direction

The VM-managed Valetudo integration runtime should own all direct Valetudo map
access. It should fetch `/api/v2/robot/state/map` in HTTP source modes, load a
TensorFleet-owned Valetudo-like fixture in fixed mock mode, and expose only a
normalized `map` section in `/api/v1/valetudo/snapshot`.

Recommended runtime map surface:

```json
{
  "available": true,
  "source": "fixed_mock",
  "updatedAt": 1718179200000,
  "metadata": {
    "id": "tensorfleet-fixed-map-001",
    "width": 300,
    "height": 200,
    "pixelSize": 5,
    "coordinateSystem": "valetudo_pixel",
    "layerCount": 5,
    "entityCount": 3,
    "segmentCount": 3,
    "zoneCount": 1
  },
  "targets": {
    "segments": [],
    "zones": []
  },
  "detail": "Valetudo map metadata and target inventory normalized; renderable map surfaces are not exposed yet."
}
```

Safety rules:

- Missing map data: `available=false`; no targets.
- Malformed map data: `available=false` for invalid top-level metadata; omit
  malformed records when the top-level map is valid; add diagnostics.
- Stale/unreachable/offline source: do not expose stale map targets as current
  truth.
- Map with no segments/zones: expose metadata only; keep target capabilities and
  cleaning commands unsupported.
- Raw capability names do not imply segment/zone command support.
- No hardware support claim is made until a real hardware snapshot is captured
  and tested.

## vm-manager / firecracker-vm impact

`firecracker-vm/tensorfleet-mgr` owns the Valetudo runtime structs, source
client, fixed mock source, fixtures, and handler tests, so this pass belongs
there plus the TypeScript adapter contract.

vm-manager inspection found the `/vms/self/tensorfleet` route is a generic
`httputil.NewSingleHostReverseProxy` to `http://<vm-ip>:9090`, trims the prefix,
and injects the internal bearer token. It does not validate or reshape
`/api/v1/valetudo/snapshot`, so no vm-manager code change is required for new
JSON fields.

## Deferred areas

- Full map rendering.
- MapCanvas Valetudo support.
- Segment/room/zone UI.
- Segment cleaning command.
- Zone cleaning command.
- Go-to command.
- User-created zone drawing.
- Map SSE/live streaming.
- Hardware validation.

---

# Valetudo No-Map Sidebar Organization Planning

Current review date: 2026-06-12.

## Implementation notes

We now have several normalized Valetudo/no-map UI surfaces: robot status, basic
controls, cleaning settings, maintenance, current statistics, attachments, and
dock components. The next task is to review the sidebar organization before
adding more backend capabilities. This pass should decide card grouping, order,
priority, collapse behavior, and attention handling without changing product
code.

Required product flow remains:

```text
Valetudo source or fixed mock
  -> VM-managed Valetudo integration runtime
  -> vm-manager proxy path if applicable
  -> Valetudo backend adapter
  -> normalized vacuum_adapter snapshot/capabilities/commands
  -> VacuumControlPanel UI
```

Product UI must continue to depend only on normalized snapshot sections and
capability descriptors. Raw Valetudo capability names, raw HTTP routes, MQTT
topics, source URLs, SSE/cache internals, vm-manager proxy paths, and
backend-specific payload shapes belong in runtime/backend code and diagnostics,
not UI branching. This sidebar organization pass does not introduce any of those
into product UI.

## Current sidebar card order (as of this pass)

The basic-robot-profile sidebar currently renders these cards in this order,
each gated by normalized capabilities and snapshot data:

1. Robot Overview (`RobotOverviewCard`)
2. Basic Cleaning Controls (`BasicControlsCard`)
3. Battery and Dock (`BatteryDockCard`)
4. Attachments (`AttachmentsCard`, conditional on `attachments` capability)
5. Dock Components (`DockComponentsCard`, conditional on `dock_components` capability)
6. Cleaning Settings (`CleaningSettingsCard`, conditional on `fan_speed` or `water_usage`)
7. Current Statistics (`CurrentStatisticsCard`, conditional on `statistics` capability)
8. Maintenance (`MaintenanceCard`, conditional on `consumables` capability)
9. Source / Health (`SourceHealthCard`)

## Review question answers

### 1. Recommended default card order

```
1. Robot Summary / Primary State
2. Basic Cleaning Controls
3. Current Statistics
4. Battery and Dock
5. Attachments
6. Dock Components
7. Cleaning Settings
8. Maintenance / Consumables
9. Source / Health / Unsupported Workflows
```

The primary change from the current order is moving Current Statistics from
position 7 to position 3. During an active cleaning run, duration and area are
immediately relevant operational feedback and belong near the controls that
started the run. The rest of the order is preserved from the current
implementation: Readiness (Battery/Dock → Attachments → Dock Components)
remains contiguous, Configure (Cleaning Settings) follows Readiness, Maintain
(Consumables) follows Configure, and Context (Source/Health) is last.

The suggested starting recommendation places Cleaning Settings at position 5
between Battery/Dock and Attachments/Dock Components. This review does not adopt
that order because it splits the Readiness group. Instead, Cleaning Settings
stays after the full Readiness block so the operator mental model flows:
confirm physical readiness → configure how to clean → start.

### 2. Primary operational controls

Primary operational controls are Robot Summary and Basic Cleaning Controls.

- **Robot Summary** gives the operator a single-glance answer to "what is the
  robot doing right now?" It covers identity, normalized activity state,
  availability, fault presence, and brief state detail.
- **Basic Cleaning Controls** are the only interactive command surface in the
  no-map sidebar. They include `start_cleaning`, `pause`, `resume`, `stop`, and
  `return_to_dock`, all gated by normalized capability availability.

These two cards must always be visible in the sidebar and must never be hidden,
collapsed, or moved below the fold for the basic-robot profile.

### 3. Status and readiness cards

Status and readiness cards, in recommended order:

- **Robot Summary** — primary activity state and fault indicator (Operate group).
- **Battery and Dock** — power level, dock relationship, charging state
  (Readiness group).
- **Attachments** — robot-side equipment and material presence
  (Readiness group, conditional on `attachments` capability).
- **Dock Components** — dock-side material readiness
  (Readiness group, conditional on `dock_components` capability).

Current Statistics straddles the Operate and Readiness boundary: it describes
operational output (area, duration) rather than physical readiness. It is
classified under the Operate group for grouping purposes.

Source / Health is a diagnostic-adjacent status card. It is visible but belongs
in the Context group at the bottom of the sidebar, not in the Readiness group.

### 4. Maintenance and upkeep cards

**Maintenance / Consumables** (`MaintenanceCard`) is the sole upkeep card. It
shows normalized `snapshot.maintenance.consumables` entries with remaining life,
status, and display labels. It is conditional on `consumables` capability support.

The Maintain group is a single card in the current surface inventory. When
Consumable reset commands are added later, they will be part of this same card.

### 5. Above-the-fold priority

Cards that should appear above the fold (the top portion visible without
scrolling) for the typical narrow sidebar width:

- Robot Summary (always)
- Basic Cleaning Controls (always)
- Current Statistics (when supported and active/recent)
- Battery and Dock (always, when supported)

Cards that will likely require scrolling for a typical sidebar:

- Attachments (conditional)
- Dock Components (conditional)
- Cleaning Settings (conditional)
- Maintenance / Consumables (conditional)
- Source / Health (always last)

The sidebar width and font size determine the exact fold line. Above-the-fold
priority is a CSS/layout concern during implementation, not an ordering concern
here. The recommended order is sufficient to achieve above-the-fold goals for
the most critical surfaces.

### 6. Collapsible defaults

Recommended collapse behavior by group:

**Operate (Robot Summary, Basic Cleaning, Current Statistics)**
- Robot Summary: always expanded, not collapsible.
- Basic Cleaning: always expanded, not collapsible.
- Current Statistics: always expanded when visible, not collapsible.
  The card is already conditionally rendered (only shown when `statistics`
  is supported and duration or area data exists), so it self-hides when not
  relevant rather than needing a user-collapsed state.

**Readiness (Battery/Dock, Attachments, Dock Components)**
- Battery and Dock: always expanded, not collapsible.
- Attachments: collapsible when all items are `ok`/`installed` and no attention
  items are present. Expanded when any item has `missing`, `full`, `empty`,
  `low`, or `error` status.
- Dock Components: same policy as Attachments.

**Configure (Cleaning Settings)**
- Collapsible once the operator has set fan speed and water usage. Expanded by
  default so the operator sees current settings before starting a run.
  Alternatively, always expanded since the card is compact.

**Maintain (Consumables)**
- Collapsible when all consumables are at `ok` status. Expanded when any
  consumable is at `warning` or `error` status.

**Context (Source / Health, Unsupported Workflows)**
- Source / Health: collapsed by default when runtime is `online` and source is
  reachable/fresh. Expanded automatically when source is stale, unreachable, or
  offline, or when runtime is degraded.
- Unsupported Workflows: collapsed by default. The no-map explanation is
  expected context, not urgent information.

Note: collapse behavior requires UI implementation work that is deferred from
this pass. This review recommends the policy but does not implement it.

### 7. Attachments and Dock Components as separate vs grouped cards

Keep **Attachments** and **Dock Components** as separate cards, but treat them
as belonging to the same visual Readiness group.

Reasons to keep them separate:

- Attachments are robot-side equipment and material state. Dock Components are
  dock-side material readiness. They have different physical ownership, different
  valid `kind` values, and different operator actions associated with attention.
- Conflating them into a single "Readiness" card risks misleading operators: a
  dustbin attachment status and a dock dustbag component status are different
  things even though both have `status=full`.
- Separate cards preserve future extensibility: dock action commands (deferred)
  will be added to or alongside the Dock Components card when implemented. Robot
  attachment actions (mop wash, mop dry, deferred) will be added to the
  Attachments card.
- The `ReadinessListCard` shared component already handles both and keeps their
  rendering compact.

A shared **visual group header** (non-interactive, CSS section separator) for
the Readiness group is a valid layout choice during implementation. It does not
require merging the two cards.

### 8. Current Statistics placement

Current Statistics belongs in the **Operate group** at position 3, immediately
after Basic Cleaning Controls.

Reasoning:

- Current Statistics is live operational feedback: how long has the run taken,
  how much area has been covered. It is most relevant when the robot is actively
  cleaning or just finished a run.
- Placing it next to controls creates a coherent operator view: "what can I do"
  (controls) → "what is it accomplishing" (statistics).
- Placing it at position 7 (current) buries it below four other cards and
  requires scrolling during an active run.
- The card is already conditionally rendered (hidden when no statistics data
  exists), so it does not crowd the sidebar for robots that do not support
  statistics.

Current Statistics is not a Readiness card (it does not inform whether the
robot is ready to clean) and not a Maintenance card (it does not describe
consumable life). The Operate group is the correct home.

### 9. Fault and attention surfacing without a diagnostics panel

Do not add a full diagnostics panel or a standalone Attention card.

Instead, use **per-card attention indicators** and **Robot Summary attention
summary**:

**Per-card attention indicators:**
- Add a compact indicator (badge, colored label, or dot) to the card header of
  any card that has an attention-worthy item. Derive this indicator only from
  normalized state owned by that card.
  - Attachments header badge: count of items with `missing`, `full`, `empty`,
    `low`, or `error` status.
  - Dock Components header badge: same policy.
  - Maintenance header badge: count of consumables with `warning` or `error`
    status.
  - Source / Health: already shows inline status values; no badge needed.

**Robot Summary attention summary:**
- The Robot Summary card already renders `fault.faults[]`, stale-source detail,
  and `activity.detail` for the `faulted`/`unavailable` states through
  `deriveVacuumPrimaryRobotState`. This should remain the primary attention
  surface for source and runtime health.
- When fault is absent but cross-surface readiness attention exists, a compact
  secondary line (e.g., "Dustbin full · Freshwater empty") can be derived from
  normalized attachment and dock component statuses and appended to the Robot
  Summary. This keeps the operator informed at the top of the sidebar without
  scrolling.

**Severity inference from (kind, status) pairs:**
- `status=full` on `kind=dustbin` or `kind=wastewater` or `kind=dustbag` is an
  attention condition (needs servicing).
- `status=empty` on `kind=freshwater` or `kind=water_tank` is an attention
  condition (cannot mop).
- `status=missing` on any kind is an attention condition.
- `status=low` on `kind=freshwater`, `kind=water_tank`, or `kind=detergent` is a
  warning condition.
- `status=ok` or `status=installed` is nominal.
- `status=unknown` should not be promoted to attention without explicit support
  in normalized data; `unknown` may mean data is temporarily unavailable.

Do not infer severity from `status` alone without considering `kind`. A
`status=full` on a freshwater tank is not an attention condition.

If severity inference from (kind, status) pairs becomes fragile, the recommended
future improvement is an explicit `severity` field on attachment items and dock
component items (`"ok" | "warning" | "error"`), populated by the adapter mapper
with full knowledge of the source semantics. This would let the UI use severity
directly without re-inferring it from kind/status combinations.

### 10. Behavior when only some optional surfaces exist

When only some optional surfaces exist, the sidebar renders only the cards whose
normalized capabilities are supported and whose data is non-empty:

- Attachments card appears only when `capabilities.attachments.supported` is
  `true` and `snapshot.attachments.items.length > 0`.
- Dock Components card appears only when `capabilities.dock_components.supported`
  is `true` and `snapshot.dock.components.length > 0`.
- Cleaning Settings card appears only when `fan_speed` or `water_usage`
  capability is supported.
- Current Statistics card appears only when `statistics` capability is supported
  and duration or area data exists.
- Maintenance card appears only when `consumables` capability is supported.

No placeholder cards or empty-state rows are rendered for unsupported
capabilities. The sidebar compresses naturally when optional surfaces are absent.

Example: attachments present but no dock components → Attachments card appears
at position 5 in the recommended order, Dock Components card is absent, and
Cleaning Settings appears at position 6 (effective position after compression).
The ordering relationship between remaining visible cards is preserved.

### 11. Avoiding Valetudo layout copy

The functional categories used in this review are reference-only. The sidebar
layout, card grouping model, labels, icon choices, spacing, visual weight, and
UX patterns are TensorFleet product decisions, not Valetudo copies.

Specific ways this layout diverges from Valetudo:

- Valetudo uses a flat list of capability-specific controls. This layout uses
  a grouped, hierarchical card model (Operate → Readiness → Configure →
  Maintain → Context).
- This layout prioritizes operational feedback (Current Statistics near controls)
  in a way that reflects the no-map operator workflow, not Valetudo's map-first
  UX.
- Attention surfacing uses per-card badges and Robot Summary secondary lines,
  not Valetudo's inline status attribute rows.
- No attempt to match Valetudo's visual design, icon set, or color language.
- Collapse behavior is TensorFleet-defined and not derived from Valetudo's
  always-expanded card list.
- Raw Valetudo capability names (`BasicControlCapability`, `FanSpeedControlCapability`,
  etc.) do not appear in any product card label, hint, or tooltip.

### 12. vm-manager and firecracker-vm expectations

**No vm-manager or firecracker-vm code changes are expected for UI-only layout
work.**

The sidebar organization, card grouping, card order, collapse behavior, and
attention indicator changes are all UI-layer work that consumes existing
normalized surfaces. They do not require:

- New runtime snapshot fields.
- New `tensorfleet-mgr` endpoints or structs.
- New vm-manager proxy routes.
- New adapter capabilities or command types.

The existing normalized surfaces already carry all data needed for the
recommended sidebar layout:
- `snapshot.identity`, `snapshot.activity`, `snapshot.fault` for Robot Summary.
- Existing command capabilities for Basic Cleaning Controls.
- `snapshot.statistics.current` for Current Statistics.
- `snapshot.battery`, `snapshot.dock` for Battery and Dock.
- `snapshot.attachments.items[]` for Attachments.
- `snapshot.dock.components[]` for Dock Components.
- `snapshot.cleaningSettings` for Cleaning Settings.
- `snapshot.maintenance.consumables[]` for Maintenance.
- `snapshot.health`, `snapshot.source`, `snapshot.availability`, `snapshot.fault`
  for Source / Health.

Future UI-only work (adding section headers, collapse buttons, attention
badges) should also not require runtime changes, as long as no new data is
surfaced. If a future attention badge requires a severity field not currently in
the normalized model, that would require adapter and potentially runtime changes,
but that is explicitly deferred by this review.

vm-manager's role remains unchanged: generic reverse proxy from
`{VM_MANAGER_URL}/vms/self/tensorfleet/...` to the guest runtime. No vm-manager
knowledge of card layout is needed or appropriate.

## Recommended card order (final)

```
1. Robot Summary / Primary State    [Operate]   always visible
2. Basic Cleaning Controls           [Operate]   always visible
3. Current Statistics                [Operate]   visible when supported + data exists
4. Battery and Dock                  [Readiness] always visible when supported
5. Attachments                       [Readiness] visible when capability supported
6. Dock Components                   [Readiness] visible when capability supported
7. Cleaning Settings                 [Configure] visible when fan_speed or water_usage supported
8. Maintenance / Consumables         [Maintain]  visible when consumables supported
9. Source / Health / Unsupported     [Context]   always last; collapsed when nominal
```

The only change from the current sidebar order is:

- **Current Statistics moves from position 7 to position 3.**
- All other relative positions are preserved.

The suggested starting recommendation is partially accepted: Current Statistics
moves up to the Operate group. However, Cleaning Settings is **not** moved to
position 5 (between Battery/Dock and Attachments/Dock Components) because that
would split the Readiness group. Cleaning Settings remains at the end of the
Readiness-Configure sequence, after all Readiness cards.

## Recommended grouping

| Group | Cards | Rationale |
|---|---|---|
| Operate | Robot Summary, Basic Cleaning, Current Statistics | What the robot is doing and accomplishing right now |
| Readiness | Battery and Dock, Attachments, Dock Components | Physical state and hardware readiness |
| Configure | Cleaning Settings | How the robot should clean |
| Maintain | Consumables | Upkeep and consumable life |
| Context | Source/Health, Unsupported Workflows | Diagnostic context, connectivity, no-map explanation |

Visual group headers (non-interactive CSS section dividers with group labels) are
optional for the first layout implementation. If the sidebar becomes dense when
all cards are visible, group headers will help operators orient quickly.

## Recommended collapse behavior

| Card | Default state | Auto-expand when |
|---|---|---|
| Robot Summary | Always expanded | — |
| Basic Cleaning | Always expanded | — |
| Current Statistics | Expanded when visible (conditionally rendered) | — |
| Battery and Dock | Always expanded | — |
| Attachments | Expanded by default; collapsible | Any item has missing/full/empty/low/error |
| Dock Components | Expanded by default; collapsible | Any component has missing/full/empty/low/error |
| Cleaning Settings | Expanded by default | — |
| Maintenance | Collapsible | Any consumable at warning/error |
| Source / Health | Collapsed when nominal | Source stale/unreachable/offline; runtime degraded |

Collapse state is local UI state. It does not require adapter or runtime
changes.

## Recommended attention behavior

Sources of attention, derived entirely from existing normalized fields:

| Source | Field | Attention condition |
|---|---|---|
| Robot fault | `fault.faults[]` | Any fault present |
| Faulted activity | `activity.status` | `"faulted"` or `"unavailable"` |
| Source stale | `source.stale` | `true` |
| Source unreachable | `source.status` | `"unreachable"` or `"offline"` |
| Runtime degraded | `health.runtimeStatus` | `"degraded"` or `"offline"` |
| Attachment attention | `attachments.items[].status` + `.kind` | `missing`, `full` (dustbin), `empty` (water_tank), `low`, `error` |
| Dock component attention | `dock.components[].status` + `.kind` | `missing`, `full` (wastewater/dustbag), `empty` (freshwater), `low`, `error` |
| Consumable warning | `maintenance.consumables[].status` | `"warning"` or `"error"` |

Attention surfacing approach:

1. **Per-card header badge**: compact item count badge on Attachments, Dock
   Components, and Maintenance card headers when attention items exist.
2. **Robot Summary secondary line**: brief cross-surface attention summary line
   in the Robot Summary card when no robot fault is active but peripheral
   attention conditions exist (e.g., "Dustbin full · Freshwater empty").
3. **Source / Health auto-expand**: Source/Health card auto-expands when source
   or runtime attention conditions are active.

Do not add a standalone Attention card or a diagnostics panel. All attention
derives from existing normalized surfaces.

Severity inference explicitly accounts for (kind, status) pairs to avoid
false positives (e.g., `full` freshwater tank is not attention). If this
becomes complex, defer to a future explicit `severity` field on attachment and
dock component items.

## Deferred from this pass

- Sidebar layout implementation (CSS, section headers, collapse behavior,
  attention badges).
- Dock action commands.
- Total statistics card.
- Operation mode card.
- Consumable reset commands.
- Map rendering and segment/room/zone/go-to behavior.
- Hardware validation for any new surface.

## Validation

This is a docs/planning pass only. No product code changed.

Required validation:

```sh
git diff --check
```

---

# Attachments and Dock Components Planning Review

Current review date: 2026-06-12.

This pass reviews Valetudo-style Attachments and Dock component concepts against
TensorFleet's normalized `vacuum_adapter` model. It is a docs-only planning pass:
no product code, runtime code, adapter code, handlers, tests, VM services, or
proxy behavior changed.

Required product flow remains:

```text
Valetudo source or fixed mock
  -> VM-managed Valetudo integration runtime
  -> vm-manager proxy path if applicable
  -> Valetudo backend adapter
  -> normalized vacuum_adapter snapshot/capabilities/commands
  -> VacuumControlPanel UI
```

Product UI must continue to depend only on normalized snapshot sections and
capability descriptors. Raw Valetudo capability names, raw HTTP routes, MQTT
topics, source URLs, SSE/cache internals, and backend-specific payload shapes
belong in runtime/backend code and diagnostics, not UI branching.

## 1. Recommendation

Choose **C. Attachments plus Dock Components read-only together**, but keep the
first implementation intentionally small:

- Add a robot-side `snapshot.attachments.items[]` surface for installed/present
  equipment and material state.
- Extend `snapshot.dock` with a `components[]` surface for dock-side material
  readiness.
- Add read-only capability descriptors named `attachments` and
  `dock_components`, each with `commands: []` and attributes that describe the
  populated state.
- Do not add dock action commands, auto-empty commands, mop wash/dry commands,
  refill commands, or consumable reset commands in the first slice.

This is small enough because both surfaces are display-only arrays with the same
status vocabulary. It is also safer than starting with dock actions because it
lets the runtime, adapter, fixed mock, and UI agree on state before any command
semantics can affect hardware.

## 2. Normalized Attachment State

Recommended adapter shape:

```ts
type VacuumAttachmentKind =
  | "dustbin"
  | "water_tank"
  | "mop"
  | "detergent"
  | "unknown";

type VacuumAttachmentStatus =
  | "installed"
  | "missing"
  | "full"
  | "empty"
  | "low"
  | "ok"
  | "unknown"
  | "error";

type VacuumAttachmentState = {
  id: string;
  label: string;
  kind: VacuumAttachmentKind;
  status: VacuumAttachmentStatus;
  available?: boolean;
  levelPercent?: number;
  detail?: string;
  updatedAt?: number | string;
};

snapshot.attachments = {
  items: VacuumAttachmentState[];
};
```

Field guidance:

- `id`: stable product-level identifier such as `dustbin`, `water_tank`, `mop`,
  or `detergent`; source-specific names stay in diagnostics.
- `label`: product display label, with source labels allowed only after mapper
  normalization and fallback.
- `kind`: stable category for grouping and icons.
- `status`: current presence/material state. Use `unknown` when the runtime can
  name the item but cannot confidently classify it.
- `available`: optional display readiness when status alone is insufficient.
  For example, a mop can be installed but unavailable because source state is
  stale.
- `levelPercent`: optional finite `0..100` material level for tanks or
  detergent when the source reports an actual level.
- `detail`: short product-safe explanation, not raw route/topic/class data.
- `updatedAt`: source/runtime timestamp when available.

Attachments are not consumables. A mop pad can have remaining life under
`maintenance.consumables`, while `attachments.items[]` answers whether a mop is
currently attached or usable. A dustbin can have a full/missing state without
being a replaceable-life consumable.

## 3. Normalized Dock Components

Recommended adapter shape:

```ts
type VacuumDockComponentKind =
  | "freshwater"
  | "wastewater"
  | "detergent"
  | "dustbag"
  | "unknown";

type VacuumDockComponentStatus =
  | "ok"
  | "missing"
  | "full"
  | "empty"
  | "low"
  | "unknown"
  | "error";

type VacuumDockComponentState = {
  id: string;
  label: string;
  kind: VacuumDockComponentKind;
  status: VacuumDockComponentStatus;
  levelPercent?: number;
  detail?: string;
  updatedAt?: number | string;
};

snapshot.dock = {
  supported?: boolean;
  state?: VacuumDockState;
  charging?: boolean;
  detail?: string;
  components?: VacuumDockComponentState[];
};
```

Field guidance:

- `id`: stable product-level identifier such as `freshwater`, `wastewater`,
  `detergent`, or `dustbag`.
- `label`: UI label after normalization.
- `kind`: stable category for grouping and icons.
- `status`: readiness state. For example, freshwater empty is attention,
  wastewater full is attention, dustbag missing is attention, and detergent low
  is warning/attention depending on UI policy.
- `levelPercent`: optional finite `0..100` level only when reported.
- `detail`: product-safe explanation.
- `updatedAt`: source/runtime timestamp when available.

Dock components extend the existing normalized dock model because they are
dock-side state. Basic `dock.state` remains the robot/dock relationship
(`docked`, `undocked`, `returning`, `charging`, `error`, `unknown`), while
`dock.components[]` describes dock-side materials.

## 4. Shared vs Separate Models

Use **separate public types** for attachments and dock components, with a shared
private mapper helper only if implementation duplication becomes real.

Reasoning:

- Attachments are robot-side equipment/material state; dock components are
  dock-side material readiness.
- They have overlapping statuses but different valid kinds and different UI
  grouping.
- Separate public types prevent accidental UI conflation such as rendering a
  robot dustbin like a dock dustbag or treating a dock freshwater tank like a
  robot water tank.
- A private helper can normalize common fields (`id`, `label`, `status`,
  `levelPercent`, `detail`, `updatedAt`) without making the public contract too
  generic.

Do not overload `maintenance.consumables`. Consumables remain replaceable-life
or cleaning-life indicators. Attachments and dock components are current
presence/readiness indicators.

## 5. Capability Descriptors

Add explicit read-only capabilities:

- `attachments`
  - `supported: true` only when `snapshot.attachments.items.length > 0` and the
    source is fresh/reachable enough to trust the normalized state.
  - `commands: []`.
  - Suggested attributes: `["items"]` plus optional item attributes such as
    `kind:dustbin`, `kind:water_tank`, `kind:mop`, `kind:detergent`.

- `dock_components`
  - `supported: true` only when `snapshot.dock.components.length > 0` and the
    source is fresh/reachable enough to trust the normalized state.
  - `commands: []`.
  - Suggested attributes: `["components"]` plus optional component attributes
    such as `kind:freshwater`, `kind:wastewater`, `kind:detergent`,
    `kind:dustbag`.

Keep `dock_actions` deferred. It should become a separate capability family only
after command semantics are designed and validated. A single `dock_state`
capability with broad attributes would be too vague because existing
`dock_state` already describes robot/dock relationship, while component
readiness and future actions have different safety expectations.

## 6. Fixed Mock Scenarios To Add Later

Add deterministic fixed mock snapshots before UI uses the new surfaces:

- Happy path: dustbin installed, water tank installed with level, mop attached,
  detergent present, freshwater ok, wastewater empty/ok, dustbag ok.
- Attention path: dustbin full, water tank low, mop missing, detergent low,
  freshwater empty, wastewater full, dustbag missing.
- Unknown path: runtime can identify components but status/level is unknown.
- Partial hardware path: attachments present but no dock components, and dock
  components present but no attachment state.
- Source stale/unreachable/offline path: omit the new surfaces and mark
  capabilities unsupported/unavailable rather than showing stale readiness as
  current truth.
- Malformed source path: invalid levels, unknown item names, duplicate IDs, and
  invalid statuses are dropped or normalized to `unknown` without crashing the
  snapshot.

Regression tests should cover fixed mock mapping, HTTP-source mapping,
capability support, missing data, malformed data, stale/unreachable source
handling, and UI gating.

## 7. Likely Valetudo HTTP Source Mapping

Likely upstream inputs, to be verified during implementation against Valetudo
source behavior and actual `/api/v2` responses:

- Robot state attributes can carry attachment/material facts such as bin,
  water tank, mop, detergent, or related readiness states when a model exposes
  them.
- Capability-specific endpoints may expose richer properties for attachments,
  consumables, dock components, or model-specific dock modules.
- Raw capability names such as auto-empty or mop dock capabilities are useful
  only as diagnostics until the runtime maps read-only component state into the
  normalized snapshot.
- SSE/cache updates, if used, should be owned by the VM-managed Valetudo
  integration runtime and folded into the same runtime snapshot. The adapter and
  UI should not know whether data came from polling, SSE, MQTT, or cache.

Implementation should fail closed: if a Valetudo source cannot provide
trustworthy component state, leave the normalized surface absent and keep the
capability unsupported or unavailable. Do not infer hardware claims from model
names or raw capability presence alone.

## 8. Missing, Malformed, Stale, Unreachable, Unsupported

Recommended behavior:

- Missing: omit `snapshot.attachments` or leave `items: []`; omit
  `snapshot.dock.components` or leave it empty; capability unsupported.
- Malformed: ignore invalid item records, clamp or drop invalid percentages,
  normalize unknown statuses/kinds to `unknown` only when the item identity is
  still meaningful, and add diagnostics for mapper debugging.
- Stale: do not show attachment/component readiness as current product state;
  mark capability unavailable or unsupported with a stale-source reason.
- Unreachable/offline: omit the surfaces and mark capabilities unavailable due
  to `source_unreachable` or `runtime_offline`.
- Unsupported: keep the surfaces absent and show no UI card, except an
  unsupported-workflow explanation if the broader no-map/sidebar experience
  needs expectation-setting.

The UI should render cards only from normalized state plus supported capability
descriptors. It should not render placeholder component rows from raw
capability diagnostics.

## 9. Future UI Cards

Once normalized runtime/mock and adapter surfaces exist, build read-only cards:

- **Attachments**: compact robot-side card showing dustbin, water tank, mop, and
  detergent rows only when normalized items exist. Use status, optional level,
  and detail; do not show consumable-life data here.
- **Dock Components**: compact dock-side card showing freshwater, wastewater,
  detergent, and dustbag rows only when normalized dock components exist. Use
  status, optional level, and detail.
- **Attention Summary**: optional integration into existing Fault/Attention or
  Robot Summary cards for statuses such as `missing`, `full`, `empty`, `low`,
  or `error`, but derive it from normalized surfaces.

Do not build dock action buttons in this slice. Future buttons for empty dustbin,
wash mop, dry mop, or refill water need separate command types, runtime command
handlers, availability rules, idempotency decisions, and hardware validation.

## 10. firecracker-vm and tensorfleet-mgr Work

The first implementation will require `firecracker-vm/tensorfleet-mgr` changes:

- Runtime snapshot structs for `attachments` and `dock.components`.
- Fixed mock snapshot construction and named scenarios.
- HTTP-source normalization for any verified Valetudo attachment/component
  inputs.
- Mapper validation for missing, malformed, stale, unreachable, and unsupported
  data.
- Handler tests for the snapshot endpoint and source modes.
- Runtime capability tier/readiness diagnostics that distinguish read-only
  component state from deferred dock actions.
- Updated `tensorfleet-mgr` binary and VM service deployment only when moving
  from planning to implementation.

The adapter/UI repo will then need:

- `vacuum_adapter` state and capability type additions.
- Valetudo runtime contract/type guard updates.
- Valetudo state/capability mapper updates.
- Fixed adapter fixtures and `scripts/vacuum-adapter-regression.ts` coverage.
- Vacuum Control no-map sidebar cards gated only by normalized capabilities and
  normalized snapshot sections.

## 11. vm-manager Impact

No vm-manager change is expected for read-only attachments and dock components.
The existing generic proxy path should remain enough because the adapter already
consumes:

```text
GET /api/v1/valetudo/health
GET /api/v1/valetudo/snapshot
POST /api/v1/valetudo/command
```

Adding fields to the runtime snapshot should flow through the existing
`/vms/self/tensorfleet/api/v1/valetudo/snapshot` proxy route if vm-manager does
not validate a typed Valetudo payload. Re-check `src/vm-manager.ts` during
implementation, but do not add backend-specific proxy routes unless the runtime
API itself changes.

## 12. Deferred Areas

Keep these out of the first implementation:

- Dock action commands.
- Auto-empty command implementation.
- Mop wash/dry command implementation.
- Water refill command implementation.
- Consumable reset commands.
- Map rendering.
- Segments, rooms, zones, and go-to behavior.
- Hardware claims or model-specific promises.

## 13. Validation

This is a docs/planning pass only. Required validation:

```sh
git diff --check
```

# Post-Layer-6A Vacuum Control Data Inventory Review

Current review date: 2026-06-11.

Historical note: this June 11 inventory is retained as context. Its
Current Statistics recommendation and "statistics missing" statements predate
the completed Current Statistics normalization slice and are superseded by the
June 12 progress report plus the Attachments and Dock Components planning
decision above.

## 1. Current decision

This is a post-Layer-6A UI data inventory and review. Layer 6A is complete for
the Valetudo mock/runtime/adapter/UI path; this pass does not change product
code and does not add UI components.

The purpose is to collect real data first, then decide which Vacuum Control
sidebar cards to build next. The review is based on current code, current tests,
current docs, and visible local screenshots. It must not claim support from
Valetudo screenshots or raw backend data unless the value is normalized through
`vacuum_adapter`.

Decision for the next implementation pass:

- Safe-now UI work should use existing normalized adapter fields only:
  identity, availability, health/source, activity, dock, battery, basic command
  capability availability, fan speed, water usage, consumable display, no-map
  state, faults, and unsupported-workflow explanations.
- Adapter work is required before product UI exposes go-to, segments, zones,
  rooms, statistics, scheduling, advanced dock features, voice, Wi-Fi, firmware,
  logs, model settings, raw map entities, or consumable reset actions.
- Runtime/mock work is required when the value does not appear in the runtime
  snapshot at all, even if Valetudo can expose it upstream.
- Raw Valetudo capability names, HTTP routes, MQTT topics, source URLs, and
  runtime transport diagnostics stay diagnostics-only.

## 2. Valetudo UI functional reference from attached screenshots

No separate attached Valetudo screenshot image files were visible in this
workspace during this pass. Local June 10 screenshots inspected from
`~/Pictures/Screenshots` showed TensorFleet Vacuum Control no-map behavior, not
Valetudo UI. Therefore, this section uses only functional categories explicitly
called out in `goal.md` as reference-only categories. It does not describe a
layout to copy, does not require visual parity, and does not license copying
Valetudo styling, spacing, grouping, icons, or UX.

Reference-only functional categories:

- Basic cleaning controls: start, pause, resume, stop, return-to-dock/home.
- Robot status: activity/state, fault/error state, dock state, charging state.
- Battery: level and charging state.
- Cleaning settings: fan speed and water usage presets.
- Consumables/maintenance: main brush, side brush, filter, sensor cleaning, mop
  pad, detergent, dustbin or other consumables when a source reports them.
- Attachments/materials: dustbin, water tank, mop pad, detergent, and similar
  equipment indicators when normalized data exists.
- Map/area concepts: map availability, rooms, zones, segments, map metadata,
  go-to targets, and cleaning areas.
- Scheduling and settings: schedules, do-not-disturb, voice/speaker, Wi-Fi,
  firmware/updater, logs, model-specific settings.
- Advanced dock features: auto-empty, mop wash, mop dry, water refill.
- Statistics: current and total cleaning statistics.

## 3. Valetudo functional component comparison

This comparison uses Valetudo only as a functional inventory for capability
driven cards, runtime behavior, and sidebar composition. It does not copy
Valetudo UI/UX, layout, spacing, icons, grouping, or visual design.

Required product flow:

```text
VM-managed Valetudo integration runtime
  -> Valetudo backend adapter
  -> normalized vacuum_adapter snapshot/capabilities/commands
  -> VacuumControlPanel UI
```

| Valetudo functional component | TensorFleet classification | Normalized TensorFleet surface to use or create | Planning action before UI |
|---|---|---|---|
| `BasicControls` | Already normalized | Existing command capabilities and commands: `start_cleaning`, `pause`, `resume`, `stop`, `return_to_dock` | UI polish only. If state-aware messaging is needed, refine normalized `availabilityReason` / `reasons`; do not branch on Valetudo command names. |
| `RobotStatus` | Already normalized | Existing `identity`, `activity`, `battery`, `dock`, `fault`, `availability`, `health`, and `source` | Split the current summary into product-owned Robot Summary, Battery, Dock, Source/Health, and Fault/Attention cards. |
| `PresetSelectionControl` for fan | Already normalized | Existing `cleaningSettings.fanSpeed` plus `set_fan_speed` | UI polish only; options and current value already come through `vacuum_adapter`. |
| `PresetSelectionControl` for water | Already normalized | Existing `cleaningSettings.waterUsage` plus `set_water_usage` | UI polish only; options and current value already come through `vacuum_adapter`. |
| `PresetSelectionControl` for mode | Needs runtime/mock plus adapter extension | Add normalized cleaning/operation mode state, capability descriptor, allowed options, current value, and setter command if command semantics are safe | Define runtime snapshot fields and mock behavior first, then adapter contract/tests, then UI. Raw `OperationModeControlCapability` stays diagnostics-only. |
| `Dock` basic state | Already normalized | Existing `dock` state and `dock_state` capability | Safe for a basic Dock card now. |
| `Dock` components/actions | Needs runtime/mock plus adapter extension | Add normalized dock components such as auto-empty, mop wash, mop dry, water refill, component state, and dock action capabilities/commands | Define component model and safe command semantics before UI. Treat raw dock capability names as diagnostics-only. |
| `Attachments` | Needs runtime/mock plus adapter extension | Add normalized attachment/material state such as dustbin installed/full, water tank installed/level, mop attached, detergent state, and material readiness | Define attachment IDs, statuses, optional quantities, and mock scenarios before UI. Consumable life is not a substitute for attachment presence. |
| `CurrentStatistics` | Needs runtime/mock plus adapter extension | Add normalized current cleaning statistics such as duration, cleaned area, cleaned count/cycle, start/update timestamps, and optional estimated coverage metrics | Best first adapter expansion candidate because it is read-only, useful, testable in mock/runtime snapshots, and does not require map rendering. |
| `TotalStatistics` | Needs runtime/mock plus adapter extension | Add normalized lifetime/aggregate statistics separate from current-run statistics | Defer until current statistics shape is stable, unless runtime exposes both with the same low-risk mapper. |
| `LiveMapPage` / `BaseMap` | Deferred | Future normalized map data: supported map capability, map metadata, grid/layers, robot pose, segment/room/zone targets, and update semantics | Keep deferred. Do not render from raw Valetudo map payloads, source URLs, SSE/cache internals, or backend-specific geometry. |
| Valetudo HTTP initial fetch plus SSE/cache live updates | Needs larger design decision | TensorFleet runtime should own polling/event handling and expose stable snapshots to the adapter | UI should remain adapter-driven. If live updates are needed, bridge runtime push/poll changes into adapter snapshots rather than letting React call Valetudo routes. |
| Valetudo in-memory mock robot | Needs runtime/mock plus adapter extension when adding new surfaces | VM-managed fixed mock/runtime scenarios, adapter fixtures, and tests | Extend mock state transitions and fixtures for any new normalized statistics, attachments, dock components, or operation mode fields before product UI uses them. |

Category summary:

- Already normalized: BasicControls, Robot Summary/status slices, Battery,
  basic Dock state, Source/Health, Fault/Attention, fan speed, and water usage.
- Needs adapter-only extension: none of the requested Valetudo-inspired
  missing cards are adapter-only today because their source values are not yet
  present in the VM-managed runtime snapshot.
- Needs runtime/mock plus adapter extension: cleaning/operation mode, dock
  components/actions, attachments/materials, current statistics, and total
  statistics.
- Needs larger design decision: runtime live update strategy for Valetudo-like
  fetch/SSE/cache behavior and safe semantics for advanced dock commands.
- Deferred: full Valetudo map rendering, room/segment/zone/go-to map behavior,
  and any UI that depends directly on raw Valetudo map internals.

Recommended first implementation slice after this planning pass:

1. Add `CurrentStatistics` as a read-only normalized surface.
2. Extend the VM-managed Valetudo runtime fixed mock and runtime snapshot with
   current-run statistics.
3. Map those fields in the Valetudo backend adapter into
   `vacuum_adapter.snapshot.statistics.current` or an equivalent normalized
   surface with an explicit capability descriptor.
4. Add tests for missing statistics, fixed mock statistics, stale/unreachable
   source behavior, and UI gating before adding the card.

Attachments plus Dock Components should be the second candidate. It is valuable
but has more safety and semantics work because attachment presence, material
levels, auto-empty, mop wash, mop dry, and refill actions can affect hardware
state and operator expectations.

## 4. Actual mock/runtime data inventory

Runtime endpoints and source modes:

| endpoint / mode | real fields found | product status |
|---|---|---|
| `GET /api/v1/valetudo/health` | `runtime.{id,version,status}`, `source.{kind,status,stale,lastSeenAt}`, `updatedAt` | Runtime health source; normalized by adapter into `snapshot.health` and `snapshot.source`. |
| `GET /api/v1/valetudo/snapshot` | Full runtime snapshot: runtime, backend, robot, source, connectivity, state, optional battery, optional dock, optional cleaning settings, optional maintenance, command availability, capability diagnostics, diagnostics, raw diagnostics, updated timestamp | Primary adapter input. |
| `POST /api/v1/valetudo/command` | `{ok,status,command,message,reason?,code?,updatedAt,diagnostics?}` | Adapter maps to normalized command result and refreshes snapshot. |
| `fixed_mock` | Fixed runtime-owned state and capabilities without external source calls | Mock-backed product-ready source for no-map UI. |
| `valetudo_mock_http` | HTTP source at `VALETUDO_MOCK_SOURCE_URL`, `VALETUDO_SOURCE_URL`, or default `http://172.16.0.1:8081`; source kind normalizes as `valetudo_mock` | Mock HTTP path; product-ready only after runtime/adapter normalization. |
| `valetudo_http` | Real Valetudo HTTP source from required `VALETUDO_SOURCE_URL`; missing URL fails closed | First hardware validation path, not broad hardware support. |
| `valetudo_mock_mqtt` | MQTT cache from `$name`, `$nodes`, `StatusStateAttribute`, `BatteryStateAttribute`, and `DockStatusStateAttribute` topics | Mock MQTT snapshot path; transport remains diagnostics/internal. |
| rejected `valetudo_mqtt` mode | Falls back to mock HTTP and does not enable production MQTT through `VALETUDO_MQTT_ENABLED` | Prevents production MQTT claims. |

Runtime snapshot fields:

- `runtime`: `id`, `version`, `status`.
- `backend`: currently `"valetudo"`.
- `robot`: `id`, `name`.
- `source`: `kind`, `status`, `stale`, `lastSeenAt`.
- `connectivity`: `reachable`, `online`.
- `state`: `value`, `label`, `started`, `paused`.
- `battery`: optional `level`, `charging`.
- `dock`: optional `state`, `docked`.
- `cleaningSettings.fanSpeed`: optional `current`, `options`.
- `cleaningSettings.waterUsage`: optional `current`, `options`.
- `maintenance.consumables[]`: `id`, `label`, optional remaining percent,
  remaining minutes, used minutes, total minutes, `status`, `detail`.
- `capabilities.commands`: command availability records keyed by runtime
  command name.
- `capabilities.diagnostics[]`: raw capability detection and implementation
  notes.
- `diagnostics`: mode, raw capability names, source diagnostics, readiness,
  last command, capability tiers, transports, notes.
- `rawDiagnostics`: source/runtime detail such as base URL, mock source kind,
  MQTT root, broker URL, last topic, and similar internal data.
- `updatedAt`: runtime snapshot timestamp.

Fixed mock data:

- Robot identity: `valetudo-fixed-mock-001`, `Valetudo Fixed Mock`.
- Default state: `docked`, label `Docked`, not started, not paused, docked,
  not charging.
- Scenarios: default/docked, cleaning, paused, returning, charging,
  source stale, source unreachable, fault/maintenance warning.
- Battery: level `82` in fixed mock source code, with charging derived from
  scenario/state.
- Dock: `docked`, `returning`, or `available` from mock state.
- Fan speed: current `medium`; options `off`, `min`, `low`, `medium`, `high`,
  `turbo`, `max`.
- Water usage: current `medium`; options `off`, `min`, `low`, `medium`, `high`,
  `max`.
- Consumables: main brush, side brush, filter, sensor cleaning, mop pad, with
  remaining minutes, total minutes, computed percent, status, and detail.
- Raw capability names include implemented product-ready data plus many
  diagnostic-only Valetudo capability names.

HTTP source data:

- Runtime fetches `/api/v2/robot` for manufacturer, model name, model details,
  implementation, stable robot ID, and display name.
- Runtime fetches `/api/v2/robot/state/attributes` and reads
  `StatusStateAttribute`, `BatteryStateAttribute`, and
  `PresetSelectionStateAttribute`.
- Runtime fetches `/api/v2/robot/capabilities` for raw capability names.
- If `FanSpeedControlCapability` is present, runtime fetches
  `/api/v2/robot/capabilities/FanSpeedControlCapability/presets`.
- If `WaterUsageControlCapability` is present, runtime fetches
  `/api/v2/robot/capabilities/WaterUsageControlCapability/presets`.
- If `ConsumableMonitoringCapability` is present, runtime fetches
  `/api/v2/robot/capabilities/ConsumableMonitoringCapability` and optional
  `/properties`.
- Runtime does not fetch map payloads, segment geometry, zones, schedules,
  logs, Wi-Fi data, firmware data, voice settings, or dock advanced state into
  the adapter-facing snapshot.

Command availability and transitions:

- Supported runtime command names: `start_cleaning`, `pause`, `resume`, `stop`,
  `return_to_dock`, `set_fan_speed`, `set_water_usage`.
- Basic commands require `BasicControlCapability`.
- `set_fan_speed` requires `FanSpeedControlCapability`.
- `set_water_usage` requires `WaterUsageControlCapability`.
- Fixed mock `start_cleaning` changes state to `cleaning`.
- Fixed mock `pause` changes `cleaning` to `paused`.
- Fixed mock `resume` changes `paused` to `cleaning`.
- Fixed mock `stop` changes `cleaning`, `paused`, or `returning` to `idle`.
- Fixed mock `return_to_dock` changes to `returning`, then settles to `docked`
  on the next read.
- Invalid state returns `status:"failed"` and `code:"invalid_state"`.
- Missing capability returns `status:"unsupported"` and
  `code:"capability_unavailable"`.
- Source unreachable returns `status:"unavailable"` and
  `code:"source_unreachable"`.
- Stale MQTT command state returns `status:"unavailable"` and
  `code:"stale_source"`.
- Unknown commands return `status:"unsupported"` and
  `code:"unsupported_command"`.
- Invalid preset requests return `invalid_request` with `missing_value` or
  `invalid_value`.

Raw runtime capability inventory:

| raw Valetudo capability | runtime status | product status |
|---|---|---|
| `BasicControlCapability` | Implemented for start, pause, resume/start mapping, stop, home | Product-facing via normalized commands. |
| `BatteryStateCapability` | Implemented when battery exists | Product-facing via normalized battery state. |
| `FanSpeedControlCapability` | Implemented current/options/setter | Product-facing via normalized cleaning settings. |
| `WaterUsageControlCapability` | Implemented current/options/setter | Product-facing via normalized cleaning settings. |
| `ConsumableMonitoringCapability` | Runtime normalizes display status | Product-facing display only when consumable entries exist; reset actions absent. |
| `CurrentStatisticsCapability`, `TotalStatisticsCapability` | Detected in fixed mock raw names | Diagnostics-only. |
| `GoToLocationCapability` | Detected raw capability | Product workflow not implemented. |
| `MapSegmentationCapability` | Detected raw capability | Product segment target geometry and commands not implemented. |
| `ZoneCleaningCapability` | Detected raw capability | Product zone geometry and commands not implemented. |
| `MappingPassCapability`, `PersistentMapControlCapability`, `MapResetCapability`, `PendingMapChangeHandlingCapability` | Detected raw names | Diagnostics/deferred. |
| `AutoEmptyDockAutoEmptyIntervalControlCapability`, `AutoEmptyDockManualTriggerCapability` | Detected raw names | Diagnostics/deferred. |
| `MopDockCleanManualTriggerCapability`, `MopDockDryManualTriggerCapability` | Detected raw names | Diagnostics/deferred. |
| `CarpetModeControlCapability`, `CarpetSensorModeControlCapability`, `CollisionAvoidantNavigationControlCapability`, `ObstacleAvoidanceControlCapability`, `OperationModeControlCapability`, `PetObstacleAvoidanceControlCapability` | Detected raw names | Diagnostics/deferred. |
| `DoNotDisturbCapability`, `KeyLockCapability`, `LocateCapability`, `ManualControlCapability`, `SpeakerTestCapability`, `SpeakerVolumeControlCapability`, `VoicePackManagementCapability`, `WifiConfigurationCapability`, `WifiScanCapability` | Detected raw names | Diagnostics/deferred. |

## 5. Normalized adapter inventory

Shared `vacuum_adapter` snapshot fields currently available to product UI:

- `identity`: `{id,label,source,model?}`.
- `availability`: `{status,connected,detail?}`.
- `capabilities`: descriptor record for all shared capability names.
- `health`: runtime status, timestamp, detail.
- `source`: source kind/status/stale/last-seen/reason.
- `dock`: support flag, normalized dock state, charging, detail.
- `cleaningSettings`: `fanSpeed` and/or `waterUsage`, each with current value,
  normalized labeled options, readiness, status, detail.
- `maintenance`: consumable array with normalized display data.
- `diagnostics`: backend/runtime/source/capability/map/pose/navigation/mapping
  diagnostics, warnings, raw detail.
- `map`: readiness, receiving flag, detail, grid, metadata, annotations.
- `pose`: readiness, available flag, coordinates, detail.
- `navigation`: state, target, plan path, progress, active/sending/canceling.
- `activity`: normalized robot activity status, label, timestamp, source,
  reason, available actions, details.
- `mission`: legacy coarse state.
- `activeMission` and `missions`: runtime-owned mission snapshots.
- `mapping`: mapping state, saved maps, persistence, progress, errors.
- `readiness`: ready flag and blocking reasons.
- `fault`: readiness, fault messages, detail.
- `battery`: readiness, percentage, charging, detail.

Valetudo adapter values currently normalized:

- Identity from runtime robot ID/name.
- Availability from runtime connectivity.
- Runtime health from runtime status.
- Source kind/status/stale/last-seen/reason.
- Activity status from runtime state/source: unavailable, faulted, paused,
  returning, cleaning, charging, docked, idle.
- Activity available actions from normalized supported and available basic
  command capabilities.
- Battery percentage and charging when runtime reports battery.
- Dock state normalized to unknown, docked, undocked, returning, charging, or
  error.
- Faults from robot fault/error state, stale source, source unreachable,
  degraded runtime, and runtime offline last error.
- Basic command descriptors: `start_cleaning`, `pause`, `resume`, `stop`,
  `return_to_dock`, plus mission aliases `pause_mission`, `resume_mission`,
  `cancel_mission`.
- Fan speed and water usage descriptors, options, and setter commands.
- Consumables descriptor only when normalized maintenance entries exist.
- Active mission for non-idle hardware cleaning / return-to-dock state, with
  coarse status and actions.

Valetudo adapter values explicitly unavailable or diagnostics-only:

- `map`: always unavailable for Valetudo; `grid:null`,
  `metadata.hasMap:false`, no annotations.
- `pose`: unavailable; no product coordinates.
- `navigation`: idle/unsupported; go-to remains diagnostics-only.
- `mapping`: idle, persistence unsupported.
- `coverage_mission` and `start_coverage`: unsupported.
- `map_annotations`, `room_semantics`, `zone_semantics`, `room_cleaning`:
  unsupported.
- `segment_cleaning` and `zone_cleaning`: detected-not-ready when raw
  capability exists; still not product-ready.
- `manual_control`: unsupported for Valetudo.
- `retry_mission_step` and `skip_mission_step`: unsupported.
- Statistics, scheduling, DND, voice, Wi-Fi, firmware, logs, advanced dock,
  model settings, and consumable resets: no normalized public fields or
  commands yet.

## 6. Existing UI consumption inventory

Current Vacuum Control consumption points:

- `VacuumControlPanel` consumes `useVacuumAdapter`, not the raw Valetudo client.
- Adapter selector chooses `turtlebot4_nav2` or `valetudo`; product behavior
  then branches on normalized capabilities.
- Header and status strip use normalized identity, availability, source,
  health, activity, dock, battery, pose, map, readiness, and selected workflow
  state.
- `NoMapCanvasPlaceholder` renders when `capabilities.map.supported` is false.
- `BasicControlsCard` renders supported basic command descriptors and dispatches
  normalized `start_cleaning`, `pause`, `resume`, `stop`, and `return_to_dock`.
- `CleaningSettingsCard` renders `snapshot.cleaningSettings.fanSpeed` and
  `waterUsage` only when corresponding normalized capabilities are supported;
  it dispatches normalized `set_fan_speed` and `set_water_usage`.
- `MaintenanceCard` renders only normalized `snapshot.maintenance.consumables`
  when `capabilities.consumables.supported`.
- `BasicRobotStatusCard` uses normalized availability, health, source,
  activity, dock, battery, and fault.
- `MissionLifecycleCard` uses coarse mission, battery, dock state, and
  return-to-dock capability in map-supported workflow sections.
- `RecentMissionsCard` uses `snapshot.missions.recent`; Valetudo currently
  supplies no recent mission history.
- `UnsupportedFeatureCard` explains unavailable mapping/navigation/Clean
  Area/room-zone workflow capability notes.
- `MapCanvas` is mounted only when map is supported; it consumes adapter map and
  pose but also directly subscribes to ROS `/map`, costmaps, overlays, and
  camera through simulation-specific paths.
- `TeleopCard` directly publishes ROS `/cmd_vel_raw` and is gated by
  `manual_control`; Valetudo marks that unsupported.
- `CameraOverlay` is a ROS image topic surface inside `MapCanvas`, not a
  Valetudo product surface.

Current no-map behavior:

- Valetudo basic profile is detected when map, navigation, clean-area,
  rooms/zones, mapping, and manual-control capabilities are absent.
- For that profile, the main canvas shows a no-map placeholder instead of
  mounting `MapCanvas`.
- Sidebar shows Basic Cleaning, Cleaning Settings, Maintenance, and Robot
  status when normalized data supports them.
- Simulation-heavy components remain separate and hidden unless capabilities
  support them.

## 7. Capability-to-UI candidate matrix

| candidate card | purpose | current data availability | source | gate | safe next? | adapter work? | runtime/mock work? | notes and risks |
|---|---|---|---|---|---|---|---|---|
| Robot summary | Name, primary state, key detail | Available | Normalized adapter | `identity`, `activity` | Yes | No | No | Already partly exists; can be expanded without backend names. |
| Connection/source summary | Online, source status, stale reason | Available | Normalized adapter | `availability`, `source` | Yes | No | No | Keep transport URLs/topics out of product card. |
| Runtime health summary | Runtime online/degraded/offline | Available | Normalized adapter | `health` | Yes, if product-level | No | No | Avoid exposing runtime ID/version outside diagnostics. |
| Basic cleaning controls | Start/pause/resume/stop/home | Available | Normalized adapter | command capabilities | Yes | No | No | Already present; next work may polish state explanations. |
| Battery | Level and charging | Available when source reports battery | Normalized adapter | `battery` capability/state | Yes | No | No | Good candidate for a dedicated compact card. |
| Dock/charging | Docked/undocked/returning/charging | Available | Normalized adapter | `dock_state` | Yes | No | No | Dock state is inferred from runtime dock/battery. |
| Activity/state | Idle/cleaning/paused/returning/faulted | Available | Normalized adapter | `activity` | Yes | No | No | Product-safe. |
| Fault/error | Stale, unreachable, degraded, robot fault | Available | Normalized adapter | `fault_state` | Yes | No | No | Keep raw event payloads out. |
| Fan speed | Current preset and options | Available when runtime reports setting | Normalized adapter | `fan_speed` | Yes | No | No | Already present. |
| Water usage | Current preset and options | Available when runtime reports setting | Normalized adapter | `water_usage` | Yes | No | No | Already present. |
| Maintenance/consumables | Display remaining life | Available when runtime reports entries | Normalized adapter | `consumables` | Yes, display-only | No | No | No reset commands yet. |
| Main brush | Consumable line item | Available in fixed mock and HTTP source when present | Normalized adapter | `consumables` | Yes | No | No | Not guaranteed on every source. |
| Side brush | Consumable line item | Available in fixed mock and HTTP source when present | Normalized adapter | `consumables` | Yes | No | No | Handles side-left/right labels. |
| Filter | Consumable line item | Available in fixed mock and HTTP source when present | Normalized adapter | `consumables` | Yes | No | No | Display-only. |
| Sensor cleaning | Consumable line item | Available in fixed mock and HTTP source when present | Normalized adapter | `consumables` | Yes | No | No | Display-only. |
| Mop pad | Consumable line item | Available in fixed mock and HTTP source when present | Normalized adapter | `consumables` | Yes | No | No | Display-only. |
| Detergent | Label mapper exists | Only if source reports such consumable | Normalized adapter when present | `consumables` | Conditional | No | Maybe source fixture if desired | Do not show placeholder support. |
| Dustbin | Label mapper exists | Only if source reports such consumable | Normalized adapter when present | `consumables` | Conditional | No | Maybe source fixture if desired | Do not imply bin sensor state. |
| Water tank | Not currently a normalized field | Missing | Missing | none | No | Yes | Yes | Need attachment/material contract. |
| Mop state | Not currently normalized | Missing | Missing | none | No | Yes | Yes | Mop pad consumable is not mop mode/state. |
| Attachments | Not normalized | Missing | Missing | none | No | Yes | Yes | Needs generic attachment model. |
| Cleaning mode | Not normalized | Raw `OperationModeControlCapability` only | Raw Valetudo only | none | No | Yes | Yes | Deferred. |
| Operation mode | Raw capability detected | Raw Valetudo only | Raw Valetudo only | none | No | Yes | Yes | Deferred. |
| Current statistics | Raw capability detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Need runtime fields and adapter type. |
| Total statistics | Raw capability detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Need runtime fields and adapter type. |
| Unavailable workflows | Explain hidden advanced workflows | Available capability notes | Normalized adapter | unsupported descriptors | Yes | No | No | Good for narrow expectation-setting. |
| Map availability | Unsupported with detail | Normalized adapter | `map` | Yes | No | No | Only "not available", not map rendering. |
| Map metadata | `hasMap:false` only | Normalized adapter | `map` | No meaningful card | Yes | Yes | Product map needs grid/metadata. |
| Map rendering | No product map grid | Missing normalized data | Raw/mock only at best | `map.supported` | No | Yes | Yes | Do not build from raw Valetudo internals. |
| Rooms | Not normalized | Missing | Missing/raw segments only | `room_semantics` | No | Yes | Yes | Need target model. |
| Zones | Not normalized | Missing | Raw capability only | `zone_semantics` | No | Yes | Yes | Need geometry contract. |
| Segments | Raw capability only | Diagnostics-only | Raw Valetudo only | `segment_cleaning` detected-not-ready | No | Yes | Yes | Need segment targets and names. |
| Segment targets | Readiness says count 0 | Missing | Missing | none | No | Yes | Yes | No mock target list exists. |
| Go-to/location target | Raw capability detected | Unsupported | Raw Valetudo only | `go_to_location` detected-not-ready | No | Yes | Yes | Need pose/map/target contract and command routing. |
| Clean Area | Unsupported | Missing | Missing | `start_coverage` | No | Yes | Yes | Current implementation depends on map grid. |
| Scheduling | Not normalized | Missing | Missing/raw upstream possible | none | No | Yes | Yes | Deferred. |
| Do-not-disturb | Raw capability detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Deferred. |
| Voice/speaker | Raw capabilities detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Deferred. |
| Wi-Fi | Raw capabilities detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Deferred. |
| Firmware/updater | Not found in normalized data | Missing | Missing | none | No | Yes | Yes | Deferred. |
| Logs | Not found in normalized data | Missing | Missing | none | No | Yes | Yes | Deferred. |
| Advanced dock | Raw auto-empty/mop dock capabilities detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Needs model-specific safe command semantics. |
| Auto-empty | Raw auto-empty capabilities detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Deferred. |
| Mop wash/dry | Raw mop dock capabilities detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Deferred. |
| Water refill | Not currently found | Missing | Missing | none | No | Yes | Yes | Deferred. |
| Model-specific settings | Several raw config capabilities detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Avoid product UI until normalized. |

## 8. Long list of possible sidebar cards

Safe now with existing normalized mock-backed data:

- Robot Summary: identity label, primary normalized status, brief detail.
- Connection Summary: connection status and source reachable/stale state.
- Runtime/Source Health Summary: product-level runtime and source status.
- Basic Cleaning Controls: start, pause, resume, stop, return-to-dock.
- Battery Card: percentage and charging state.
- Dock Card: docked/undocked/returning/charging/error state.
- Activity Card: idle, cleaning, paused, returning, docked, charging, faulted.
- Cleaning Settings Card: fan speed and water usage preset controls.
- Maintenance Card: normalized consumable display.
- Fault Card: stale source, source unreachable, degraded runtime, robot error.
- Map Unavailable Card: explains no normalized map is available.
- Unavailable Workflows Card: explains navigation, Clean Area, rooms/zones, and
  segment workflows are deferred.

Possible now only as conditional rows inside safe cards:

- Main brush, side brush, filter, sensor cleaning, mop pad, detergent, dustbin:
  safe only when actual `maintenance.consumables[]` entries exist.
- Runtime degraded/offline details: safe if phrased as product health, not raw
  runtime IDs or transport endpoints.
- Source stale/unreachable reason: safe if using normalized `source.reason` and
  capability disabled reasons.

Needs adapter changes:

- Attachment status, dustbin installed/full, water tank installed, mop attached,
  mop active state, detergent tank state.
- Current statistics and total statistics.
- Consumable reset actions.
- Map metadata beyond "not available".
- Rooms, zones, segments, segment targets, go-to targets.
- Scheduling, do-not-disturb, locate, key lock, voice/speaker, Wi-Fi,
  firmware/updater, logs.
- Advanced dock cards: auto-empty, mop wash, mop dry, water refill.
- Model-specific settings and operation/carpet/obstacle/pet/collision settings.

Needs runtime/mock changes:

- Fetch and normalize map data, robot pose, segment geometry, named rooms,
  zones, go-to execution state, cleaning statistics, schedule data, DND state,
  voice/speaker state, Wi-Fi scan/config state, firmware/update/log state,
  advanced dock state, attachments, consumable reset endpoints, and
  model-specific settings.
- Add mock fixtures and tests for every new normalized field before product UI
  depends on it.

Should stay deferred:

- Anything that depends only on raw Valetudo capability names.
- Anything that would expose HTTP routes, MQTT topics, source URLs, broker URLs,
  or raw capability class names as product requirements.
- Any map/segment/zone/go-to implementation built directly from raw Valetudo
  internals without updating the adapter/runtime contract.
- Activity/history/recent-command cards unless explicitly approved later.
- Main-panel diagnostics panels.

## 9. Map feasibility

Current Valetudo product map feasibility: not ready.

Evidence:

- The Valetudo adapter returns `capabilities.map.supported=false`.
- The Valetudo adapter returns `snapshot.map.grid=null`.
- The Valetudo adapter returns `snapshot.map.metadata.hasMap=false`.
- The Valetudo adapter returns `snapshot.map.annotations=[]`.
- The Valetudo adapter diagnostics say map data is diagnostics-only until
  product map rendering is implemented.
- The runtime HTTP snapshot path fetches robot info, state attributes,
  capabilities, fan/water presets, and consumables; it does not fetch Valetudo
  map payloads into the adapter-facing snapshot.
- The runtime readiness summary keeps `segmentTargetCount=0`.
- Existing `MapCanvas` is simulation-heavy and subscribes to ROS `/map` and
  costmap topics; it is hidden for Valetudo no-map states.

Conclusion:

- Map UI cannot honestly be built for Valetudo without hardware or at least a
  runtime/mock/adapter map contract update.
- A "map unavailable" explanation can be improved now.
- Product map rendering from raw Valetudo map internals should not be
  implemented until the VM runtime exposes normalized map data and the adapter
  declares `capabilities.map.supported=true`.
- Valetudo map screenshots can be used only as a functional reference for
  categories such as map, rooms, zones, segments, and target selection. They are
  not a layout or UX source.

## 10. Exclusions for next UI pass

- No diagnostics panel in the main Vacuum Control panel.
- No activity/history/recent-command card unless the user later approves.
- No copied Valetudo UI/UX, layout, spacing, icons, grouping, or visual design.
- No raw Valetudo capability names in product controls.
- No HTTP/MQTT/source URL/broker/topic details in product controls.
- No hardware claims.
- No production MQTT claims.
- No map rendering, go-to, segment, room, zone, Clean Area, scheduling,
  advanced dock, voice, Wi-Fi, firmware, logs, or model settings without a
  normalized runtime/adapter contract.
- No consumable reset actions until runtime command semantics and adapter
  commands exist.

## 11. Recommended next implementation options

Ranked safe-now options using existing normalized data:

1. Split the current Robot status card into clearer Robot Summary, Battery, Dock,
   and Source/Health cards while keeping the same normalized inputs.
2. Improve Basic Cleaning disabled-state explanations using capability
   `availabilityReason` and structured reasons already present.
3. Keep Cleaning Settings as a dedicated card and refine fan/water option
   presentation without changing command behavior.
4. Keep Maintenance display-only, possibly grouping consumables by status while
   avoiding reset actions.
5. Improve the no-map/unavailable-workflows explanation so operators understand
   what is intentionally unsupported.

Needs adapter changes before UI:

- Statistics cards.
- Attachment/material cards.
- Consumable reset actions.
- Segment, room, zone, and go-to target cards.
- Schedule, DND, speaker, Wi-Fi, firmware, logs, advanced dock, and
  model-settings cards.

Needs runtime/mock changes before UI:

- Any value not present in `ValetudoRuntimeSnapshot`, including map data,
  segment targets, zones, statistics, schedules, attachment states, advanced
  dock states, logs, updater state, Wi-Fi scan/config, and voice settings.

Should stay deferred:

- Map rendering, segment cleaning, zone cleaning, room cleaning, Clean Area,
  go-to/navigation, advanced dock actions, production MQTT, and copied Valetudo
  parity work.

## 12. Validation

This is a docs/research pass only. Product code was not changed.

Required validation:

```sh
git diff --check
```

Full build/test suites are not required by this pass because no code changed.

## 13. Progress report update

`progress_report.md` was updated after this review. The report records:

- What changed.
- Which docs were updated.
- What data was inspected.
- Key findings.
- Validation performed.
- Remaining gaps.
- Next recommended step.

Future implementation runs must update `progress_report.md` every time.
