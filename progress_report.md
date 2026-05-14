# Runtime-Owned Vacuum Mission Architecture

## Progress Report — Runtime-Owned Mission Architecture

### 1. What changed

Implemented the follow-up fix so a new destination can be staged after clearing a terminal navigation snapshot.

The latest screenshot showed Clear destination working, but the next map click brought the canceled run back instead of staging a fresh target. The cause was that map-click target selection cleared the terminal snapshot dismissal key, so the runtime-hydrated canceled destination immediately overrode the new draft.

Changes in this pass:

- `VacuumControlPanel` now tracks a dismissed terminal navigation destination key.
- Clear destination still refuses to clear while a navigation mission is active.
- After a completed/canceled/failed navigation mission, Clear destination hides the terminal target, destination marker, route/progress display, and action-card target locally.
- Selecting a new destination no longer clears that dismissal; the fresh draft stays visible instead of being replaced by the canceled runtime target.
- Terminal route visuals now require a visible target, so a dismissed terminal mission leaves the map in idle/draftable state.
- Starting a new run clears the dismissal, so the next runtime mission can hydrate normally.
- No runtime mission execution state is cleared by this UI action.
- User validated the live webview behavior: Clear destination works, and a
  subsequent map click stages a new destination.
- Updated architecture docs to reflect runtime-owned TurtleBot4/Nav2
  navigation, mission snapshot hydration, and terminal destination dismissal.

### 2. Which mode this affects

- Mapping: unchanged in execution; mission action naming remains normalized to commands the adapter can dispatch.
- Navigation: fixed post-run local dismissal and fresh target staging after a canceled/completed/failed runtime-hydrated destination.
- Clean Area: unchanged; still frontend-orchestrated and still not compliant with the final runtime-owned architecture.
- Shared adapter/runtime architecture: unchanged; the runtime mission snapshot remains authoritative.

### 3. Ownership check

- Is this still owned by React/webview state?
  - Draft target selection is still React-owned before start.
  - The dismissed terminal-destination key is UI-only presentation state.
  - A new clicked target remains a UI draft until submitted.
  - Active navigation execution is not owned by React.
- Is this now owned by the runtime/backend?
  - Yes. The VM mission runtime owns the Nav2 goal and exposes the mission snapshot through `/vacuum_mission/status` and `/vacuum_mission/get_snapshot`.
- What state is the UI only rendering?
  - `snapshot.activeMission`, `snapshot.navigation.currentTarget`, `snapshot.navigation.active`, navigation progress, pose, map, readiness, and capabilities.
- What command does the UI submit?
  - Start navigation submits `start_navigation`.
  - Cancel submits `cancel_mission` when available, otherwise `cancel_navigation`.
  - Clear destination submits no robot command because it only dismisses terminal UI state after the mission is no longer active.

### 4. Webview close/reopen behavior

- mapping
  - Mapping continues in the existing VM mapping runtime. UI hydrates from adapter mapping status.
- navigation
  - Navigation continues in the VM mission runtime.
  - On reopen, the adapter queries `/vacuum_mission/get_snapshot` and maps the returned mission into destination, map marker, progress, and action card state.
  - If the reopened mission is terminal, the user can now Clear destination to dismiss the terminal destination locally without sending a backend command.
  - After dismissal, clicking the map stages a new draft destination even though the runtime still remembers the last terminal mission summary.
- clean area
  - Clean Area still depends on React for waypoint sequencing and coverage progress. Closing the webview during Clean Area is still a known architecture gap.

### 5. Real hardware compatibility check

- Does this expose TurtleBot4/Nav2 specifics to product UI?
  - No. The UI still consumes adapter snapshots, normalized navigation state, and capability flags.
- Does this require Nav2 waypoint sequencing as a public concept?
  - No.
- Can the same adapter shape be implemented by Valetudo later?
  - Yes. The UI action dismisses product-level terminal navigation display and does not depend on backend identity.
- What capability flags decide whether controls are shown/enabled?
  - `start_navigation`
  - `cancel_mission`
  - `cancel_navigation` as fallback
  - `mission_state`
  - existing readiness/map/pose/navigation fields
- What operations are explicitly unsupported?
  - TurtleBot4/Nav2 still reports `start_coverage`, `pause_mission`, `resume_mission`, `retry_mission_step`, and `skip_mission_step` unsupported for current coverage/runtime behavior.

### 6. Files changed

- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
  - Adds terminal navigation destination dismissal so Clear destination works on runtime-hydrated completed/canceled/failed runs.
  - Keeps the dismissal while staging a fresh draft target.
  - Treats terminal navigation visual state as idle when the terminal target has been dismissed.
- `VACUUM_STACK_PLAN.md`
  - Documents that TurtleBot4/Nav2 navigation is runtime-owned through
    `start_navigation`, `/vacuum_mission/status`, and
    `/vacuum_mission/get_snapshot`.
  - Clarifies that Clean Area is still the remaining frontend-owned mission
    execution gap.
- `extension.md`
  - Updates Layer 3 extension facts for runtime-owned navigation hydration and
    terminal destination dismissal.
- `steps.md`
  - Updates Layer 3 implementation notes from direct `go_to_location`
    navigation to runtime-owned `start_navigation` mission execution.
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/useTurtleBot4Nav2Adapter.ts`
  - Adds optimistic navigation mission snapshot after `start_navigation`.
  - Parses runtime mission snapshots from both topic payloads and service responses.
  - Queries `/vacuum_mission/get_snapshot` on mount and after mission commands.
  - Imports mission service constants used by the snapshot query path.
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/capabilityMapper.ts`
  - Adds `/vacuum_mission/get_snapshot`.
  - Treats mission state as runtime-backed when either the status topic or snapshot service exists.
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/stateMapper.ts`
  - Keeps hydrated runtime navigation missions as the source for navigation state and actions.
- `panels-standalone/src/vacuum-adapter/state.ts`
  - Adds mapping command names to normalized mission actions.
- `scripts/vacuum-adapter-regression.ts`
  - Covers hydrated runtime navigation mission state and action filtering.
- `/home/shane/firecracker-vm/assets/opt/ros2_ws/src/turtlebot4_firecracker_bringup/scripts/vacuum_mission_node.py`
  - Adds `/vacuum_mission/get_snapshot` runtime service.
- Live VM `root@172.16.0.10`
  - Updated `/opt/ros2_ws/src/turtlebot4_firecracker_bringup/scripts/vacuum_mission_node.py`.
  - Updated `/opt/ros2_ws/install/turtlebot4_firecracker_bringup/lib/turtlebot4_firecracker_bringup/vacuum_mission_node.py`.
  - Updated `/opt/ros2_ws/install/turtlebot4_firecracker_bringup/lib/turtlebot4_firecracker_bringup/vacuum_mission_node`.
  - Restarted `turtlebot4-navigation@jazzy.service`.

### 7. Tests / validation run

Passed:

```sh
bun run test:vacuum-adapter
npm run build:panels
git diff --check
python3 -c "import ast, pathlib; ast.parse(pathlib.Path('/home/shane/firecracker-vm/assets/opt/ros2_ws/src/turtlebot4_firecracker_bringup/scripts/vacuum_mission_node.py').read_text())"
sshpass -p root scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null /home/shane/firecracker-vm/assets/opt/ros2_ws/src/turtlebot4_firecracker_bringup/scripts/vacuum_mission_node.py root@172.16.0.10:/tmp/vacuum_mission_node.py
sshpass -p root ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null root@172.16.0.10 'systemctl restart turtlebot4-navigation@jazzy.service'
```

Live VM verification:

```text
/vacuum_mission/cancel
/vacuum_mission/get_snapshot
/vacuum_mission/start_navigation
turtlebot4-navigation@jazzy.service active
/vacuum_mission/get_snapshot returned activeMission:null while idle
bt_navigator active [3]
controller_server active [3]
```

Live webview validation:

```text
Clear destination hides the canceled terminal run.
Clicking the map after clearing stages a fresh destination.
Run again can use the fresh destination.
```

Known existing failure:

```sh
npx tsc --noEmit
```

The repo-wide type check still fails on unrelated unused variables:

- `packages/tensorfleet-auth/src/oauth-core.ts:178` `callbackBaseUrl`
- `packages/tensorfleet-auth/src/oauth-core.ts:202` `actualPort`

Also attempted:

```sh
python3 -m py_compile /home/shane/firecracker-vm/assets/opt/ros2_ws/src/turtlebot4_firecracker_bringup/scripts/vacuum_mission_node.py
```

That failed because Python tried to write `__pycache__` under a read-only filesystem path, so the no-write AST parse was used instead.

### 8. Remaining risks

- The Clear destination and fresh destination staging fixes passed
  build/regression checks and were validated in the live webview.
- Clean Area execution and authoritative progress are still React-owned.
- Mission history/recent summaries are still minimal.

### 9. Next recommended step

Move Clean Area route sequencing and coverage progress behind
`start_coverage` in the VM mission runtime.
