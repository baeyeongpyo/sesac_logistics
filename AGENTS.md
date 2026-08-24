# Agent Instructions

This project uses `llm-wiki-core` with an optional Agent OS overlay.

Before doing project work, read and follow this file when it exists:

```text
llm-wiki-core/templates/agent-os-agents.md
```

If you cannot read that file, ask the user to paste `llm-wiki-core/templates/agent-os-agents.md` into this `agents.md`/`AGENTS.md` file.

## Navigation pose GUI context

The interactive map editor is `tools/navigation_pose_gui.py`. It uses
`docs/navigation_layout_1to1.svg` as the coordinate source and the matching PNG
as the rendered background. Run it from this project with:

```bash
./tools/navigation_pose_gui.py
```

### Coordinate frame

- Units are centimetres.
- The origin is the lower-left corner of the main floor.
- +X points right and +Y points up.
- Yaw is in degrees, counter-clockwise from +X, normalized to `[-180, 180)`.
- Vehicle and pallet sticker `x_cm`/`y_cm` values are object-center poses.
- Path points A and B are the center of the forklift's **front face**, not its
  body center.

### Runtime data read order

Runtime files are derived live data, not canonical wiki truth. Treat them as
read-only while the GUI is running.

1. Read `runtime/navigation_pose_live.json` first for the latest sticker poses,
   attachment state, A/B poses, calculated path, and planner status.
2. Read `runtime/navigation_pose_sessions.json` for recording indices,
   comments, timestamps, durations, and event counts.
3. Read or filter `runtime/navigation_pose_events.jsonl` by `session_id` only
   when movement history is needed. Do not load a long JSONL file in full when
   a streaming/filtering command can answer the question.
4. Read `runtime/navigation_pose_events_trash.jsonl` or
   `runtime/navigation_pose_sessions_trash.jsonl` only when the user asks about
   deleted-session recovery.

The live snapshot is atomically replaced and may update about 16 times per
second. For a question about "current" state, use its newest `updated_at` value.
If the file is missing, ask the user to start the GUI.

### Sticker and attachment semantics

Each entry in `items` has `id`, `type`, `x_cm`, `y_cm`, `yaw_deg`,
`length_cm`, and `width_cm`. A pallet with `attached_to: <vehicle_id>` uses
`offset_x_cm`, `offset_y_cm`, and `offset_yaw_deg` in that vehicle's local
frame and moves rigidly with it. An unattached pallet has `attached_to: null`.

### Path semantics

Read path information under `path_planner` in the live snapshot.

- `point_a` and `point_b` are forklift front-face center coordinates.
- `point_a_yaw_deg` and `point_b_yaw_deg` are the required front directions.
- Each `path` entry is `[front_x_cm, front_y_cm, yaw_deg]`; `path_modes` aligns
  with its segments and uses `R`, `READY`, `TURN`, or `F`.
- From a work pose the planner may first reverse straight (`R`) until the full
  loaded/unloaded footprint can turn. `ready_pose` is the transition pose.
- After Ready it may turn/transit, but the final B approach is forward (`F`),
  heading-aligned, and reaches B at `point_b_yaw_deg` with the front face.
- Collision checks use oriented rectangles for the vehicle and attached
  pallets, sample the swept area between poses, apply `safety_margin_cm`, and
  avoid floor boundaries, fixed structures, and other stickers.
- Never describe a path as usable when `status` says it failed, is stale, or
  requires recalculation.

When discussing a route, report at least the A/B front poses, selected vehicle,
safety margin, planner status, path length/pose count when available, and the
specific obstacle or endpoint collision when planning failed.

### Forklift operation flow

Use this state sequence when interpreting or planning tasks:

1. Arrive at a dock/tag, center-align, and pick the pallet.
2. In `loaded` state, reverse straight until there is enough swept-area
   clearance to turn; that pose becomes `Ready`.
3. Deliver normal/fresh pallets to the innermost instructed available slot and
   enter the slot front-first, or pick a pallet from the fresh zone front-first.
4. After a fresh-zone pickup, reverse loaded until the next turn-safe `Ready`.
5. Travel to the instructed empty Y1-Y4 position and place the pallet front-first.
6. After every placement, reverse unloaded until turn-safe and transition to
   `Ready` again.

Do not reject a task merely because forward motion from the work pose is
blocked. First evaluate the straight reverse escape and its complete swept area.

## Vehicle ROS 2 access and DDS domains

Use the existing interactive Bash functions when inspecting the real vehicles.
When a non-interactive tool shell does not know these functions, invoke them as
`bash -ic 'ros21'` or `bash -ic 'ros22'`.

| Vehicle | Shell command | SSH host | Docker shell command | ROS domain |
| --- | --- | --- | --- | --- |
| Vehicle 1 | `ros21` | `intelions@192.168.100.38` | `cd "$HOME/docker" && exec ./exec_shell.sh` | `215` |
| Vehicle 2 | `ros22` | `intelions@192.168.100.35` | `cd "$HOME/docker" && exec ./exec_shell.sh` | `216` |

The `auto_dock`, `fork_controller`, and `tag_entity_mapper` nodes map DDS domain
`215` to vehicle 1 and `216` to vehicle 2 when their `vehicle` parameter is
zero. Set or verify the appropriate `ROS_DOMAIN_ID` when running ROS commands;
do not infer the vehicle from the SSH hostname alone.

The two vehicles use separate DDS domains. Therefore the absolute topic
`/fork/command` is intentionally shared as a name and does not need a
`/robot_N` prefix: each vehicle sees only the copy in its own DDS domain.
Vehicle-specific workflow, result, and status topics may still use the explicit
`/robot_1/...` or `/robot_2/...` form. A leading `/` makes a topic absolute, so
a ROS node namespace does not alter `/fork/command`.
