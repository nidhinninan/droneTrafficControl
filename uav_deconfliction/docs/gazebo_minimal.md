# Minimal Harmonic Deconfliction Demo Plan

## Recommendation (cleanest/fastest demo)

Use **Gazebo Harmonic + the existing file-based `trajectory_player_plugin`** (already in this repo). It’s cleaner than ROS for your case because:

- No ROS stack, no message plumbing.
- The plugin already supports **live pose updates and per-tick color changes** (reads `r,g,b` from the file and writes to the model’s `Material`).
- We can implement the whole demo with **one `.world` file + one Python script**.

## Target outcome (your 3 base features)

- **Dynamic Playback**: Python streams poses over time from mission JSON.
- **Green Trails**: World uses a simple **persistent** `particle_emitter` or prefereably some type of solid line, that is set to green for each drone.
- **Conflict Warning**:
  - Compute spatiotemporal conflict via `conflict_detector.check_trajectory_conflict_with_temporal`.
  - From `conflict_time` onward, Python writes drone colors as **red** and the `particle_emitter` or solid line as **red**. Mark the the earlier and future drone (both the conflicting drones) trajectories also **red**.
  - A dedicated `conflict_marker` sphere is kept offscreen, then moved to `conflict_location` at `conflict_time`.

## Minimal architecture

```mermaid
flowchart TD
  missionsJson[demo_double_spiral.json] --> demoScript[simple_harmonic_demo.py]
  demoScript -->|compute_conflict_time_location| conflictDetector[conflict_detector.check_trajectory_conflict_with_temporal]
  demoScript -->|write_last_line_pose_color| positionFiles[/tmp/gazebo_drone_positions/*.txt]
  worldFile[double_spiral_demo.world] --> gzSim[gz sim]
  positionFiles --> trajectoryPlugin[trajectory_player_plugin]
  trajectoryPlugin --> gzSim
```

## Concrete repo changes (CUT OUT ALL FLUFF)

### 1) Add one JSON scenario (hardcoded waypoints)

Create a JSON file with exactly 2 missions, using the same double-loop spiral scenario as `uav_deconfliction/tests/test_conflict_detector.py`:

- Mission IDs: `M001`, `M002`
- Waypoints: the 15 spiral points for centers `(0,0,50)` and `(30,20,55)`
- Timestamps: `i*10` seconds

Add file:

- [`uav_deconfliction/data/demo_double_spiral.json`](uav_deconfliction/data/demo_double_spiral.json)

Schema (minimal):

- `safety_buffer_m`
- `temporal_tolerance_s`
- `missions: [{mission_id, waypoints: [{x,y,z,t}]}]`

### 2) Replace the bloated Gazebo scripts with 1 tiny demo script

Create a single script:

- [`uav_deconfliction/src/simple_harmonic_demo.py`](uav_deconfliction/src/simple_harmonic_demo.py)

Responsibilities (keep it small):

- **Load missions** from `demo_double_spiral.json` into `Mission`/`Waypoint`.
- **Detect conflict** once at startup:
  - Call `conflict_detector.check_trajectory_conflict_with_temporal(m1, m2, safety_buffer, temporal_tolerance, num_samples)`
  - Print: `conflict_time`, `conflict_location`, `separation_distance` (this satisfies “timestamp shown”).
- **Playback loop** (real-time-ish):
  - Use existing spline helpers for smooth motion:
    - `geometry_utils.create_trajectory_curve(waypoints)` and `geometry_utils.evaluate_trajectory_at_parameter(tck,u)`
    - Map time->u using waypoint timestamps + the spline’s `u_knots` (same idea as your current `sample_mission_trajectory`, but inline and minimal).
  - Every tick, write one line to:
    - `/tmp/gazebo_drone_positions/drone_M001.txt`
    - `/tmp/gazebo_drone_positions/drone_M002.txt`
  - File format already supported by plugin:
    - `timestamp,x,y,z,roll,pitch,yaw,r,g,b`
  - Color policy:
    - Before conflict: keep drone body colors distinct (e.g. blue for M001, green for M002).
    - After `conflict_time`: both drones written as **(1,0,0)**.
- **Conflict marker sphere**:
  - Add a file `/tmp/gazebo_drone_positions/conflict_marker.txt`.
  - Before conflict: keep marker far away (e.g. `z=-999`).
  - After conflict: move it to `conflict_location` and keep it there (always red).

CLI args (minimal):

- `--scenario uav_deconfliction/data/demo_double_spiral.json`
- `--rate_hz 30`
- `--speed 1.0`

### 3) Add a static Gazebo Harmonic world (no generator code)

Instead of a big “world generator”, commit one hand-written minimal `.world` file:

- [`uav_deconfliction/outputs/generated_worlds/double_spiral_demo.world`](uav_deconfliction/outputs/generated_worlds/double_spiral_demo.world)

Contents (minimal):

- Required Harmonic systems: Physics + SceneBroadcaster + UserCommands (+ ParticleEmitter system if needed for trails).
- `model drone_M001`:
  - simple `box` visual
  - `particle_emitter` configured green (your “green trails”)
  - plugin tag:
    - `name="gazebo::TrajectoryPlayer"`
    - `filename` set to the built library name
    - `position_file` `/tmp/gazebo_drone_positions/drone_M001.txt`
- `model drone_M002` similarly.
- `model conflict_marker`:
  - `sphere` visual, colored red
  - plugin tag reading `/tmp/gazebo_drone_positions/conflict_marker.txt`
  - sphere radius set to visually represent the buffer (e.g. radius = `safety_buffer_m / 2`)

### 4) Remove/retire the bloated files and tests

The goal is a demo, not a framework.

Actions:

- **Delete or archive** (move to `uav_deconfliction/src/legacy/`):
  - [`uav_deconfliction/src/gazebo_bridge.py`](uav_deconfliction/src/gazebo_bridge.py) (replace with the 1-file demo)
  - [`uav_deconfliction/src/gazebo_trajectory_player.py`](uav_deconfliction/src/gazebo_trajectory_player.py) (redundant)
- **Remove demo-only tests** (optional, but aligned with “cut fluff”):
  - [`uav_deconfliction/tests/test_temporal_logic.py`](uav_deconfliction/tests/test_temporal_logic.py)
  - Keep core tests for `conflict_detector` if you still want CI coverage.

## Build/run instructions (exact demo steps)

### Build plugin (only if not already built)

From [`uav_deconfliction/gazebo_plugins/trajectory_player`](uav_deconfliction/gazebo_plugins/trajectory_player):

- `mkdir -p build && cd build`
- `cmake .. && make -j`

### Set plugin path for Harmonic

Before launching Gazebo, export the plugin search path to include the build output containing `libtrajectory_player_plugin.so`:

- `export GZ_SIM_SYSTEM_PLUGIN_PATH=.../uav_deconfliction/gazebo_plugins/trajectory_player/build:$GZ_SIM_SYSTEM_PLUGIN_PATH`

(We’ll verify the exact filename string used in the `.world` plugin `filename=...` so Gazebo resolves it correctly.)

### Start Gazebo

- `gz sim -r uav_deconfliction/outputs/generated_worlds/double_spiral_demo.world`

### Run the playback writer

In another terminal:

- `python3 uav_deconfliction/src/simple_harmonic_demo.py --scenario uav_deconfliction/data/demo_double_spiral.json`

Expected:

- drones move along spiral trajectories
- green trail particles
- at printed `conflict_time`, drones turn red and the red sphere appears at conflict point

## Minimal acceptance criteria

- Running the two commands above yields motion in Gazebo.
- Console prints one line like:
  - `CONFLICT at t=...s loc=(x,y,z) sep=...m`
- Visual: drones become red from that time onward, and conflict sphere appears.

## Notes on scope / compromises (intentional)

- “Trajectory turns red” is implemented as **drone body color turns red**. Trails remain green (as requested) and are not dynamically recolored (would require extra systems/plugins).
- The conflict marker is not “spawned” via Gazebo services; it is pre-placed offscreen and moved at conflict time (same visual result, far less code).

## TO_DO LIST
- Create `uav_deconfliction/data/demo_double_spiral.json` with hardcoded double-spiral waypoints + timestamps for M001/M002, plus safety_buffer and temporal_tolerance.
- Add `uav_deconfliction/outputs/generated_worlds/double_spiral_demo.world` with two drone models + green particle emitters + TrajectoryPlayer plugin tags, plus a conflict_marker sphere model wired to its own position file.
- Implement `uav_deconfliction/src/simple_harmonic_demo.py` to load JSON, compute conflict time/location using `conflict_detector.check_trajectory_conflict_with_temporal`, stream pose+color to /tmp position files, and move the conflict marker at conflict time.
- Delete or move to `src/legacy/` the bloated `gazebo_bridge.py` and `gazebo_trajectory_player.py` and remove demo-only tests like `tests/test_temporal_logic.py` (or mark them skipped) to keep the repo minimal for this demo.