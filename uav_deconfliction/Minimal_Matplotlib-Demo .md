# Minimal Matplotlib Deconfliction Demo Plan

## Goal

Cut all Gazebo/bridge/test bloat out of the main path and deliver a **working, reproducible Matplotlib animation demo** that matches your reference plot behavior:

- **Dynamic playback** of two 3D trajectories
- **Green trails** (solid trails; optionally “particle-like” via small markers)
- **Conflict warning**: from the first detected spatiotemporal conflict time onward, both drones turn **red**
- **Conflict point + timestamp shown** on the plot, and a red sphere/marker appears at the conflict location
- **Export a GIF** for easy sharing

## Approach (lowest hanging fruit)

- Don’t touch simulator integration for now.
- Treat the demo as a visualization of **mission data**.
- Implement conflict detection in the simplest reliable way:
- Use your existing function `conflict_detector.check_trajectory_conflict_with_temporal` (already computes `conflict_time` and `conflict_location`).
- If you want *even less dependency on the deconfliction stack*, we can fall back to “same-index time” distance check; but default is to reuse your existing code so behavior aligns with your project.

## Repo cleanup strategy (park Gazebo work)

Move all Gazebo-related work into a new dev folder so it doesn’t block or confuse the Matplotlib path.

- Create folder:
- [`uav_deconfliction/dev/gazebo_legacy/`](uav_deconfliction/dev/gazebo_legacy/)

- Move (not delete) these files:
- [`uav_deconfliction/src/gazebo_trajectory_player.py`](uav_deconfliction/src/gazebo_trajectory_player.py) → `uav_deconfliction/dev/gazebo_legacy/gazebo_trajectory_player.py`
- [`uav_deconfliction/src/gazebo_bridge.py`](uav_deconfliction/src/gazebo_bridge.py) → `uav_deconfliction/dev/gazebo_legacy/gazebo_bridge.py`
- (optional) anything else Gazebo-specific you don’t want in the mainline

- Park the Gazebo tests:
- [`uav_deconfliction/tests/test_temporal_logic.py`](uav_deconfliction/tests/test_temporal_logic.py) → `uav_deconfliction/dev/gazebo_legacy/test_temporal_logic.py`

- Add a README explaining what’s parked and why:
- [`uav_deconfliction/dev/gazebo_legacy/README.md`](uav_deconfliction/dev/gazebo_legacy/README.md)

README should include:

- What these files were intended to do
- Why they’re parked (bugs / too much configuration)
- What would be needed to revive them (minimal checklist)
- A pointer to the Matplotlib demo as the current reference visualization

## Minimal data input (JSON)

Create a single scenario JSON file with hardcoded waypoints + timestamps representing the double-loop spiral conflict scenario.

- Add file:
- [`uav_deconfliction/data/demo_double_spiral.json`](uav_deconfliction/data/demo_double_spiral.json)

Minimal schema:

- `safety_buffer_m`: float (e.g. 5.0)
- `temporal_tolerance_s`: float (e.g. 1.0)
- `fps`: int (e.g. 20)
- `missions`: list of 2 missions
- `mission_id`
- `waypoints`: list of `{x,y,z,t}`

(We will generate these values once from the known spiral function so they’re *hardcoded* in JSON for reproducibility.)

## Minimal Matplotlib animation script

Add one small, single-purpose script:

- [`uav_deconfliction/src/simple_viz.py`](uav_deconfliction/src/simple_viz.py)

Responsibilities (keep it tight):

1. **Load JSON** and create `Mission`/`Waypoint` objects (reuse [`uav_deconfliction/src/data_models.py`](uav_deconfliction/src/data_models.py) types).
2. **Compute conflict** once:

- `conflict_detector.check_trajectory_conflict_with_temporal(m1, m2, safety_buffer_m, temporal_tolerance_s, num_samples)`
- Capture:
 - `conflict_time`
 - `conflict_location`
 - `separation_distance`

3. **Resample missions to a common animation timeline**:

- Create a uniform time grid based on `fps` and mission duration.
- Interpolate each mission position at time `t` with simple linear interpolation between waypoints (minimal, deterministic).
 - This avoids SciPy splines and keeps dependencies at `numpy + matplotlib`.
-