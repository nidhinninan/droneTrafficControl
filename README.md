# UAV Deconfliction Project Walkthrough

I have successfully implemented the complete spatial **and temporal (4D)** deconfliction system for UAV trajectories.

## Project Structure
- `uav_deconfliction/`
    - `main.py`: **CLI Entry Point** for running scenarios and validating files.
    - `data/`:
        - `example_mission.json`: Sample mission file.
        - `example_traffic.json`: Sample traffic list.
    - `src/`:
        - `data_models.py`: Core classes (`Mission`, `Waypoint`, `Conflict`).
        - `geometry_utils.py`: B-spline math for smooth curves and **4D time interpolation**.
        - `conflict_detector.py`: Low-level conflict logic with safety buffers.
        - `mission_validator.py`: High-level interface for validation, reporting, and batch processing.
        - `test_data_generator.py`: Utilities to generate spiraling, crossing, and complex mission scenarios.
    - `tests/`: 
        - `test_validation.py`: Comprehensive automated test suite (spatial focus).
        - `test_temporal_logic.py`: **New** temporal logic verification suite.
    - `outputs/`: 
        - Visualization plots and test artifacts.

## CLI Usage
The system can be controlled via the command line using `main.py`.

### 1. Run Test Scenarios
Execute predefined scenarios to visualize system capabilities:
```bash
# Run a crossing conflict scenario
./uav_deconfliction/main.py --scenario crossing

# Run a complex random scenario
./uav_deconfliction/main.py --scenario complex --safety-buffer 10.0
```
Available scenarios: `crossing`, `parallel`, `altitude`, `complex`, `spiral`, `near_miss`.

### 2. Validate Custom Missions
Validate your own mission files against traffic data:
```bash
./uav_deconfliction/main.py \
    --validate uav_deconfliction/data/example_mission.json \
    --traffic uav_deconfliction/data/example_traffic.json \
    --output-dir my_results
```
Results (report JSON and 3D visualization) are saved to the specified output directory.

## System Capabilities
1.  **Curve-Based Trajectories**: Uses B-splines to model realistic, smooth flight paths.
2.  **Spatial Conflict Detection**: Accurately calculates the closest approach in 3D space.
3.  **Temporal Deconfliction (4D)**: Checks if spatial conflicts actually occur at the same time.
    - Calculates exact timestamp at point of closest approach.
    - Allows missions to cross spatially if separated by time.
4.  **Validation Reports**: detailed reports with spatiotemporal analysis.

## Automated Testing

### 1. Spatial Tests (`tests/test_validation.py`)
- `test_conflict_spatial_violation`: Detects 3D intersection.
- `test_conflict_parallel_too_close`: Verifies buffer sensitivity.
- `test_altitude_separation`: Confirms 3D height checking.
- `test_multiple_conflicts`: Batch validation logic.

### 2. Temporal Tests (`tests/test_temporal_logic.py`)
- `test_temporal_separation_clear`: Verifies that missions crossing in space but separated by time (e.g., 10s delay) are **APPROVED**.
- `test_spatiotemporal_conflict`: Verifies that missions crossing in space AND time are **REJECTED**.

All tests passed successfully.

## Visualization

### Matplotlib Animation (Recommended)

The primary visualization is a self-contained Matplotlib 3D animation that demonstrates the deconfliction system.

**Features:**
- Dynamic 3D playback of drone trajectories
- Green trails showing flight paths
- **Automatic red warning** when conflict is detected
- Conflict marker and timestamp displayed
- GIF export for easy sharing

**Usage:**
```bash
cd uav_deconfliction
python src/simple_viz.py
```

![Demo Animation](uav_deconfliction/outputs/demo_conflict.gif)

This will:
1. Load the scenario from `data/demo_double_spiral.json`
2. Detect conflicts using spatiotemporal analysis
3. Display an animated 3D plot
4. Export animation to `outputs/demo_conflict.gif`

---

### Gazebo Harmonic Integration (Experimental)

> [!WARNING]
> The Gazebo integration is currently **experimental and buggy**. Known issues include:
> - Trajectory visibility problems (drones may not render correctly)
> - Coordinate system mismatches between mission data and Gazebo world
> - Camera positioning issues making it difficult to view trajectories
> 
> **Recommendation:** Use the Matplotlib demo above for reliable visualization.

The Gazebo plugin files have been moved to `dev/gazebo_legacy/` for future development.

#### Original Setup (for reference)
1. **Compile the Plugin**:
   ```bash
   cd uav_deconfliction/gazebo_plugins/trajectory_player/build
   cmake .. && make
   export GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH}:$(pwd)
   cd ../../../..
   ```

2. **Run Simulation**:
   ```bash
   python3 uav_deconfliction/src/gazebo_bridge.py --missions uav_deconfliction/data/missions.json --generate-world demoworld.sdf
   python3 uav_deconfliction/src/gazebo_trajectory_player.py --world demoworld.sdf --missions uav_deconfliction/data/missions.json --speed 0.5
   ```