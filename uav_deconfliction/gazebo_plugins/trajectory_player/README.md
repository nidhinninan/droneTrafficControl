# Trajectory Player Gazebo Plugin (Harmonic)

## Build Instructions

### Prerequisites
- Gazebo Harmonic (`gz-harmonic` / `libgz-sim8-dev`)
- CMake

### Build Steps

1. Create build directory:
```bash
cd gazebo_plugins/trajectory_player
mkdir build
cd build
```

2. Configure and build:
```bash
cmake ..
make
```

3. Export Plugin Path:
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH}:$(pwd)
```

### Usage

The `gazebo_bridge.py --generate-world` command will add this plugin to the SDF file.
Update `gazebo_bridge.py` to ensure it generates `<plugin filename="trajectory_player_plugin">` compatible tags for Harmonic.
