"""
Gazebo Bridge Module

This module serves as the interface between the droneTrafficControl system and Gazebo visualization.
It converts validated Mission objects into time-sampled trajectory data suitable for playback.
It includes functionality to detect spatiotemporal conflicts and visually annotate them (e.g., turning red).

NOTE: This module handles VISUALIZATION ONLY. It does not perform physics simulation.
"""

import time
import subprocess
import argparse
import sys
import os
import signal
import atexit
import json
import shutil
import math
import xml.etree.ElementTree as ET
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np
import matplotlib.cm as cm

# Adjust path to import siblings if run as script
try:
    from data_models import Mission, Waypoint, Conflict
    import geometry_utils
    from mission_validator import validate_mission_with_temporal
    from test_data_generator import generate_temporally_separated_missions, generate_crossing_missions_3d
except ImportError:
    # Add src to path
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from data_models import Mission, Waypoint, Conflict
    import geometry_utils
    from mission_validator import validate_mission_with_temporal
    from test_data_generator import generate_temporally_separated_missions, generate_crossing_missions_3d

# ============================================================================
# DATA STRUCTURES
# ============================================================================

@dataclass
class TrajectoryPlaybackData:
    """
    Represents a time-sampled trajectory for Gazebo playback.
    
    Attributes:
        mission_id: Unique identifier for the mission.
        sampled_positions: List of (time, x, y, z) tuples.
        start_time: Absolute start time of the mission (seconds).
        end_time: Absolute end time of the mission (seconds).
        color: Base color (r, g, b) in 0-1 range.
        conflict_start_time: If set, the time at which a conflict begins.
                             Used to switch color to red during playback.
    """
    mission_id: str
    sampled_positions: List[Tuple[float, float, float, float]]
    start_time: float
    end_time: float
    color: Tuple[float, float, float]
    conflict_start_time: Optional[float] = None
    
    def get_position_at_time(self, t: float) -> Optional[Tuple[float, float, float]]:
        """
        Returns interpolated (x, y, z) at time t.
        Returns None if t is outside the mission duration.
        """
        if t < self.start_time or t > self.end_time:
            return None
            
        # Linear search for now (optimization: binary search)
        # Assuming sampled_positions are sorted by time
        for i in range(len(self.sampled_positions) - 1):
            t1, x1, y1, z1 = self.sampled_positions[i]
            t2, x2, y2, z2 = self.sampled_positions[i+1]
            
            if t1 <= t <= t2:
                # Interpolate
                ratio = (t - t1) / (t2 - t1) if (t2 - t1) > 0 else 0
                x = x1 + ratio * (x2 - x1)
                y = y1 + ratio * (y2 - y1)
                z = z1 + ratio * (z2 - z1)
                return (x, y, z)
                
        return None

    def get_color_at_time(self, t: float) -> Tuple[float, float, float]:
        """
        Returns the color to display at time t.
        If a conflict has started by time t, returns RED (1, 0, 0).
        Otherwise returns the base color.
        """
        if self.conflict_start_time is not None and t >= self.conflict_start_time:
            return (1.0, 0.0, 0.0) # RED
        return self.color

    def get_duration(self) -> float:
        return self.end_time - self.start_time

    def __repr__(self):
        return f"TrajectoryPlaybackData({self.mission_id}, dur={self.get_duration():.1f}s, conflict={self.conflict_start_time})"


# ============================================================================
# TRAJECTORY SAMPLING & DATASET CREATION
# ============================================================================

def sample_mission_trajectory(mission: Mission, sample_rate_hz: float = 20.0) -> TrajectoryPlaybackData:
    """
    Converts a Mission object into a discretely sampled trajectory.
    
    Mapping Strategy:
    - Time range [start_time, end_time] is sampled uniformly.
    - Each timestamp is mapped to the spline parameter u [0, 1].
    - Since spline u is not constantly linear with time (unless segments are equal duration),
      we must map time -> segment -> local_u -> global_u.
      
    For this implementation, we assume linear time progression between waypoints.
    """
    if len(mission.waypoints) < 2:
        raise ValueError(f"Mission {mission.mission_id} has insufficient waypoints")

    # Generate Spline
    tck, u_knots = geometry_utils.create_trajectory_curve(mission.waypoints)
    
    start_time = mission.start_time
    end_time = mission.end_time
    duration = end_time - start_time
    
    if duration <= 0:
         raise ValueError(f"Mission {mission.mission_id} has zero duration")

    # Generate sample times
    # np.arange excludes end, so we add a small epsilon or use linspace
    num_samples = int(duration * sample_rate_hz)
    times = np.linspace(start_time, end_time, num_samples)
    
    sampled_positions = []
    
    # Pre-calculate segment time boundaries for mapping
    # normalized_times = [(w.timestamp - start_time)/duration for w in mission.waypoints] # Not strictly linear in u if segments vary
    
    # Better approach: Map time t to spline u
    # We know u=0 at t=start, u=1 at t=end
    # The 'u' returned by splprep corresponds to the waypoints.
    # We can interpolate u for a given t based on the waypoint timestamps.
    wp_times = [w.timestamp for w in mission.waypoints]
    
    # Interpolate mapping from T -> U
    # This assumes the spline parameterization roughly follows the time distribution of waypoints
    # which is true for splprep unless weighted otherwise.
    u_samples = np.interp(times, wp_times, u_knots)
    
    for t, u in zip(times, u_samples):
        x, y, z = geometry_utils.evaluate_trajectory_at_parameter(tck, u)
        sampled_positions.append((float(t), x, y, z))
        
    return TrajectoryPlaybackData(
        mission_id=mission.mission_id,
        sampled_positions=sampled_positions,
        start_time=start_time,
        end_time=end_time,
        color=(1.0, 1.0, 1.0) # Default white, updated later
    )

def assign_mission_colors(playback_data_list: List[TrajectoryPlaybackData], colormap='rainbow'):
    """
    Assigns visually distinct colors to each trajectory object in place.
    """
    cmap = cm.get_cmap(colormap)
    num_missions = len(playback_data_list)
    
    for i, data in enumerate(playback_data_list):
        # Get color from colormap (returns r,g,b,a)
        # Use 0.8 scaling to avoid extremely light colors if using some maps
        idx = i / num_missions if num_missions > 1 else 0.5
        rgba = cmap(idx)
        data.color = (rgba[0], rgba[1], rgba[2])

def detect_conflicts_and_annotate(missions: List[Mission], playback_data_map: Dict[str, TrajectoryPlaybackData]):
    """
    Runs spatiotemporal validation to detect conflicts.
    If a conflict is found, updates the corresponding TrajectoryPlaybackData 
    with conflict_start_time.
    """
    # Simply check every pair. O(N^2) but N is small (num missions)
    for i, m1 in enumerate(missions):
        # Check against all others
        others = [m for j, m in enumerate(missions) if i != j]
        
        # Use existing validator with temporal check
        is_clear, conflicts, _ = validate_mission_with_temporal(
            m1, others, safety_buffer=5.0, temporal_tolerance=1.5
        )
        
        if not is_clear and conflicts:
            # Find earliest conflict time for this mission
            earliest_time = min(c.conflict_time for c in conflicts)
            
            # Update data object
            if m1.mission_id in playback_data_map:
                pb_data = playback_data_map[m1.mission_id]
                # If already set (from another pair), keep the earlier one
                if pb_data.conflict_start_time is None or earliest_time < pb_data.conflict_start_time:
                    pb_data.conflict_start_time = earliest_time


def create_playback_dataset(missions: List[Mission], sample_rate_hz: float = 20.0) -> List[TrajectoryPlaybackData]:
    """
    Orchestrates the creation of playback data from missions.
    - Samples trajectories
    - Detects conflicts
    - Assigns colors
    """
    pb_list = []
    pb_map = {}
    
    # 1. Sample
    for m in missions:
        try:
            pb = sample_mission_trajectory(m, sample_rate_hz)
            pb_list.append(pb)
            pb_map[m.mission_id] = pb
        except Exception as e:
            print(f"Skipping mission {m.mission_id}: {e}")
            
    # 2. Assign base colors
    assign_mission_colors(pb_list)
    
    # 3. Detect conflicts and mark start times
    detect_conflicts_and_annotate(missions, pb_map)
    
    # Sort by start time
    pb_list.sort(key=lambda x: x.start_time)
    
    print(f"Created playback data for {len(pb_list)} missions.")
    return pb_list

# ============================================================================
# FILE-BASED POSITION UPDATES
# ============================================================================

POSITION_DIR = "/tmp/gazebo_drone_positions"

def cleanup_position_files():
    """Clears communication directory."""
    if os.path.exists(POSITION_DIR):
        shutil.rmtree(POSITION_DIR)
    os.makedirs(POSITION_DIR, exist_ok=True)
    print(f"Cleaned position directory: {POSITION_DIR}")

def write_position_file(model_name: str, timestamp: float, pos: Tuple[float,float,float], color: Tuple[float,float,float]):
    """
    Writes position and color command to file.
    Format: timestamp, x, y, z, roll, pitch, yaw, r, g, b
    """
    filepath = os.path.join(POSITION_DIR, f"{model_name}.txt")
    # Using 'a' (append) or 'w' (overwrite)?
    # Gazebo plugin likely reads the last line. Overwriting is safer to prevent huge files.
    # But usually file locking might be an issue. Let's append but maybe truncate occasionally if needed.
    # For now, append is fine for short tests.
    
    # Simple orientation (0,0,0) for now. Ideally calculate from velocity.
    line = f"{timestamp:.3f},{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f},0,0,0,{color[0]:.2f},{color[1]:.2f},{color[2]:.2f}\n"
    
    with open(filepath, 'a') as f:
        f.write(line)

# ============================================================================
# PLAYBACK MANAGER
# ============================================================================

class GazeboPlaybackManager:
    """
    Manages the playback loop and Gazebo updates.
    """
    def __init__(self, dataset: List[TrajectoryPlaybackData], update_rate_hz: int = 30):
        self.dataset = dataset
        self.update_rate_hz = update_rate_hz
        self.dt = 1.0 / update_rate_hz
        
        # Determine global time range
        if not dataset:
            self.max_time = 0
        else:
            self.max_time = max(d.end_time for d in dataset) + 2.0 # +2s buffer
            
        self.sim_time = 0.0
        
    def run_playback(self, time_scale: float = 1.0, realtime: bool = True):
        print(f"Starting playback (Scale: {time_scale}x, Realtime: {realtime})")
        print(f"Duration: {self.max_time:.1f}s")
        
        start_wall_time = time.time()
        
        steps = 0
        try:
            while self.sim_time <= self.max_time:
                loop_start = time.time()
                
                # Update each drone
                active_count = 0
                for data in self.dataset:
                    pos = data.get_position_at_time(self.sim_time)
                    if pos:
                        active_count += 1
                        color = data.get_color_at_time(self.sim_time)
                        write_position_file(data.mission_id, self.sim_time, pos, color)
                
                # Progress logging
                if steps % (self.update_rate_hz * 2) == 0: # Every 2s
                    print(f"Sim Time: {self.sim_time:.1f}s | Active Drones: {active_count}")
                    
                # Advance time
                self.sim_time += self.dt * time_scale
                steps += 1
                
                if realtime:
                    elapsed = time.time() - loop_start
                    sleep_time = (self.dt / time_scale) - elapsed  # Assuming we sleep for wall-time equivalent
                    # Actually, if time_scale=2.0 (2x speed), we sleep for half the dt.
                    
                    target_loop_dur = self.dt / time_scale
                    sleep_time = target_loop_dur - elapsed
                    
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                        
        except KeyboardInterrupt:
            print("Playback interrupted by user.")
            
        print("Playback complete.")

# ============================================================================
# GAZEBO PROCESS HELPERS
# ============================================================================

GAZEBO_PROC = None

def launch_gazebo_with_world(world_path: str, headless: bool = False):
    global GAZEBO_PROC
    cmd = ["gzserver" if headless else "gazebo", "--verbose", world_path]
    print(f"Launching Gazebo: {' '.join(cmd)}")
    
    # Create a process group so we can kill it reliably
    GAZEBO_PROC = subprocess.Popen(cmd, preexec_fn=os.setsid)
    return GAZEBO_PROC

def kill_gazebo():
    global GAZEBO_PROC
    if GAZEBO_PROC:
        print("Terminating Gazebo...")
        try:
            os.killpg(os.getpgid(GAZEBO_PROC.pid), signal.SIGTERM)
            GAZEBO_PROC.wait(timeout=5)
        except:
            print("Force killing Gazebo...")
            os.killpg(os.getpgid(GAZEBO_PROC.pid), signal.SIGKILL)
        GAZEBO_PROC = None

# Register cleanup
atexit.register(kill_gazebo)

# ============================================================================
# MAIN & TESTS
# ============================================================================

def test_playback_collision_scenario(headless=True):
    """
    Runs the specific test requested:
    1. Generates safe (temporally separated) missions.
    2. Generates unsafe (colliding) missions.
    3. Plays them back to verify behavior (colliding ones turn red).
    """
    print("\n=== Setting up Collision Scenario Test ===")
    
    # 1. Generate Safe Data
    m1, m2 = generate_temporally_separated_missions()
    m1.mission_id = "Safe_A"
    m2.mission_id = "Safe_B"
    
    # 2. Generate Unsafe Data (shifted in time/space to occur after safe ones)
    m3, m4 = generate_crossing_missions_3d()
    
    # Shift unsafe missions to start at t=20s so they don't overlap with safe ones
    time_offset = 20.0
    spatial_offset = 50.0 # Move them away in X
    
    for w in m3.waypoints:
        w.timestamp += time_offset
        w.x += spatial_offset
    # Manually update start/end times since we modified waypoints after init
    m3.start_time = m3.waypoints[0].timestamp
    m3.end_time = m3.waypoints[-1].timestamp
    m3.mission_id = "Collision_X"

    for w in m4.waypoints:
        w.timestamp += time_offset
        w.x += spatial_offset
    m4.start_time = m4.waypoints[0].timestamp
    m4.end_time = m4.waypoints[-1].timestamp
    m4.mission_id = "Collision_Y"
    
    missions = [m1, m2, m3, m4]
    
    print("Missions generated:")
    for m in missions:
        print(f" - {m.mission_id}: {m.start_time:.1f}s to {m.end_time:.1f}s")

    # 3. Create Dataset (This runs conflict detection)
    print("\nProcessing dataset (Detecting conflicts)...")
    dataset = create_playback_dataset(missions)
    
    # Verify conflict detection
    for data in dataset:
        if data.mission_id in ["Collision_X", "Collision_Y"]:
            if data.conflict_start_time is None:
                print(f"WARNING: Expected conflict for {data.mission_id} NOT detected!")
            else:
                print(f"SUCCESS: Conflict detected for {data.mission_id} starting at {data.conflict_start_time:.2f}s")
    
    # 4. Prepare Files
    cleanup_position_files()
    
    # 5. Run Playback (Mock run without actual Gazebo if world not provided, or simply write files)
    print("\nStarting Playback Loop (writing position files)...")
    manager = GazeboPlaybackManager(dataset, update_rate_hz=30)
    
    # Run fast for test
    manager.run_playback(time_scale=10.0, realtime=True) # 10x speed
    
    print("\nTest Complete. check /tmp/gazebo_drone_positions/")

# ============================================================================
# WORLD GENERATION
# ============================================================================

@dataclass
class WorldGeneratorConfig:
    world_name: str = "drone_traffic_world"
    physics_update_rate: int = 100
    ground_plane_size: float = 500.0
    grid_enabled: bool = True
    grid_spacing: float = 10.0
    drone_model_uri: str = "model://quadrotor"
    camera_follow_mode: bool = False
    trail_visualization: bool = True
    trail_length_seconds: float = 5.0

class GazeboWorldGenerator:
    """
    Generates Gazebo .world XML files based on mission data.
    """
    def __init__(self, config: WorldGeneratorConfig = None):
        if config is None:
            self.config = WorldGeneratorConfig()
        else:
            self.config = config

    def generate_world_xml(self, playback_dataset: List[TrajectoryPlaybackData], output_path: str):
        """
        Generates the complete .world file.
        """
        sdf_root = ET.Element('sdf', version='1.9')
        world = ET.SubElement(sdf_root, 'world', name=self.config.world_name)

        # Gazebo Harmonic System Plugins
        # Physics
        p_physics = ET.SubElement(world, 'plugin', filename='gz-sim-physics-system', name='gz::sim::systems::Physics')
        
        # User Commands (for GUI interaction)
        p_user = ET.SubElement(world, 'plugin', filename='gz-sim-user-commands-system', name='gz::sim::systems::UserCommands')
        
        # Scene Broadcaster (essential for visualization)
        p_scene = ET.SubElement(world, 'plugin', filename='gz-sim-scene-broadcaster-system', name='gz::sim::systems::SceneBroadcaster')
        
        # Particle Emitter System (for trails)
        p_particles = ET.SubElement(world, 'plugin', filename='gz-sim-particle-emitter-system', name='gz::sim::systems::ParticleEmitter')

        # 1. Physics Configuration
        physics = ET.SubElement(world, 'physics', name='1ms', type='ignored')
        ET.SubElement(physics, 'max_step_size').text = '0.01'
        ET.SubElement(physics, 'real_time_factor').text = '1.0'
        ET.SubElement(physics, 'real_time_update_rate').text = str(self.config.physics_update_rate)

        # 2. Lighting (Sun) - Inline definition for Harmonic
        sun_light = ET.SubElement(world, 'light', type='directional', name='sun')
        ET.SubElement(sun_light, 'cast_shadows').text = 'true'
        ET.SubElement(sun_light, 'pose').text = '0 0 10 0 0 0'
        diffuse_light = ET.SubElement(sun_light, 'diffuse')
        diffuse_light.text = '0.8 0.8 0.8 1'
        specular_light = ET.SubElement(sun_light, 'specular')
        specular_light.text = '0.2 0.2 0.2 1'
        attenuation = ET.SubElement(sun_light, 'attenuation')
        ET.SubElement(attenuation, 'range').text = '1000'
        ET.SubElement(attenuation, 'constant').text = '0.9'
        ET.SubElement(attenuation, 'linear').text = '0.01'
        ET.SubElement(attenuation, 'quadratic').text = '0.001'
        direction = ET.SubElement(sun_light, 'direction')
        direction.text = '-0.5 0.1 -0.9'

        # 3. Ground Plane - Inline definition for Harmonic
        ground_model = ET.SubElement(world, 'model', name='ground_plane')
        ET.SubElement(ground_model, 'static').text = 'true'
        ground_link = ET.SubElement(ground_model, 'link', name='link')
        
        ground_collision = ET.SubElement(ground_link, 'collision', name='collision')
        ground_collision_geo = ET.SubElement(ground_collision, 'geometry')
        ground_plane_elem = ET.SubElement(ground_collision_geo, 'plane')
        ET.SubElement(ground_plane_elem, 'normal').text = '0 0 1'
        ET.SubElement(ground_plane_elem, 'size').text = '500 500'
        
        ground_visual = ET.SubElement(ground_link, 'visual', name='visual')
        ground_visual_geo = ET.SubElement(ground_visual, 'geometry')
        ground_visual_plane = ET.SubElement(ground_visual_geo, 'plane')
        ET.SubElement(ground_visual_plane, 'normal').text = '0 0 1'
        ET.SubElement(ground_visual_plane, 'size').text = '500 500'
        ground_visual_mat = ET.SubElement(ground_visual, 'material')
        ET.SubElement(ground_visual_mat, 'ambient').text = '0.8 0.8 0.8 1'
        ET.SubElement(ground_visual_mat, 'diffuse').text = '0.8 0.8 0.8 1'

        # 4. Reference Grid (Disabled - Makes file too large)
        # if self.config.grid_enabled:
        #     grid_model = self.generate_reference_grid()
        #     world.append(grid_model)

        # 5. Drones
        for data in playback_dataset:
            drone_model = self.generate_drone_model_spawn(data)
            world.append(drone_model)

        # 6. Camera
        camera_gui = self.generate_camera_config(playback_dataset)
        world.append(camera_gui)

        # Write to file
        tree = ET.ElementTree(sdf_root)
        ET.indent(tree, space="  ", level=0)
        
        # Ensure dir exists
        out_path_obj = Path(output_path)
        out_path_obj.parent.mkdir(parents=True, exist_ok=True)
        
        tree.write(output_path, encoding='utf-8', xml_declaration=True)
        print(f"Generated world file: {output_path}")

    def generate_reference_grid(self) -> ET.Element:
        """Creates a model with grid lines."""
        model = ET.Element('model', name='reference_grid')
        ET.SubElement(model, 'static').text = '1'
        
        # Determine number of lines
        half_size = self.config.ground_plane_size / 2
        count = int(self.config.ground_plane_size / self.config.grid_spacing)
        
        link = ET.SubElement(model, 'link', name='link')
        
        for i in range(count + 1):
            offset = -half_size + i * self.config.grid_spacing
            
            # X-line (Parallel to X)
            self._add_line_visual(link, f"line_x_{i}", 0, offset, 
                                  None, self.config.ground_plane_size, 0.05) # pos, len, thickness
            
            # Y-line (Parallel to Y)
            self._add_line_visual(link, f"line_y_{i}", offset, 0, 
                                  None, self.config.ground_plane_size, 0.05, rotate_90=True)
                                  
        return model

    def _add_line_visual(self, link_element, name, x, y, z, length, thickness, rotate_90=False):
        visual = ET.SubElement(link_element, 'visual', name=name)
        pose = ET.SubElement(visual, 'pose')
        if rotate_90:
             pose.text = f"{x} {y} 0.01 0 0 1.5708"
        else:
             pose.text = f"{x} {y} 0.01 0 0 0"
             
        geo = ET.SubElement(visual, 'geometry')
        box = ET.SubElement(geo, 'box')
        size = ET.SubElement(box, 'size')
        size.text = f"{length} {thickness} 0.001"
        
        mat = ET.SubElement(visual, 'material')
        # Harmonic prefers simple color tags or PBR
        # Using grey color
        ambient = ET.SubElement(mat, 'ambient')
        ambient.text = "0.5 0.5 0.5 1"
        diffuse = ET.SubElement(mat, 'diffuse')
        diffuse.text = "0.5 0.5 0.5 1"

    def generate_drone_model_spawn(self, data: TrajectoryPlaybackData) -> ET.Element:
        model = ET.Element('model', name=f"drone_{data.mission_id}")
        
        # Initial pose
        if data.sampled_positions:
            _, x, y, z = data.sampled_positions[0]
            ET.SubElement(model, 'pose').text = f"{x} {y} {z} 0 0 0"
        
        link = ET.SubElement(model, 'link', name='body')
        
        # Visual
        visual = ET.SubElement(link, 'visual', name='visual')
        geo = ET.SubElement(visual, 'geometry')
        # Fallback to Box since mesh is likely missing in this environment
        box = ET.SubElement(geo, 'box')
        size = ET.SubElement(box, 'size')
        size.text = "0.5 0.5 0.2" # Simple drone size
        # mesh = ET.SubElement(geo, 'mesh')
        # ET.SubElement(mesh, 'uri').text = self.config.drone_model_uri + "/meshes/quadrotor.dae"
        # Fallback to simple box if desired, but sticking to requested XML struct loosely
        # NOTE: If user doesn't have the mesh, this might fail to load visual.
        # Let's check for fallback in a more robust usage, but here we output as requested.
        
        # Color
        r, g, b = data.color
        mat = ET.SubElement(visual, 'material')
        ambient = ET.SubElement(mat, 'ambient')
        ambient.text = f"{r} {g} {b} 1"
        diffuse = ET.SubElement(mat, 'diffuse')
        diffuse.text = f"{r} {g} {b} 1"
        
        # Minimal Inertial
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = "0.01"
        inertia = ET.SubElement(inertial, 'inertia')
        ET.SubElement(inertia, 'ixx').text = "0.001"
        ET.SubElement(inertia, 'iyy').text = "0.001"
        ET.SubElement(inertia, 'izz').text = "0.001"
        
        # Trail (Particle Emitter)
        if self.config.trail_visualization:
            emitter = ET.SubElement(link, 'particle_emitter', name='trail_emitter', type='box')
            ET.SubElement(emitter, 'emitting').text = 'true'
            ET.SubElement(emitter, 'size').text = '0.1 0.1 0.1'
            ET.SubElement(emitter, 'particle_size').text = '0.1 0.1 0.1'
            ET.SubElement(emitter, 'lifetime').text = str(self.config.trail_length_seconds)
            ET.SubElement(emitter, 'min_velocity').text = '0.0'
            ET.SubElement(emitter, 'max_velocity').text = '0.0'
            ET.SubElement(emitter, 'scale_rate').text = '0'
            ET.SubElement(emitter, 'rate').text = '20' # Particles per second
            
            # Green Color
            mat = ET.SubElement(emitter, 'material')
            diffuse = ET.SubElement(mat, 'diffuse')
            diffuse.text = "0 1 0 1" # Green
            ambient = ET.SubElement(mat, 'ambient')
            ambient.text = "0 1 0 1"
            
        # Plugin
        # Plugin (Harmonic)
        # Filename is the shared library name (without lib prefix often works in Gz, but standard is libname.so or name)
        # We named the target `trajectory_player_plugin` in CMake.
        plugin = ET.SubElement(model, 'plugin', name='gazebo::TrajectoryPlayer', filename='trajectory_player_plugin')
        ET.SubElement(plugin, 'position_file').text = f"/tmp/gazebo_drone_positions/drone_{data.mission_id}.txt"
        ET.SubElement(plugin, 'update_rate').text = str(30)
        # Note: Gz Harmonic plugins often load by finding the library in GZ_SIM_SYSTEM_PLUGIN_PATH
        
        return model

    def generate_camera_config(self, dataset: List[TrajectoryPlaybackData]) -> ET.Element:
        gui = ET.Element('gui', fullscreen='0')
        camera = ET.SubElement(gui, 'camera', name='user_camera')
        
        # Calculate bounding box center
        all_x = []
        all_y = []
        all_z = []
        for d in dataset:
            for _, x, y, z in d.sampled_positions:
                all_x.append(x)
                all_y.append(y)
                all_z.append(z)
                
        if not all_x:
            cx, cy, cz = 0, 0, 0
        else:
            cx = (min(all_x) + max(all_x)) / 2
            cy = (min(all_y) + max(all_y)) / 2
            cz = (min(all_z) + max(all_z)) / 2
            
        # Offset camera
        cam_x = cx
        cam_y = cy - 80 # Back up
        cam_z = cz + 60 # Go up
        
        # Look at center (approximate via pitch)
        # Pitch down ~45 deg (0.78 rad)
        ET.SubElement(camera, 'pose').text = f"{cam_x} {cam_y} {cam_z} 0 0.6 1.57" 
        # Actually 1.57 yaw points +Y, 0 yaw points +X. Gazebo coords: X forward, Z up.
        # If we are at (0, -80, 60), we need to look +Y and -Z.
        # Yaw=1.57 (90deg) faces +Y. Pitch=0.6 faces down.
        
        ET.SubElement(camera, 'view_controller').text = 'orbit'
        
        if self.config.camera_follow_mode and dataset:
            track = ET.SubElement(camera, 'track_visual')
            ET.SubElement(track, 'name').text = f"drone_{dataset[0].mission_id}"
            ET.SubElement(track, 'use_model_frame').text = '1'
            ET.SubElement(track, 'min_dist').text = '10.0'
            ET.SubElement(track, 'max_dist').text = '50.0'

        return gui
        
def test_world_generation():
    print("=== Testing World Generation ===")
    config = WorldGeneratorConfig(world_name="test_world_gen")
    generator = GazeboWorldGenerator(config)
    
    # Generate Dummy Data
    m1, m2 = generate_temporally_separated_missions()
    m1.mission_id = "Gen_A"
    m2.mission_id = "Gen_B"
    missions = [m1, m2]
    dataset = create_playback_dataset(missions)
    
    output_path = "outputs/generated_worlds/test.world"
    generator.generate_world_xml(dataset, output_path)
    
    # Quick validate
    if os.path.exists(output_path):
        with open(output_path, 'r') as f:
            content = f.read()
            if "<sdf" in content and "Gen_A" in content:
                print("SUCCESS: World file looks valid.")
            else:
                print("FAILURE: World file missing expected content.")
    else:
        print("FAILURE: File not created.")

def main():
    parser = argparse.ArgumentParser(description="Gazebo Trajectory Playback")
    parser.add_argument('--world', help="Path to .world file (for playback)")
    parser.add_argument('--missions', help="Path to missions JSON")
    parser.add_argument('--test', action='store_true', help="Run built-in collision test")
    parser.add_argument('--test-world', action='store_true', help="Run world generation test")
    parser.add_argument('--generate-world', help="Output path for generated world file")
    parser.add_argument('--headless', action='store_true', help="Run Gazebo headless")
    parser.add_argument('--speed', type=float, default=1.0, help="Playback speed")
    
    args = parser.parse_args()
    
    if args.test:
        test_playback_collision_scenario(args.headless)
        return
        
    if args.test_world:
        test_world_generation()
        return

    # Normal Operation
    if args.missions and args.generate_world:
        # Load missions from JSON (TODO: Implement generic loader, for now just use generator for demo logic)
        print("Note: JSON loading not fully implemented in this demo script. Using generated missions.")
        m1, m2 = generate_temporally_separated_missions()
        dataset = create_playback_dataset([m1, m2])
        
        gen = GazeboWorldGenerator()
        gen.generate_world_xml(dataset, args.generate_world)
        return

    print("Use --test or --test-world to run demos.")

if __name__ == "__main__":
    main()
