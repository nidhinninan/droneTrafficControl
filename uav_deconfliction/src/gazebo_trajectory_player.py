"""
Gazebo Trajectory Player
Controls drone position updates in Gazebo without requiring full ROS installation.
Runs alongside Gazebo and updates drone positions based on simulation time.
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
from typing import List, Tuple, Optional, Dict
from pathlib import Path

# Try importing from sibling module
try:
    from gazebo_bridge import TrajectoryPlaybackData, create_playback_dataset
    from test_data_generator import generate_temporally_separated_missions
except ImportError:
    # Add src to path
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from gazebo_bridge import TrajectoryPlaybackData, create_playback_dataset
    from test_data_generator import generate_temporally_separated_missions

# ============================================================================
# CONSTANTS & FILE IO
# ============================================================================

POSITION_DIR = "/tmp/gazebo_drone_positions"

def cleanup_position_files():
    """Clears communication directory."""
    if os.path.exists(POSITION_DIR):
        shutil.rmtree(POSITION_DIR)
    os.makedirs(POSITION_DIR, exist_ok=True)
    print(f"Cleaned position directory: {POSITION_DIR}")

def write_position_file(model_name: str, timestamp: float, pos: Tuple[float,float,float], 
                        orientation: Tuple[float,float,float] = (0,0,0), 
                        color: Tuple[float,float,float] = (1,1,1)):
    """
    Writes position and color command to file.
    Format: timestamp, x, y, z, roll, pitch, yaw, r, g, b
    """
    filepath = os.path.join(POSITION_DIR, f"{model_name}.txt")
    
    # Append to file
    line = f"{timestamp:.3f},{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}," \
           f"{orientation[0]:.3f},{orientation[1]:.3f},{orientation[2]:.3f}," \
           f"{color[0]:.2f},{color[1]:.2f},{color[2]:.2f}\n"
    
    try:
        with open(filepath, 'a') as f:
            f.write(line)
    except IOError as e:
        print(f"Error writing to {filepath}: {e}")

# ============================================================================
# DRONE CONTROLLER
# ============================================================================

class DronePositionController:
    """
    Manages position updates for one drone.
    """
    def __init__(self, model_name: str, trajectory_data: TrajectoryPlaybackData):
        self.model_name = model_name
        self.trajectory_data = trajectory_data
        self.is_active = False
        print(f"Initialized controller for {model_name}")

    def update(self, sim_time: float):
        """
        Updates drone position if mission is active at this time.
        """
        # Check start
        if sim_time < self.trajectory_data.start_time:
            return

        # Check end
        if sim_time > self.trajectory_data.end_time:
            if self.is_active:
                print(f"Mission {self.model_name} completed at {sim_time:.1f}s")
                self.is_active = False
                # Optional: Move to holding area or stop updating
            return

        # Activate if just starting
        if not self.is_active:
            print(f"Mission {self.model_name} started at {sim_time:.1f}s")
            self.is_active = True

        # Get state
        pos = self.trajectory_data.get_position_at_time(sim_time)
        color = self.trajectory_data.get_color_at_time(sim_time)
        
        if pos:
            # TODO: Calculate orientation from velocity vector if available
            orientation = (0.0, 0.0, 0.0) 
            self.update_gazebo_model_pose(pos, orientation, color)

    def update_gazebo_model_pose(self, position, orientation, color):
        # File-based approach (Option B)
        write_position_file(self.model_name, 0.0, position, orientation, color) # Timestamp in file content not strictly used by some readers, but good to have

# ============================================================================
# PLAYBACK MANAGER
# ============================================================================

class GazeboPlaybackManager:
    """
    Orchestrates multiple drone controllers and simulation time.
    """
    def __init__(self, playback_dataset: List[TrajectoryPlaybackData], update_rate_hz: int = 30):
        self.drone_controllers = []
        for data in playback_dataset:
            # Model name in Gazebo usually matches mission_id or likely "drone_{mission_id}"
            # Based on world generator, it is "drone_{mission_id}"
            model_name = f"drone_{data.mission_id}" # or data.mission_id depending on how it's spawned
            # But wait, world generator uses "drone_{mission_id}".
            # Let's align with that.
            # However, if we are using just the raw mission_id, we should be careful.
            # Let's assume the user wants flexible mapping, but for now strict 1:1.
            ctrl = DronePositionController(model_name, data)
            self.drone_controllers.append(ctrl)
            
        self.update_rate_hz = update_rate_hz
        self.sim_time = 0.0
        self.drone_controllers_map = {c.model_name: c for c in self.drone_controllers}

    def get_max_mission_duration(self) -> float:
        if not self.drone_controllers:
            return 0.0
        return max(c.trajectory_data.end_time for c in self.drone_controllers) + 2.0

    def run_playback(self, time_scale: float = 1.0, realtime: bool = True):
        max_time = self.get_max_mission_duration()
        print(f"Starting playback. Duration: {max_time:.1f}s, Speed: {time_scale}x")
        
        dt = 1.0 / self.update_rate_hz
        
        try:
            while self.sim_time <= max_time:
                loop_start = time.time()
                
                # Update all
                for ctrl in self.drone_controllers:
                    ctrl.update(self.sim_time)
                    
                # Advance time
                self.sim_time += dt * time_scale
                
                # Log
                if int(self.sim_time / time_scale * 10) % 20 == 0: # Roughly every 2s equivalent
                     pass # Too spammy if fast, maybe just print every N wall seconds
                
                if int(self.sim_time) % 5 == 0 and abs(self.sim_time - int(self.sim_time)) < (dt*time_scale):
                     print(f"Sim Time: {self.sim_time:.1f}s")

                if realtime:
                    elapsed = time.time() - loop_start
                    sleep_time = (dt / time_scale) - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                        
        except KeyboardInterrupt:
            print("Playback paused/stopped by user.")
            
        print("Playback finished.")

# ============================================================================
# GAZEBO PROCESS
# ============================================================================

GAZEBO_PROC = None

def launch_gazebo_with_world(world_path: str, headless: bool = False):
    global GAZEBO_PROC
    # Gazebo Harmonic command: gz sim [options] [file]
    # -r: run immediately
    # -s: server only (headless)
    cmd = ["gz", "sim", "-r"]
    if headless:
        cmd.append("-s")
    cmd.append(world_path)
    
    print(f"Launching Gazebo Harmonic: {' '.join(cmd)}")
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
            os.killpg(os.getpgid(GAZEBO_PROC.pid), signal.SIGKILL)
        GAZEBO_PROC = None

atexit.register(kill_gazebo)

# ============================================================================
# MAIN & TEST
# ============================================================================

def test_playback_simple():
    print("=== Testing Simple Playback ===")
    
    # Generate dummy missions
    m1, m2 = generate_temporally_separated_missions()
    m1.mission_id = "Test_A"
    
    # Create dataset
    dataset = create_playback_dataset([m1])
    
    # Clean files
    cleanup_position_files()
    
    # Init manager
    manager = GazeboPlaybackManager(dataset, update_rate_hz=30)
    
    # Run short burst
    print("Running 5s simulation...")
    manager.sim_time = 0.0
    # Override max duration for test
    original_get_max = manager.get_max_mission_duration
    manager.get_max_mission_duration = lambda: 5.0
    
    manager.run_playback(time_scale=10.0, realtime=True)
    
    # Check output
    expected_file = os.path.join(POSITION_DIR, f"drone_{m1.mission_id}.txt")
    if os.path.exists(expected_file):
        print(f"SUCCESS: Position file created: {expected_file}")
    else:
        print(f"FAILURE: Position file missing: {expected_file}")

def main():
    parser = argparse.ArgumentParser(description="Gazebo Trajectory Player")
    parser.add_argument('--world', help="Path to .world file")
    parser.add_argument('--missions', help="Path to missions JSON")
    parser.add_argument('--test', action='store_true', help="Run unit test")
    parser.add_argument('--headless', action='store_true', help="Run Gazebo headless")
    parser.add_argument('--speed', type=float, default=1.0, help="Playback speed")
    
    args = parser.parse_args()
    
    if args.test:
        test_playback_simple()
        return

    if args.world and args.missions:
        # 1. Load missions
        # TODO: Proper JSON loader. For now, we need at least one mission to run.
        # If user provides JSON, we assume it matches our internal schema or we mock it for demo.
        print("Loading missions...")
        # Mock load for demo capability if real JSON not ready
        # In real usage, we'd parse args.missions
        
        # 2. Prepare files
        cleanup_position_files()
        
        # 3. Launch Gazebo
        launch_gazebo_with_world(args.world, args.headless)
        
        # 4. Wait for it (naive sleep)
        print("Waiting 10s for Gazebo to load...")
        time.sleep(10)
        
        # 5. Playback
        # NOTE: Without real missions loaded, we can't play. 
        # Assuming for this step we just want the structure.
        print("Playback logic requires loaded missions. (Not implemented in this shell)")
        
        # Keep alive
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
            
        kill_gazebo()
        return
        
    print("Use --test to verify.")

if __name__ == "__main__":
    main()
