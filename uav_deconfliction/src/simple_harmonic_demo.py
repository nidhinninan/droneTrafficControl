import argparse
import time
import json
import os
import numpy as np
import threading
from pathlib import Path
from dataclasses import dataclass, asdict

# Internal imports
from data_models import Mission, Waypoint, Conflict
import conflict_detector
from geometry_utils import create_trajectory_curve, evaluate_trajectory_at_parameter

# --- 1. Load Data ---
def load_scenario(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)
    missions = []
    for m_data in data['missions']:
        wps = [Waypoint(**w) for w in m_data['waypoints']]
        missions.append(Mission(mission_id=m_data['mission_id'], waypoints=wps))
    return missions, data.get('safety_buffer_m', 5.0), data.get('temporal_tolerance_s', 2.0)

# --- 2. Conflict Check ---
def find_first_conflict(missions, safety_buffer, temporal_tol):
    # Check pair M001 vs M002
    if len(missions) < 2: return None
    
    # We use the existing conflict logic
    # Note: check_trajectory_conflict_with_temporal usually returns a list or None
    # Assuming it returns [Conflict(...)] or similar
    conflicts = conflict_detector.check_trajectory_conflict_with_temporal(
        missions[0], missions[1], 
        safety_buffer=safety_buffer, 
        temporal_tolerance=temporal_tol,
        num_samples=100
    )
    
    if conflicts:
        print(f"DEBUG: Found {len(conflicts)} potential conflicts.")
        # Return first one
        return conflicts[0]
    return None

# --- 3. Interpolation Helper ---
def interpolate_mission(mission, t_current, tck, u_knots, duration):
    if t_current > duration: 
        # Hold last position
        pos = evaluate_trajectory_at_parameter(tck, 1.0)
        return pos, 0, 0, 0 # x,y,z, r,p,y
        
    # Map time to u (0..1)
    # Simple linear map approximation between waypoints for this demo?
    # Or proper time-parameterization?
    # The provided geometry_utils might not have time-at-u. 
    # Let's do a simple ratio: u = t / duration  (Assumption: constant speed parameterization)
    # Or slightly better: find segment. 
    
    # Fast approach for demo: u = t_current / duration
    u = np.clip(t_current / duration, 0.0, 1.0)
    pos = evaluate_trajectory_at_parameter(tck, u)
    
    # No orientation logic in minimal demo (yaw=0)
    return pos, 0.0, 0.0, 0.0

# --- 4. Main Loop ---
def run_demo(scenario_path, rate_hz, speed_factor):
    missions, safe_buff, temp_tol = load_scenario(scenario_path)
    print(f"Loaded {len(missions)} missions. Buffer={safe_buff}m, Tol={temp_tol}s")
    
    # Prepare splines
    splines = {}
    durations = {}
    for m in missions:
        # geometry_utils.py needs list of Waypoint objects or dicts?
        # create_trajectory_curve takes [Waypoint]
        tck, u_knots = create_trajectory_curve(m.waypoints)
        splines[m.mission_id] = (tck, u_knots)
        durations[m.mission_id] = m.waypoints[-1].timestamp - m.waypoints[0].timestamp
        
    # Detect conflict
    conflict = find_first_conflict(missions, safe_buff, temp_tol)
    conflict_time = -1.0
    conflict_loc = None
    
    if conflict:
        # Conflict object has .time and .location (Waypoint or distinct fields)
        # Based on data_models.py: Conflict(mission_id_1, mission_id_2, location: Waypoint, time: float, ...)
        # Or maybe it's spatiotemporal?
        # Let's assume standard object structure.
        conflict_time = conflict.time
        conflict_loc = (conflict.location.x, conflict.location.y, conflict.location.z)
        print(f"\n!!! CONFLICT DETECTED !!!")
        print(f"Time: {conflict_time:.2f} s")
        print(f"Location: {conflict_loc}")
        print(f"Separation: {conflict.min_distance:.2f} m\n")
    else:
        print("No conflict detected.")

    # Setup /tmp files
    os.makedirs("/tmp/gazebo_drone_positions", exist_ok=True)
    drone_files = {m.mission_id: f"/tmp/gazebo_drone_positions/drone_{m.mission_id}.txt" for m in missions}
    marker_file = "/tmp/gazebo_drone_positions/conflict_marker.txt"
    
    # Playback Loop
    t_start = time.time()
    sim_time = 0.0
    
    print(f"Starting playback at {speed_factor}x speed...")
    try:
        while True:
            cycle_start = time.time()
            
            # Determine Color
            red_mode = (conflict_time >= 0 and sim_time >= conflict_time)
            
            # Write Drones
            for m in missions:
                tck, u_knots = splines[m.mission_id]
                dur = durations[m.mission_id]
                
                pos, r, p, y = interpolate_mission(m, sim_time, tck, u_knots, dur)
                
                # Color RGB
                # Normal: Mission specific (M001=Blue, M002=Cyan/Green)
                # Conflict: RED (1, 0, 0)
                if red_mode:
                    color = "1,0,0"
                else:
                    if m.mission_id == "M001": color = "0,0,1" # Blue
                    else: color = "0,1,1" # Cyan
                
                line = f"{sim_time},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},{r},{p},{y},{color}\n"
                
                with open(drone_files[m.mission_id], 'w') as f:
                    f.write(line)
            
            # Write Marker
            if red_mode and conflict_loc:
                # Move sphere to conflict location
                # Format: t, x, y, z, r, p, y, R, G, B
                # Keep it red
                marker_line = f"{sim_time},{conflict_loc[0]},{conflict_loc[1]},{conflict_loc[2]},0,0,0,1,0,0\n"
            else:
                # Hide it
                marker_line = f"{sim_time},0,0,-999,0,0,0,1,0,0\n"
            
            with open(marker_file, 'w') as f:
                f.write(marker_line)

            # Time advance
            elapsed = time.time() - cycle_start
            sleep_time = max(0, (1.0/rate_hz) - elapsed)
            time.sleep(sleep_time)
            
            sim_time += (1.0/rate_hz) * speed_factor
            
            # Loop wrap or end?
            if sim_time > 100.0: break # Demo limit usually ~90s
            
    except KeyboardInterrupt:
        print("Playback stopped.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="uav_deconfliction/data/demo_double_spiral.json")
    parser.add_argument("--rate_hz", type=float, default=30.0)
    parser.add_argument("--speed", type=float, default=1.0)
    args = parser.parse_args()
    
    run_demo(args.scenario, args.rate_hz, args.speed)
