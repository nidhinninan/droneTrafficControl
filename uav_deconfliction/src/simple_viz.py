#!/usr/bin/env python3
"""
Simple Matplotlib Visualization for UAV Deconfliction Demo

A minimal, self-contained script that:
1. Loads mission data from JSON
2. Detects spatiotemporal conflicts using the existing conflict detector
3. Animates two drones in 3D with green trails
4. Changes color to red when conflict occurs
5. Exports animation as GIF

Dependencies: numpy, matplotlib (no scipy required for animation)
"""

import json
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path

# Add src to path for imports
src_dir = Path(__file__).parent
sys.path.insert(0, str(src_dir))

from data_models import Mission, Waypoint, Conflict
import conflict_detector


# =============================================================================
# JSON LOADING
# =============================================================================

def load_scenario(json_path: str):
    """Load mission data from JSON file."""
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    missions = []
    for m_data in data['missions']:
        waypoints = [Waypoint(**wp) for wp in m_data['waypoints']]
        missions.append(Mission(mission_id=m_data['mission_id'], waypoints=waypoints))
    
    return {
        'missions': missions,
        'safety_buffer': data.get('safety_buffer_m', 5.0),
        'temporal_tolerance': data.get('temporal_tolerance_s', 1.5),
        'fps': data.get('fps', 20)
    }


# =============================================================================
# LINEAR INTERPOLATION (numpy only, no scipy)
# =============================================================================

def interpolate_position(waypoints: list, t: float):
    """
    Linear interpolation between waypoints at time t.
    Returns (x, y, z) position.
    """
    # Clamp to mission time bounds
    t_start = waypoints[0].timestamp
    t_end = waypoints[-1].timestamp
    t = np.clip(t, t_start, t_end)
    
    # Find the segment containing t
    for i in range(len(waypoints) - 1):
        wp1, wp2 = waypoints[i], waypoints[i + 1]
        if wp1.timestamp <= t <= wp2.timestamp:
            # Linear interpolation within segment
            dt = wp2.timestamp - wp1.timestamp
            if dt == 0:
                return (wp1.x, wp1.y, wp1.z)
            alpha = (t - wp1.timestamp) / dt
            x = wp1.x + alpha * (wp2.x - wp1.x)
            y = wp1.y + alpha * (wp2.y - wp1.y)
            z = wp1.z + alpha * (wp2.z - wp1.z)
            return (x, y, z)
    
    # Fallback to last waypoint
    return (waypoints[-1].x, waypoints[-1].y, waypoints[-1].z)


def generate_full_trajectory(waypoints: list, num_points: int = 100):
    """Generate full trajectory points for trail visualization."""
    t_start = waypoints[0].timestamp
    t_end = waypoints[-1].timestamp
    times = np.linspace(t_start, t_end, num_points)
    
    positions = [interpolate_position(waypoints, t) for t in times]
    return np.array(positions)


# =============================================================================
# CONFLICT DETECTION
# =============================================================================

def detect_conflict(missions: list, safety_buffer: float, temporal_tolerance: float):
    """
    Use existing conflict detector to find the first conflict.
    Returns conflict object or None.
    """
    if len(missions) < 2:
        return None
    
    conflict = conflict_detector.check_trajectory_conflict_with_temporal(
        missions[0], missions[1],
        safety_buffer=safety_buffer,
        temporal_tolerance=temporal_tolerance,
        num_samples=200
    )
    
    return conflict


# =============================================================================
# 3D ANIMATION
# =============================================================================

def create_animation(scenario: dict, output_path: str = None):
    """
    Create 3D animation showing drone trajectories with conflict detection.
    """
    missions = scenario['missions']
    safety_buffer = scenario['safety_buffer']
    temporal_tolerance = scenario['temporal_tolerance']
    fps = scenario['fps']
    
    # Detect conflict
    conflict = detect_conflict(missions, safety_buffer, temporal_tolerance)
    conflict_time = conflict.conflict_time if conflict else float('inf')
    conflict_loc = conflict.conflict_location if conflict else None
    
    if conflict:
        print(f"\n{'='*60}")
        print("⚠️  CONFLICT DETECTED!")
        print(f"{'='*60}")
        print(f"  Time:       {conflict_time:.2f} seconds")
        print(f"  Location:   ({conflict_loc[0]:.1f}, {conflict_loc[1]:.1f}, {conflict_loc[2]:.1f})")
        print(f"  Separation: {conflict.separation_distance:.2f} m (buffer: {safety_buffer} m)")
        print(f"{'='*60}\n")
    else:
        print("\n✅ No conflict detected - trajectories are safe.\n")
    
    # Calculate time bounds
    all_times = []
    for m in missions:
        all_times.extend([wp.timestamp for wp in m.waypoints])
    t_min, t_max = min(all_times), max(all_times)
    
    # Generate full trajectories for ghost trails
    full_traj = [generate_full_trajectory(m.waypoints, 200) for m in missions]
    
    # Setup figure
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # Colors
    colors_normal = ['#00AA00', '#0066CC']  # Green, Blue
    color_conflict = '#FF0000'  # Red
    
    # Initialize plot elements
    # Ghost trails (full trajectory, semi-transparent)
    ghost_lines = []
    for i, traj in enumerate(full_traj):
        line, = ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 
                       color=colors_normal[i], alpha=0.2, linewidth=1)
        ghost_lines.append(line)
    
    # Active trails (up to current time)
    trail_lines = []
    for i in range(len(missions)):
        line, = ax.plot([], [], [], color=colors_normal[i], linewidth=2.5, alpha=0.8)
        trail_lines.append(line)
    
    # Drone markers
    drone_markers = []
    for i in range(len(missions)):
        marker, = ax.plot([], [], [], 'o', markersize=12, 
                         color=colors_normal[i], markeredgecolor='black', markeredgewidth=1)
        drone_markers.append(marker)
    
    # Conflict marker (hidden initially)
    conflict_marker, = ax.plot([], [], [], 's', markersize=15, color=color_conflict, 
                               markeredgecolor='darkred', markeredgewidth=2, alpha=0)
    
    # Text annotation
    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=12,
                          fontfamily='monospace', fontweight='bold')
    conflict_text = ax.text2D(0.02, 0.88, '', transform=ax.transAxes, fontsize=11,
                              color=color_conflict, fontweight='bold')
    
    # Set axis limits with padding
    all_x = np.concatenate([t[:, 0] for t in full_traj])
    all_y = np.concatenate([t[:, 1] for t in full_traj])
    all_z = np.concatenate([t[:, 2] for t in full_traj])
    
    padding = 10
    ax.set_xlim(all_x.min() - padding, all_x.max() + padding)
    ax.set_ylim(all_y.min() - padding, all_y.max() + padding)
    ax.set_zlim(all_z.min() - padding, all_z.max() + padding)
    
    ax.set_xlabel('X (meters)', fontsize=10)
    ax.set_ylabel('Y (meters)', fontsize=10)
    ax.set_zlabel('Z (meters)', fontsize=10)
    ax.set_title('UAV Trajectory Deconfliction Demo', fontsize=14, fontweight='bold')
    
    # Animation frames
    duration = t_max - t_min
    num_frames = int(duration * fps)
    frame_times = np.linspace(t_min, t_max, num_frames)
    
    # Trail history storage
    trail_history = [[] for _ in missions]
    
    def init():
        for line in trail_lines:
            line.set_data([], [])
            line.set_3d_properties([])
        for marker in drone_markers:
            marker.set_data([], [])
            marker.set_3d_properties([])
        conflict_marker.set_data([], [])
        conflict_marker.set_3d_properties([])
        time_text.set_text('')
        conflict_text.set_text('')
        return trail_lines + drone_markers + [conflict_marker, time_text, conflict_text]
    
    def update(frame):
        t = frame_times[frame]
        is_conflict = t >= conflict_time
        
        for i, mission in enumerate(missions):
            # Get current position
            pos = interpolate_position(mission.waypoints, t)
            trail_history[i].append(pos)
            
            # Update trail
            trail_data = np.array(trail_history[i])
            trail_lines[i].set_data(trail_data[:, 0], trail_data[:, 1])
            trail_lines[i].set_3d_properties(trail_data[:, 2])
            
            # Update drone marker
            drone_markers[i].set_data([pos[0]], [pos[1]])
            drone_markers[i].set_3d_properties([pos[2]])
            
            # Change color on conflict
            color = color_conflict if is_conflict else colors_normal[i]
            trail_lines[i].set_color(color)
            drone_markers[i].set_color(color)
        
        # Update conflict marker
        if is_conflict and conflict_loc:
            conflict_marker.set_data([conflict_loc[0]], [conflict_loc[1]])
            conflict_marker.set_3d_properties([conflict_loc[2]])
            conflict_marker.set_alpha(1.0)
        
        # Update text
        time_text.set_text(f'Time: {t:.1f}s')
        
        if is_conflict:
            conflict_text.set_text(
                f'⚠️ CONFLICT @ t={conflict_time:.1f}s\n'
                f'   Location: ({conflict_loc[0]:.0f}, {conflict_loc[1]:.0f}, {conflict_loc[2]:.0f})'
            )
        else:
            conflict_text.set_text('')
        
        return trail_lines + drone_markers + [conflict_marker, time_text, conflict_text]
    
    # Create animation
    anim = FuncAnimation(fig, update, init_func=init, frames=num_frames,
                        interval=1000/fps, blit=False, repeat=True)
    
    # Save GIF if output path provided
    if output_path:
        print(f"Saving animation to {output_path}...")
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        anim.save(output_path, writer='pillow', fps=fps)
        print(f"✅ Saved: {output_path}")
    
    return fig, anim


# =============================================================================
# MAIN
# =============================================================================

def main():
    # Paths
    script_dir = Path(__file__).parent
    project_dir = script_dir.parent
    json_path = project_dir / 'data' / 'demo_double_spiral.json'
    output_path = project_dir / 'outputs' / 'demo_conflict.gif'
    
    print(f"\n{'='*60}")
    print("    UAV Deconfliction Demo - Matplotlib Visualization")
    print(f"{'='*60}\n")
    
    # Load scenario
    print(f"Loading scenario from: {json_path}")
    scenario = load_scenario(str(json_path))
    print(f"Loaded {len(scenario['missions'])} missions")
    print(f"Safety buffer: {scenario['safety_buffer']}m")
    print(f"Temporal tolerance: {scenario['temporal_tolerance']}s")
    
    # Create animation
    fig, anim = create_animation(scenario, str(output_path))
    
    # Show interactive window
    print("\n📺 Displaying animation (close window to exit)...")
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
