import sys
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

from data_models import Waypoint, Mission
import geometry_utils
from geometry_utils import generate_double_loop_spiral_waypoints
from conflict_detector import validate_mission_pair, check_multiple_missions

def visualize_scenario(mission1, mission2, conflict, title, output_path):
    """
    Visualizes two missions and any conflict between them.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot Mission 1
    tck1, _ = geometry_utils.create_trajectory_curve(mission1.waypoints)
    samples1 = geometry_utils.sample_trajectory_points(tck1, 200)
    ax.plot(samples1[:, 0], samples1[:, 1], samples1[:, 2], label=f'{mission1.mission_id}', color='blue')
    
    # Plot Mission 2
    tck2, _ = geometry_utils.create_trajectory_curve(mission2.waypoints)
    samples2 = geometry_utils.sample_trajectory_points(tck2, 200)
    ax.plot(samples2[:, 0], samples2[:, 1], samples2[:, 2], label=f'{mission2.mission_id}', color='green')
    
    # Plot Conflict if exists
    if conflict:
        loc = conflict.conflict_location
        ax.scatter([loc[0]], [loc[1]], [loc[2]], color='red', s=150, marker='X', label='Conflict Point', zorder=10)
        ax.text(loc[0], loc[1], loc[2]+5, f"Dist: {conflict.separation_distance:.2f}m", color='red')
        
    ax.set_title(title)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    
    # Ensure output directory exists
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path)
    print(f"Plot saved to {output_path}")
    plt.close(fig)

def test_conflict_detection():
    print("=== Testing Conflict Detector ===")
    
    # 1. Setup Missions
    # Create two missions with slightly offset spiral paths
    # Center (0,0,50) vs (30,20,55) - we know min dist is ~2.16m from previous test
    
    wps1_tuples = generate_double_loop_spiral_waypoints(num_waypoints=15, center=(0, 0, 50))
    wps2_tuples = generate_double_loop_spiral_waypoints(num_waypoints=15, center=(30, 20, 55))
    
    wps1 = [Waypoint(x=p[0], y=p[1], z=p[2], timestamp=float(i*10)) for i, p in enumerate(wps1_tuples)]
    wps2 = [Waypoint(x=p[0], y=p[1], z=p[2], timestamp=float(i*10)) for i, p in enumerate(wps2_tuples)]
    
    m1 = Mission("M001", wps1)
    m2 = Mission("M002", wps2)
    
    # 2. Test Safe Scenario (Buffer < 2.16m)
    print("\n[Test Case 1] Checking SAFE condition (Buffer=2.0m)...")
    has_conflict, conflict = validate_mission_pair(m1, m2, safety_buffer=2.0, verbose=True)
    if not has_conflict:
        print("SUCCESS: Correctly identified as SAFE.")
        visualize_scenario(m1, m2, None, "SAFE: Buffer=2.0m (Sep ~2.16m)", "outputs/test_safe_scenario.png")
    else:
        print("FAILURE: Incorrectly identified as CONFLICT.")
        
    # 3. Test Conflict Scenario (Buffer > 2.16m)
    print("\n[Test Case 2] Checking CONFLICT condition (Buffer=5.0m)...")
    has_conflict, conflict = validate_mission_pair(m1, m2, safety_buffer=5.0, verbose=True)
    if has_conflict:
        print("SUCCESS: Correctly identified as CONFLICT.")
        print(f"Violation Margin: {conflict.get_violation_margin():.2f}m") 
        # Safety (5.0) - Actual (~2.16) = ~2.84
        visualize_scenario(m1, m2, conflict, "CONFLICT: Buffer=5.0m (Sep ~2.16m)", "outputs/test_conflict_scenario.png")
    else:
        print("FAILURE: Incorrectly identified as SAFE.")

    # 4. Test Batch Processing
    print("\n[Test Case 3] Checking Batch Processing...")
    # Add a third, far away mission
    wps3_tuples = generate_double_loop_spiral_waypoints(num_waypoints=15, center=(1000, 1000, 100))
    wps3 = [Waypoint(x=p[0], y=p[1], z=p[2], timestamp=float(i*10)) for i, p in enumerate(wps3_tuples)]
    m3 = Mission("M003", wps3)
    
    conflicts = check_multiple_missions(m1, [m2, m3], safety_buffer=5.0)
    if len(conflicts) == 1 and conflicts[0].mission_ids == ("M001", "M002"):
        print("SUCCESS: Correctly identified 1 conflict (M001 vs M002) and ignored M003.")
    else:
        print(f"FAILURE: Expected 1 conflict, found {len(conflicts)}.")

if __name__ == "__main__":
    test_conflict_detection()
