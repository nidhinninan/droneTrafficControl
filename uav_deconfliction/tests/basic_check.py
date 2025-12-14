import sys
import os

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

from data_models import Waypoint, Mission, Conflict

def test_models():
    print("Testing data models...")
    
    # 1. Test Waypoint
    w1 = Waypoint(x=0.0, y=0.0, z=10.0, timestamp=0.0)
    w2 = Waypoint(x=10.0, y=10.0, z=10.0, timestamp=10.0)
    print(f"Waypoint created: {w1}")
    
    # 2. Test Mission
    mission = Mission("M001", [w1, w2])
    # Expected duration: w2.timestamp (10.0) - w1.timestamp (0.0) = 10.0
    print(f"Mission created duration: {mission.get_duration()}")
    
    # Test segments
    segments = mission.get_all_segments()
    assert len(segments) == 1
    assert segments[0] == (w1, w2)
    print("Segments verified.")
    
    # Test validation
    try:
        w_bad = Waypoint(x=5.0, y=5.0, z=5.0, timestamp=5.0) # timestamp 5 < 10
        Mission("M_BAD", [w2, w_bad])
        print("ERROR: Validation failed to catch unsorted waypoints!")
    except ValueError as e:
        print(f"Validation successful: {e}")

    # 3. Test Conflict
    conflict = Conflict(
        mission_ids=("M001", "M002"),
        conflict_location=(5.0, 5.0, 10.0),
        conflict_time=5.0,
        separation_distance=2.0,
        safety_buffer=5.0,
        description="Too close!"
    )
    
    assert conflict.is_violation() == True
    assert conflict.get_violation_margin() == 3.0
    print("Conflict logic verified.")
    
    print("\nAll tests passed!")

if __name__ == "__main__":
    test_models()
