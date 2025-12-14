import sys
import os
import json

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

from data_models import Waypoint, Mission
from geometry_utils import generate_double_loop_spiral_waypoints
from mission_validator import (
    validate_mission, 
    batch_validate_missions, 
    categorize_conflicts_by_severity,
    export_validation_results
)

def test_mission_validation():
    print("=== Testing Mission Validator ===")
    
    # 1. Setup Missions
    wps1_tuples = generate_double_loop_spiral_waypoints(num_waypoints=15, center=(0, 0, 50))
    wps2_tuples = generate_double_loop_spiral_waypoints(num_waypoints=15, center=(30, 20, 55)) # Conflict
    wps3_tuples = generate_double_loop_spiral_waypoints(num_waypoints=15, center=(1000, 1000, 100)) # Safe

    wps1 = [Waypoint(x=p[0], y=p[1], z=p[2], timestamp=float(i*10)) for i, p in enumerate(wps1_tuples)]
    wps2 = [Waypoint(x=p[0], y=p[1], z=p[2], timestamp=float(i*10)) for i, p in enumerate(wps2_tuples)]
    wps3 = [Waypoint(x=p[0], y=p[1], z=p[2], timestamp=float(i*10)) for i, p in enumerate(wps3_tuples)]
    
    m1 = Mission("M001", wps1)
    m2 = Mission("M002", wps2)
    m3 = Mission("M003", wps3)
    
    # 2. Test Validation (Safe)
    print("\n[Test 1] Validating M001 vs M003 (Safe)...")
    is_clear, conflicts, report = validate_mission(m1, [m3], safety_buffer=5.0)
    if is_clear and len(conflicts) == 0:
        print("SUCCESS: Validated as safe.")
    else:
        print("FAILURE: Validation failed for safe scenario.")

    # 3. Test Validation (Conflict)
    print("\n[Test 2] Validating M001 vs M002 (Conflict)...")
    is_clear, conflicts, report = validate_mission(m1, [m2], safety_buffer=5.0)
    if not is_clear and len(conflicts) > 0:
        print("SUCCESS: Validated as conflict.")
        print(f"Generated report length: {len(report)} chars")
    else:
        print("FAILURE: Validation failed for conflict scenario.")
    
    # 4. Test Severity Categorization
    print("\n[Test 3] Testing Categorization...")
    # Buffer is 5.0, separation is ~2.16 (based on prev tests)
    # Ratio = 2.16 / 5.0 = 0.43 (< 0.5) -> CRITICAL
    cats = categorize_conflicts_by_severity(conflicts, safety_buffer=5.0)
    if len(cats["critical"]) == 1:
        print("SUCCESS: Correctly categorized as CRITICAL.")
    else:
        print(f"FAILURE: Expected critical, got {cats}")

    # 5. Test Export
    print("\n[Test 4] Testing JSON Export...")
    out_path = "outputs/validation_test.json"
    success = export_validation_results((is_clear, conflicts, report), out_path, 'json')
    if success and os.path.exists(out_path):
        print(f"SUCCESS: Exported to {out_path}")
        with open(out_path) as f:
            data = json.load(f)
            print(f"JSON Status: {data.get('mission_status')}")
    else:
        print("FAILURE: Export failed.")

    # 6. Test Batch
    print("\n[Test 5] Batch Validation...")
    results = batch_validate_missions([m1, m3], [m2], safety_buffer=5.0)
    # m1 vs m2 -> Conflict
    # m3 vs m2 -> Safe
    if not results["M001"][0] and results["M003"][0]:
        print("SUCCESS: Batch validation correct.")
    else:
        print("FAILURE: Batch validation incorrect.")

if __name__ == "__main__":
    test_mission_validation()
