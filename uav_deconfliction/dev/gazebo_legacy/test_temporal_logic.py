
import unittest
import sys
from pathlib import Path
import numpy as np

# Ensure src is in path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from mission_validator import validate_mission, validate_mission_with_temporal
from test_data_generator import generate_temporally_separated_missions, generate_crossing_missions_3d, visualize_test_scenario
from data_models import Mission, Waypoint

class TestTemporalValidation(unittest.TestCase):
    
    def test_temporal_separation_clear(self):
        """
        Verify that missions crossing in space but separated in time are CLEARED.
        """
        print("\n=== Test: Temporal Separation ===")
        m1, m2 = generate_temporally_separated_missions()
        
        # 1. SPATIAL CHECK (Should Fail)
        is_clear, conflicts, _ = validate_mission(m1, [m2], safety_buffer=5.0)
        self.assertFalse(is_clear, "Spatial-only check should fail for crossing paths")
        print("✓ Spatial check correctly identified conflict (as expected)")
        
        # 2. TEMPORAL CHECK (Should Pass)
        is_clear_temp, conflicts_temp, report = validate_mission_with_temporal(
            m1, [m2], safety_buffer=5.0, temporal_tolerance=1.0
        )
        
        self.assertTrue(is_clear_temp, "Temporal check should pass for time-separated missions")
        self.assertEqual(len(conflicts_temp), 0)
        print("✓ Temporal check correctly cleared the mission")
        # print(report)

    def test_spatiotemporal_conflict(self):
        """
        Verify that missions crossing in space AND time are REJECTED.
        """
        print("\n=== Test: Spatiotemporal Conflict ===")
        # Use standard crossing missions (timestamps approx same 0..10s)
        m1, m2 = generate_crossing_missions_3d()
        m1.mission_id = "COLLISION_A"
        m2.mission_id = "COLLISION_B"
        
        # TEMPORAL CHECK (Should Fail)
        is_clear, conflicts, report = validate_mission_with_temporal(
            m1, [m2], safety_buffer=5.0, temporal_tolerance=1.0
        )
        
        self.assertFalse(is_clear, "Should detect conflict when overlapping in space and time")
        self.assertTrue(len(conflicts) > 0)
        
        c = conflicts[0]
        print(f"✓ Conflict detected at time t={c.conflict_time:.2f}s")
        print(f"  Visual description:\n{c.description}")
        
        # Verify conflict time is reasonable (crossing happens at index 5 -> t=5.0)
        self.assertAlmostEqual(c.conflict_time, 5.0, delta=1.0)
        
        # VISUALIZATION
        print("Generating visualization for conflict...")
        visualize_test_scenario(
            [m1, m2], 
            title="Spatiotemporal Conflict Test", 
            conflicts=conflicts,
            output_path="test_outputs/test_spatiotemporal_conflict.png"
        )

if __name__ == '__main__':
    unittest.main()
