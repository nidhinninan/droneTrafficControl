#!/usr/bin/env python3
"""
Verification script for ROS2 Bridge conversion logic.
Mocking MAVROS messages to test utils.py without ROS2 installed.
"""

import sys
import os
import unittest
from types import SimpleNamespace

# Setup path to import utils
sys.path.append(os.path.dirname(__file__))

import utils
from data_models import Mission, Waypoint

# Mock mavros_msgs.msg.Waypoint
class MockWaypoint:
    def __init__(self, lat, lon, alt, command=16):
        self.x_lat = lat
        self.y_long = lon
        self.z_alt = alt
        self.command = command

class TestConversion(unittest.TestCase):
    def test_basic_conversion(self):
        print("\nTesting Basic Conversion...")
        # Define waypoints (approx 1 degree lat ~ 111km)
        # 0.00001 deg is approx 1 meter
        waypoints = [
            MockWaypoint(0.0, 0.0, 10.0),          # Home (0,0,10)
            MockWaypoint(0.0001, 0.0, 10.0),       # ~11.1m North
            MockWaypoint(0.0001, 0.0001, 20.0),    # ~11.1m East + Up
        ]
        
        speed = 5.0
        mission = utils.convert_mavros_waypoints_to_mission(
            waypoints, 
            mission_id="test_mission", 
            speed_mps=speed
        )
        
        self.assertIsNotNone(mission)
        self.assertEqual(len(mission.waypoints), 3)
        
        # Check WP 0 (Origin)
        self.assertAlmostEqual(mission.waypoints[0].x, 0.0, places=1)
        self.assertAlmostEqual(mission.waypoints[0].y, 0.0, places=1)
        self.assertEqual(mission.waypoints[0].timestamp, 0.0)
        
        # Check WP 1 (North)
        # 0.0001 deg lat is roughly 11.132 meters
        expected_y = 0.0001 * 111319.9
        self.assertAlmostEqual(mission.waypoints[1].y, expected_y, places=1)
        self.assertAlmostEqual(mission.waypoints[1].x, 0.0, places=1)
        
        # Check Timestamps
        dist01 = expected_y
        expected_t1 = dist01 / speed
        self.assertAlmostEqual(mission.waypoints[1].timestamp, expected_t1, places=1)
        
        print("✅ Basic Conversion Passed")
        print(f"   Mission Duration: {mission.get_duration():.2f}s")
        print(f"   Waypoints: {mission.waypoints}")

    def test_home_override(self):
        print("\nTesting Home Override...")
        waypoints = [
            MockWaypoint(10.0001, 10.0, 50.0),
            MockWaypoint(10.0002, 10.0, 50.0)
        ]
        
        # Set home exactly at the waypoint location
        mission = utils.convert_mavros_waypoints_to_mission(
            waypoints,
            home_lat=10.0,
            home_lon=10.0,
            home_alt=0.0
        )
        
        # Should be approx 11.1m North
        expected_y = 0.0001 * 111319.9
        self.assertAlmostEqual(mission.waypoints[0].y, expected_y, places=1)
        self.assertAlmostEqual(mission.waypoints[0].z, 50.0, places=1)
        print("✅ Home Override Passed")

if __name__ == '__main__':
    unittest.main()
