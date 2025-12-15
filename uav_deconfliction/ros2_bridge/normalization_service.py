#!/usr/bin/env python3
"""
Normalization Service Node
Provides a ROS2 service to validate and normalize mission data.
"""

import sys
import os
import json
import time

import rclpy
from rclpy.node import Node

# Internal imports (robust handling)
current_dir = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.abspath(os.path.join(current_dir, '../src'))
if os.path.isdir(src_path):
    sys.path.insert(0, src_path)

try:
    # Try package-relative import first
    from . import utils
except ImportError:
    import utils

# Try to import custom service definition
try:
    from deconfliction_msgs.srv import NormalizeMission
except ImportError:
    print("Warning: deconfliction_msgs not found. Service will not start.")
    NormalizeMission = None

class NormalizationService(Node):
    def __init__(self):
        super().__init__('normalization_service')
        
        if NormalizeMission is None:
            self.get_logger().error("deconfliction_msgs module is missing. Cannot start service.")
            # We don't exit to allow node introspection, but functionality is disabled
            return

        self.srv = self.create_service(
            NormalizeMission, 
            'normalize_mission', 
            self.normalize_callback
        )
        self.get_logger().info('Normalization Service Ready.')

    def normalize_callback(self, request, response):
        self.get_logger().info('Received normalization request.')
        
        try:
            # 1. Parse Input
            raw_data = json.loads(request.input_json)
            
            # 2. Validation & Normalization
            normalized_data, error = self.validate_and_normalize(raw_data)
            
            if error:
                response.success = False
                response.message = f"Validation Error: {error}"
                response.normalized_mission_json = ""
                self.get_logger().warn(f"Validation failed: {error}")
            else:
                response.success = True
                response.message = "Mission normalized successfully."
                response.normalized_mission_json = json.dumps(normalized_data)
                self.get_logger().info("Mission normalized successfully.")
                
        except json.JSONDecodeError:
            response.success = False
            response.message = "Invalid JSON input."
            response.normalized_mission_json = ""
            self.get_logger().error("Failed to decode JSON input.")
        except Exception as e:
            response.success = False
            response.message = f"Internal Error: {str(e)}"
            response.normalized_mission_json = ""
            self.get_logger().error(f"Internal error: {e}")
            
        return response

    def validate_and_normalize(self, data):
        """
        Validates structure and applies defaults.
        Returns (normalized_dict, error_string_or_None).
        """
        # A. Basic Structure
        if not isinstance(data, dict):
            return None, "Input must be a JSON object"
            
        # B. Drone ID
        drone_id = data.get("drone_id", "unknown_drone")
        if not drone_id:
            drone_id = "unknown_drone"
            
        # C. Waypoints
        waypoints = data.get("waypoints", [])
        if not isinstance(waypoints, list):
            return None, "'waypoints' must be a list"
            
        if len(waypoints) < 2:
            return None, "Mission must have at least 2 waypoints"
            
        # Validate/Normalize individual waypoints
        norm_waypoints = []
        for i, wp in enumerate(waypoints):
            if not isinstance(wp, dict):
                return None, f"Waypoint {i} is not a valid object"
            
            # Check coords
            x = wp.get("x")
            y = wp.get("y")
            z = wp.get("z")
            
            if any(v is None for v in [x, y, z]):
                 return None, f"Waypoint {i} missing x, y, or z coordinates"
            
            # Simple Range Check (Safety)
            if any(abs(v) > 100000 for v in [x, y, z]): # 100km limit
                 return None, f"Waypoint {i} coordinates out of reasonable bounds (>100km)"

            # Ensure sequence
            seq = wp.get("sequence", i)
            
            # Ensure timestamp (t) exists or default to 0.0 (will be fixed by bridge if needed, but we check existence)
            # data_models.Waypoint needs it.
            t = wp.get("timestamp", wp.get("t", 0.0))
            
            norm_waypoints.append({
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "sequence": int(seq),
                "timestamp": float(t)
            })

        # D. Time Window
        time_window = data.get("time_window", {})
        now = time.time()
        
        start = time_window.get("start", now)
        end = time_window.get("end")
        
        # If end is missing, estimate it from waypoints if they have timestamps
        if not end:
            last_t = norm_waypoints[-1]["timestamp"]
            end = start + last_t if last_t > 0 else start + 600 # Default 10 min
            
        if end <= start:
             return None, "Invalid time window: end time must be after start time"

        # E. Safety Buffer
        safety_buffer = data.get("safety_buffer", 5.0)
        if safety_buffer <= 0:
            return None, "Safety buffer must be positive"

        # Construct Normalized Object
        normalized = {
            "mission_id": data.get("mission_id", f"{drone_id}_norm_{int(now)}"),
            "drone_id": str(drone_id),
            "waypoints": norm_waypoints,
            "time_window": {
                "start": float(start),
                "end": float(end)
            },
            "safety_buffer": float(safety_buffer),
            "metadata": {
                "normalized_at": now,
                "source": data.get("source", "unknown")
            }
        }
        
        return normalized, None

def main(args=None):
    rclpy.init(args=args)
    node = NormalizationService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
