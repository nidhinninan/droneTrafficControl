#!/usr/bin/env python3
"""
QGroundControl .plan Converter
Converts QGC mission plans to the internal JSON format used by the Deconfliction System.
"""

import json
import argparse
import sys
import os
import math
from typing import Dict, Any

# Add current directory to path to find utils
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    import utils
except ImportError:
    print("Error: Could not import 'utils.py'. Make sure you are running this from the ros2_bridge directory.")
    sys.exit(1)

# MAVLink Commands
MAV_CMD_NAV_WAYPOINT = 16

def load_plan(filepath: str) -> Dict[str, Any]:
    with open(filepath, 'r') as f:
        return json.load(f)

def parse_mission_items(plan_data: Dict[str, Any], speed_mps: float = 5.0, default_alt: float = 10.0) -> Dict[str, Any]:
    mission_items = plan_data.get('mission', {}).get('items', [])
    
    if not mission_items:
        print("Warning: No mission items found in plan file.")
        return {}

    converted_waypoints = []
    
    # Establish Reference (Home)
    # QGC plans usually store 'plannedHomePosition' separately, or we use the first WP
    home_pos = plan_data.get('mission', {}).get('plannedHomePosition')
    
    if home_pos:
        ref_lat = home_pos[0]
        ref_lon = home_pos[1]
        ref_alt = home_pos[2]
    else:
        # scan for first valid waypoint
        first_wp = next((item for item in mission_items if item['command'] == MAV_CMD_NAV_WAYPOINT), None)
        if not first_wp:
            raise ValueError("No valid waypoints found to establish home reference.")
        ref_lat = first_wp['params'][4] # Param 5 (Lat)
        ref_lon = first_wp['params'][5] # Param 6 (Lon)
        ref_alt = first_wp['params'][6] # Param 7 (Alt)
    
    print(f"Reference Home: ({ref_lat:.6f}, {ref_lon:.6f}, {ref_alt:.1f}m)")

    current_time = 0.0
    prev_pos = (0.0, 0.0, 0.0)

    for i, item in enumerate(mission_items):
        cmd = item.get('command')
        
        if cmd != MAV_CMD_NAV_WAYPOINT:
            print(f"Skipping unsupported command {cmd} at sequence {item.get('sequence', i)}")
            continue
            
        # Extract Params (MAVLink standard 7 params)
        # For Waypoint: p1=Hold, p2=AcceptRadius, p3=PassRadius, p4=Yaw, p5=Lat, p6=Lon, p7=Alt
        params = item.get('params', [])
        if len(params) < 7:
            print(f"Skipping malformed waypoint at index {i}")
            continue
            
        lat = params[4]
        lon = params[5]
        alt = params[6]
        
        # Convert to ENU
        x, y, z = utils.geodetic_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt)
        curr_pos = (x, y, z)
        
        # Calculate Time
        if i > 0 and converted_waypoints:
            dist = utils.calculate_distance(prev_pos, curr_pos)
            dt = dist / speed_mps
            current_time += dt
        
        converted_waypoints.append({
            "x": x,
            "y": y,
            "z": z,
            "sequence": len(converted_waypoints)
        })
        
        prev_pos = curr_pos

    return {
        "waypoints": converted_waypoints,
        "duration": current_time
    }

def main():
    parser = argparse.ArgumentParser(description="Convert QGC .plan to Deconfliction JSON")
    parser.add_argument("plan_file", help="Path to input .plan file")
    parser.add_argument("--output", "-o", help="Path to output JSON file", default=None)
    parser.add_argument("--drone-id", default="drone_1", help="Drone ID string")
    parser.add_argument("--speed", type=float, default=5.0, help="Assumed flight speed (m/s)")
    parser.add_argument("--ros", action="store_true", help="Publish to ROS2 topic (requires rclpy)")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.plan_file):
        print(f"Error: File not found: {args.plan_file}")
        sys.exit(1)
        
    print(f"Loading plan: {args.plan_file}")
    plan_data = load_plan(args.plan_file)
    
    try:
        result = parse_mission_items(plan_data, args.speed)
    except ValueError as e:
        print(f"Error parsing plan: {e}")
        sys.exit(1)
        
    waypoints = result.get('waypoints', [])
    duration = result.get('duration', 0.0)
    
    print(f"Parsed {len(waypoints)} valid waypoints.")
    print(f"Estimated Duration: {duration:.1f}s")
    
    # Construct Output Payload
    # Assuming start time is immediate for this conversion context
    import time
    start_time = time.time()
    
    payload = {
        "drone_id": args.drone_id,
        "waypoints": waypoints,
        "time_window": {
            "start": start_time,
            "end": start_time + duration
        }
    }
    
    # 1. Output to File
    if args.output:
        out_path = args.output
    else:
        # Default to same name .json
        base = os.path.splitext(args.plan_file)[0]
        out_path = f"{base}_converted.json"
        
    with open(out_path, 'w') as f:
        json.dump(payload, f, indent=4)
    print(f"Saved to: {out_path}")
    
    # 2. Publish to ROS2 (Optional)
    if args.ros:
        try:
            import rclpy
            from std_msgs.msg import String
            
            rclpy.init()
            node = rclpy.create_node('qgc_plan_publisher')
            pub = node.create_publisher(String, '/deconfliction/mission_input', 10)
            
            msg = String()
            msg.data = json.dumps(payload)
            
            # Publish multiple times to ensure connection
            import time
            time.sleep(1)
            pub.publish(msg)
            print(f"Published to /deconfliction/mission_input")
            time.sleep(0.5)
            
            node.destroy_node()
            rclpy.shutdown()
        except ImportError:
            print("Error: rclpy not found. Cannot publish to ROS.")
        except Exception as e:
            print(f"Error publishing to ROS: {e}")

if __name__ == "__main__":
    main()
