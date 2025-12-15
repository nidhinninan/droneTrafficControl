#!/usr/bin/env python3
"""
ROS2 Bridge Node for UAV Deconfliction.
Subscribes to MAVROS mission waypoints and converts them to internal Mission format.
"""

import sys
import os
import rclpy
from rclpy.node import Node

# Add source directory to path for internal modules
# This logic is maintained to allow access to shared data_models when running in dev environment
current_dir = os.path.dirname(os.path.abspath(__file__))
# If running installed, this path might not exist unless symlinked
src_path = os.path.abspath(os.path.join(current_dir, '../src'))
if os.path.isdir(src_path):
    sys.path.insert(0, src_path)

try:
    from mavros_msgs.msg import WaypointList
except ImportError:
    print("Warning: mavros_msgs not found. This node requires MAVROS to be installed.")
    class WaypointList:
        pass

# Internal imports
try:
    # Try package-relative import first (for installed package usage)
    from . import utils
except ImportError:
    # Fallback for script usage
    import utils

try:
    from mission_validator import validate_mission_with_temporal
    from data_models import Mission
except ImportError:
    # Fallback/Mock if shared modules are missing (e.g. strict package install)
    class Mission:
        def __init__(self, mission_id, waypoints):
             self.mission_id = mission_id
             self.waypoints = waypoints
        def get_duration(self): return self.waypoints[-1].timestamp if self.waypoints else 0.0
    
    validate_mission_with_temporal = None
    print("Warning: Shared 'data_models' not found. Running in decoupled mode.")

import json
from std_msgs.msg import String

class DeconflictionBridge(Node):
    def __init__(self):
        super().__init__('deconfliction_bridge')
        
        # Parameters
        self.declare_parameter('speed_mps', 5.0)
        self.declare_parameter('safety_buffer', 5.0)
        self.declare_parameter('home_lat', 0.0) # 0.0 means use first waypoint
        self.declare_parameter('home_lon', 0.0)
        self.declare_parameter('home_alt', 0.0)
        self.declare_parameter('drone_id', 'drone_1')
        
        self.speed = self.get_parameter('speed_mps').value
        self.safety_buffer = self.get_parameter('safety_buffer').value
        self.drone_id = self.get_parameter('drone_id').value
        
        # Subscriber
        self.subscription = self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.waypoints_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher
        self.publisher_ = self.create_publisher(
            String,
            '/deconfliction/mission_input',
            10
        )
        
        # Internal State
        self.latest_mission = None
        self.traffic_missions = [] # List of Mission objects (could be populated via another topic)
        
        self.get_logger().info('Deconfliction Bridge Node Started.')
        self.get_logger().info(f'Listening on /mavros/mission/waypoints (Assumed Speed: {self.speed} m/s)')
        self.get_logger().info(f'Publishing to /deconfliction/mission_input for Drone ID: {self.drone_id}')

    def waypoints_callback(self, msg):
        self.get_logger().info(f'Received {len(msg.waypoints)} waypoints from MAVROS.')
        
        # Handle parameters that might change or need explicit 0 check
        h_lat = self.get_parameter('home_lat').value
        h_lon = self.get_parameter('home_lon').value
        h_alt = self.get_parameter('home_alt').value
        
        home_pos = None if (h_lat == 0.0 and h_lon == 0.0) else (h_lat, h_lon, h_alt)
        
        # Convert
        mission_id = f'{self.drone_id}_mission_{self.get_clock().now().nanoseconds}'
        mission = utils.convert_mavros_waypoints_to_mission(
            msg.waypoints,
            mission_id=mission_id,
            speed_mps=self.speed,
            home_lat=home_pos[0] if home_pos else None,
            home_lon=home_pos[1] if home_pos else None,
            home_alt=home_pos[2] if home_pos else None
        )
        
        if not mission:
            self.get_logger().warn('Conversion failed or no valid waypoints found.')
            return
            
        self.latest_mission = mission
        self.get_logger().info(f'Mission Parsed! Duration: {mission.get_duration():.2f}s, Waypoints: {len(mission.waypoints)}')
        
        # Publish JSON
        self.publish_mission_json(mission)
        
        # Save Mission to Disk (Persistence)
        self.save_mission(mission)
        
        # Trigger Validation (Internal)
        # self.validate_current_mission() # Optional usage based on design

    def publish_mission_json(self, mission: Mission):
        """Publishes the mission as a JSON string to the output topic."""
        try:
            # Construct time window
            # Assuming start time is NOW since we just received it
            now_sec = self.get_clock().now().seconds_nanoseconds()[0]
            start_time = float(now_sec)
            end_time = start_time + float(mission.get_duration())
            
            payload = {
                "drone_id": self.drone_id,
                "waypoints": [],
                "time_window": {
                    "start": start_time,
                    "end": end_time
                }
            }
            
            for i, wp in enumerate(mission.waypoints):
                payload["waypoints"].append({
                    "x": wp.x,
                    "y": wp.y,
                    "z": wp.z,
                    "sequence": i
                })
                
            json_str = json.dumps(payload)
            msg = String()
            msg.data = json_str
            self.publisher_.publish(msg)
            
            self.get_logger().info(f'📤 Published JSON mission update to /deconfliction/mission_input')
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish JSON: {e}')

    def save_mission(self, mission: Mission):
        import json
        
        # Create output dir
        output_dir = os.path.join(current_dir, '../outputs/received_missions')
        os.makedirs(output_dir, exist_ok=True)
        
        filename = f"{mission.mission_id}.json"
        filepath = os.path.join(output_dir, filename)
        
        try:
            data = {
                "mission_id": mission.mission_id,
                "waypoints": [
                    {
                        "x": wp.x,
                        "y": wp.y,
                        "z": wp.z,
                        "timestamp": wp.timestamp
                    }
                    for wp in mission.waypoints
                ],
                "description": f"Imported from MAVROS at {self.get_clock().now().to_msg().sec}"
            }
            
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=4)
                
            self.get_logger().info(f'💾 Mission plan saved to {filepath}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save mission: {e}')

    def validate_current_mission(self):
        if not self.latest_mission:
            return

        self.get_logger().info('Validating Mission...')
        
        # Placeholder traffic for demonstration if empty
        if not self.traffic_missions:
             self.get_logger().info('No background traffic loaded. Mission will likely be clear.')
        
        # Using the spatiotemporal validator we confirmed in analysis
        is_clear, conflicts, report = validate_mission_with_temporal(
            self.latest_mission, 
            self.traffic_missions, 
            safety_buffer=self.safety_buffer
        )
        
        if is_clear:
            self.get_logger().info('✅ Mission APPROVED.')
        else:
            self.get_logger().warn(f'❌ Mission REJECTED. {len(conflicts)} Conflicts Detected.')
            for c in conflicts:
                self.get_logger().warn(f'   Conflict at {c.conflict_location} (Dist: {c.separation_distance:.2f}m)')

def main(args=None):
    rclpy.init(args=args)
    node = DeconflictionBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
