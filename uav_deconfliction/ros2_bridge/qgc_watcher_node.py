#!/usr/bin/env python3
"""
QGC Plan Watcher Node
Monitors a directory for new .plan files and publishes them to the deconfliction system.
"""

import sys
import os
import time
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Internal imports
current_dir = os.path.dirname(os.path.abspath(__file__))
# Try package-relative import first
try:
    from . import qgc_converter
except ImportError:
    import qgc_converter

class QGCWatcherNode(Node):
    def __init__(self):
        super().__init__('qgc_watcher_node')
        
        # Parameters
        self.declare_parameter('watch_dir', os.path.join(os.getenv('HOME'), 'Documents', 'QGroundControl', 'Missions'))
        self.declare_parameter('drone_id', 'drone_qgc')
        self.declare_parameter('poll_interval', 2.0)
        self.declare_parameter('speed_mps', 5.0)
        
        self.watch_dir = self.get_parameter('watch_dir').value
        self.drone_id = self.get_parameter('drone_id').value
        self.poll_interval = self.get_parameter('poll_interval').value
        self.speed = self.get_parameter('speed_mps').value
        
        # Publisher
        self.publisher_ = self.create_publisher(
            String,
            '/deconfliction/mission_input',
            10
        )
        
        # State
        self.processed_files = set()
        
        # Ensure dir exists
        if not os.path.exists(self.watch_dir):
            try:
                os.makedirs(self.watch_dir, exist_ok=True)
                self.get_logger().info(f"Created watch directory: {self.watch_dir}")
            except OSError as e:
                self.get_logger().error(f"Failed to create watch directory: {e}")
        
        self.get_logger().info(f"Watching {self.watch_dir} for new .plan files...")
        
        # Timer
        self.timer = self.create_timer(self.poll_interval, self.check_directory)

    def check_directory(self):
        if not os.path.exists(self.watch_dir):
            return

        try:
            files = [f for f in os.listdir(self.watch_dir) if f.endswith('.plan')]
            
            for f in files:
                filepath = os.path.join(self.watch_dir, f)
                
                # Check modification time or just existence in set
                # For MVP, we just check if we've seen this filename in this session
                # Improvement: Check mtime to handle updates
                if f not in self.processed_files:
                    self.process_plan(filepath)
                    self.processed_files.add(f)
                    
        except Exception as e:
            self.get_logger().error(f"Error checking directory: {e}")

    def process_plan(self, filepath):
        self.get_logger().info(f"Processing new plan: {filepath}")
        
        try:
            # use the converter logic
            plan_data = qgc_converter.load_plan(filepath)
            result = qgc_converter.parse_mission_items(plan_data, self.speed)
            
            if not result:
                self.get_logger().warn(f"No valid waypoints in {filepath}")
                return

            waypoints = result.get('waypoints', [])
            duration = result.get('duration', 0.0)
            
            # Construct Payload
            # We construct the JSON payload directly here to match the topic format
            import time
            start_time = float(time.time())
            
            payload = {
                "drone_id": self.drone_id,
                "waypoints": waypoints,
                "time_window": {
                    "start": start_time,
                    "end": start_time + duration
                },
                "source": "qgc_watcher"
            }
            
            msg = String()
            msg.data = json.dumps(payload)
            self.publisher_.publish(msg)
            
            self.get_logger().info(f"📤 Published QGC plan {filepath} to system.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert plan {filepath}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = QGCWatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
