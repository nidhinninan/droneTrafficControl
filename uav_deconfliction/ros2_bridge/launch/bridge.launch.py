from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_bridge', # Note: This assumes the package is built/installed as 'ros2_bridge' or script is runnable
            executable='bridge_node.py', # Since we are running as script usually, convert to executable path if needed
            name='deconfliction_bridge',
            output='screen',
            parameters=[
                {'speed_mps': 8.0},
                {'safety_buffer': 5.0}
            ]
        )
    ])
