from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================
    
    # 1. Coordinate Frame Origins (Default: 0.0 implies "use first waypoint")
    arg_home_lat = DeclareLaunchArgument(
        'home_lat', default_value='0.0',
        description='Reference Home Latitude (set to 0.0 to use first waypoint)'
    )
    arg_home_lon = DeclareLaunchArgument(
        'home_lon', default_value='0.0',
        description='Reference Home Longitude'
    )
    arg_home_alt = DeclareLaunchArgument(
        'home_alt', default_value='0.0',
        description='Reference Home Altitude'
    )

    # 2. Safety Parameters
    arg_safety_buffer = DeclareLaunchArgument(
        'safety_buffer', default_value='5.0',
        description='Minimum spatial separation distance in meters'
    )
    
    # 3. Flight Parameters
    arg_speed = DeclareLaunchArgument(
        'speed_mps', default_value='5.0',
        description='Assumed autonomous flight speed in m/s'
    )

    # 4. QGC Watcher Config
    default_watch_dir = os.path.join(os.getenv('HOME', '/tmp'), 'qgc_missions')
    arg_watch_dir = DeclareLaunchArgument(
        'qgc_plan_dir', default_value=default_watch_dir,
        description='Directory to watch for QGroundControl .plan files'
    )

    # ========================================================================
    # NODES
    # ========================================================================

    # 1. MAVROS Bridge Node
    # Subscribes to /mavros/mission/waypoints
    bridge_node = Node(
        package='ros2_bridge',
        executable='bridge_node',
        name='mavros_bridge',
        output='screen',
        parameters=[{
            'home_lat': LaunchConfiguration('home_lat'),
            'home_lon': LaunchConfiguration('home_lon'),
            'home_alt': LaunchConfiguration('home_alt'),
            'safety_buffer': LaunchConfiguration('safety_buffer'),
            'speed_mps': LaunchConfiguration('speed_mps'),
            'drone_id': 'drone_mavros'
        }]
    )

    # 2. Normalization Service
    # Provides /normalize_mission service
    normalization_node = Node(
        package='ros2_bridge',
        executable='normalization_service',
        name='mission_normalizer',
        output='screen',
        # No specific params for now, but could add limits here
    )

    # 3. QGC Plan Watcher
    # Watches directory for .plan files
    watcher_node = Node(
        package='ros2_bridge',
        executable='qgc_watcher',
        name='qgc_watcher',
        output='screen',
        parameters=[{
            'watch_dir': LaunchConfiguration('qgc_plan_dir'),
            'drone_id': 'drone_qgc_import',
            'speed_mps': LaunchConfiguration('speed_mps')
        }]
    )

    return LaunchDescription([
        # Args
        arg_home_lat, arg_home_lon, arg_home_alt,
        arg_safety_buffer, arg_speed, arg_watch_dir,
        
        # Nodes
        bridge_node,
        normalization_node,
        watcher_node
    ])
