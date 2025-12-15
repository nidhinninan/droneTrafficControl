"""
Utility functions for ROS2 Bridge.
Handles conversion between MAVLink (Lat/Lon) and internal used Cartesian (x, y, z) coordinates.
"""

import math
from typing import List, Tuple
import sys
import os

# Add parrent/src to path to find data_models
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

try:
    from data_models import Waypoint, Mission
except ImportError:
    # Fallback for when running from a different context
    from src.data_models import Waypoint, Mission

# Constants
EARTH_RADIUS = 6378137.0  # Meters

def geodetic_to_enu(lat: float, lon: float, alt: float, 
                    ref_lat: float, ref_lon: float, ref_alt: float) -> Tuple[float, float, float]:
    """
    Convert Geodetic coordinates (lat, lon, alt) to Local ENU (East-North-Up).
    
    Args:
        lat, lon: Latitude and Longitude in degrees.
        alt: Altitude in meters.
        ref_lat, ref_lon: Reference (Home) Latitude and Longitude in degrees.
        ref_alt: Reference Altitude in meters.
        
    Returns:
        (x, y, z) tuple in meters relative to reference.
    """
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)
    
    d_lat = lat_rad - ref_lat_rad
    d_lon = lon_rad - ref_lon_rad
    
    # Simple Equirectangular projection for small distances (flat earth assumption)
    # Sufficient for drone missions < 100km
    x = d_lon * math.cos(ref_lat_rad) * EARTH_RADIUS
    y = d_lat * EARTH_RADIUS
    z = alt - ref_alt
    
    return (x, y, z)

def calculate_distance(p1: Tuple[float, float, float], p2: Tuple[float, float, float]) -> float:
    """Euclidean distance between two 3D points."""
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 + (p2[2]-p1[2])**2)

def convert_mavros_waypoints_to_mission(
    mavros_waypoints: list, 
    mission_id: str = "imported_mission",
    speed_mps: float = 5.0,
    home_lat: float = None, 
    home_lon: float = None, 
    home_alt: float = None
) -> Mission:
    """
    Converts a list of MAVROS Waypoint objects to an internal Mission object.
    
    Args:
        mavros_waypoints: List of mavros_msgs/Waypoint objects.
        mission_id: ID to assign to the new mission.
        speed_mps: Assumed flight speed in m/s for timestamp generation.
        home_lat, home_lon, home_alt: Optional manual home position override.
            If None, the first waypoint is used as the origin (0,0,0).
            
    Returns:
        A populated Mission object.
    """
    internal_waypoints = []
    
    # Filter valid waypoints (Command 16 = NAV_WAYPOINT)
    # Also consider checking frame, but for MVP we assume global relative alt (3) or glboal (0)
    valid_wps = [wp for wp in mavros_waypoints if wp.command == 16]
    
    if not valid_wps:
        return None

    # Determine Home/Reference
    if home_lat is None:
        # Use first waypoint as origin
        ref_lat = valid_wps[0].x_lat
        ref_lon = valid_wps[0].y_long
        ref_alt = valid_wps[0].z_alt
    else:
        ref_lat, ref_lon, ref_alt = home_lat, home_lon, home_alt

    # Conversion Loop
    current_time = 0.0
    prev_pos = (0.0, 0.0, 0.0) # Will be updated in first iteration if not starting at 0,0,0
    
    for i, wp in enumerate(valid_wps):
        # 1. Convert Position
        x, y, z = geodetic_to_enu(wp.x_lat, wp.y_long, wp.z_alt, ref_lat, ref_lon, ref_alt)
        curr_pos = (x, y, z)
        
        # 2. Calculate Timestamp
        if i == 0:
            current_time = 0.0
        else:
            dist = calculate_distance(prev_pos, curr_pos)
            dt = dist / speed_mps
            current_time += dt
            
        # 3. Create Waypoint
        internal_waypoints.append(Waypoint(x=x, y=y, z=z, timestamp=current_time))
        prev_pos = curr_pos
        
    return Mission(mission_id=mission_id, waypoints=internal_waypoints)
