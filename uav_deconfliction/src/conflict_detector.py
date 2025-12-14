"""
Conflict Detector Module

This module implements spatial conflict detection for smooth 3D trajectory curves.
It leverages B-spline representations to determine if two UAV missions come within
a specified safety buffer distance of each other.

Current Capabilities:
- Spatial Conflict Detection: Checks if 3D paths are too close.
- Batch Processing: Check one mission against many.

Future Enhancements:
- Temporal Analysis: Verifying if drones are at the conflict point at the same time.
"""

from typing import Optional, List, Tuple, Any
import numpy as np

# Internal imports
try:
    from data_models import Mission, Conflict, Waypoint
    import geometry_utils
except ImportError:
    # Fallback for relative imports or running as script
    from .data_models import Mission, Conflict, Waypoint
    from . import geometry_utils

def check_trajectory_conflict(mission1: Mission, mission2: Mission, safety_buffer: float, num_samples: int = 100) -> Optional[Conflict]:
    """
    Checks for spatial conflict between two missions using smooth B-spline trajectories.

    It generates trajectory curves for both missions, finds the point of closest approach
    in 3D space, and checks if the separation distance is less than the safety buffer.

    Args:
        mission1: First mission object.
        mission2: Second mission object.
        safety_buffer: Minimum required separation distance in meters.
        num_samples: Number of samples for discretization (higher = more accurate but slower).

    Returns:
        A Conflict object if a violation is detected, otherwise None.
    
    Example:
        >>> conflict = check_trajectory_conflict(m1, m2, safety_buffer=5.0)
        >>> if conflict:
        ...     print(conflict.description)
    """
    # Validate inputs
    if safety_buffer <= 0:
        print(f"Warning: Safety buffer must be positive. Received {safety_buffer}.")
        return None
    
    if len(mission1.waypoints) < 2 or len(mission2.waypoints) < 2:
        print(f"Error: Missions must have at least 2 waypoints. Skipping checks for {mission1.mission_id} vs {mission2.mission_id}.")
        return None

    # a. Trajectory Generation
    try:
        tck1, _ = geometry_utils.create_trajectory_curve(mission1.waypoints)
        tck2, _ = geometry_utils.create_trajectory_curve(mission2.waypoints)
    except Exception as e:
        print(f"Error creating trajectories for {mission1.mission_id} or {mission2.mission_id}: {e}")
        return None

    # b. Spatial Conflict Detection
    # Returns (min_dist, p1, p2, u1, u2)
    min_dist, p1, p2, u1, u2 = geometry_utils.closest_approach_between_curves(tck1, tck2, num_samples)
    
    # c. Safety Buffer Validation
    if not geometry_utils.violates_safety_buffer(min_dist, safety_buffer):
        # Safe
        # print(f"No conflict between {mission1.mission_id} and {mission2.mission_id}: separation {min_dist:.2f}m >= buffer {safety_buffer:.2f}m")
        return None
    
    # d. Conflict Object Creation
    # Use the midpoint or the point on the first curve as the "location"
    # p1 is a numpy array [x, y, z]
    conflict_loc = (float(p1[0]), float(p1[1]), float(p1[2]))
    
    description = format_conflict_description(
        mission1.mission_id, 
        mission2.mission_id, 
        conflict_loc, 
        min_dist, 
        safety_buffer
    )
    
    conflict = Conflict(
        mission_ids=(mission1.mission_id, mission2.mission_id),
        conflict_location=conflict_loc,
        conflict_time=0.0, # Placeholder: Temporal analysis not implemented
        separation_distance=float(min_dist),
        safety_buffer=safety_buffer,
        description=description
    )
    
    # print(f"CONFLICT DETECTED: {mission1.mission_id} vs {mission2.mission_id} at {min_dist:.2f}m")
    return conflict

def format_conflict_description(mission_id1: str, mission_id2: str, location: Tuple[float, float, float], separation_distance: float, safety_buffer: float) -> str:
    """
    Creates a human-readable string describing the spatial conflict.

    Args:
        mission_id1: ID of first mission.
        mission_id2: ID of second mission.
        location: (x, y, z) tuple of approximate closest approach.
        separation_distance: The actual minimum distance found.
        safety_buffer: The required minimum distance.

    Returns:
        Formatted description string.
    """
    violation = safety_buffer - separation_distance
    return (
        f"SPATIAL CONFLICT DETECTED\n"
        f"    Missions: {mission_id1} <-> {mission_id2}\n"
        f"    Closest Approach Location: ({location[0]:.2f}, {location[1]:.2f}, {location[2]:.2f}) meters\n"
        f"    Actual Separation: {separation_distance:.2f} m\n"
        f"    Required Safety Buffer: {safety_buffer:.2f} m\n"
        f"    Violation Margin: {violation:.2f} m\n"
        f"    \n"
        f"    Note: Temporal analysis not implemented - conflict location represents spatial closest approach only."
    )

def validate_mission_pair(mission1: Mission, mission2: Mission, safety_buffer: float, num_samples: int = 100, verbose: bool = False) -> Tuple[bool, Optional[Conflict]]:
    """
    Convenience wrapper to check a pair of missions.

    Args:
        mission1: First mission.
        mission2: Second mission.
        safety_buffer: Safety buffer distance.
        num_samples: Sampling density.
        verbose: If True, prints status messages.

    Returns:
        Tuple (has_conflict, conflict_object).
    """
    if verbose:
        print(f"Checking {mission1.mission_id} vs {mission2.mission_id} (Buffer: {safety_buffer}m)...")
    
    conflict = check_trajectory_conflict(mission1, mission2, safety_buffer, num_samples)
    
    if conflict:
        if verbose:
            print(conflict.description)
        return (True, conflict)
    else:
        if verbose:
            print(f"  -> SAFE.")
        return (False, None)

def check_multiple_missions(primary_mission: Mission, other_missions: List[Mission], safety_buffer: float, num_samples: int = 100) -> List[Conflict]:
    """
    Checks a primary mission against a list of other missions.

    Args:
        primary_mission: The mission being validated.
        other_missions: List of existing missions to check against.
        safety_buffer: Safety buffer distance.
        num_samples: Sampling density.

    Returns:
        List of detected Conflict objects, sorted by severity (closest separation first).
    """
    print(f"Checking primary mission {primary_mission.mission_id} against {len(other_missions)} other missions...")
    conflicts = []
    
    for other in other_missions:
        if primary_mission.mission_id == other.mission_id:
            continue # Skip self by ID
            
        conflict = check_trajectory_conflict(primary_mission, other, safety_buffer, num_samples)
        if conflict:
            conflicts.append(conflict)
    
    # Sort by separation distance (ascending = closest/most dangerous first)
    conflicts.sort(key=lambda c: c.separation_distance)
    
    print(f"Found {len(conflicts)} conflicts.")
    return conflicts


# ============================================================================
# TEMPORAL CONFLICT DETECTION - FUTURE IMPLEMENTATION PLACEHOLDER
# ============================================================================

def check_trajectory_conflict_with_temporal(mission1: Mission, mission2: Mission, safety_buffer: float, temporal_tolerance: float = 0.5, num_samples: int = 100) -> Optional[Conflict]:
    """
    Checks for SPATIOTEMPORAL conflict between two missions.
    
    A conflict is reported ONLY if:
    1. Spatial separation < safety_buffer
    2. Temporal separation at that point < temporal_tolerance
    
    Args:
        mission1: First mission.
        mission2: Second mission.
        safety_buffer: Min spatial separation (meters).
        temporal_tolerance: Min temporal separation (seconds) to be considered 'simultaneous'.
        num_samples: Sampling density.
        
    Returns:
        Conflict object if both conditions met, else None.
    """
    # 1. Spatial Pre-check (Fast)
    if safety_buffer <= 0:
        return None
    if len(mission1.waypoints) < 2 or len(mission2.waypoints) < 2:
        return None

    try:
        tck1, _ = geometry_utils.create_trajectory_curve(mission1.waypoints)
        tck2, _ = geometry_utils.create_trajectory_curve(mission2.waypoints)
    except Exception as e:
        print(f"Error creating trajectories: {e}")
        return None

    # 2. Find Spatial Closest Approach
    min_dist, p1, p2, u1, u2 = geometry_utils.closest_approach_between_curves(tck1, tck2, num_samples)
    
    # If spatially safe, return None immediately
    if not geometry_utils.violates_safety_buffer(min_dist, safety_buffer):
        return None
        
    # 3. Temporal Check
    # Estimate time at the point of closest approach
    t1 = geometry_utils.estimate_time_at_parameter(u1, mission1.waypoints)
    t2 = geometry_utils.estimate_time_at_parameter(u2, mission2.waypoints)
    
    time_diff = abs(t1 - t2)
    
    if time_diff >= temporal_tolerance:
        # Spatially close, but separated by time -> SAFE
        return None
        
    # 4. Conflict Confirmed
    conflict_loc = (float(p1[0]), float(p1[1]), float(p1[2]))
    
    description = (
        f"SPATIOTEMPORAL CONFLICT DETECTED\n"
        f"    Missions: {mission1.mission_id} <-> {mission2.mission_id}\n"
        f"    Location: ({conflict_loc[0]:.2f}, {conflict_loc[1]:.2f}, {conflict_loc[2]:.2f})\n"
        f"    Spatial Separation: {min_dist:.2f} m\n"
        f"    Time Difference: {time_diff:.2f} s (Tolerance: {temporal_tolerance}s)\n"
    )
    
    conflict = Conflict(
        mission_ids=(mission1.mission_id, mission2.mission_id),
        conflict_location=conflict_loc,
        conflict_time=t1, # Time of occurrence for mission 1
        separation_distance=float(min_dist),
        safety_buffer=safety_buffer,
        description=description
    )
    
    return conflict
