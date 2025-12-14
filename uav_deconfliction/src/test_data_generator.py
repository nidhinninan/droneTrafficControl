"""
Test Data Generator Module

This module creates various test scenarios to validate the deconfliction system.
It generates Mission objects with realistic waypoint configurations that form 
smooth B-spline trajectories. All missions are designed for 3D spatial testing.
"""

from typing import List, Tuple, Optional
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

try:
    from data_models import Mission, Waypoint, Conflict
    import geometry_utils
except ImportError:
    from .data_models import Mission, Waypoint, Conflict
    from . import geometry_utils

# ============================================================================
# BASIC MISSION GENERATORS
# ============================================================================

def generate_curved_mission(mission_id: str, waypoints_3d: List[Tuple[float, float, float]], base_velocity: float = 10.0) -> Mission:
    """
    Creates a Mission object from a list of 3D waypoints.

    This is the core function for creating test missions. The waypoints provided
    will be interpolated into smooth curves by geometry_utils during analysis.
    Timestamps are currently placeholders (sequential) as temporal analysis 
    is not yet implemented.

    Args:
        mission_id: Unique string identifier.
        waypoints_3d: List of (x, y, z) coordinates.
        base_velocity: Nominal speed (m/s) used for placeholder timestamps.

    Returns:
        A Mission object ready for testing.
    """
    waypoints = []
    for i, p in enumerate(waypoints_3d):
        # Placeholder timestamp logic: sequential seconds
        # In a real system, this would calculate dist / velocity
        t = float(i) 
        waypoints.append(Waypoint(x=p[0], y=p[1], z=p[2], timestamp=t))
        
    return Mission(mission_id=mission_id, waypoints=waypoints)


def generate_double_loop_spiral_mission(mission_id: str, num_waypoints: int = 15, radius: float = 50.0, height_variation: float = 30.0, center: Tuple[float, float, float] = (0, 0, 50)) -> Mission:
    """
    Creates a mission following a 3D double-loop spiral pattern.

    Uses the parametric equations from geometry_utils to generate an interesting
    3D curved trajectory. This shows off the system's ability to handle complex
    non-linear paths with altitude variations.

    Args:
        mission_id: Identifier.
        num_waypoints: Number of points to define the spiral.
        radius: Horizontal radius of the loops.
        height_variation: Vertical amplitude.
        center: (x, y, z) center point.

    Returns:
        Mission object with spiral trajectory.
    """
    # leverage existing geometry utility
    points = geometry_utils.generate_double_loop_spiral_waypoints(
        num_waypoints=num_waypoints,
        radius=radius,
        height_variation=height_variation,
        center=center
    )
    return generate_curved_mission(mission_id, points)


def generate_straight_path_mission(mission_id: str, start_point: Tuple[float, float, float], end_point: Tuple[float, float, float], num_waypoints: int = 10) -> Mission:
    """
    Creates a mission with waypoints along a straight line in 3D space.

    Even though the waypoints are collinear, the underlying system treats
    them as a potentially smooth spline (which degenerates to a line).

    Args:
        mission_id: Identifier.
        start_point: (x, y, z) start.
        end_point: (x, y, z) end.
        num_waypoints: Number of waypoints to generate.

    Returns:
        Mission object.
    
    Example:
        >>> m = generate_straight_path_mission("Line1", (0,0,0), (100,0,0))
    """
    # Linear interpolation
    p0 = np.array(start_point)
    p1 = np.array(end_point)
    
    points = []
    for i in range(num_waypoints):
        fraction = i / (num_waypoints - 1)
        pt = p0 + (p1 - p0) * fraction
        points.append(tuple(pt))
        
    return generate_curved_mission(mission_id, points)


# ============================================================================
# SPATIAL CONFLICT TEST SCENARIOS
# ============================================================================

def generate_crossing_missions_3d(safety_buffer: float = 5.0) -> Tuple[Mission, Mission]:
    """
    Creates two missions that cross paths in 3D space.

    Mission A: Straight horizontal path along X.
    Mission B: Straight horizontal path along Y.
    Intersect at (50, 0, 50).

    This scenario tests the system's ability to detect trajectory intersections.
    Since both exist in the same space without temporal separation, this 
    should trigger a spatial conflict.

    Returns:
        Tuple (Mission A, Mission B).
    """
    m1 = generate_straight_path_mission(
        "CROSS_A", 
        start_point=(0, 0, 50), 
        end_point=(100, 0, 50), 
        num_waypoints=10
    )
    
    m2 = generate_straight_path_mission(
        "CROSS_B", 
        start_point=(50, -50, 50), 
        end_point=(50, 50, 50), 
        num_waypoints=10
    )
    
    return m1, m2


def generate_parallel_missions_3d(separation_distance: float, num_waypoints: int = 10, length: float = 100.0) -> Tuple[Mission, Mission]:
    """
    Creates two missions flying parallel paths in 3D space.

    Mission A: Along X-axis at Y=0.
    Mission B: Along X-axis at Y=separation_distance.

    Expected Behavior:
    - If separation_distance < safety_buffer -> CONFLICT.
    - If separation_distance >= safety_buffer -> SAFE.

    Args:
        separation_distance: Distance between parallel tracks.
        num_waypoints: Waypoints per mission.
        length: Length of each track.

    Returns:
        Tuple (Mission A, Mission B).
    """
    m1 = generate_straight_path_mission(
        "PARA_A",
        start_point=(0, 0, 50),
        end_point=(length, 0, 50),
        num_waypoints=num_waypoints
    )
    
    m2 = generate_straight_path_mission(
        "PARA_B",
        start_point=(0, separation_distance, 50),
        end_point=(length, separation_distance, 50),
        num_waypoints=num_waypoints
    )
    
    return m1, m2


def generate_altitude_separated_missions_3d(horizontal_overlap: bool = True, altitude_separation: float = 50.0) -> Tuple[Mission, Mission]:
    """
    Creates two missions that overlap horizontally but are separated by altitude.

    Tests 3D spatial reasoning to ensure functionality isn't limited to 2D checks.

    Expected Behavior:
    - If altitude_separation < safety_buffer -> CONFLICT.
    - If altitude_separation >= safety_buffer -> SAFE.

    Args:
        horizontal_overlap: If True, XY paths are identical.
        altitude_separation: Vertical distance in meters.

    Returns:
        Tuple (Mission A, Mission B).
    """
    # Mission A at z=50
    points_a = []
    # Create a small curve
    x_vals = np.linspace(0, 100, 10)
    for x in x_vals:
        points_a.append((x, 0, 50.0))
        
    m1 = generate_curved_mission("ALT_A", points_a)
    
    # Mission B at z=50+sep
    points_b = []
    offset_y = 0.0 if horizontal_overlap else 5.0
    for x in x_vals:
        points_b.append((x, offset_y, 50.0 + altitude_separation))
        
    m2 = generate_curved_mission("ALT_B", points_b)
    
    return m1, m2


def generate_offset_spirals(offset_distance: float, mission_ids: Tuple[str, str] = ("SPIRAL_1", "SPIRAL_2")) -> Tuple[Mission, Mission]:
    """
    Creates two double-loop spiral missions with a horizontal offset.

    Spiral 1: Centered at (0, 0, 50).
    Spiral 2: Centered at (offset_distance, 0, 55).

    Use Cases:
    - Small offset -> Conflict likely.
    - Large offset -> Clear.

    Returns:
        Tuple (Spiral 1, Spiral 2).
    """
    s1 = generate_double_loop_spiral_mission(
        mission_ids[0],
        radius=50.0,
        center=(0, 0, 50)
    )
    
    s2 = generate_double_loop_spiral_mission(
        mission_ids[1],
        radius=50.0,
        center=(offset_distance, 0, 55) # Slight vertical shift too
    )
    
    return s1, s2


def generate_complex_scenario_3d(num_missions: int = 5, airspace_size: float = 200.0, altitude_range: Tuple[float, float] = (30, 100)) -> List[Mission]:
    """
    Creates a realistic multi-mission scenario with variety.
    
    Includes a mix of straight lines, curves, and crossing paths at random
    positions within the defined airspace. Useful for batch processing tests.

    Args:
        num_missions: Total missions to generate.
        airspace_size: Cube size for the scenario.
        altitude_range: Min and max Z values.

    Returns:
        List of randomized Mission objects.
    """
    missions = []
    np.random.seed(42) # Deterministic for testing
    
    min_z, max_z = altitude_range
    
    for i in range(num_missions):
        m_type = np.random.choice(['straight', 'curve', 'crossing'])
        m_id = f"COMPLEX_{i:02d}"
        
        start = (
            np.random.uniform(0, airspace_size),
            np.random.uniform(0, airspace_size),
            np.random.uniform(min_z, max_z)
        )
        end = (
            np.random.uniform(0, airspace_size),
            np.random.uniform(0, airspace_size),
            np.random.uniform(min_z, max_z)
        )
        
        if m_type == 'straight' or m_type == 'crossing':
            m = generate_straight_path_mission(m_id, start, end)
        else: # curve (spiral-ish segment)
            m = generate_double_loop_spiral_mission(
                m_id, 
                num_waypoints=10, 
                radius=np.random.uniform(20, 50),
                center=start
            )
            
        missions.append(m)
        
    return missions


def generate_near_miss_scenario(miss_distance: float = 4.0, safety_buffer: float = 5.0) -> Tuple[Mission, Mission]:
    """
    Creates two missions that pass very close but don't quite intersect.
    
    Useful for testing the exact boundary conditions of the safety buffer.

    Args:
        miss_distance: The closest approach distance.
        safety_buffer: Reference for context (unused in generation, strictly output).

    Returns:
        Tuple (Mission A, Mission B).
    """
    # Mission A: (0,0,50) -> (100,0,50)
    # Mission B: (0, miss, 50) -> (100, miss, 50)
    # Parallel lines separated exactly by miss_distance
    
    return generate_parallel_missions_3d(
        separation_distance=miss_distance,
        num_waypoints=10
    )


# ============================================================================
# VISUALIZATION UTILITIES
# ============================================================================

def visualize_test_scenario(missions: List[Mission], title: str = "Test Scenario", conflicts: Optional[List[Conflict]] = None, output_path: Optional[str] = None):
    """
    Visualizes a test scenario with multiple missions.

    Creates a 3D matplotlib plot showing trajectory curves, waypoints,
    and highlights any conflict locations.

    Args:
        missions: List of missions to plot.
        title: Plot title.
        conflicts: List of Conflict objects to highlight.
        output_path: File path to save the plot image.

    Returns:
        The matplotlib Figure object.
    
    Example:
        >>> m1, m2 = generate_crossing_missions_3d()
        >>> visualize_test_scenario([m1, m2], output_path="cross.png")
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    colors = plt.cm.jet(np.linspace(0, 1, len(missions)))
    
    for i, mission in enumerate(missions):
        # Generate smooth curve for visualization
        try:
            tck, _ = geometry_utils.create_trajectory_curve(mission.waypoints)
            samples = geometry_utils.sample_trajectory_points(tck, 100)
            
            # Plot curve
            ax.plot(samples[:, 0], samples[:, 1], samples[:, 2], 
                   label=f'{mission.mission_id}', color=colors[i])
            
            # Plot waypoints
            wps = np.array([(w.x, w.y, w.z) for w in mission.waypoints])
            ax.scatter(wps[:, 0], wps[:, 1], wps[:, 2], color=colors[i], s=20, alpha=0.5)
            
        except Exception as e:
            print(f"Could not plot {mission.mission_id}: {e}")
            
    # Highlight conflicts
    if conflicts:
        for c in conflicts:
            loc = c.conflict_location
            ax.scatter([loc[0]], [loc[1]], [loc[2]], color='red', s=200, marker='X', zorder=100, label='Conflict')
            if len(conflicts) <= 5: # Avoid clutter if too many
                # Show separation and time if available
                time_str = f"\nt={c.conflict_time:.2f}s" if c.conflict_time is not None else ""
                ax.text(loc[0], loc[1], loc[2]+5, f"{c.separation_distance:.1f}m{time_str}", color='red')
    
    ax.set_title(title)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    
    # Avoid duplicate labels in legend
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys())
    
    if output_path:
        import os
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path)
        print(f"Scenario plot saved to {output_path}")
        
    plt.close(fig) # Close to prevent memory leaks if taking many plots
    return fig


# ============================================================================
# TEMPORAL TEST DATA GENERATION - FUTURE IMPLEMENTATION PLACEHOLDER
# ============================================================================

def generate_temporally_separated_missions(spatial_path_type: str = "crossing") -> Tuple[Mission, Mission]:
    """
    Creates two missions that cross paths in space but are separated in time.
    
    1. Generates standard crossing missions.
    2. Adjusts Mission B waypoints so its timestamps start 20 seconds LATER.
    
    This ensures that while their lines intersect, the drones are never there at the same time.
    """
    # Reuse spatial logic
    m1, m2 = generate_crossing_missions_3d()
    
    # modify m2 waypoints to have delayed timestamps
    # m1 takes roughly 10s (index based). 
    # If we delay m2 by 20s, it enters when m1 is long gone.
    time_offset = 20.0 
    
    for wp in m2.waypoints:
        wp.timestamp += time_offset
        
    m1.mission_id = "TEMP_SAFE_1"
    m2.mission_id = "TEMP_SAFE_2_DELAYED"
    
    return m1, m2

if __name__ == "__main__":
    # Simple self-test if run directly
    print("Generating sample test scenario...")
    missions = generate_complex_scenario_3d(num_missions=3)
    for m in missions:
        print(f"Generated {m.mission_id} with {len(m.waypoints)} waypoints")
    
    visualize_test_scenario(missions, title="Test Data Generator - Self Test", output_path="outputs/gen_test.png")
