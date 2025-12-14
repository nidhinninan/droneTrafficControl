import numpy as np
from scipy.interpolate import splprep, splev
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Tuple, Any, Optional

# Import Waypoint for type hints and usage
try:
    from data_models import Waypoint
except ImportError:
    # Fallback if running as a script from src/ or similar
    try:
        from .data_models import Waypoint
    except ImportError:
        # If running directly from src directory
        import sys
        import os
        sys.path.append(os.path.dirname(os.path.abspath(__file__)))
        from data_models import Waypoint

def create_trajectory_curve(waypoints: List[Waypoint]) -> Tuple[Tuple[Any, Any, int], np.ndarray]:
    """
    Creates a smooth flight trajectory curve passing through all waypoints using B-splines.
    
    This function generates a continuous flyable path that avoids the sharp corners
    typical of straight-line segments. It uses cubic splines (k=3) for smooth
    curvature (continuous second derivative) or lower degree for fewer waypoints.

    Args:
        waypoints: A list of Waypoint objects, sorted by timestamp.

    Returns:
        A tuple (tck, u) where:
        - tck: A tuple (knots, coefficients, degree) containing the B-spline representation.
        - u: An array of parameter values [0, 1] corresponding to each waypoint position.
    """
    xs = [w.x for w in waypoints]
    ys = [w.y for w in waypoints]
    zs = [w.z for w in waypoints]
    
    # Determine spline degree k
    # Need at least k+1 points for degree k. 
    # Default is k=3 (cubic), but reduce if fewer points.
    n_points = len(waypoints)
    k = min(3, n_points - 1)
    
    if k < 1:
        raise ValueError("Need at least 2 waypoints to create a trajectory.")

    # splprep returns:
    # tck: tuple (t, c, k) containing the vector of knots, the B-spline coefficients, and the degree of the spline.
    # u: array of the parameters for each input point.
    # s=0 ensures the spline passes through all points (interpolating).
    tck, u = splprep([xs, ys, zs], s=0, k=k)
    
    return tck, u

def evaluate_trajectory_at_parameter(tck, u_value: float) -> Tuple[float, float, float]:
    """
    Evaluates the 3D position of the trajectory at a specific parameter u.

    Args:
        tck: The B-spline representation (knots, coefficients, degree).
        u_value: A parameter value between 0.0 (start) and 1.0 (end).

    Returns:
        A tuple (x, y, z) representing the 3D coordinate at that parameter.

    Example:
        >>> pos_start = evaluate_trajectory_at_parameter(tck, 0.0) # Returns first waypoint coords
        >>> pos_mid = evaluate_trajectory_at_parameter(tck, 0.5)   # Returns a point ~midway through trajectory
    """
    # splev returns a list of arrays [x_val, y_val, z_val]
    out = splev(u_value, tck)
    return (float(out[0]), float(out[1]), float(out[2]))

def sample_trajectory_points(tck, num_samples: int = 100) -> np.ndarray:
    """
    Generates uniformly spaced sample points along the trajectory curve.

    These points allow for discrete distance calculations and visualization.
    More samples increase accuracy but increase computation time.

    Args:
        tck: The B-spline representation.
        num_samples: Number of points to sample (default 100).

    Returns:
        A numpy array of shape (num_samples, 3) containing 3D coordinates.
    """
    u_new = np.linspace(0, 1, num_samples)
    # splev returns [x_arr, y_arr, z_arr]
    res = splev(u_new, tck)
    # Transpose to get shape (num_samples, 3)
    points = np.array(res).T
    return points

def calculate_distance_3d(point1: np.ndarray, point2: np.ndarray) -> float:
    """
    Calculates the Euclidean distance between two 3D points.

    Args:
        point1: First point (x, y, z) or (x, y).
        point2: Second point (x, y, z) or (x, y).

    Returns:
        Euclidean distance.
    
    Example:
        >>> d = calculate_distance_3d(np.array([0,0,0]), np.array([3,4,0])) # Returns 5.0
    """
    return np.linalg.norm(np.array(point1) - np.array(point2))

def closest_approach_between_curves(tck1, tck2, num_samples: int = 100) -> Tuple[float, np.ndarray, np.ndarray, float, float]:
    """
    Finds the minimum distance between two smooth trajectories using discrete sampling.

    This function approximates the closest approach by sampling both curves densely
    and finding the minimum pairwise distance. This handles non-trivial geometries
    like spirals or loops where the closest point might not be a waypoint.

    Args:
        tck1: B-spline representation of the first trajectory.
        tck2: B-spline representation of the second trajectory.
        num_samples: Number of samples to use for discretization (higher = more accurate).

    Returns:
        A tuple (min_dist, p1, p2, u1, u2) containing:
        - min_dist: The minimum separation distance found.
        - p1: The 3D point on curve 1 at closest approach.
        - p2: The 3D point on curve 2 at closest approach.
        - u1: The parameter value on curve 1.
        - u2: The parameter value on curve 2.
    """
    # 1. Sample both curves
    u_vals = np.linspace(0, 1, num_samples)
    points1 = sample_trajectory_points(tck1, num_samples)
    points2 = sample_trajectory_points(tck2, num_samples)
    
    # 2. Compute pairwise distance matrix (num_samples x num_samples)
    dist_matrix = cdist(points1, points2, metric='euclidean')
    
    # 3. Find index of minimum value
    min_idx = np.unravel_index(np.argmin(dist_matrix), dist_matrix.shape)
    idx1, idx2 = min_idx
    
    min_dist = dist_matrix[idx1, idx2]
    p1 = points1[idx1]
    p2 = points2[idx2]
    u1 = u_vals[idx1]
    u2 = u_vals[idx2]
    
    return min_dist, p1, p2, u1, u2

def violates_safety_buffer(actual_distance: float, required_safety_buffer: float) -> bool:
    """
    Checks if the measured separation distance violates the required safety buffer.

    Args:
        actual_distance: The closest distance between two drones.
        required_safety_buffer: The minimum allowed distance.

    Returns:
        True if actual_distance < required_safety_buffer, else False.

    Example:
        >>> violates_safety_buffer(3.2, 5.0) 
        True
        >>> violates_safety_buffer(7.5, 5.0)
        False
    """
    return actual_distance < required_safety_buffer

def calculate_trajectory_length(tck, num_samples: int = 1000) -> float:
    """
    Calculates the total arc length of the trajectory.

    Samples the curve densely and sums the distances between consecutive points.

    Args:
        tck: The B-spline representation.
        num_samples: Number of segments to use for summation (default 1000).

    Returns:
        Total length in meters.
    """
    points = sample_trajectory_points(tck, num_samples)
    # Calculate difference between consecutive points
    diffs = np.diff(points, axis=0)
    # Calculate euclidean distance for each segment
    segment_lengths = np.linalg.norm(diffs, axis=1)
    return float(np.sum(segment_lengths))

# ==========================================
# TEST AND VISUALIZATION FUNCTIONS
# ==========================================

def generate_double_loop_spiral_waypoints(num_waypoints: int = 15, radius: float = 50, height_variation: float = 30, center=(0, 0, 50)) -> List[Tuple[float, float, float]]:
    """
    Generates waypoints forming a 3D double-loop spiral pattern.

    Mathematically creates a path that loops twice (4*pi) while oscillating vertically.
    
    Args:
        num_waypoints: Number of waypoints.
        radius: Horizontal radius of the loops.
        height_variation: Vertical amplitude.
        center: (x, y, z) center.

    Returns:
        List of (x, y, z) tuples.
    """
    cx, cy, cz = center
    waypoints = []
    
    # Generate points along the parameter theta from 0 to 4*pi
    thetas = np.linspace(0, 4 * np.pi, num_waypoints)
    
    for theta in thetas:
        # Double loop effect with slight modulation (1 + 0.3*sin(2*theta))
        mod = 1 + 0.3 * np.sin(2 * theta)
        x = cx + radius * np.cos(theta) * mod
        y = cy + radius * np.sin(theta) * mod
        # Gradual vertical oscillation
        z = cz + height_variation * np.sin(theta / 2.0)
        waypoints.append((x, y, z))
        
    return waypoints

def visualize_trajectory_with_waypoints(waypoints: List[Tuple[float, float, float]], trajectory_samples: np.ndarray, title: str = "Drone Trajectory", output_path: Optional[str] = None):
    """
    Visualizes the smooth trajectory and original waypoints in 3D.

    Args:
        waypoints: List of (x, y, z) tuples.
        trajectory_samples: Array of points (N, 3) from the smooth curve.
        title: Plot title.
        output_path: If provided, saves the figure to this path.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Unpack sample points
    xs_smooth = trajectory_samples[:, 0]
    ys_smooth = trajectory_samples[:, 1]
    zs_smooth = trajectory_samples[:, 2]
    
    # Unpack waypoints
    wx = [p[0] for p in waypoints]
    wy = [p[1] for p in waypoints]
    wz = [p[2] for p in waypoints]
    
    # Plot smooth curve
    ax.plot(xs_smooth, ys_smooth, zs_smooth, label='Smooth Trajectory', color='blue', linewidth=2)
    
    # Plot waypoints
    ax.scatter(wx, wy, wz, c='red', s=50, marker='o', label='Waypoints', depthshade=False)
    
    # Connect waypoints with dashed lines to show order/linear path
    ax.plot(wx, wy, wz, color='gray', linestyle='--', linewidth=0.5, alpha=0.5, label='Linear Path')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)
    ax.legend()
    ax.grid(True)
    
    # Set equal aspect ratio approximate
    # (Matplotlib 3D doesn't have 'equal' aspect built-in simply, but we can set limits)
    max_range = np.array([xs_smooth.max()-xs_smooth.min(), ys_smooth.max()-ys_smooth.min(), zs_smooth.max()-zs_smooth.min()]).max() / 2.0
    mid_x = (xs_smooth.max()+xs_smooth.min()) * 0.5
    mid_y = (ys_smooth.max()+ys_smooth.min()) * 0.5
    mid_z = (zs_smooth.max()+zs_smooth.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    if output_path:
        plt.savefig(output_path)
        print(f"Plot saved to {output_path}")
    else:
        plt.show()

def test_trajectory_curve_generation():
    print("\n=== Test 1: Trajectory Curve Generation ===")
    
    # a. Generate Waypoints
    wp_tuples = generate_double_loop_spiral_waypoints(num_waypoints=15)
    
    # Convert to Waypoint objects
    # Only x, y, z matter for curve generation, timestamp is just required by constructor
    waypoints = [Waypoint(x=p[0], y=p[1], z=p[2], timestamp=float(i)) for i, p in enumerate(wp_tuples)]
    
    # b. Print waypoints
    print(f"Generated {len(waypoints)} waypoints.")
    # print(waypoints[0])
    
    # c. Create Trajectory
    try:
        tck, u = create_trajectory_curve(waypoints)
        print(f"Trajectory created. Spline degree: {tck[2]}")
        print(f"Number of knots: {len(tck[0])}")
    except Exception as e:
        print(f"Error creating trajectory: {e}")
        return

    # d. Sample Curve
    samples = sample_trajectory_points(tck, num_samples=200)
    
    # e. Calculate Length
    length = calculate_trajectory_length(tck)
    print(f"Total Trajectory Length: {length:.2f} meters")
    
    # f. Visualize
    output_file = "outputs/trajectory_test.png"
    # Make sure output dir exists
    import os
    os.makedirs(os.path.dirname(output_file) if os.path.dirname(output_file) else '.', exist_ok=True)
    
    visualize_trajectory_with_waypoints(wp_tuples, samples, title="Test 1: Double Loop Spiral", output_path=output_file)

def test_closest_approach_between_spirals():
    print("\n=== Test 2: Closest Approach Between Two Spirals ===")
    
    # 1. Generate two spirals
    wp_tuples1 = generate_double_loop_spiral_waypoints(num_waypoints=15, center=(0, 0, 50))
    wp_tuples2 = generate_double_loop_spiral_waypoints(num_waypoints=15, center=(30, 20, 55)) # Shifted
    
    wps1 = [Waypoint(x=p[0], y=p[1], z=p[2], timestamp=float(i)) for i, p in enumerate(wp_tuples1)]
    wps2 = [Waypoint(x=p[0], y=p[1], z=p[2], timestamp=float(i)) for i, p in enumerate(wp_tuples2)]
    
    # 2. Create Curves
    tck1, _ = create_trajectory_curve(wps1)
    tck2, _ = create_trajectory_curve(wps2)
    
    # 3. Calculate Closest Approach
    min_dist, p1, p2, u1, u2 = closest_approach_between_curves(tck1, tck2, num_samples=200)
    
    print(f"Minimum Distance: {min_dist:.4f} meters")
    print(f"Closest Point on Curve 1 (u={u1:.2f}): {p1}")
    print(f"Closest Point on Curve 2 (u={u2:.2f}): {p2}")
    
    # 4. Visualization
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    samples1 = sample_trajectory_points(tck1, 200)
    samples2 = sample_trajectory_points(tck2, 200)
    
    ax.plot(samples1[:, 0], samples1[:, 1], samples1[:, 2], label='Trajectory 1', color='blue')
    ax.plot(samples2[:, 0], samples2[:, 1], samples2[:, 2], label='Trajectory 2', color='green')
    
    # Draw line connecting closest points
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color='red', linewidth=3, label='Closest Approach')
    
    # Mark points
    ax.scatter([p1[0]], [p1[1]], [p1[2]], color='red', s=100, marker='X')
    ax.scatter([p2[0]], [p2[1]], [p2[2]], color='red', s=100, marker='X')
    
    # Annotation
    mid_x = (p1[0] + p2[0]) / 2
    mid_y = (p1[1] + p2[1]) / 2
    mid_z = (p1[2] + p2[2]) / 2
    ax.text(mid_x, mid_y, mid_z + 2, f"Dist: {min_dist:.2f}m", color='red', fontsize=12)
    
    ax.set_title("Test 2: Closest Approach of Two UAVs")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    
    output_file = "outputs/closest_approach_test.png"
    plt.savefig(output_file)
    print(f"Plot saved to {output_file}")


if __name__ == "__main__":
    # Ensure outputs directory exists
    import os
    if not os.path.exists('outputs'):
        os.makedirs('outputs')
        
    test_trajectory_curve_generation()
    test_closest_approach_between_spirals()
