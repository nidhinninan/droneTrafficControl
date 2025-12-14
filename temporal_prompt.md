# Prompt: Implement Temporal (4D) Deconfliction for UAV Trajectories

**Objective:**
Extend the existing 3D spatial deconfliction system to become a 4D system (3D space + time) by implementing the temporal logic placeholders. Use the existing B-spline trajectory architecture.

**Context:**
The current system (`uav_deconfliction/`) correctly identifies spatial conflicts using B-splines (`scipy.interpolate.splprep`). It has placeholders for temporal checks in `conflict_detector.py` and `mission_validator.py`. Drone missions consist of `Waypoints` which already contain `timestamp` fields.

**Core Logic (Adaptation):**
We will adapt the segment-based "constant velocity" logic to the spline architecture.
1.  **Time Interpolation**: Since the spatial curve is parameterized by `u` (0 to 1), we must map any `u` value back to a specific timestamp.
    *   *Assumption*: Drones move with constant velocity *between waypoints*.
    *   *Implementation*: Map the spline parameter `u` to the corresponding segment indices and interpolate time linearly between the adjacent waypoints.
2.  **Conflict Definition**: A conflict exists if and *only if*:
    *   **Spatial**: The minimal distance between trajectories < `safety_buffer`.
    *   **Temporal**: The time difference at the point of closest approach is < `temporal_tolerance`.

---

## Instructions by File

### 1. Update `src/geometry_utils.py`
Add time calculation functions to support 4D analysis.

*   **Function**: `estimate_time_at_parameter(u, waypoints)`
    *   **Inputs**:
        *   `u` (float): The B-spline parameter (0.0 to 1.0) where the conflict occurs.
        *   `waypoints` (List[Waypoint]): The mission waypoints containing timestamps.
    *   **Logic**:
        *   The spline represents the path through *N* waypoints.
        *   In a uniform spline (as currently implemented with default `splprep`), `u` values correspond roughly to the normalized index of waypoints (e.g., with 4 waypoints, they are roughly at u=0.0, 0.33, 0.66, 1.0).
        *   *Better Approach*: Use the `u` value returned by `closest_approach_between_curves`. If necessary, assume `u` scales linearly with the *accumulated chord length* (distance) along the path, or simply map `u` linearly to the waypoint list index if `splprep` was called without custom weights.
        *   *Simplified Requirement*: Calculate `total_u_span` (usually 0 to 1). Find which "segment" of the spline `u` falls into based on the number of waypoints. Linearly interpolate the timestamp between the two bounding waypoints.
        *   *Return*: The estimated timestamp (float).

### 2. Update `src/conflict_detector.py`
Implement the placeholder `check_trajectory_conflict_with_temporal`.

*   **Function**: `check_trajectory_conflict_with_temporal`
    *   **Signature**: `(mission1, mission2, safety_buffer, temporal_tolerance=0.5, num_samples=100)`
    *   **Logic**:
        1.  Call `geometry_utils.closest_approach_between_curves` (same as spatial check).
        2.  Check if `min_dist < safety_buffer`. If **False**, return `None` (Safe).
        3.  If **Spatial Conflict**:
            *   Retrieve `u1` and `u2` (the parameters on the two curves where closest approach occurs).
            *   Calculate `t1 = geometry_utils.estimate_time_at_parameter(u1, mission1.waypoints)`.
            *   Calculate `t2 = geometry_utils.estimate_time_at_parameter(u2, mission2.waypoints)`.
            *   Calculate `time_diff = abs(t1 - t2)`.
        4.  **Temporal Check**:
            *   If `time_diff < temporal_tolerance`, it is a **Real Conflict**.
            *   Else, it is **Safe** (spatially close, but at different times).
        5.  **Return**:
            *   If conflict: Create `Conflict` object (and populate the previously null `conflict_time` field with `t1`).
            *   If safe: Return `None`.

### 3. Update `src/mission_validator.py`
Implement the high-level temporal validation placeholders.

*   **Function**: `validate_mission_with_temporal`
    *   Mirror `validate_mission` but call `check_trajectory_conflict_with_temporal`.
    *   Update logic to handle the new `temporal_tolerance` parameter.
*   **Reporting**:
    *   Update `generate_validation_report` to include "Temporal Tolerance" in the header.
    *   If a conflict is detected, display the `Time Difference` in the conflict details to show why it failed.

### 4. Update `src/test_data_generator.py`
Implement the temporal test checking capability.

*   **Function**: `generate_temporally_separated_missions()`
    *   Create two missions with crossing paths (use `generate_crossing_missions_3d`).
    *   **Crucial Step**: Adjust the `timestamp`s of the second mission so that it reaches the crossing point 10 seconds *after* the first mission.
    *   Return the two missions.

### 5. Create `tests/test_temporal_logic.py`
Create a new test file using `unittest`.

*   **Test**: `test_temporal_separation_clear`
    *   Use `generate_temporally_separated_missions`.
    *   Assert `validate_mission` (spatial only) returns **False** (Conflict).
    *   Assert `validate_mission_with_temporal` returns **True** (Clear).
*   **Test**: `test_spatiotemporal_conflict`
    *   Create two crossing missions with identical timestamps.
    *   Assert `validate_mission_with_temporal` returns **False** (Conflict).

---

**Output Requirements:**
*   Modify the specified files in place.
*   Ensure backward compatibility (spatial-only functions must still work).
*   Run the new `tests/test_temporal_logic.py` to verify 4D functionality.
