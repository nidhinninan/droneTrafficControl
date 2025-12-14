"""
Mission Validator Module

This module serves as the high-level interface for UAV mission validation.
It orchestrates conflict detection, generates safety reports, and handles
result exportation. This is the primary entry point for determining if a
mission plan is safe to execute in the current airspace.

Current Capabilities:
- Spatial Safety Checks: Validates missions against 3D spatial conflict criteria.
- Report Generation: Creates detailed human-readable validation reports.
- Data Export: Supports JSON, CSV, and text export of validation results.
- Batch Processing: Validation of multiple missions against traffic.

Future Enhancements:
- Temporal Deconfliction: Time-based safety logic (currently placeholders).
"""

from typing import List, Tuple, Optional, Dict, Any
import json
import csv
import os
from pathlib import Path

# Internal imports
try:
    from data_models import Mission, Conflict, Waypoint
    import conflict_detector
    import geometry_utils
except ImportError:
    # Fallback for relative imports or running as script
    from .data_models import Mission, Conflict, Waypoint
    from . import conflict_detector
    from . import geometry_utils


# ============================================================================
# MAIN VALIDATION FUNCTIONS
# ============================================================================

def validate_mission(primary_mission: Mission, other_missions: List[Mission], safety_buffer: float = 5.0, num_samples: int = 100, verbose: bool = False) -> Tuple[bool, List[Conflict], str]:
    """
    Validates a primary mission against a set of existing traffic missions.

    Performs a comprehensive check for spatial conflicts. If any part of the
    trajectory violates the safety buffer with any other mission, validation fails.

    Args:
        primary_mission: The mission plan to validate.
        other_missions: List of other active missions/traffic.
        safety_buffer: Minimum required separation distance in meters (default 5.0).
        num_samples: Sampling density for collision checking (default 100).
        verbose: If True, prints detailed progress logs.

    Returns:
        A tuple containing:
        - is_clear (bool): True if safe, False if conflicts detected.
        - conflicts (List[Conflict]): List of conflict details (empty if safe).
        - summary_report (str): Detailed text report of the validation.

    Example:
        >>> safe, conflicts, report = validate_mission(my_mission, traffic)
        >>> if safe:
        ...     print("Mission Safe!")
    """
    # a. INPUT VALIDATION
    if not primary_mission or len(primary_mission.waypoints) < 2:
        error_msg = f"Invalid primary mission {primary_mission.mission_id if primary_mission else 'None'}: Insufficient waypoints."
        if verbose: print(error_msg)
        return (False, [], error_msg)

    if safety_buffer <= 0:
        error_msg = f"Invalid safety buffer: {safety_buffer}. Must be positive."
        if verbose: print(error_msg)
        return (False, [], error_msg)

    if verbose:
        print(f"Validating {primary_mission.mission_id} against {len(other_missions)} traffic missions (Buffer: {safety_buffer}m)")

    # b. TRAJECTORY GENERATION FOR PRIMARY MISSION (Pre-check)
    try:
        # Just ensure we can generate it, though check_trajectory_conflict will do it again.
        # Ideally, we cache this, but for simplicity we rely on the detector function.
        _ = geometry_utils.create_trajectory_curve(primary_mission.waypoints)
        if verbose: print(f"Primary mission {primary_mission.mission_id} trajectory structure valid.")
    except Exception as e:
        error_msg = f"Failed to generate trajectory for primary mission: {e}"
        if verbose: print(error_msg)
        return (False, [], error_msg)

    # c. CONFLICT DETECTION LOOP
    conflicts = []
    
    # Check duplicates or self-check edge cases
    active_traffic = [m for m in other_missions if m.mission_id != primary_mission.mission_id]

    if verbose and not active_traffic:
        print("No conflicting traffic passed in.")

    # We can reuse the batch logic from conflict_detector, but we implement the loop here as specified
    # to maintain the requested structure and verbose logging control.
    for i, other in enumerate(active_traffic):
        if verbose:
            print(f"Checking against mission {i+1}/{len(active_traffic)} ({other.mission_id})...")
        
        try:
            conflict = conflict_detector.check_trajectory_conflict(
                primary_mission, other, safety_buffer, num_samples
            )
            if conflict:
                conflicts.append(conflict)
        except Exception as e:
            print(f"Warning: Error checking {primary_mission.mission_id} vs {other.mission_id}: {e}")
            # Continue checking others

    if verbose:
        print(f"Conflict detection complete. Found {len(conflicts)} conflicts.")

    # d. DETERMINE VALIDATION STATUS
    is_clear = (len(conflicts) == 0)
    
    if is_clear:
        if verbose: print("✓ Mission APPROVED - No spatial conflicts detected")
    else:
        if verbose: print(f"✗ Mission REJECTED - {len(conflicts)} spatial conflict(s) detected")

    # e. GENERATE SUMMARY REPORT
    summary_report = generate_validation_report(primary_mission, conflicts, is_clear)

    # f. RETURN RESULTS
    return (is_clear, conflicts, summary_report)


def generate_validation_report(primary_mission: Mission, conflicts: List[Conflict], is_clear: bool) -> str:
    """
    Generates a detailed text report of the validation results.
    
    Args:
        primary_mission: The validated mission.
        conflicts: List of detected conflicts.
        is_clear: Boolean safety status.

    Returns:
        Formatted string report.
    """
    # Helper to calculate length safely
    traj_len = "N/A"
    try:
        tck, _ = geometry_utils.create_trajectory_curve(primary_mission.waypoints)
        traj_len = f"{geometry_utils.calculate_trajectory_length(tck):.2f}"
    except:
        pass

    safety_buffer = conflicts[0].safety_buffer if conflicts else "N/A" # Get from conflict or unknown if clear
    # If clear, we don't have safety_buffer in arguments explicitly here unless we change signature.
    # We can infer it or just say "As Requested". 
    # NOTE: The prompted signature for this function did not include safety_buffer, so we'll omit or genericize.
    
    status_str = "✓ APPROVED" if is_clear else "✗ REJECTED"
    summary_str = "No spatial conflicts found - mission is clear to execute" if is_clear else "Spatial conflicts detected - mission cannot proceed as planned"
    
    report = []
    report.append("=" * 80)
    report.append("UAV STRATEGIC DECONFLICTION - MISSION VALIDATION REPORT")
    report.append("=" * 80)
    report.append("")
    report.append(f"PRIMARY MISSION: {primary_mission.mission_id}")
    report.append(f"Waypoints: {len(primary_mission.waypoints)}")
    report.append(f"Trajectory Length: {traj_len} meters")
    report.append("")
    report.append(f"VALIDATION STATUS: {status_str}")
    report.append("")
    report.append("CONFLICT SUMMARY:")
    report.append(f"Total Conflicts Detected: {len(conflicts)}")
    report.append(summary_str)
    report.append("")
    
    if conflicts:
        for i, c in enumerate(conflicts):
            report.append(f"CONFLICT #{i+1}:")
            # Indent the description slightly
            desc_lines = c.description.split('\n')
            for line in desc_lines:
                report.append(line)
            report.append("---")
            report.append("")

    report.append("RECOMMENDATIONS:")
    if is_clear:
        report.append("- Mission approved for execution")
        report.append("- Maintain safety buffer during flight")
        report.append("- Monitor for any dynamic airspace changes")
    else:
        report.append("- Modify mission waypoints to avoid conflict zones")
        report.append("- Consider altitude adjustments if conflicts are near same Z-level")
        report.append("- Reschedule mission (temporal deconfliction not yet implemented)")
        report.append("- Contact air traffic coordination for assistance")
    
    report.append("")
    report.append("NOTE: This report reflects SPATIAL analysis only. Temporal deconfliction")
    report.append("      (time-based separation) has not been implemented yet.")
    report.append("=" * 80)
    
    return "\n".join(report)


# ============================================================================
# ANALYSIS HELPERS
# ============================================================================

def find_worst_conflict(conflicts: List[Conflict]) -> Optional[Conflict]:
    """
    Identifies the conflict with the closest separation distance (most dangerous).

    Args:
        conflicts: List of Conflict objects.

    Returns:
        The Conflict object with min separation, or None if list is empty.
    """
    if not conflicts:
        return None
    return min(conflicts, key=lambda c: c.separation_distance)


def categorize_conflicts_by_severity(conflicts: List[Conflict], safety_buffer: float) -> Dict[str, List[Conflict]]:
    """
    Sorts conflicts into severity buckets based on violation margin.

    Categories:
    - CRITICAL: Separation < 50% of buffer
    - SEVERE: Separation < 75% of buffer
    - MODERATE: Separation < 100% of buffer

    Args:
        conflicts: List of Conflict objects.
        safety_buffer: The buffer used for validation.

    Returns:
        Dictionary with keys 'critical', 'severe', 'moderate'.
    """
    categories = {
        "critical": [],
        "severe": [],
        "moderate": []
    }
    
    for c in conflicts:
        ratio = c.separation_distance / safety_buffer
        if ratio < 0.5:
            categories["critical"].append(c)
        elif ratio < 0.75:
            categories["severe"].append(c)
        else:
            categories["moderate"].append(c)
            
    return categories


# ============================================================================
# I/O AND BATCH HELPERS
# ============================================================================

def export_validation_results(validation_results: Tuple[bool, List[Conflict], str], output_path: str, format: str = 'json') -> bool:
    """
    Exports validation results to a file.

    Args:
        validation_results: The tuple returned by validate_mission (is_clear, conflicts, report).
        output_path: Destination file path.
        format: 'json', 'txt', or 'csv'.

    Returns:
        True if successful, False on error.
    """
    is_clear, conflicts, summary_report = validation_results
    
    try:
        # Create directory if needed
        out_dir = os.path.dirname(output_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
            
        if format.lower() == 'json':
            data = {
                "mission_status": "APPROVED" if is_clear else "REJECTED",
                "is_clear": is_clear,
                "num_conflicts": len(conflicts),
                "conflicts": [],
                "summary": summary_report
            }
            
            for c in conflicts:
                c_dict = {
                    "mission_ids": c.mission_ids,
                    "location": c.conflict_location,
                    "separation_distance": c.separation_distance,
                    "safety_buffer": c.safety_buffer,
                    "violation_margin": c.get_violation_margin(),
                    "description": c.description
                }
                data["conflicts"].append(c_dict)
                
            with open(output_path, 'w') as f:
                json.dump(data, f, indent=2)

        elif format.lower() == 'txt':
            with open(output_path, 'w') as f:
                f.write(summary_report)
                
        elif format.lower() == 'csv':
            # Flat CSV format for conflicts
            with open(output_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["mission_id_1", "mission_id_2", "loc_x", "loc_y", "loc_z", "separation_dist", "safety_buffer", "violation_margin"])
                
                for c in conflicts:
                    writer.writerow([
                        c.mission_ids[0],
                        c.mission_ids[1],
                        c.conflict_location[0],
                        c.conflict_location[1],
                        c.conflict_location[2],
                        c.separation_distance,
                        c.safety_buffer,
                        c.get_violation_margin()
                    ])
        else:
            print(f"Error: Unknown format '{format}'")
            return False
            
        return True
        
    except Exception as e:
        print(f"Error exporting results to {output_path}: {e}")
        return False


def batch_validate_missions(missions_to_validate: List[Mission], existing_traffic: List[Mission], safety_buffer: float = 5.0, num_samples: int = 100) -> Dict[str, Tuple[bool, List[Conflict], str]]:
    """
    Validates a list of missions against existing traffic efficiently.

    Args:
        missions_to_validate: List of new missions to check.
        existing_traffic: Background traffic missions.
        safety_buffer: Safety buffer distance.
        num_samples: Sampling density.

    Returns:
        Dictionary mapping mission_id -> (is_clear, conflicts, report).
    """
    results = {}
    total = len(missions_to_validate)
    print(f"Batch validating {total} missions...")
    
    for i, mission in enumerate(missions_to_validate):
        res = validate_mission(mission, existing_traffic, safety_buffer, num_samples, verbose=False)
        results[mission.mission_id] = res
        print(f"Validated {i+1}/{total} missions")
        
    return results


# ============================================================================
# TEMPORAL VALIDATION - FUTURE IMPLEMENTATION PLACEHOLDER
# ============================================================================

def validate_mission_with_temporal(primary_mission: Mission, other_missions: List[Mission], safety_buffer: float = 5.0, num_samples: int = 100):
   """
   FUTURE IMPLEMENTATION: Mission Validation with Temporal Deconfliction
   
   This function will extend validate_mission() to include temporal reasoning.
   It will ensure that missions are not only spatially separated, but also
   that drones don't occupy the same space at the same time.
   
   Planned approach:
   - Perform spatial conflict detection as in validate_mission()
   - For each spatial conflict detected:
       * Calculate when primary mission drone reaches conflict location
       * Calculate when other mission drone reaches conflict location
       * Check if these time windows overlap
       * Only report conflict if BOTH spatial AND temporal violations occur
   - This will allow missions with crossing paths that are temporally separated
   
   Parameters:
       primary_mission (Mission): Mission to validate with timestamped waypoints
       other_missions (List[Mission]): Existing traffic with timestamps
       safety_buffer (float): Minimum spatial separation required
       num_samples (int): Trajectory sampling density
   
   Returns:
       Tuple[bool, List[Conflict], str]: Same as validate_mission but with
                                         temporal conflicts only
   
   Key differences from spatial-only validation:
   - Fewer conflicts detected (temporal separation filters out some)
   - Conflict objects include accurate conflict_time values
   - Report indicates both spatial and temporal analysis completed
   
   TODO:
   - Integrate conflict_detector.check_trajectory_conflict_with_temporal()
   - Update report generation to show temporal conflict details
   - Add time-based conflict categorization
   - Consider mission time window constraints (start_time, end_time)
   """
   raise NotImplementedError("Temporal validation not yet implemented")

def find_earliest_conflict(conflicts: List[Conflict]) -> Optional[Conflict]:
   """
   FUTURE HELPER FUNCTION: Find First Temporal Conflict
   
   Once temporal analysis is implemented, this will identify which conflict
   occurs earliest in time along the mission timeline.
   
   Parameters:
       conflicts (List[Conflict]): List of conflicts with conflict_time populated
   
   Returns:
       Optional[Conflict]: The conflict occurring earliest in time, or None
   
   Use case: Help operators understand the first problem they'll encounter
             in chronological order during mission execution
   
   TODO:
   - Filter conflicts where conflict_time is not None
   - Use min() with key=lambda c: c.conflict_time
   - Handle edge cases where all conflicts lack timestamps
   """
   raise NotImplementedError("Temporal conflict ordering not yet implemented")

def suggest_temporal_deconfliction_strategies(primary_mission: Mission, conflicts: List[Conflict]) -> List[str]:
   """
   FUTURE HELPER FUNCTION: Suggest Time-Based Conflict Resolution
   
   Given a mission with temporal conflicts, suggest ways to resolve them
   by adjusting mission timing without changing the spatial path.
   
   Potential strategies:
   - Delay mission start time by X seconds
   - Speed up/slow down mission execution
   - Break mission into segments with wait times
   - Identify time windows when path is clear
   
   Parameters:
       primary_mission (Mission): The mission with conflicts
       conflicts (List[Conflict]): Temporal conflicts to resolve
   
   Returns:
       List[str]: Human-readable suggestions for temporal deconfliction
   
   TODO: Implement temporal optimization algorithms
   """
   raise NotImplementedError("Temporal deconfliction strategies not yet implemented")
