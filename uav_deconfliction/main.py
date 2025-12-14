#!/usr/bin/env python3
"""
UAV Deconfliction System - Main CLI Entry Point

This script serves as the command-line interface for the UAV spatial deconfliction system.
It supports running predefined spatial test scenarios or validating custom mission files.
All conflict detection is SPATIAL-ONLY (using B-spline trajectories); temporal logic
is currently a placeholder.
"""

import argparse
import sys
import os
import json
import logging
from pathlib import Path
from typing import List, Dict, Any

# Ensure src module is in path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

try:
    from data_models import Mission, Waypoint
    from mission_validator import validate_mission, generate_validation_report, export_validation_results
    import test_data_generator
    from geometry_utils import visualize_trajectory_with_waypoints
except ImportError as e:
    print(f"Critical Error: Failed to import source modules. {e}")
    sys.exit(1)

# Configure Logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

def parse_arguments():
    """Defines and parses command-line arguments."""
    parser = argparse.ArgumentParser(
        description="UAV Deconfliction System (Spatial Analysis)",
        formatter_class=argparse.RawTextHelpFormatter
    )

    # Mode selection (mutually exclusive)
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument(
        "--scenario",
        choices=['crossing', 'parallel', 'altitude', 'complex', 'spiral', 'near_miss'],
        help="Run a predefined test scenario:\n"
             "  crossing  : Two missions intersecting (spatial conflict)\n"
             "  parallel  : Two parallel missions (check buffer)\n"
             "  altitude  : Overlapping paths separated by altitude\n"
             "  complex   : Multi-mission random scenario\n"
             "  spiral    : Complex curved spiral trajectories\n"
             "  near_miss : Boundary condition test"
    )
    mode_group.add_argument(
        "--validate",
        metavar="MISSION_FILE",
        help="Path to the primary mission JSON file to validate"
    )

    # Custom validation arguments
    parser.add_argument(
        "--traffic",
        metavar="TRAFFIC_FILE",
        help="Path to operational traffic JSON file (required if --validate is set)"
    )

    # Configuration
    parser.add_argument(
        "--safety-buffer",
        type=float,
        default=5.0,
        help="Minimum safety separation distance in meters (default: 5.0)"
    )
    parser.add_argument(
        "--output-dir",
        default="./outputs",
        help="Directory to save reports and visualizations (default: ./outputs)"
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable detailed debug logging"
    )
    
    # Visualization options
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Disable generation of 3D visualization plots"
    )

    return parser.parse_args()

def load_mission_from_json(filepath: str) -> Mission:
    """Parses a JSON file into a Mission object."""
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        required_keys = ['mission_id', 'waypoints']
        for key in required_keys:
            if key not in data:
                raise ValueError(f"JSON missing required key: '{key}'")

        waypoints = []
        for i, wp_data in enumerate(data['waypoints']):
            # Allow flexible input: "timestamp" or "t"
            t = wp_data.get('timestamp', wp_data.get('t', float(i)))
            waypoints.append(Waypoint(
                x=wp_data['x'],
                y=wp_data['y'],
                z=wp_data['z'],
                timestamp=t
            ))
            
        return Mission(mission_id=data['mission_id'], waypoints=waypoints)
        
    except FileNotFoundError:
        logger.error(f"File not found: {filepath}")
        sys.exit(1)
    except json.JSONDecodeError:
        logger.error(f"Invalid JSON format in: {filepath}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Failed to load mission: {e}")
        sys.exit(1)

def load_traffic_from_json(filepath: str) -> List[Mission]:
    """Parses a JSON list of missions."""
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
            
        if not isinstance(data, list):
            raise ValueError("Traffic file must contain a JSON list of mission objects")
            
        missions = []
        for m_data in data:
            # Re-use logic (simplified here for brevity, usually factor out)
            waypoints = []
            for i, wp in enumerate(m_data['waypoints']):
                 t = wp.get('timestamp', wp.get('t', float(i)))
                 waypoints.append(Waypoint(wp['x'], wp['y'], wp['z'], t))
            missions.append(Mission(m_data['mission_id'], waypoints))
            
        return missions
    except Exception as e:
        logger.error(f"Failed to load traffic: {e}")
        sys.exit(1)

def run_scenario(scenario: str, safety_buffer: float, output_dir: str, skip_viz: bool):
    """Executes a predefined test scenario."""
    logger.info(f"Running scenario: {scenario.upper()}")
    
    # Generate Data
    if scenario == 'crossing':
        m1, m2 = test_data_generator.generate_crossing_missions_3d(safety_buffer)
        primary = m1
        traffic = [m2]
    elif scenario == 'parallel':
        # Default to close parallel to show detection capability
        m1, m2 = test_data_generator.generate_parallel_missions_3d(separation_distance=4.0)
        primary = m1
        traffic = [m2]
    elif scenario == 'altitude':
        m1, m2 = test_data_generator.generate_altitude_separated_missions_3d(altitude_separation=60.0)
        primary = m1
        traffic = [m2]
    elif scenario == 'spiral':
        m1, m2 = test_data_generator.generate_offset_spirals(offset_distance=30.0)
        primary = m1
        traffic = [m2]
    elif scenario == 'complex':
        missions = test_data_generator.generate_complex_scenario_3d(num_missions=5)
        primary = missions[0]
        traffic = missions[1:]
    elif scenario == 'near_miss':
        m1, m2 = test_data_generator.generate_near_miss_scenario(miss_distance=safety_buffer+0.5, safety_buffer=safety_buffer)
        primary = m1
        traffic = [m2]
    else:
        logger.error(f"Unknown scenario: {scenario}")
        return

    # Validate
    logger.info(f"Validating {primary.mission_id} against {len(traffic)} traffic missions...")
    is_clear, conflicts, report = validate_mission(primary, traffic, safety_buffer)
    
    # Output Results
    logger.info("Validation complete.")
    print("\n" + "="*60)
    print(report)
    print("="*60 + "\n")

    # Export
    os.makedirs(output_dir, exist_ok=True)
    report_path = os.path.join(output_dir, f"report_{scenario}.txt")
    with open(report_path, "w") as f:
        f.write(report)
    logger.info(f"Report saved to {report_path}")

    # Visualize
    if not skip_viz:
        viz_path = os.path.join(output_dir, f"viz_{scenario}.png")
        all_missions = [primary] + traffic
        test_data_generator.visualize_test_scenario(
            all_missions, 
            title=f"Scenario: {scenario.title()} (Buffer {safety_buffer}m)",
            conflicts=conflicts, 
            output_path=viz_path
        )
        logger.info(f"Visualization saved to {viz_path}")

def validate_custom_mission(mission_file: str, traffic_file: str, safety_buffer: float, output_dir: str, skip_viz: bool):
    """Orchestrates validation of custom files."""
    if not traffic_file:
        logger.error("--traffic file is required when using --validate")
        sys.exit(1)

    logger.info(f"Loading custom mission from {mission_file}...")
    primary_mission = load_mission_from_json(mission_file)
    
    logger.info(f"Loading traffic from {traffic_file}...")
    traffic_missions = load_traffic_from_json(traffic_file)
    
    logger.info(f"Validating {primary_mission.mission_id} against {len(traffic_missions)} traffic missions...")
    is_clear, conflicts, report = validate_mission(primary_mission, traffic_missions, safety_buffer)
    
    # Console Output
    print("\n" + "="*60)
    print(report)
    print("="*60 + "\n")
    
    # File Export
    os.makedirs(output_dir, exist_ok=True)
    base_name = f"validation_{primary_mission.mission_id}"
    json_path = os.path.join(output_dir, f"{base_name}.json")
    
    validation_results = (is_clear, conflicts, report)
    export_validation_results(validation_results, json_path, format='json')
    logger.info(f"Results exported to {json_path}")
    
    # Visualize
    if not skip_viz:
        viz_path = os.path.join(output_dir, f"{base_name}.png")
        all_missions = [primary_mission] + traffic_missions
        test_data_generator.visualize_test_scenario(
            all_missions,
            title=f"Validation: {primary_mission.mission_id}",
            conflicts=conflicts,
            output_path=viz_path
        )
        logger.info(f"Visualization saved to {viz_path}")

def main():
    args = parse_arguments()
    
    if args.verbose:
        logger.setLevel(logging.DEBUG)
        
    logger.info("Starting UAV Deconfliction System...")
    logger.info(f"Mode: {'Scenario' if args.scenario else 'File Validation'}")
    logger.info(f"Safety Buffer: {args.safety_buffer}m")
    
    if args.scenario:
        run_scenario(args.scenario, args.safety_buffer, args.output_dir, args.no_viz)
    elif args.validate:
        validate_custom_mission(args.validate, args.traffic, args.safety_buffer, args.output_dir, args.no_viz)
        
    logger.info("Process completed successfully.")

if __name__ == "__main__":
    main()
