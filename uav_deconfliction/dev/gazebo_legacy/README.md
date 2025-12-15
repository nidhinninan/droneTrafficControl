# Gazebo Legacy Files

This folder contains parked Gazebo integration code that was set aside to focus on the simpler Matplotlib visualization demo.

## What's Here

- **gazebo_trajectory_player.py** - Played back UAV trajectories in Gazebo simulation
- **test_temporal_logic.py** - Tests that required Gazebo environment

## Why Parked

1. Gazebo ROS bridge configuration adds significant setup complexity
2. Coordinate system mismatches between waypoint data and Gazebo world
3. Camera/visibility issues made it hard to see the actual trajectories
4. The core deconfliction logic can be demonstrated more easily with Matplotlib

## How to Revive

1. Set up ROS 2 Humble + Gazebo Garden environment
2. Configure the coordinate transforms (Gazebo uses ENU, may need NED conversion)
3. Fix the drone model visibility/scale issues
4. Wire up the trajectory player to publish to Gazebo topics

## Current Reference

See [simple_viz.py](../src/simple_viz.py) for the working Matplotlib demo.
