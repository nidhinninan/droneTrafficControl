import json
import xml.etree.ElementTree as ET
from xml.dom import minidom
import numpy as np
from pathlib import Path

# Local imports
from data_models import Mission, Waypoint
from geometry_utils import create_trajectory_curve, evaluate_trajectory_at_parameter

def generate_world_with_markers(json_path, output_world_path):
    print(f"Generating world from {json_path}...")
    
    # 1. Load Data
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    missions = []
    for m in data['missions']:
        wps = [Waypoint(**w) for w in m['waypoints']]
        missions.append(Mission(mission_id=m['mission_id'], waypoints=wps))

    # 2. Base SDF Template
    sdf_str = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="double_spiral_demo">
    
    <!-- Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <!-- Physics -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Sun -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Conflict Marker -->
    <model name="conflict_marker">
      <pose>0 0 -999 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><sphere><radius>1.5</radius></sphere></geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
            <emissive>1 0 0 1</emissive>
          </material>
        </visual>
      </link>
      <plugin name="gazebo::TrajectoryPlayer" filename="trajectory_player_plugin">
        <position_file>/tmp/gazebo_drone_positions/conflict_marker.txt</position_file>
        <update_rate>10</update_rate>
      </plugin>
    </model>
    
    <!-- Camera -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose relative_to='drone_M001'>-8 -8 5 0 0.3 0.78</pose>
        <track_visual>
          <name>drone_M001</name>
          <min_dist>5</min_dist>
          <max_dist>20</max_dist>
          <static>false</static>
          <use_model_frame>true</use_model_frame>
          <xyz>-8 -8 5</xyz>
          <inherit_yaw>true</inherit_yaw>
        </track_visual>
      </camera>
    </gui>

  </world>
</sdf>"""
    
    root = ET.fromstring(sdf_str)
    world = root.find('world')

    # 3. Add Drones & Markers
    for m in missions:
        # Determine Color
        if m.mission_id == "M001":
            color_vec = "0 0 1 1" # Blue
            viz_name = "visual_blue"
        else:
            color_vec = "0 1 1 1" # Cyan
            viz_name = "visual_cyan"
            
        # -- Add Drone Model --
        model = ET.SubElement(world, 'model', name=f"drone_{m.mission_id}")
        pose = ET.SubElement(model, 'pose').text = "0 0 0 0 0 0" # Initial pose overridden by plugin
        link = ET.SubElement(model, 'link', name='body')
        
        # Inertial
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = "0.5"
        inertia = ET.SubElement(inertial, 'inertia')
        ET.SubElement(inertia, 'ixx').text = "0.01"
        ET.SubElement(inertia, 'iyy').text = "0.01"
        ET.SubElement(inertia, 'izz').text = "0.01"
        
        # Visual
        visual = ET.SubElement(link, 'visual', name='visual')
        geo = ET.SubElement(visual, 'geometry')
        ET.SubElement(geo, 'box').append(ET.fromstring("<size>0.5 0.5 0.2</size>"))
        mat = ET.SubElement(visual, 'material')
        ET.SubElement(mat, 'ambient').text = color_vec
        ET.SubElement(mat, 'diffuse').text = color_vec
        
        # Plugin
        plugin = ET.SubElement(model, 'plugin', name='gazebo::TrajectoryPlayer', filename='trajectory_player_plugin')
        ET.SubElement(plugin, 'position_file').text = f"/tmp/gazebo_drone_positions/drone_{m.mission_id}.txt"
        ET.SubElement(plugin, 'update_rate').text = "30"

        # -- Add Visualization Markers (Baked into a Static Model) --
        # We create ONE static model per mission to hold all the little spheres
        marker_model = ET.SubElement(world, 'model', name=f"path_viz_{m.mission_id}")
        ET.SubElement(marker_model, 'static').text = "true"
        marker_link = ET.SubElement(marker_model, 'link', name='link')
        
        # Interpolate Path
        tck, u_knots = create_trajectory_curve(m.waypoints)
        # Sample every ~0.5 units? 
        # Total length approximation?
        # Just sample 200 points
        for i, u in enumerate(np.linspace(0, 1, 200)):
            pos = evaluate_trajectory_at_parameter(tck, u)
            
            # Add visual element for this dot
            vis = ET.SubElement(marker_link, 'visual', name=f"dot_{i}")
            
            # Position the visual relative to model origin (0,0,0)
            vis_pose = ET.SubElement(vis, 'pose')
            vis_pose.text = f"{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f} 0 0 0"
            
            vis_geo = ET.SubElement(vis, 'geometry')
            ET.SubElement(vis_geo, 'sphere').append(ET.fromstring("<radius>0.10</radius>"))
            
            vis_mat = ET.SubElement(vis, 'material')
            # Slight transparency or just solid color
            ET.SubElement(vis_mat, 'ambient').text = color_vec
            ET.SubElement(vis_mat, 'diffuse').text = color_vec
            ET.SubElement(vis_mat, 'emissive').text = color_vec # Make it glow slightly

    # 4. Save
    xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
    with open(output_world_path, "w") as f:
        f.write(xmlstr)
    
    print(f"Generated {output_world_path} with static path markers.")

if __name__ == "__main__":
    generate_world_with_markers(
        "uav_deconfliction/data/demo_double_spiral.json",
        "uav_deconfliction/outputs/generated_worlds/double_spiral_demo.world"
    )
