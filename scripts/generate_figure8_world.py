"""
Gazebo 3D World Generator for Quadplane Condor Airfield
======================================================
1. Removes all cyan/blue dots for a pristine clean look.
2. Creates an expansive, realistic international airport runway & tarmac:
   - Main Runway: 350m x 45m with centerline stripes, edge lines, and threshold zebra markings.
   - Large Central Helipad & VTOL Pad: 12m diameter with yellow border & 'H'.
   - Apron / Tarmac & Taxiway providing clear visual depth cues during climb and descent.
   - Pristine green airfield grass surrounding the runway.
   - Quadplane Condor embedded at origin on the Helipad.
"""

from pathlib import Path


def generate_figure8_world():
    # 1. Main Runway Centerline Stripes (from Y = -160m to +160m, spaced every 15m)
    runway_stripes_xml = ""
    for i in range(-11, 12):
        y_pos = i * 14.0
        # Skip center helipad area
        if abs(y_pos) < 10.0:
            continue
        stripe = f"""
    <!-- Runway Centerline Stripe {i+11} -->
    <model name="stripe_{i+11}">
      <static>true</static>
      <pose>0 {y_pos:.1f} 0.007 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry>
            <box>
              <size>0.9 8.0 0.005</size>
            </box>
          </geometry>
          <material>
            <ambient>0.95 0.95 0.95 1.0</ambient>
            <diffuse>0.95 0.95 0.95 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""
        runway_stripes_xml += stripe

    # 2. Runway Edge White Lines (Left & Right boundaries at X = -21m and +21m)
    runway_edge_lines_xml = """
    <!-- Runway Left Edge Line -->
    <model name="runway_edge_left">
      <static>true</static>
      <pose>-21.5 0 0.007 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry>
            <box>
              <size>0.6 340.0 0.005</size>
            </box>
          </geometry>
          <material>
            <ambient>0.95 0.95 0.95 1.0</ambient>
            <diffuse>0.95 0.95 0.95 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Runway Right Edge Line -->
    <model name="runway_edge_right">
      <static>true</static>
      <pose>21.5 0 0.007 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry>
            <box>
              <size>0.6 340.0 0.005</size>
            </box>
          </geometry>
          <material>
            <ambient>0.95 0.95 0.95 1.0</ambient>
            <diffuse>0.95 0.95 0.95 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

    # 3. Threshold Piano Keys (North & South ends of runway)
    threshold_keys_xml = ""
    for end_name, y_end in [("south", -160.0), ("north", 160.0)]:
        for key_idx in range(-5, 6):
            if key_idx == 0:
                continue
            x_pos = key_idx * 3.4
            key = f"""
    <!-- Threshold Key {end_name}_{key_idx} -->
    <model name="thresh_{end_name}_{key_idx+5}">
      <static>true</static>
      <pose>{x_pos:.1f} {y_end:.1f} 0.007 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry>
            <box>
              <size>1.6 15.0 0.005</size>
            </box>
          </geometry>
          <material>
            <ambient>0.95 0.95 0.95 1.0</ambient>
            <diffuse>0.95 0.95 0.95 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""
            threshold_keys_xml += key

    # 4. Complete SDF World Definition
    sdf_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.80665</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <scene>
      <grid>false</grid>
      <ambient>0.55 0.55 0.60 1.0</ambient>
      <background>0.65 0.78 0.92 1.0</background>
      <shadows>true</shadows>
    </scene>

    <!-- Sun Lighting -->
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1.3</intensity>
      <direction>0.3 0.4 -0.85</direction>
      <diffuse>0.98 0.98 0.98 1.0</diffuse>
      <specular>0.3 0.3 0.3 1.0</specular>
      <attenuation>
        <range>2000</range>
        <constant>1</constant>
      </attenuation>
    </light>

    <!-- Airfield Green Grass Base (800m x 800m) -->
    <model name="airfield_ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>800 800</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="grass_visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>800 800</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.24 0.42 0.24 1.0</ambient>
            <diffuse>0.28 0.48 0.28 1.0</diffuse>
            <specular>0.05 0.05 0.05 1.0</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Main Expansive Asphalt Runway (350m x 45m) -->
    <model name="runway_main">
      <static>true</static>
      <pose>0 0 0.003 0 0 0</pose>
      <link name="link">
        <visual name="asphalt">
          <geometry>
            <box>
              <size>45.0 350.0 0.004</size>
            </box>
          </geometry>
          <material>
            <ambient>0.13 0.15 0.17 1.0</ambient>
            <diffuse>0.17 0.19 0.21 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Apron / Tarmac Zone (West of Runway: 60m x 80m) -->
    <model name="apron_tarmac">
      <static>true</static>
      <pose>-55.0 0 0.003 0 0 0</pose>
      <link name="link">
        <visual name="concrete">
          <geometry>
            <box>
              <size>65.0 90.0 0.004</size>
            </box>
          </geometry>
          <material>
            <ambient>0.20 0.22 0.25 1.0</ambient>
            <diffuse>0.24 0.26 0.29 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Runway Markings: Edge Lines, Centerline & Threshold Keys -->
{runway_edge_lines_xml}
{runway_stripes_xml}
{threshold_keys_xml}

    <!-- Grand High-Visibility Takeoff & Landing Helipad (H-Pad) at Center (0,0) -->
    <model name="helipad_h">
      <static>true</static>
      <pose>0 0 0.005 0 0 0</pose>
      <link name="helipad_link">
        <!-- Base Dark Octagon Disc -->
        <visual name="pad_base">
          <geometry>
            <cylinder>
              <radius>7.0</radius>
              <length>0.008</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.10 0.12 0.15 1.0</ambient>
            <diffuse>0.14 0.16 0.19 1.0</diffuse>
          </material>
        </visual>
        <!-- Outer Glowing Yellow Border Ring -->
        <visual name="outer_yellow_ring">
          <pose>0 0 0.002 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>6.6</radius>
              <length>0.01</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.95 0.75 0.0 1.0</ambient>
            <diffuse>1.0 0.82 0.0 1.0</diffuse>
          </material>
        </visual>
        <!-- Inner Landing Pad Surface -->
        <visual name="inner_surface">
          <pose>0 0 0.004 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>5.8</radius>
              <length>0.01</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.12 0.14 0.17 1.0</ambient>
            <diffuse>0.16 0.18 0.22 1.0</diffuse>
          </material>
        </visual>
        <!-- White 'H' Left Bar (North-South) -->
        <visual name="h_left">
          <pose>-1.6 0 0.008 0 0 0</pose>
          <geometry>
            <box>
              <size>0.6 3.8 0.01</size>
            </box>
          </geometry>
          <material>
            <ambient>1.0 1.0 1.0 1.0</ambient>
            <diffuse>1.0 1.0 1.0 1.0</diffuse>
          </material>
        </visual>
        <!-- White 'H' Right Bar (North-South) -->
        <visual name="h_right">
          <pose>1.6 0 0.008 0 0 0</pose>
          <geometry>
            <box>
              <size>0.6 3.8 0.01</size>
            </box>
          </geometry>
          <material>
            <ambient>1.0 1.0 1.0 1.0</ambient>
            <diffuse>1.0 1.0 1.0 1.0</diffuse>
          </material>
        </visual>
        <!-- White 'H' Crossbar (East-West) -->
        <visual name="h_mid">
          <pose>0 0 0.008 0 0 0</pose>
          <geometry>
            <box>
              <size>2.6 0.6 0.01</size>
            </box>
          </geometry>
          <material>
            <ambient>1.0 1.0 1.0 1.0</ambient>
            <diffuse>1.0 1.0 1.0 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Embedded Quadplane Condor at Origin (Pointing True North) -->
    <include>
      <uri>model://quadplane_condor</uri>
      <name>quadplane_condor_0</name>
      <pose>0 0 0.35 0 0 0</pose>
    </include>

    <!-- Geographic Origin (Hanoi Coordinates) -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>21.028511</latitude_deg>
      <longitude_deg>105.804817</longitude_deg>
      <elevation>0.0</elevation>
    </spherical_coordinates>

  </world>
</sdf>
"""

    # 1. Save directly into project models/worlds
    project_world_dir = Path(__file__).resolve().parents[1] / "models" / "worlds"
    project_world_dir.mkdir(parents=True, exist_ok=True)
    proj_world_file = project_world_dir / "condor_figure8.sdf"
    with open(proj_world_file, "w") as f:
        f.write(sdf_content)
    print(f"✅ Generated condor_figure8.sdf in project: {proj_world_file}")

    # 2. Also save to PX4-Autopilot directory if present
    px4_world_dir = Path.home() / "PX4-Autopilot/Tools/simulation/gz/worlds"
    if px4_world_dir.is_dir():
        out_file = px4_world_dir / "figure8.sdf"
        with open(out_file, "w") as f:
            f.write(sdf_content)
        out_default = px4_world_dir / "default.sdf"
        with open(out_default, "w") as f:
            f.write(sdf_content)
        out_file2 = px4_world_dir / "condor_figure8.sdf"
        with open(out_file2, "w") as f:
            f.write(sdf_content)
        print(f"✅ Synced world files to PX4-Autopilot: {out_file2}")


if __name__ == "__main__":
    generate_figure8_world()
