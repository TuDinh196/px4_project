import math
import os


def create_sdf():
    A = 40.0
    num_pillars = 40

    pillars_xml = ""
    for i in range(num_pillars):
        # Calculate t from 0 to 2*pi
        t = (i / num_pillars) * 2 * math.pi

        # Lemniscate formula from Python script (NED)
        # N = A * sin(t)
        # E = A * sin(t) * cos(t)
        n = A * math.sin(t)
        e = A * math.sin(t) * math.cos(t)

        # Convert NED to ENU for Gazebo
        x = e  # X is East
        y = n  # Y is North
        z = 10.0  # Center of a 20m high cylinder

        # Color gradient or orange
        color = "1.0 0.4 0.0 0.8"  # Orange, slightly transparent

        pillar = f"""
    <model name="pillar_{i}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.4</radius>
              <length>20.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <!-- No collision tag means drone passes through -->
      </link>
    </model>
"""
        pillars_xml += pillar

    sdf_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="figure8">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1 1</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>500 500</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range>
        <constant>1</constant>
      </attenuation>
    </light>

    <!-- Coordinate reference for PX4 (Hanoi matched) -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>21.028511</latitude_deg>
      <longitude_deg>105.804817</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

    <!-- The Figure-8 Track -->
{pillars_xml}
  </world>
</sdf>
"""

    out_path = os.path.expanduser(
        "~/PX4-Autopilot/Tools/simulation/gz/worlds/figure8.sdf"
    )
    with open(out_path, "w") as f:
        f.write(sdf_content)
    print(f"Generated {out_path}")


if __name__ == "__main__":
    create_sdf()
