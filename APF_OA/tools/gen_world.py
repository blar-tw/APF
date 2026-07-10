#!/usr/bin/env python3
"""Generate worlds/apf_test.sdf from apf_oa/world_spec.py.

Keeps the SDF, the fixed-list obstacle publisher, and the RViz markers in
sync: edit world_spec.py, rerun this, run.sh copies the world into the PX4
worlds dir at launch.

Usage: python3 tools/gen_world.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from apf_oa import world_spec  # noqa: E402

# Physics + plugin set proven by HOLO-DWA's dwa_test.sdf (PX4 v1.14.4 + gz Garden)
HEADER = """<sdf version='1.9'>
  <world name='apf_test'>
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::Contact' filename='gz-sim-contact-system'/>
    <plugin name='gz::sim::systems::Imu' filename='gz-sim-imu-system'/>
    <plugin name='gz::sim::systems::AirPressure' filename='gz-sim-air-pressure-system'/>
    <plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'>
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.8 0.9 1.0 1</background>
      <shadows>true</shadows>
    </scene>

    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane><normal>0 0 1</normal><size>1 1</size></plane>
          </geometry>
          <surface><friction><ode/></friction><bounce/><contact/></surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <light name='sunUTC' type='directional'>
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic>
      </attenuation>
    </light>
"""

GOAL_MARKER = """
    <!-- Goal marker: flat visual-only pad, no collision (nothing to repel from) -->
    <model name='goal_marker'>
      <static>true</static>
      <pose>{gx} {gy} 0.025 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><box><size>0.8 0.8 0.05</size></box></geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
            <emissive>0 0.5 0 1</emissive>
          </material>
        </visual>
      </link>
    </model>
"""

CYLINDER = """
    <model name='{name}'>
      <static>true</static>
      <pose>{x} {y} {zc} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        </collision>
        <visual name='visual'>
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
          <material>
            <ambient>{color} 1</ambient>
            <diffuse>{color} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

FOOTER = """  </world>
</sdf>
"""


def main():
    out = os.path.join(os.path.dirname(__file__), '..', 'worlds', 'apf_test.sdf')
    gx, gy, _ = world_spec.GOAL_DEFAULT
    parts = [HEADER, GOAL_MARKER.format(gx=gx, gy=gy)]
    for name, x, y, r, h in world_spec.CYLINDERS:
        tall = h > world_spec.CRUISE_ALT
        color = '0.8 0.2 0.2' if tall else '0.9 0.6 0.1'
        parts.append(CYLINDER.format(name=name, x=x, y=y, zc=h / 2.0,
                                     r=r, h=h, color=color))
    parts.append(FOOTER)
    with open(out, 'w') as f:
        f.write(''.join(parts))
    print(f"wrote {os.path.normpath(out)} "
          f"({len(world_spec.CYLINDERS)} cylinders, goal marker at ({gx}, {gy}))")


if __name__ == '__main__':
    main()
