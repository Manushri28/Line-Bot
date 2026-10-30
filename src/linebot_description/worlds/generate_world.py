#!/usr/bin/env python3
"""Generate line_track.sdf and waypoints.csv from a list of (x, y) waypoints."""
import csv
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
LINE_W = 0.03  # line width, metres


# ---------- helpers to build paths ----------
def ellipse(A=1.5, B=1.0, n=100):
    """Closed loop starting at (0,0) heading +x."""
    return [(A * math.sin(2 * math.pi * i / n),
             B * (1 - math.cos(2 * math.pi * i / n))) for i in range(n + 1)]


def polyline(corners, step=0.02):
    """Densify straight segments between corner points."""
    pts = []
    for (x1, y1), (x2, y2) in zip(corners[:-1], corners[1:]):
        n = max(1, int(math.hypot(x2 - x1, y2 - y1) / step))
        pts += [(x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n) for i in range(n)]
    pts.append(corners[-1])
    return pts


# >>> REPLACE with the official / unknown path <<<
WAYPOINTS = ellipse()
# Example square-ish path:
# WAYPOINTS = polyline([(0, 0), (1.5, 0), (1.5, 1.5), (0, 1.5), (0, 0)])

BLACK = ("<material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse>"
         "<specular>0 0 0 1</specular></material>")


def build():
    vis = []
    # round dots at every waypoint hide gaps at corners
    for i, (x, y) in enumerate(WAYPOINTS):
        vis.append(f"""
      <visual name="dot{i}">
        <pose>{x:.4f} {y:.4f} 0.001 0 0 0</pose>
        <geometry><cylinder><radius>{LINE_W/2}</radius><length>0.002</length></cylinder></geometry>
        {BLACK}
      </visual>""")
    for i in range(len(WAYPOINTS) - 1):
        (x1, y1), (x2, y2) = WAYPOINTS[i], WAYPOINTS[i + 1]
        L = math.hypot(x2 - x1, y2 - y1)
        if L < 1e-6:
            continue
        yaw = math.atan2(y2 - y1, x2 - x1)
        vis.append(f"""
      <visual name="seg{i}">
        <pose>{(x1+x2)/2:.4f} {(y1+y2)/2:.4f} 0.001 0 0 {yaw:.5f}</pose>
        <geometry><box><size>{L:.4f} {LINE_W} 0.002</size></box></geometry>
        {BLACK}
      </visual>""")

    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="line_track">
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- ONLY the 4 standard grading plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.6 0.7 0.9 1</background>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>-0.3 0.1 -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
          <material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse><specular>0 0 0 1</specular></material>
        </visual>
      </link>
    </model>

    <model name="line_path">
      <static>true</static>
      <link name="link">{''.join(vis)}
      </link>
    </model>
  </world>
</sdf>
"""
    with open(os.path.join(HERE, "line_track.sdf"), "w") as f:
        f.write(sdf)
    with open(os.path.join(HERE, "waypoints.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y"])
        w.writerows(WAYPOINTS)
    print(f"Wrote line_track.sdf and waypoints.csv ({len(WAYPOINTS)} points)")


if __name__ == "__main__":
    build()