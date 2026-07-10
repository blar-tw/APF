"""Single source of truth for the apf_test world obstacles.

Used by tools/gen_world.py (SDF generation), obstacle_sensor_node's
fixed-list mode, and the RViz markers - edit here, regenerate, and every
consumer stays consistent. Coordinates are Gazebo world ENU (x East, y North).
"""

import numpy as np

# name, x, y, radius, height  (Gazebo world frame, base on the ground)
# Tall cylinders (3.5 m) block the 2 m cruise altitude -> forces lateral
# avoidance; the low one (1.2 m) sits on the path -> 3D APF climbs over it.
CYLINDERS = [
    ("cyl_tall_1", 3.5, 0.7, 0.4, 3.5),
    ("cyl_tall_2", 6.0, -1.2, 0.4, 3.5),
    ("cyl_tall_3", 8.0, 0.9, 0.4, 3.5),
    ("cyl_low_1", 10.0, 0.0, 0.5, 1.2),
    ("cyl_tall_4", 11.5, 1.8, 0.4, 3.5),
]

# Defaults shared by the nodes and run.sh (Gazebo world frame)
GOAL_DEFAULT = (12.0, 0.0, 2.0)
CRUISE_ALT = 2.0


def sample_cylinder_points(x, y, radius, height,
                           n_around=16, dz=0.4, z_min=0.2):
    """Sample points on a cylinder surface (ENU): rings every dz plus a
    top-cap ring, so the planner sees both the side and the top edge."""
    pts = []
    angles = np.linspace(0.0, 2.0 * np.pi, n_around, endpoint=False)
    ring_x = x + radius * np.cos(angles)
    ring_y = y + radius * np.sin(angles)
    for z in np.arange(z_min, height + 1e-6, dz):
        pts.append(np.stack([ring_x, ring_y, np.full_like(ring_x, z)], axis=1))
    # top cap: a smaller ring on the lid marks the fly-over surface
    cap_r = radius * 0.5
    pts.append(np.stack([x + cap_r * np.cos(angles),
                         y + cap_r * np.sin(angles),
                         np.full_like(ring_x, height)], axis=1))
    return np.concatenate(pts, axis=0)


def obstacle_points_enu():
    """All obstacle surface points (N, 3) in Gazebo world ENU."""
    return np.concatenate(
        [sample_cylinder_points(x, y, r, h) for _, x, y, r, h in CYLINDERS],
        axis=0,
    )
