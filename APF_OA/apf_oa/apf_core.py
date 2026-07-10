"""Artificial Potential Field core - pure numpy, no ROS dependencies.

All vectors are 3D in the PX4 local NED frame (x North, y East, z Down).
The planner treats the summed force as a desired velocity (kinematic APF),
clamped to v_max before being sent to the flight controller.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Config:
    # Attractive: F_att = k_att * (goal - pos), saturated beyond att_saturation
    k_att: float = 0.8
    att_saturation: float = 3.0

    # Repulsive (Khatib): per obstacle point within influence_radius,
    # F = k_rep * (1/d - 1/d0) / d^2, directed away from the point
    k_rep: float = 1.2
    influence_radius: float = 2.5
    max_rep_force: float = 6.0
    min_obstacle_dist: float = 0.05  # numerical floor for 1/d terms

    # Sector reduction: keep only the nearest point per (azimuth, elevation)
    # sector before summing. Bounds the repulsion independently of point-cloud
    # density - a raw sum over a dense cloud otherwise dwarfs the attraction
    # and stalls the drone short of the goal (seen in sim_offline).
    n_az_sectors: int = 12
    n_el_sectors: int = 3

    # Velocity command
    v_max: float = 1.5

    # Goal
    goal_threshold: float = 0.5

    # Local minima detection, two criteria over a stuck_ticks window while
    # still away from the goal:
    #   1. speed stays below stuck_speed_eps (hover stall)
    #   2. net displacement stays below stuck_dist_eps (tight oscillation -
    #      speed alone misses a drone bouncing in front of a wall)
    stuck_speed_eps: float = 0.1
    stuck_dist_eps: float = 0.3
    stuck_ticks: int = 40


def attractive_force(pos, goal, cfg: Config):
    """Pull toward the goal, proportional to distance (saturated far away)."""
    pos = np.asarray(pos, dtype=float)
    goal = np.asarray(goal, dtype=float)
    diff = goal - pos
    dist = np.linalg.norm(diff)
    if dist < 1e-9:
        return np.zeros(3)
    if dist > cfg.att_saturation:
        # Constant pull beyond the saturation radius so a far goal does not
        # drown out the repulsive term
        return cfg.k_att * cfg.att_saturation * diff / dist
    return cfg.k_att * diff


def reduce_to_sector_nearest(pos, pts, cfg: Config):
    """Keep only the nearest obstacle point per (azimuth, elevation) sector.

    Caps the number of repulsion sources at n_az * n_el regardless of how
    dense the cloud is, so fixed-list sampling and a depth-camera cloud
    produce comparable force magnitudes.
    """
    diff = pts - pos[None, :]
    dist = np.linalg.norm(diff, axis=1)
    az = np.arctan2(diff[:, 1], diff[:, 0])
    horiz = np.linalg.norm(diff[:, :2], axis=1)
    el = np.arctan2(-diff[:, 2], horiz)  # NED: -z is up

    az_bin = np.minimum(((az + np.pi) / (2 * np.pi) * cfg.n_az_sectors).astype(int),
                        cfg.n_az_sectors - 1)
    el_bin = np.minimum(((el + np.pi / 2) / np.pi * cfg.n_el_sectors).astype(int),
                        cfg.n_el_sectors - 1)
    sector = az_bin * cfg.n_el_sectors + el_bin

    order = np.argsort(dist)
    _, first_idx = np.unique(sector[order], return_index=True)
    return pts[order[first_idx]]


def repulsive_force(pos, obstacle_points, cfg: Config):
    """Khatib repulsion summed over sector-nearest points within influence_radius.

    obstacle_points: (N, 3) array in the same frame as pos. Returns zeros for
    an empty set or when everything is out of range.
    """
    pos = np.asarray(pos, dtype=float)
    pts = np.asarray(obstacle_points, dtype=float).reshape(-1, 3)
    if pts.shape[0] == 0:
        return np.zeros(3)

    dist = np.linalg.norm(pts - pos[None, :], axis=1)
    pts = pts[dist < cfg.influence_radius]
    if pts.shape[0] == 0:
        return np.zeros(3)

    pts = reduce_to_sector_nearest(pos, pts, cfg)

    diff = pos[None, :] - pts               # away from each representative
    d = np.clip(np.linalg.norm(diff, axis=1), cfg.min_obstacle_dist, None)
    d0 = cfg.influence_radius
    mag = cfg.k_rep * (1.0 / d - 1.0 / d0) / (d * d)
    force = np.sum(diff / d[:, None] * mag[:, None], axis=0)

    norm = np.linalg.norm(force)
    if norm > cfg.max_rep_force:
        force = force / norm * cfg.max_rep_force
    return force


def force_to_velocity(force, cfg: Config):
    """Treat the total force as a desired velocity, clamped to v_max."""
    force = np.asarray(force, dtype=float)
    norm = np.linalg.norm(force)
    if norm > cfg.v_max:
        return force / norm * cfg.v_max
    return force.copy()


def apf_step(pos, goal, obstacle_points, cfg: Config):
    """One APF evaluation: returns (v_cmd(3,), info dict for logging)."""
    f_att = attractive_force(pos, goal, cfg)
    f_rep = repulsive_force(pos, obstacle_points, cfg)
    f_total = f_att + f_rep
    v_cmd = force_to_velocity(f_total, cfg)
    info = {
        "f_att": f_att,
        "f_rep": f_rep,
        "f_total": f_total,
        "dist_goal": float(np.linalg.norm(np.asarray(goal, dtype=float) - np.asarray(pos, dtype=float))),
    }
    return v_cmd, info


class StuckDetector:
    """Local-minima heuristic: stuck when, away from the goal, either the
    speed stays near zero for stuck_ticks consecutive ticks, or the position
    barely moves over a stuck_ticks window (oscillation trap)."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.count = 0
        self.stuck = False
        self.episodes = 0
        self._pos_window = []

    def update(self, speed, dist_to_goal, pos=None):
        near_goal = dist_to_goal <= self.cfg.goal_threshold

        no_progress = False
        if pos is not None and not near_goal:
            self._pos_window.append(np.asarray(pos, dtype=float))
            if len(self._pos_window) > self.cfg.stuck_ticks:
                self._pos_window.pop(0)
            if len(self._pos_window) == self.cfg.stuck_ticks:
                spread = np.linalg.norm(
                    np.max(self._pos_window, axis=0) - np.min(self._pos_window, axis=0))
                no_progress = bool(spread < self.cfg.stuck_dist_eps)
        elif near_goal:
            self._pos_window.clear()

        if near_goal or (speed >= self.cfg.stuck_speed_eps and not no_progress):
            self.count = 0
            self.stuck = False
            return False

        self.count += 1
        now_stuck = no_progress or self.count >= self.cfg.stuck_ticks
        if now_stuck and not self.stuck:
            self.episodes += 1  # rising edge
        self.stuck = now_stuck
        return self.stuck

    def reset(self):
        self.count = 0
        self.stuck = False
        self._pos_window.clear()


# --- frame helpers (Gazebo world ENU <-> PX4 local NED) ---

def enu_to_ned(v):
    """(E, N, U) -> (N, E, D). Works for positions and vectors."""
    v = np.asarray(v, dtype=float)
    return np.array([v[1], v[0], -v[2]])


def ned_to_enu(v):
    """(N, E, D) -> (E, N, U)."""
    v = np.asarray(v, dtype=float)
    return np.array([v[1], v[0], -v[2]])


def rotate_by_quat(q_wxyz, vec):
    """Rotate vec by quaternion (w, x, y, z).

    With PX4 VehicleOdometry.q this maps body FRD -> local NED.
    """
    w, x, y, z = (float(c) for c in q_wxyz)
    v = np.asarray(vec, dtype=float)
    u = np.array([x, y, z])
    return 2.0 * np.dot(u, v) * u + (w * w - np.dot(u, u)) * v + 2.0 * w * np.cross(u, v)
