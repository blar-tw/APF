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
    # F = k_rep * (1/d - 1/d0) / d^2, directed away from the point.
    # k_rep and influence_radius were raised (1.2->1.8, 2.5->3.0) so avoidance
    # starts earlier and pushes harder - this restores the clearance margin the
    # command-smoothing lag would otherwise eat into (the drone reacts sooner).
    k_rep: float = 1.8
    influence_radius: float = 3.0
    max_rep_force: float = 6.0
    min_obstacle_dist: float = 0.05  # numerical floor for 1/d terms

    # Sector reduction: keep only the nearest point per (azimuth, elevation)
    # sector before summing. Bounds the repulsion independently of point-cloud
    # density - a raw sum over a dense cloud otherwise dwarfs the attraction
    # and stalls the drone short of the goal (seen in sim_offline).
    n_az_sectors: int = 12
    n_el_sectors: int = 3

    # Tangential ("swirl") force: a horizontal component perpendicular to the
    # net repulsion, biased toward the goal side. A purely radial push points
    # straight back when an obstacle sits between drone and goal, so attraction
    # and repulsion cancel and the drone bounces in place; rotating part of that
    # push 90 deg turns the bounce into a smooth go-around and breaks the
    # symmetric-obstacle local minimum. Fraction of the horizontal repulsion
    # magnitude; 0 disables.
    tangential_gain: float = 1.5
    # The swirl is tapered off close to an obstacle: below swirl_safe_radius the
    # drone would otherwise keep circling the surface (velocity command aimed
    # tangentially) instead of being pushed clear, so within that band the pure
    # radial repulsion takes over and restores clearance. Full swirl is reached
    # swirl_taper_band metres further out; the go-around still happens at the
    # mid-range where obstacles are first felt.
    swirl_safe_radius: float = 0.5
    swirl_taper_band: float = 0.5

    # Velocity command
    v_max: float = 1.5

    # Acceleration (slew-rate) limit on the velocity command, m/s^2. Bounds how
    # fast the command may change between ticks, low-pass filtering the steep
    # near-obstacle field that otherwise makes a memoryless APF chatter. Also a
    # realistic soft-start / soft-turn for the flight controller. inf disables.
    max_accel: float = 4.0

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


def swirl_taper(dist_nearest, cfg: Config):
    """0 within swirl_safe_radius (radial push dominates -> clearance), ramping
    to 1 over swirl_taper_band (full go-around swirl at mid-range)."""
    band = max(cfg.swirl_taper_band, 1e-6)
    return float(np.clip((dist_nearest - cfg.swirl_safe_radius) / band, 0.0, 1.0))


def tangential_force(f_rep, goal_dir, dist_nearest, cfg: Config):
    """Swirl term: the horizontal repulsion rotated 90 deg toward the goal side.

    Scales with the horizontal repulsion magnitude, so it only acts near
    obstacles and vanishes in open space. Kept horizontal so the vertical
    fly-over behaviour is untouched. The rotation sign is chosen once per call
    from the goal direction (deterministic - no per-tick flip-flop of its own),
    and it is tapered off within swirl_safe_radius so the drone is pushed clear
    of a surface instead of orbiting it (see swirl_taper).
    """
    if cfg.tangential_gain <= 0.0:
        return np.zeros(3)
    taper = swirl_taper(dist_nearest, cfg)
    if taper <= 0.0:
        return np.zeros(3)
    r = np.asarray(f_rep, dtype=float)[:2]        # horizontal repulsion
    nr = float(np.linalg.norm(r))
    if nr < 1e-9:
        return np.zeros(3)
    u = r / nr
    perp = np.array([-u[1], u[0]])                # +90 deg in the N-E plane
    g = np.asarray(goal_dir, dtype=float)[:2]
    if float(np.dot(perp, g)) < 0.0:
        perp = -perp                              # steer around toward the goal
    t = cfg.tangential_gain * taper * nr * perp
    return np.array([t[0], t[1], 0.0])


def force_to_velocity(force, cfg: Config):
    """Treat the total force as a desired velocity, clamped to v_max."""
    force = np.asarray(force, dtype=float)
    norm = np.linalg.norm(force)
    if norm > cfg.v_max:
        return force / norm * cfg.v_max
    return force.copy()


def limit_acceleration(v_prev, v_desired, dt, cfg: Config):
    """Clamp the per-tick change of the velocity command to max_accel * dt.

    A temporal low-pass on the command: the memoryless field can swing from
    full push to full pull between ticks near an obstacle, and that is the
    oscillation the drone shows. Bounding the step turns those swings into a
    smooth turn. Feed it the previous *command* (not the measured velocity) so
    the limiter stays deterministic and independent of odometry noise.
    """
    v_prev = np.asarray(v_prev, dtype=float)
    v_desired = np.asarray(v_desired, dtype=float)
    if not np.isfinite(cfg.max_accel):
        return v_desired.copy()
    dv = v_desired - v_prev
    max_step = cfg.max_accel * dt
    n = float(np.linalg.norm(dv))
    if n > max_step:
        dv = dv / n * max_step
    return v_prev + dv


def apf_step(pos, goal, obstacle_points, cfg: Config, v_prev=None, dt=None):
    """One APF evaluation: returns (v_cmd(3,), info dict for logging).

    When v_prev and dt are supplied the command is acceleration-limited against
    the previous command (recommended - this is what suppresses near-obstacle
    oscillation). Called without them it returns the raw field velocity, which
    keeps the force-field unit tests independent of the smoothing.
    """
    pos = np.asarray(pos, dtype=float)
    goal = np.asarray(goal, dtype=float)
    pts = np.asarray(obstacle_points, dtype=float).reshape(-1, 3)
    dist_nearest = (float(np.min(np.linalg.norm(pts - pos[None, :], axis=1)))
                    if pts.shape[0] else np.inf)
    f_att = attractive_force(pos, goal, cfg)
    f_rep = repulsive_force(pos, obstacle_points, cfg)
    f_tan = tangential_force(f_rep, goal - pos, dist_nearest, cfg)
    f_obstacle = f_rep + f_tan
    f_total = f_att + f_obstacle
    v_desired = force_to_velocity(f_total, cfg)
    if v_prev is not None and dt is not None:
        v_cmd = limit_acceleration(v_prev, v_desired, dt, cfg)
    else:
        v_cmd = v_desired
    info = {
        "f_att": f_att,
        # full obstacle response (radial + swirl) so the CSV and RViz arrow
        # show what the drone actually feels; repulsive_force stays pure radial
        "f_rep": f_obstacle,
        "f_tan": f_tan,
        "f_total": f_total,
        "v_desired": v_desired,
        "dist_goal": float(np.linalg.norm(goal - pos)),
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
