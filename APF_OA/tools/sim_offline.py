#!/usr/bin/env python3
"""Offline kinematic APF check - no ROS, no Gazebo (seconds, not minutes).

Rolls a point drone (pos += v_cmd * dt, perfect tracking) through the
world_spec obstacles with the same apf_core the flight node uses. Reports
reached / stuck, flight time, path length, and minimum obstacle clearance.
Use it to pre-screen k_att / k_rep / influence_radius before a Gazebo run.

Usage:
  python3 tools/sim_offline.py                 # default goal (12, 0, 2)
  python3 tools/sim_offline.py --goal 8 -3 2
  python3 tools/sim_offline.py --k-rep 2.0 --influence 3.0
  python3 tools/sim_offline.py --trap          # add a head-on wall of cylinders (local-minima test)
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from apf_oa import apf_core, world_spec  # noqa: E402


def cylinder_clearance(pos_enu, cylinders):
    """Analytic 3D distance from a point to the closest cylinder surface."""
    best = float('inf')
    x, y, z = pos_enu
    for _, cx, cy, r, h in cylinders:
        dxy = np.hypot(x - cx, y - cy)
        if z <= h:
            d = dxy - r  # beside the shaft (negative = inside)
        else:
            d = np.hypot(max(dxy - r, 0.0), z - h)  # above: rim/top distance
        best = min(best, d)
    return best


def run(goal_enu, cfg, cylinders, dt=0.05, t_max=120.0, verbose=True):
    obstacles_enu = np.concatenate(
        [world_spec.sample_cylinder_points(x, y, r, h)
         for _, x, y, r, h in cylinders], axis=0)
    obs_ned = np.stack([obstacles_enu[:, 1], obstacles_enu[:, 0],
                        -obstacles_enu[:, 2]], axis=1)
    goal_ned = apf_core.enu_to_ned(goal_enu)

    pos = np.array([0.0, 0.0, -goal_enu[2]])  # start above origin at cruise alt
    v_prev = np.zeros(3)
    detector = apf_core.StuckDetector(cfg)
    min_clear = float('inf')
    path_len = 0.0
    stuck_at = None

    n_ticks = int(t_max / dt)
    for i in range(n_ticks):
        v, info = apf_core.apf_step(pos, goal_ned, obs_ned, cfg,
                                    v_prev=v_prev, dt=dt)
        v_prev = v
        pos = pos + v * dt
        path_len += float(np.linalg.norm(v)) * dt
        min_clear = min(min_clear, cylinder_clearance(apf_core.ned_to_enu(pos), cylinders))

        if detector.update(float(np.linalg.norm(v)), info['dist_goal'], pos=pos) and stuck_at is None:
            stuck_at = apf_core.ned_to_enu(pos).round(2)
        if info['dist_goal'] < cfg.goal_threshold:
            if verbose:
                print(f"REACHED in {i * dt:.1f}s | path {path_len:.1f} m "
                      f"| min clearance {min_clear:.2f} m")
            return True, i * dt, min_clear, stuck_at

    if verbose:
        end = apf_core.ned_to_enu(pos).round(2)
        print(f"NOT reached after {t_max:.0f}s | end ENU {end} "
              f"| min clearance {min_clear:.2f} m"
              + (f" | STUCK at ENU {stuck_at}" if stuck_at is not None else ""))
    return False, t_max, min_clear, stuck_at


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal', nargs=3, type=float,
                    default=list(world_spec.GOAL_DEFAULT))
    ap.add_argument('--k-att', type=float, default=0.8)
    ap.add_argument('--k-rep', type=float, default=1.2)
    ap.add_argument('--influence', type=float, default=2.5)
    ap.add_argument('--v-max', type=float, default=1.5)
    ap.add_argument('--trap', action='store_true',
                    help='add a 3-cylinder wall straight across the path (local-minima scenario)')
    args = ap.parse_args()

    cfg = apf_core.Config(k_att=args.k_att, k_rep=args.k_rep,
                          influence_radius=args.influence, v_max=args.v_max)
    cylinders = list(world_spec.CYLINDERS)
    if args.trap:
        cylinders += [(f'trap_{i}', 6.0, dy, 0.4, 3.5) for i, dy in
                      enumerate((-0.9, 0.0, 0.9))]
        print("trap mode: 3-cylinder wall added at x=6, y in {-0.9, 0, 0.9}")

    run(tuple(args.goal), cfg, cylinders)


if __name__ == '__main__':
    main()
