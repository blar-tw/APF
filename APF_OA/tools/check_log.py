#!/usr/bin/env python3
"""Summarize an APF flight CSV: outcome, timing, clearance, stuck episodes.

Usage:
  python3 tools/check_log.py                 # newest CSV in ../logs
  python3 tools/check_log.py path/to/log.csv
"""

import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from apf_oa import apf_core, world_spec  # noqa: E402
from tools.sim_offline import cylinder_clearance  # noqa: E402


def newest_log():
    logs = sorted(glob.glob(os.path.join(
        os.path.dirname(__file__), '..', '..', 'logs', 'apf_*.csv')))
    if not logs:
        sys.exit('no CSV found in logs/')
    return logs[-1]


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else newest_log()
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f'{path}: empty log (planner never reached NAVIGATE?)')

    t = np.array([float(r['t']) for r in rows])
    pos = np.array([[float(r['n']), float(r['e']), float(r['d'])] for r in rows])
    dist = np.array([float(r['dist_goal']) for r in rows])
    stuck = np.array([int(r['stuck']) for r in rows])
    episodes = int(rows[-1]['stuck_episodes'])

    clear = np.array([
        cylinder_clearance(apf_core.ned_to_enu(p), world_spec.CYLINDERS)
        for p in pos])
    path_len = float(np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1)))

    reached = dist[-1] < 0.6 or dist.min() < 0.5
    print(f"log: {os.path.relpath(path)}")
    print(f"  outcome:        {'REACHED' if reached else 'NOT reached'} "
          f"(final dist {dist[-1]:.2f} m, min {dist.min():.2f} m)")
    print(f"  duration:       {t[-1] - t[0]:.1f} s NAVIGATE ({len(rows)} ticks)")
    print(f"  path length:    {path_len:.1f} m")
    print(f"  min clearance:  {clear.min():.2f} m "
          f"(at t={t[np.argmin(clear)]:.1f}s, altitude {-pos[np.argmin(clear), 2]:.2f} m)")
    print(f"  max altitude:   {-pos[:, 2].min():.2f} m  (fly-over check)")
    print(f"  stuck episodes: {episodes} ({int(stuck.sum())} stuck ticks)")


if __name__ == '__main__':
    main()
