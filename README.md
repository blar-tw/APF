# Artificial Potential Field Obstacle Avoidance

> 3D APF (Artificial Potential Field) obstacle avoidance: a multirotor in PX4
> SITL + Gazebo simulation, sensing obstacles with a depth camera (OakD-Lite
> on x500_depth), computing the combined attractive/repulsive force in real
> time to fly toward the goal. Sibling project:
> [HOLO-DWA](https://github.com/blar-tw/HOLO-DWA) (same environment, DWA algorithm).
## Demo
![APF](APF.gif)
The demo has been accelerated, and there are still some minor fluctuations when UAV is near the obstacle. I will further optimize this in a future iteration.

## Requirements

- WSL2 + Ubuntu 22.04 (or native), ROS 2 Humble
- PX4-Autopilot v1.14.4 + Gazebo Garden (the `x500_depth` model and `4002`
  airframe are built in)
- `ros_gz` bridge (must match the Gazebo version — see HOLO-DWA's
  [installation.md](../HOLO-DWA/docs/installation.md))
- Micro-XRCE-DDS-Agent, `px4_msgs`, `tmux`, Python 3.10 + `numpy`
- Needs network access once on first launch: Gazebo downloads the OakD-Lite
  model from Fuel (cached afterward)

## Quick Start

**Before running for the first time** (once per airframe): `gz_x500_depth`
(airframe 4002) defaults to disallowing Offboard without RC and will
failsafe immediately. After launch, in PX4's `pxh>` console set:

```
param set NAV_DLL_ACT 0
param set COM_RCL_EXCEPT 4
param save
```

```bash
cd ~/ws
colcon build --packages-select apf_oa --symlink-install
cd src/APF
./run.sh                      # goal (12, 0), altitude 2 m, fixed obstacle list, opens RViz if a display is present
./run.sh 8.0 -3.0             # custom goal_x goal_y (Gazebo world coordinates)
./run.sh 8.0 -3.0 2.5         # plus custom goal altitude
OBST_SOURCE=depth ./run.sh    # switch obstacle source to the depth-camera point cloud
HEADLESS=1 ./run.sh           # no GUI (more stable tracking under WSL2)
./run.sh kill                 # tear everything down
```

After a run, inspect the results:

```bash
python3 APF_OA/tools/check_log.py     # summary of the latest flight CSV
```

## How It Works

```
Gazebo (x500_depth, world: apf_test)
   └─ OakD-Lite depth camera ──► /depth_camera/points (gz)
        ▼ ros_gz_bridge (→ /depth_points)
 obstacle_sensor_node ──► /apf/obstacle_points (PointCloud2, NED)
        │   source=fixed:  sampled points on world_spec cylinder surfaces
        │                  (omniscient, for verification)
        │   source=depth:  depth point cloud → FLU→FRD→NED conversion +
        │                  ground filtering / downsampling
        ▼
 apf_planner_node (20 Hz)
        │   INIT → TAKEOFF → NAVIGATE → GOAL_REACHED
        │   NAVIGATE: apf_core.apf_step() → 3D velocity command
        ▼
 /fmu/in/trajectory_setpoint ──► XRCE-DDS Agent ──► PX4 SITL
```

APF core ([apf_core.py](APF_OA/apf_oa/apf_core.py), pure numpy, no ROS):

- **Attractive force**: `F_att = k_att · (goal − pos)`, saturating beyond
  `att_saturation` (constant-speed pull at long range).
- **Repulsive force** (Khatib): within `influence_radius` of an obstacle
  point, `F = k_rep·(1/d − 1/d0)/d²` pushes away from it; **sector
  reduction** (12 azimuth × 3 elevation bins, nearest point per sector) is
  applied first, then summed — this decouples repulsion magnitude from
  point-cloud density, so fixed / depth sources behave consistently (without
  this step, a dense point cloud's total repulsion overwhelms the attraction
  and the drone stalls mid-flight).
- **Combined force → velocity**: the combined force is treated as the
  desired velocity, magnitude clamped to `v_max`, and all 3 axes
  (vx, vy, vz) are sent to PX4 directly (per-axis NaN passthrough).
- **Local-minima detection**: while still far from the goal, either
  "velocity stays near zero for consecutive ticks" **or** "displacement is
  minimal over a time window (oscillation)" flags the drone as stuck,
  logged to both the log and the CSV (no escape mechanism yet — see Step 5
  below).

Obstacle heights are deliberately designed: 4 tall cylinders at 3.5 m (block
the 2 m cruise altitude → force a horizontal detour) + 1 short cylinder at
1.2 m blocking the path directly (z-axis repulsion → flown straight over),
demonstrating both behaviors of 3D APF in a single run.

## Files

| Path | Role | How to test |
|------|------|---------|
| [`APF_OA/apf_oa/apf_core.py`](APF_OA/apf_oa/apf_core.py) | APF algorithm core (attraction/repulsion/velocity conversion/stuck detection), pure numpy | `python3 -m pytest test/` (24 tests) |
| [`APF_OA/apf_oa/world_spec.py`](APF_OA/apf_oa/world_spec.py) | Single source of truth for obstacle specs (shared by SDF, fixed list, and RViz markers) | After editing, run `tools/gen_world.py` to regenerate |
| [`APF_OA/apf_oa/obstacle_sensor_node.py`](APF_OA/apf_oa/obstacle_sensor_node.py) | Obstacle source node: fixed (known list) / depth (camera point cloud → NED) | `ros2 topic echo /apf/obstacle_points --field width` |
| [`APF_OA/apf_oa/apf_planner_node.py`](APF_OA/apf_oa/apf_planner_node.py) | Flight node: offboard state machine + APF navigation + CSV logging + RViz visualization | Run `./run.sh` and watch the 1 Hz status line; `tools/check_log.py` to inspect the CSV |
| [`APF_OA/apf_oa/pc2_util.py`](APF_OA/apf_oa/pc2_util.py) | Lightweight PointCloud2 ↔ numpy conversion | Covered by the integration tests |
| [`APF_OA/config/apf_params.yaml`](APF_OA/config/apf_params.yaml) | All parameters: k_att / k_rep / influence_radius / goal / stuck detection, etc. | Edit, then `./run.sh kill` and `./run.sh` again (symlink-install, no rebuild needed) |
| [`APF_OA/launch/apf_nodes.launch.py`](APF_OA/launch/apf_nodes.launch.py) | ROS-side launch for the three components (sensor + planner + RViz) | `ros2 launch apf_oa apf_nodes.launch.py rviz:=false` |
| [`APF_OA/worlds/apf_test.sdf`](APF_OA/worlds/apf_test.sdf) | Generated Gazebo world (5 cylinders + goal marker) | Verified with `gz sdf -k`; run.sh syncs it into PX4 automatically on every run |
| [`APF_OA/tools/gen_world.py`](APF_OA/tools/gen_world.py) | Generates the SDF above from world_spec | `python3 tools/gen_world.py` |
| [`APF_OA/tools/sim_offline.py`](APF_OA/tools/sim_offline.py) | Offline dynamics simulation (no ROS/Gazebo needed, runs in seconds); run this before tuning | `python3 tools/sim_offline.py [--trap]` |
| [`APF_OA/tools/check_log.py`](APF_OA/tools/check_log.py) | Flight CSV summary (goal reached / clearance / stuck count) | `python3 tools/check_log.py` |
| [`run.sh`](run.sh) | One-shot tmux launch of the full stack (PX4+gz / agent / bridge / nodes) | `./run.sh`, tear down with `./run.sh kill` |
| [`WORKLOG.md`](WORKLOG.md) | Decision log and TODOs | — |

CSV columns: `t, state, position (n,e,d), velocity, F_att, F_rep, F_total,
v_cmd, dist_goal, num obstacle points, stuck, stuck_episodes` →
`logs/apf_*.csv`.

## Tuning

Edit [`config/apf_params.yaml`](APF_OA/config/apf_params.yaml) and re-run
`./run.sh` (no rebuild needed). Suggested tuning order: sweep offline with
`tools/sim_offline.py --k-rep X --influence Y` first, and only move to
Gazebo once the behavior looks right.
