#!/usr/bin/env bash
#
# run.sh - one-shot launcher for the APF_OA gz simulation stack.
# Same tmux pattern as HOLO-DWA/run.sh.
#
# Panes:
#   1. PX4 SITL + Gazebo   (gz_x500_depth, world apf_test)
#   2. Micro XRCE-DDS Agent (udp4 :8888)
#   3. ros_gz_bridge        (depth camera point cloud -> /depth_points)
#   4. ros2 launch apf_oa apf_nodes.launch.py  (sensor + planner [+ rviz])
#
# Usage:
#   ./run.sh                       # defaults (goal 12.0, 0.0, alt 2.0)
#   ./run.sh 8.0 -3.0              # goal_x goal_y (Gazebo world coords)
#   ./run.sh 8.0 -3.0 2.5          # + goal altitude
#   ./run.sh kill                  # tear the session down
#
# Environment overrides:
#   OBST_SOURCE   fixed | depth    obstacle source        (default: fixed)
#   RVIZ          true | false     start RViz             (default: true if DISPLAY set)
#   HEADLESS      1                run gz without GUI
#   PX4_DIR       PX4-Autopilot checkout                  (default: ~/PX4-Autopilot)
#   ROS_SETUP     ROS 2 setup script                      (default: /opt/ros/humble/setup.bash)
#   SESSION       tmux session name                       (default: apf-oa)

set -euo pipefail
tmux set-option -g mouse on 2>/dev/null || true

# --- resolve paths ----------------------------------------------------------
WS_DIR="${WS_DIR:-${HOME}/ws}"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_DIR}/install/setup.bash"
APF_DIR="${WS_DIR}/src/APF"
WORLD_SRC="${APF_DIR}/APF_OA/worlds/apf_test.sdf"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-apf_test}"
SESSION="${SESSION:-apf-oa}"
OBST_SOURCE="${OBST_SOURCE:-fixed}"
if [[ -z "${RVIZ:-}" ]]; then
  [[ -n "${DISPLAY:-}" && -z "${HEADLESS:-}" ]] && RVIZ=true || RVIZ=false
fi

# gz topic of the OakD-Lite depth point cloud (namespace tracks the world name)
DEPTH_TOPIC="/depth_camera/points"

# --- process cleanup ---------------------------------------------------------
kill_sim_procs() {
  local sig="${1:-TERM}"
  pkill -"${sig}" -f 'gz sim'            2>/dev/null || true
  pkill -"${sig}" -x  px4                2>/dev/null || true
  pkill -"${sig}" -f 'MicroXRCEAgent'    2>/dev/null || true
  pkill -"${sig}" -f 'parameter_bridge'  2>/dev/null || true
  pkill -"${sig}" -f 'apf_planner_node'  2>/dev/null || true
  pkill -"${sig}" -f 'obstacle_sensor_node' 2>/dev/null || true
  pkill -"${sig}" -f 'apf_nodes.launch'  2>/dev/null || true
  pkill -"${sig}" -f 'rviz2'             2>/dev/null || true
  pkill -"${sig}" -f 'make px4_sitl'     2>/dev/null || true
}

SIM_PATTERN='gz sim|MicroXRCEAgent|[p]x4|parameter_bridge|apf_planner_node|obstacle_sensor_node|rviz2'

cleanup_sim() {
  kill_sim_procs TERM
  for _ in $(seq 1 8); do
    pgrep -f "${SIM_PATTERN}" >/dev/null 2>&1 || break
    sleep 0.5
  done
  kill_sim_procs KILL
}

# --- teardown shortcut --------------------------------------------------------
if [[ "${1:-}" == "kill" || "${1:-}" == "stop" ]]; then
  tmux kill-session -t "${SESSION}" 2>/dev/null && echo "Killed tmux session '${SESSION}'." \
    || echo "No tmux session '${SESSION}' running."
  cleanup_sim
  echo "Cleaned up sim processes."
  exit 0
fi

# --- navigation goal ----------------------------------------------------------
GOAL_X="${1:-12.0}"
GOAL_Y="${2:-0.0}"
GOAL_Z="${3:-2.0}"

# --- sanity checks --------------------------------------------------------------
fail() { echo "ERROR: $*" >&2; exit 1; }

command -v tmux >/dev/null 2>&1 || fail "tmux not found (sudo apt install tmux)."
[[ -d "${PX4_DIR}" ]]   || fail "PX4-Autopilot not found at '${PX4_DIR}' (set PX4_DIR)."
[[ -f "${ROS_SETUP}" ]] || fail "ROS setup not found at '${ROS_SETUP}' (set ROS_SETUP)."
[[ -f "${WS_SETUP}" ]]  || fail "workspace not built: '${WS_SETUP}' missing (colcon build in ${WS_DIR})."
[[ -f "${WORLD_SRC}" ]] || fail "world not generated: run 'python3 APF_OA/tools/gen_world.py' first."

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  fail "tmux session '${SESSION}' already running. './run.sh kill' first."
fi

if pgrep -f "${SIM_PATTERN}" >/dev/null 2>&1; then
  echo "Found leftover sim processes - cleaning up first..."
  cleanup_sim
fi

# Sync the generated world into PX4's world dir (idempotent)
cp "${WORLD_SRC}" "${PX4_DIR}/Tools/simulation/gz/worlds/apf_test.sdf"

SOURCE_ROS="source '${ROS_SETUP}'; source '${WS_SETUP}'"
HEADLESS_PREFIX=""
[[ -n "${HEADLESS:-}" ]] && HEADLESS_PREFIX="HEADLESS=1 "

echo "Launching '${SESSION}':"
echo "  PX4_DIR      = ${PX4_DIR}"
echo "  world        = ${PX4_GZ_WORLD}   headless=${HEADLESS:-0}"
echo "  goal         = (${GOAL_X}, ${GOAL_Y}, alt ${GOAL_Z})"
echo "  obstacles    = ${OBST_SOURCE}   rviz=${RVIZ}"

# --- tmux layout (track panes by pane-id, not positional index) -----------------
P_SIM="$(tmux new-session -d -P -F '#{pane_id}' -s "${SESSION}" -n sim -c "${PX4_DIR}")"
tmux send-keys -t "${P_SIM}" \
  "${HEADLESS_PREFIX}PX4_GZ_WORLD=${PX4_GZ_WORLD} make px4_sitl gz_x500_depth" C-m

P_AGENT="$(tmux split-window -t "${P_SIM}" -h -P -F '#{pane_id}' -c "${WS_DIR}")"
tmux send-keys -t "${P_AGENT}" \
  "${SOURCE_ROS}; MicroXRCEAgent udp4 -p 8888" C-m

P_BRIDGE="$(tmux split-window -t "${P_AGENT}" -v -P -F '#{pane_id}' -c "${WS_DIR}")"
tmux send-keys -t "${P_BRIDGE}" \
  "${SOURCE_ROS}; sleep 12; ros2 run ros_gz_bridge parameter_bridge \
'${DEPTH_TOPIC}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' \
--ros-args -r '${DEPTH_TOPIC}':=/depth_points" C-m

P_NODES="$(tmux split-window -t "${P_SIM}" -v -P -F '#{pane_id}' -c "${WS_DIR}")"
tmux send-keys -t "${P_NODES}" \
  "${SOURCE_ROS}; sleep 18; ros2 launch apf_oa apf_nodes.launch.py \
goal_x:=${GOAL_X} goal_y:=${GOAL_Y} goal_z:=${GOAL_Z} source:=${OBST_SOURCE} rviz:=${RVIZ}" C-m

tmux select-layout -t "${SESSION}":sim tiled
tmux select-pane   -t "${P_SIM}"

# NO_ATTACH=1 leaves the session detached (batch / CI use)
if [[ -n "${NO_ATTACH:-}" ]]; then
  echo "Session '${SESSION}' running detached (attach: tmux attach -t ${SESSION})"
  exit 0
fi
echo "Attaching... (detach: Ctrl-b d   |   teardown: ./run.sh kill)"
tmux attach-session -t "${SESSION}"
