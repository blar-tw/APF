#!/usr/bin/env python3
"""APF navigation node for PX4 offboard control.

State machine (same skeleton as HOLO-DWA's scanner.py):
  INIT -> (20 heartbeats, set Offboard, arm) -> TAKEOFF -> NAVIGATE -> GOAL_REACHED

NAVIGATE runs one 3D APF step per 20 Hz tick over the obstacle cloud from
/apf/obstacle_points (PX4 local NED) and commands a full 3D velocity setpoint
(PX4 per-axis NaN passthrough). The nose is kept on the direction of travel so
the forward-facing depth camera looks where the drone is going.

Every tick is appended to a CSV (position, forces, command, stuck flag) for
offline analysis; an RViz Path + goal/force markers are published in the ENU
`map` frame.
"""

import csv
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy, qos_profile_sensor_data)
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleOdometry)
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from apf_oa import apf_core
from apf_oa.pc2_util import cloud_to_xyz

CSV_FIELDS = [
    't', 'state', 'n', 'e', 'd', 'vn', 've', 'vd',
    'fatt_n', 'fatt_e', 'fatt_d', 'frep_n', 'frep_e', 'frep_d',
    'ftot_n', 'ftot_e', 'ftot_d', 'vcmd_n', 'vcmd_e', 'vcmd_d',
    'dist_goal', 'n_obs', 'stuck', 'stuck_episodes',
]


class ApfPlannerNode(Node):
    def __init__(self):
        super().__init__('apf_planner_node')

        # PX4 pub/sub QoS (same as HOLO-DWA scanner.py)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- parameters (overridable via config/apf_params.yaml) ---
        self.declare_parameter('goal_x', 12.0)   # Gazebo world ENU
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_z', 2.0)    # altitude, m up
        self.declare_parameter('goal_threshold', 0.5)
        self.declare_parameter('k_att', 0.8)
        self.declare_parameter('att_saturation', 3.0)
        self.declare_parameter('k_rep', 1.8)
        self.declare_parameter('influence_radius', 3.0)
        self.declare_parameter('max_rep_force', 6.0)
        self.declare_parameter('tangential_gain', 1.5)
        self.declare_parameter('swirl_safe_radius', 0.5)
        self.declare_parameter('swirl_taper_band', 0.5)
        self.declare_parameter('v_max', 1.5)
        self.declare_parameter('max_accel', 4.0)
        self.declare_parameter('stuck_speed_eps', 0.1)
        self.declare_parameter('stuck_ticks', 40)
        self.declare_parameter('log_dir', os.path.expanduser('~/ws/src/APF/logs'))

        self.cfg = apf_core.Config(
            k_att=float(self.get_parameter('k_att').value),
            att_saturation=float(self.get_parameter('att_saturation').value),
            k_rep=float(self.get_parameter('k_rep').value),
            influence_radius=float(self.get_parameter('influence_radius').value),
            max_rep_force=float(self.get_parameter('max_rep_force').value),
            tangential_gain=float(self.get_parameter('tangential_gain').value),
            swirl_safe_radius=float(self.get_parameter('swirl_safe_radius').value),
            swirl_taper_band=float(self.get_parameter('swirl_taper_band').value),
            v_max=float(self.get_parameter('v_max').value),
            max_accel=float(self.get_parameter('max_accel').value),
            goal_threshold=float(self.get_parameter('goal_threshold').value),
            stuck_speed_eps=float(self.get_parameter('stuck_speed_eps').value),
            stuck_ticks=int(self.get_parameter('stuck_ticks').value),
        )
        self.stuck_detector = apf_core.StuckDetector(self.cfg)

        # --- PX4 interface ---
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                 self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/apf/obstacle_points',
                                 self.cloud_cb, qos_profile_sensor_data)

        # --- RViz ---
        self.path_pub = self.create_publisher(Path, '/apf/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/apf/planner_markers', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        self.last_path_sec = 0.0

        # --- state ---
        self.dt = 0.05              # control period (s), matches the 20 Hz timer
        self.nav_state = 'INIT'
        self.heartbeat_counter = 0
        self.pos = np.zeros(3)      # NED
        self.vel = np.zeros(3)      # NED
        self.v_cmd_prev = np.zeros(3)  # last commanded velocity, for accel limit
        self.yaw = 0.0
        self.have_odom = False
        self.obstacles = None       # (N, 3) NED
        self.last_cloud_sec = 0.0
        self.hold_pos = np.zeros(3)
        self.hold_yaw = 0.0
        self.last_status_sec = 0.0
        self.last_warn_sec = 0.0
        self.t0 = time.time()

        # goal: Gazebo world ENU parameter -> local NED
        gx = float(self.get_parameter('goal_x').value)
        gy = float(self.get_parameter('goal_y').value)
        gz = float(self.get_parameter('goal_z').value)
        self.goal_ned = apf_core.enu_to_ned((gx, gy, gz))
        self.takeoff_alt = -abs(gz)  # climb straight to the goal altitude

        # --- CSV logging ---
        log_dir = self.get_parameter('log_dir').value
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(
            log_dir, time.strftime('apf_%Y%m%d_%H%M%S.csv'))
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(CSV_FIELDS)
        self.tick_count = 0

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz
        self.get_logger().info(
            f"goal ENU=({gx:.1f}, {gy:.1f}, {gz:.1f}) -> NED={np.round(self.goal_ned, 2)} "
            f"| k_att={self.cfg.k_att} k_rep={self.cfg.k_rep} d0={self.cfg.influence_radius} "
            f"| log: {self.csv_path}")

    # --- callbacks ---

    def odom_cb(self, msg):
        if np.all(np.isfinite(msg.position)):
            self.pos = np.array(msg.position, dtype=float)
            self.have_odom = True
        if np.all(np.isfinite(msg.velocity)):
            self.vel = np.array(msg.velocity, dtype=float)
        if np.all(np.isfinite(msg.q)):
            w, x, y, z = msg.q
            self.yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def cloud_cb(self, msg):
        self.obstacles = cloud_to_xyz(msg)
        self.last_cloud_sec = self.get_clock().now().nanoseconds / 1e9

    # --- 20 Hz control loop ---

    def tick(self):
        self.publish_heartbeat(use_velocity=(self.nav_state == 'NAVIGATE'))

        if self.nav_state == 'INIT':
            if self.heartbeat_counter == 20:
                self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.nav_state = 'TAKEOFF'
                self.get_logger().info('Offboard + arm sent, taking off')

        elif self.nav_state == 'TAKEOFF':
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_alt,
                                           yaw=self.yaw_to_goal())
            if self.have_odom and self.pos[2] < (self.takeoff_alt + 0.2):
                self.nav_state = 'NAVIGATE'
                self.v_cmd_prev = self.vel.copy()  # seed accel limiter from real motion
                self.get_logger().info('Reached takeoff altitude, APF navigation starts')

        elif self.nav_state == 'NAVIGATE':
            self.navigate()

        elif self.nav_state == 'GOAL_REACHED':
            self.publish_position_setpoint(*self.hold_pos, yaw=self.hold_yaw)

        self.heartbeat_counter += 1

    def navigate(self):
        dist_goal = float(np.linalg.norm(self.goal_ned - self.pos))
        if dist_goal < self.cfg.goal_threshold:
            self.hold_pos = self.pos.copy()
            self.hold_yaw = self.yaw
            self.nav_state = 'GOAL_REACHED'
            self.csv_file.flush()  # NAVIGATE stops logging; push out the buffered tail
            self.get_logger().info('>>> GOAL REACHED <<<')
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self.obstacles is None:
            # Hover until the obstacle source is up rather than flying blind
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_alt,
                                           yaw=self.yaw_to_goal())
            self.v_cmd_prev = self.vel.copy()  # resume the accel limiter from rest
            if now - self.last_warn_sec > 2.0:
                self.get_logger().warn('No /apf/obstacle_points yet - hovering')
                self.last_warn_sec = now
            return
        if now - self.last_cloud_sec > 2.0 and now - self.last_warn_sec > 2.0:
            self.get_logger().warn(
                f'Obstacle cloud is stale ({now - self.last_cloud_sec:.1f}s old)')
            self.last_warn_sec = now

        v_cmd, info = apf_core.apf_step(self.pos, self.goal_ned, self.obstacles,
                                        self.cfg, v_prev=self.v_cmd_prev, dt=self.dt)
        self.v_cmd_prev = v_cmd

        speed = float(np.linalg.norm(self.vel))
        stuck = self.stuck_detector.update(speed, dist_goal, pos=self.pos)
        if stuck and self.stuck_detector.count == self.cfg.stuck_ticks:
            self.get_logger().warn(
                f'LOCAL MINIMUM suspected at NED {np.round(self.pos, 2)} '
                f'(episode {self.stuck_detector.episodes})')

        # Face the direction of travel (depth camera FOV), or the goal when slow
        vxy = math.hypot(v_cmd[0], v_cmd[1])
        yaw = math.atan2(v_cmd[1], v_cmd[0]) if vxy > 0.3 else self.yaw_to_goal()
        self.publish_velocity_setpoint(v_cmd, yaw=yaw)

        self.log_tick(info, v_cmd, dist_goal, stuck)
        self.publish_viz(info)

        if now - self.last_status_sec >= 1.0:
            self.last_status_sec = now
            self.get_logger().info(
                f"[APF] pos NED=({self.pos[0]:+.2f},{self.pos[1]:+.2f},{self.pos[2]:+.2f}) "
                f"v_cmd=({v_cmd[0]:+.2f},{v_cmd[1]:+.2f},{v_cmd[2]:+.2f}) "
                f"|Fatt|={np.linalg.norm(info['f_att']):.2f} "
                f"|Frep|={np.linalg.norm(info['f_rep']):.2f} "
                f"obs={self.obstacles.shape[0]} dist={dist_goal:.2f}m"
                + (' [STUCK]' if stuck else ''))

    # --- helpers ---

    def yaw_to_goal(self):
        dn = self.goal_ned[0] - self.pos[0]
        de = self.goal_ned[1] - self.pos[1]
        if math.hypot(dn, de) < 1e-6:
            return self.yaw
        return math.atan2(de, dn)

    def log_tick(self, info, v_cmd, dist_goal, stuck):
        self.csv.writerow([
            round(time.time() - self.t0, 3), self.nav_state,
            *np.round(self.pos, 4), *np.round(self.vel, 4),
            *np.round(info['f_att'], 4), *np.round(info['f_rep'], 4),
            *np.round(info['f_total'], 4), *np.round(v_cmd, 4),
            round(dist_goal, 4), self.obstacles.shape[0],
            int(stuck), self.stuck_detector.episodes,
        ])
        self.tick_count += 1
        if self.tick_count % 20 == 0:
            self.csv_file.flush()

    def publish_viz(self, info):
        now = self.get_clock().now()
        stamp = now.to_msg()
        now_sec = now.nanoseconds / 1e9

        pos_enu = apf_core.ned_to_enu(self.pos)
        if now_sec - self.last_path_sec > 0.5:
            self.last_path_sec = now_sec
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = stamp
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos_enu
            ps.pose.orientation.w = 1.0
            self.path_msg.poses.append(ps)
            if len(self.path_msg.poses) > 2000:
                self.path_msg.poses.pop(0)
            self.path_msg.header.stamp = stamp
            self.path_pub.publish(self.path_msg)

        arr = MarkerArray()
        goal = Marker()
        goal.header.frame_id = 'map'
        goal.header.stamp = stamp
        goal.ns, goal.id, goal.type = 'apf', 0, Marker.SPHERE
        g_enu = apf_core.ned_to_enu(self.goal_ned)
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = g_enu
        goal.pose.orientation.w = 1.0
        goal.scale.x = goal.scale.y = goal.scale.z = 2 * self.cfg.goal_threshold
        goal.color.g, goal.color.a = 1.0, 0.6
        arr.markers.append(goal)

        force = Marker()
        force.header.frame_id = 'map'
        force.header.stamp = stamp
        force.ns, force.id, force.type = 'apf', 1, Marker.ARROW
        f_enu = apf_core.ned_to_enu(info['f_total'])
        tip = pos_enu + f_enu
        force.points = []
        for p in (pos_enu, tip):
            pt = Point()
            pt.x, pt.y, pt.z = (float(c) for c in p)
            force.points.append(pt)
        force.scale.x, force.scale.y = 0.06, 0.15
        force.color.r, force.color.b, force.color.a = 1.0, 1.0, 0.9
        arr.markers.append(force)
        self.marker_pub.publish(arr)

    # --- low-level PX4 publishers (same pattern as scanner.py) ---

    def publish_heartbeat(self, use_velocity):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = use_velocity
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_position_setpoint(self, n, e, d, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(n), float(e), float(d)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def publish_velocity_setpoint(self, v_ned, yaw=0.0):
        """Full 3D velocity command via PX4 per-axis NaN passthrough."""
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.position = [nan, nan, nan]
        msg.velocity = [float(v_ned[0]), float(v_ned[1]), float(v_ned[2])]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def publish_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1, msg.param2 = param1, param2
        msg.command = command
        msg.target_system = msg.target_component = 1
        msg.source_system = msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def destroy_node(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ApfPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
