#!/usr/bin/env python3
"""Obstacle source for the APF planner.

Two modes (parameter `source`):
  fixed - publish the known world_spec cylinder surface points (Step 2:
          validates the APF pipeline with ground truth, no perception noise)
  depth - convert the x500_depth OakD-Lite point cloud (/depth_points from
          ros_gz_bridge, gz sensor frame) into the PX4 local NED frame using
          the latest odometry

Either way the planner consumes one topic: /apf/obstacle_points, an
unorganized xyz PointCloud2 in the PX4 local NED frame. RViz ground-truth
cylinder markers and an ENU copy of the cloud are also published.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from apf_oa import apf_core, world_spec
from apf_oa.pc2_util import cloud_to_xyz, xyz_to_cloud

# OakD-Lite mount on x500_depth (model.sdf include pose, gz body FLU)
CAM_OFFSET_FRD = np.array([0.12, -0.03, -0.242])


class ObstacleSensorNode(Node):
    def __init__(self):
        super().__init__('obstacle_sensor_node')

        self.declare_parameter('source', 'fixed')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('max_range', 8.0)     # depth: ignore returns farther than this
        self.declare_parameter('stride', 8)          # depth: pixel subsampling step
        self.declare_parameter('ground_min_alt', 0.3)  # depth: drop returns below this altitude
        self.declare_parameter('max_points', 800)    # depth: cap points fed to the planner

        self.source = self.get_parameter('source').value
        self.max_range = float(self.get_parameter('max_range').value)
        self.stride = int(self.get_parameter('stride').value)
        self.ground_min_alt = float(self.get_parameter('ground_min_alt').value)
        self.max_points = int(self.get_parameter('max_points').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.points_pub = self.create_publisher(PointCloud2, '/apf/obstacle_points', 10)
        self.viz_cloud_pub = self.create_publisher(PointCloud2, '/apf/obstacle_points_viz', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/apf/obstacle_markers', 10)

        # Ground-truth cylinder markers for RViz in both modes (map frame, ENU)
        self.marker_timer = self.create_timer(1.0, self.publish_markers)

        if self.source == 'fixed':
            pts_enu = world_spec.obstacle_points_enu()
            self.fixed_pts_ned = np.stack(
                [pts_enu[:, 1], pts_enu[:, 0], -pts_enu[:, 2]], axis=1)
            self.fixed_pts_enu = pts_enu
            self.create_timer(1.0 / rate, self.publish_fixed)
            self.get_logger().info(
                f"source=fixed: {self.fixed_pts_ned.shape[0]} surface points from world_spec")
        else:
            self.min_pub_period = 1.0 / rate
            self.last_pub_sec = 0.0
            # Latest odometry, needed to place camera points into local NED
            self.pos = np.zeros(3)
            self.q = np.array([1.0, 0.0, 0.0, 0.0])
            self.have_odom = False
            self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                     self.odom_cb, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, '/depth_points',
                                     self.depth_cb, qos_profile_sensor_data)
            self.get_logger().info("source=depth: waiting for /depth_points + odometry")

    # --- fixed mode ---

    def publish_fixed(self):
        stamp = self.get_clock().now().to_msg()
        self.points_pub.publish(xyz_to_cloud(self.fixed_pts_ned, 'ned', stamp))
        self.viz_cloud_pub.publish(xyz_to_cloud(self.fixed_pts_enu, 'map', stamp))

    # --- depth mode ---

    def odom_cb(self, msg):
        if np.all(np.isfinite(msg.position)) and np.all(np.isfinite(msg.q)):
            self.pos = np.array(msg.position, dtype=float)
            self.q = np.array(msg.q, dtype=float)
            self.have_odom = True

    def depth_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if not self.have_odom or (now - self.last_pub_sec) < self.min_pub_period:
            return
        self.last_pub_sec = now

        # Organized 640x480 cloud -> pixel-stride subsample before any math
        # (points are in the gz sensor frame: x forward, y left, z up)
        if msg.height > 1:
            xyz = self._strided_xyz(msg)
        else:
            xyz = cloud_to_xyz(msg)[::self.stride]
        if xyz.shape[0] == 0:
            return

        # Range filter in the sensor frame (x is the optical axis)
        r = np.linalg.norm(xyz, axis=1)
        xyz = xyz[(xyz[:, 0] > 0.2) & (r < self.max_range)]
        if xyz.shape[0] == 0:
            self._publish_ned(np.zeros((0, 3)))
            return

        # sensor FLU -> body FRD, add camera mount offset, rotate to NED
        frd = np.stack([xyz[:, 0], -xyz[:, 1], -xyz[:, 2]], axis=1) + CAM_OFFSET_FRD
        w, x, y, z = self.q
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])
        ned = frd @ rot.T + self.pos

        # Drop ground returns (altitude = -D in NED)
        ned = ned[-ned[:, 2] > self.ground_min_alt]
        if ned.shape[0] > self.max_points:
            idx = np.linspace(0, ned.shape[0] - 1, self.max_points).astype(int)
            ned = ned[idx]
        self._publish_ned(ned)

    def _strided_xyz(self, msg):
        """Stride-subsample an organized cloud, then drop NaNs."""
        full = self._raw_xyz(msg)
        grid = full.reshape(msg.height, msg.width, 3)
        sub = grid[::self.stride, ::self.stride].reshape(-1, 3)
        return sub[np.isfinite(sub).all(axis=1)]

    def _raw_xyz(self, msg):
        n = msg.width * msg.height
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if msg.height > 1 and msg.row_step != msg.width * msg.point_step:
            rows = buf.reshape(msg.height, msg.row_step)
            buf = rows[:, :msg.width * msg.point_step].reshape(-1)
        raw = buf[:n * msg.point_step].reshape(n, msg.point_step)
        offs = {f.name: f.offset for f in msg.fields}
        xyz = np.empty((n, 3), dtype=np.float32)
        for i, name in enumerate('xyz'):
            xyz[:, i] = raw[:, offs[name]:offs[name] + 4].copy().view(np.float32)[:, 0]
        return xyz

    def _publish_ned(self, ned):
        stamp = self.get_clock().now().to_msg()
        self.points_pub.publish(xyz_to_cloud(ned, 'ned', stamp))
        if ned.shape[0]:
            enu = np.stack([ned[:, 1], ned[:, 0], -ned[:, 2]], axis=1)
        else:
            enu = ned
        self.viz_cloud_pub.publish(xyz_to_cloud(enu, 'map', stamp))

    # --- RViz ground truth ---

    def publish_markers(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, (name, x, y, r, h) in enumerate(world_spec.CYLINDERS):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(h / 2.0)
            m.pose.orientation.w = 1.0
            m.scale.x = float(2 * r)
            m.scale.y = float(2 * r)
            m.scale.z = float(h)
            tall = h > world_spec.CRUISE_ALT
            m.color.r, m.color.g, m.color.b = (0.8, 0.2, 0.2) if tall else (0.9, 0.6, 0.1)
            m.color.a = 0.85
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
