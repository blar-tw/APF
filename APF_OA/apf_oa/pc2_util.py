"""Minimal numpy <-> sensor_msgs/PointCloud2 helpers (xyz only)."""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract (N, 3) float xyz from a PointCloud2, NaN rows dropped."""
    x_off = y_off = z_off = None
    for f in msg.fields:
        if f.name == 'x':
            x_off = f.offset
        elif f.name == 'y':
            y_off = f.offset
        elif f.name == 'z':
            z_off = f.offset
    if x_off is None or y_off is None or z_off is None:
        return np.zeros((0, 3))

    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))

    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    # gz bridge clouds are densely packed; guard against row padding anyway
    if msg.height > 1 and msg.row_step != msg.width * msg.point_step:
        rows = buf.reshape(msg.height, msg.row_step)
        buf = rows[:, :msg.width * msg.point_step].reshape(-1)
    pts_raw = buf[:n * msg.point_step].reshape(n, msg.point_step)

    xyz = np.empty((n, 3), dtype=np.float32)
    for i, off in enumerate((x_off, y_off, z_off)):
        xyz[:, i] = pts_raw[:, off:off + 4].copy().view(np.float32)[:, 0]
    return xyz[np.isfinite(xyz).all(axis=1)]


def xyz_to_cloud(xyz: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Pack an (N, 3) array into an unorganized float32 PointCloud2."""
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = xyz.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * xyz.shape[0]
    msg.is_dense = True
    msg.data = xyz.tobytes()
    return msg
