import math

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def trajectory_to_path(xyz, frame_id="map", stamp=None):
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    if stamp is not None:
        path_msg.header.stamp = stamp

    for idx, point in enumerate(xyz):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        if stamp is not None:
            pose.header.stamp = stamp
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = float(point[2])

        if idx + 1 < len(xyz):
            dx = float(xyz[idx + 1][0] - point[0])
            dy = float(xyz[idx + 1][1] - point[1])
            yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-6 else 0.0
        elif idx > 0:
            dx = float(point[0] - xyz[idx - 1][0])
            dy = float(point[1] - xyz[idx - 1][1])
            yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-6 else 0.0
        else:
            yaw = 0.0
        pose.pose.orientation = yaw_to_quaternion(yaw)
        path_msg.poses.append(pose)

    return path_msg


def xyz_to_cloud(points, frame_id="map", stamp=None):
    import numpy as np  # pylint: disable=import-outside-toplevel

    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    header = None
    if stamp is not None:
        from std_msgs.msg import Header  # pylint: disable=import-outside-toplevel

        header = Header()
        header.frame_id = frame_id
        header.stamp = stamp
    return point_cloud2.create_cloud(header, fields, points.tolist())
