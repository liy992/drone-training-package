"""Point cloud helpers for the training bridge node."""

from __future__ import annotations

import math

from sensor_msgs.msg import PointCloud2


def point_cloud_stamp_to_seconds(cloud: PointCloud2) -> float:
    return float(cloud.header.stamp.sec) + float(cloud.header.stamp.nanosec) * 1e-9


def point_cloud_layout_key(cloud: PointCloud2):
    field_key = tuple(
        (field.name, field.offset, field.datatype, field.count) for field in cloud.fields
    )
    return (field_key, bool(cloud.is_bigendian), int(cloud.point_step))


def prune_accumulated_clouds(clouds, latest_stamp_s: float, window_s: float, max_frames: int):
    if not clouds:
        return []

    if window_s > 0.0:
        clouds = [
            cloud
            for cloud in clouds
            if latest_stamp_s - point_cloud_stamp_to_seconds(cloud) <= window_s
        ]

    if max_frames > 0:
        clouds = clouds[-max_frames:]

    return clouds


def combine_point_clouds(clouds, frame_id: str) -> PointCloud2:
    clouds = list(clouds)
    if not clouds:
        raise ValueError("Cannot combine an empty point cloud list")

    first = clouds[0]
    layout_key = point_cloud_layout_key(first)
    for cloud in clouds[1:]:
        if point_cloud_layout_key(cloud) != layout_key:
            raise ValueError("Point cloud layouts differ")

    combined = PointCloud2()
    combined.header.stamp = clouds[-1].header.stamp
    combined.header.frame_id = frame_id
    combined.height = 1
    combined.fields = list(first.fields)
    combined.is_bigendian = first.is_bigendian
    combined.point_step = first.point_step
    combined.is_dense = all(cloud.is_dense for cloud in clouds)
    combined.data = b"".join(bytes(cloud.data) for cloud in clouds)
    combined.width = (
        len(combined.data) // combined.point_step
        if combined.point_step > 0
        else sum(cloud.width * max(cloud.height, 1) for cloud in clouds)
    )
    combined.row_step = combined.width * combined.point_step
    return combined


def cloud_has_xyz_fields(cloud: PointCloud2) -> bool:
    field_names = {field.name for field in cloud.fields}
    return {"x", "y", "z"}.issubset(field_names)


def quaternion_to_rotation_matrix(q):
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def transform_point_cloud(cloud: PointCloud2, transform, target_frame: str) -> PointCloud2:
    transformed = PointCloud2()
    transformed.header = cloud.header
    transformed.header.frame_id = target_frame
    transformed.height = cloud.height
    transformed.width = cloud.width
    transformed.fields = list(cloud.fields)
    transformed.is_bigendian = cloud.is_bigendian
    transformed.point_step = cloud.point_step
    transformed.row_step = cloud.row_step
    transformed.is_dense = cloud.is_dense

    if not cloud_has_xyz_fields(cloud):
        transformed.data = bytes(cloud.data)
        return transformed

    from sensor_msgs_py import point_cloud2

    points = point_cloud2.read_points(cloud, skip_nans=True)
    output = points.copy()
    rotation = quaternion_to_rotation_matrix(transform.transform.rotation)
    tx = float(transform.transform.translation.x)
    ty = float(transform.transform.translation.y)
    tz = float(transform.transform.translation.z)

    x = points["x"].copy()
    y = points["y"].copy()
    z = points["z"].copy()
    output["x"] = rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z + tx
    output["y"] = rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z + ty
    output["z"] = rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z + tz

    transformed = point_cloud2.create_cloud(transformed.header, cloud.fields, output)
    transformed.is_dense = cloud.is_dense
    return transformed


def filter_points_near_position(cloud: PointCloud2, center_xyz, min_range: float, xy_radius: float = 0.0, z_radius: float = 0.0):
    if (
        min_range <= 0.0
        and (xy_radius <= 0.0 or z_radius <= 0.0)
    ) or not cloud_has_xyz_fields(cloud):
        return cloud

    from sensor_msgs_py import point_cloud2

    points = point_cloud2.read_points(cloud, skip_nans=True)
    if len(points) == 0:
        return cloud

    dx = points["x"] - float(center_xyz[0])
    dy = points["y"] - float(center_xyz[1])
    dz = points["z"] - float(center_xyz[2])
    keep = True
    if min_range > 0.0:
        keep = (dx * dx + dy * dy + dz * dz) >= float(min_range) ** 2
    if xy_radius > 0.0 and z_radius > 0.0:
        outside_body = (
            (dx * dx + dy * dy) >= float(xy_radius) ** 2
        ) | (abs(dz) >= float(z_radius))
        keep = keep & outside_body

    if bool(keep.all()):
        return cloud

    filtered = point_cloud2.create_cloud(cloud.header, cloud.fields, points[keep])
    filtered.is_dense = cloud.is_dense
    return filtered
