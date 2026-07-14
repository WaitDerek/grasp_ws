import math

import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped


def _quat_xyzw_to_matrix(quat: np.ndarray) -> np.ndarray:
    x, y, z, w = quat
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _matrix_to_quat_xyzw(rot: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    quat = np.array([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quat / norm


def pose_from_json(source_frame: str, stamp, result: dict) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = source_frame
    pose.header.stamp = stamp
    pose.pose.position.x = float(result["position"]["x"])
    pose.pose.position.y = float(result["position"]["y"])
    pose.pose.position.z = float(result["position"]["z"])
    pose.pose.orientation.x = float(result["orientation"]["x"])
    pose.pose.orientation.y = float(result["orientation"]["y"])
    pose.pose.orientation.z = float(result["orientation"]["z"])
    pose.pose.orientation.w = float(result["orientation"]["w"])
    return pose


def transform_pose(pose: PoseStamped, transform: TransformStamped) -> PoseStamped:
    translation = np.array(
        [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ],
        dtype=np.float64,
    )
    transform_quat = np.array(
        [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ],
        dtype=np.float64,
    )
    pose_quat = np.array(
        [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ],
        dtype=np.float64,
    )
    transform_rot = _quat_xyzw_to_matrix(transform_quat)
    pose_rot = _quat_xyzw_to_matrix(pose_quat)
    pose_pos = np.array(
        [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
        dtype=np.float64,
    )

    out_pos = transform_rot @ pose_pos + translation
    out_rot = transform_rot @ pose_rot
    out_quat = _matrix_to_quat_xyzw(out_rot)

    transformed = PoseStamped()
    transformed.header.frame_id = transform.header.frame_id
    transformed.header.stamp = transform.header.stamp
    transformed.pose.position.x = float(out_pos[0])
    transformed.pose.position.y = float(out_pos[1])
    transformed.pose.position.z = float(out_pos[2])
    transformed.pose.orientation.x = float(out_quat[0])
    transformed.pose.orientation.y = float(out_quat[1])
    transformed.pose.orientation.z = float(out_quat[2])
    transformed.pose.orientation.w = float(out_quat[3])
    return transformed
