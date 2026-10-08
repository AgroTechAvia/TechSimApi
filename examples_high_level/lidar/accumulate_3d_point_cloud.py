"""Accumulate 3D lidar scans in the map frame and display them on shutdown.

Run the simulator with a ``Lidar3D0`` socket, start this script, and stop
collection with Ctrl+C. The drone may be flown by another controller while the
script is running.
"""
from __future__ import annotations

import copy
import signal
import threading
import time

import numpy as np

from agrotechsimapi import HighLevelSimClient, PointCloud


ip = "127.0.0.1"
port = 5762  # TCP-порт MSP; RPC симулятора использует отдельный порт 8080.
sim_port = 8080

SCAN_RATE_HZ = 10.0
CHANNEL_COUNT = 32
POINTS_PER_CHANNEL = 512
ANGLE_BELOW_ZERO = np.deg2rad(45.0)
ANGLE_ABOVE_ZERO = np.deg2rad(15.0)
RANGE_MIN = 0
RANGE_MAX = 50
VOXEL_SIZE_METERS = 0.01


def _pose(kinematics: dict) -> tuple[np.ndarray, np.ndarray]:
    position = np.asarray(kinematics["location"][:3], dtype=np.float64)
    qx, qy, qz, qw = (float(value) for value in kinematics["orientation"][:4])
    quaternion = np.asarray([qw, qx, qy, qz], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm == 0.0 or not np.isfinite(norm):
        raise ValueError("Simulator returned an invalid orientation quaternion")
    return position, quaternion / norm


def _rotate_vectors(vectors: np.ndarray, quaternions: np.ndarray) -> np.ndarray:
    """Rotate matching vectors by unit quaternions stored as (w, x, y, z)."""
    xyz = quaternions[:, 1:]
    uv = np.cross(xyz, vectors)
    uuv = np.cross(xyz, uv)
    return vectors + 2.0 * (quaternions[:, :1] * uv + uuv)


def scan_to_map(
    cloud: PointCloud,
    kinematics_before: dict,
    kinematics_after: dict,
) -> np.ndarray:
    """Transform valid local points using interpolated drone telemetry."""
    local_points = cloud.valid_points.astype(np.float64, copy=False)
    if local_points.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    position_before, quaternion_before = _pose(kinematics_before)
    position_after, quaternion_after = _pose(kinematics_after)
    if np.dot(quaternion_before, quaternion_after) < 0.0:
        quaternion_after = -quaternion_after

    offsets = cloud.time_offset_ns[cloud.valid].astype(np.float64)
    duration = max(float(cloud.scan_duration_ns), 1.0)
    alpha = np.clip(offsets / duration, 0.0, 1.0)[:, None]
    positions = position_before + alpha * (position_after - position_before)

    # Normalized linear interpolation is stable for the small rotation between
    # telemetry samples that bracket one approximately 50 ms scan.
    quaternions = quaternion_before + alpha * (quaternion_after - quaternion_before)
    quaternions /= np.linalg.norm(quaternions, axis=1, keepdims=True)

    # The RPC points are expressed in the lidar frame, not directly in the
    # drone body frame. Applying the socket mounting transform is essential
    # when Lidar3D0 has a non-zero rotation (for example a 180-degree roll).
    mount_qx, mount_qy, mount_qz, mount_qw = cloud.sensor_orientation_body
    mount_quaternion_wxyz = np.asarray(
        [mount_qw, mount_qx, mount_qy, mount_qz], dtype=np.float64
    )
    mount_quaternions = np.broadcast_to(
        mount_quaternion_wxyz,
        (len(local_points), 4),
    )
    body_points = (
        _rotate_vectors(local_points, mount_quaternions)
        + cloud.sensor_position_body
    )
    return (_rotate_vectors(body_points, quaternions) + positions).astype(np.float32)


def _voxel_downsample_numpy(points: np.ndarray) -> np.ndarray:
    """Fallback voxel filter used when Open3D is not installed."""
    voxel_indices = np.floor(points / VOXEL_SIZE_METERS).astype(np.int64)
    _, first_indices = np.unique(voxel_indices, axis=0, return_index=True)
    return points[np.sort(first_indices)]


def show_cloud(points: np.ndarray) -> None:
    """Open an interactive Z-up point-cloud viewer."""
    try:
        import open3d as o3d
    except ImportError:
        import matplotlib.pyplot as plt

        points = _voxel_downsample_numpy(points)
        print(
            "Open3D is not installed; using the slower Matplotlib fallback. "
            "Install it with: pip install open3d"
        )
        figure = plt.figure("AgroTechSim accumulated 3D lidar cloud")
        axes = figure.add_subplot(111, projection="3d")
        axes.scatter(
            points[:, 0], points[:, 1], points[:, 2],
            s=0.25, c=points[:, 2], cmap="viridis",
        )
        axes.set_xlabel("X, m")
        axes.set_ylabel("Y, m")
        axes.set_zlabel("Z up, m")
        axes.set_title(f"Voxel-filtered point cloud ({len(points):,} points)")
        axes.set_box_aspect(np.maximum(np.ptp(points, axis=0), 0.01))
        axes.view_init(elev=25.0, azim=-135.0)
        plt.show()
        return

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64, copy=False))
    point_cloud = point_cloud.voxel_down_sample(voxel_size=VOXEL_SIZE_METERS)

    filtered_points = np.asarray(point_cloud.points)
    z_min = float(filtered_points[:, 2].min())
    z_span = max(float(np.ptp(filtered_points[:, 2])), 1e-6)
    normalized_height = (filtered_points[:, 2] - z_min) / z_span
    point_cloud.colors = o3d.utility.Vector3dVector(np.column_stack((
        0.15 + 0.85 * normalized_height,
        0.35 + 0.45 * (1.0 - np.abs(2.0 * normalized_height - 1.0)),
        1.0 - 0.85 * normalized_height,
    )))

    center = filtered_points.mean(axis=0)
    extent = np.maximum(np.ptp(filtered_points, axis=0), 0.01)
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=max(0.25, float(np.linalg.norm(extent)) * 0.05),
        origin=[0.0, 0.0, 0.0],
    )
    print(
        f"Voxel filter {VOXEL_SIZE_METERS:.2f} m: "
        f"{len(points):,} -> {len(filtered_points):,} displayed points"
    )
    o3d.visualization.draw_geometries(
        [point_cloud, coordinate_frame],
        window_name="AgroTechSim 3D lidar point cloud (Z up)",
        width=1280,
        height=720,
        zoom=0.7,
        front=[-1.0, -1.0, -0.55],
        lookat=center.tolist(),
        up=[0.0, 0.0, 1.0],
    )


def main() -> None:
    stop_requested = threading.Event()

    def request_stop(_signum=None, _frame=None):
        stop_requested.set()

    signal.signal(signal.SIGINT, request_stop)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, request_stop)

    client = HighLevelSimClient()
    accumulated_scans: list[np.ndarray] = []
    period = 1.0 / SCAN_RATE_HZ

    try:
        client.connect(ip, port, sim_port=sim_port)
        print("3D lidar collection started at 10 Hz. Press Ctrl+C to stop and display the cloud.")
        while not stop_requested.is_set():
            iteration_started = time.monotonic()
            before = copy.deepcopy(client.get_sim_kinematics())
            if before is None:
                time.sleep(0.01)
                continue

            cloud = client.get_lidar_point_cloud(
                angle_below_zero=ANGLE_BELOW_ZERO,
                angle_above_zero=ANGLE_ABOVE_ZERO,
                range_min=RANGE_MIN,
                range_max=RANGE_MAX,
                channel_count=CHANNEL_COUNT,
                points_per_channel=POINTS_PER_CHANNEL,
            )
            after = copy.deepcopy(client.get_sim_kinematics())
            if after is not None:
                accumulated_scans.append(scan_to_map(cloud, before, after))

            remaining = period - (time.monotonic() - iteration_started)
            stop_requested.wait(max(0.0, remaining))
    finally:
        client.disconnect()

    nonempty_scans = [scan for scan in accumulated_scans if len(scan)]
    if not nonempty_scans:
        print("Collection stopped: no lidar hits were received.")
        return

    combined_cloud = np.concatenate(nonempty_scans, axis=0)
    print(f"Collection stopped: {len(combined_cloud):,} points accumulated. Opening 3D view...")
    show_cloud(combined_cloud)


if __name__ == "__main__":
    main()
