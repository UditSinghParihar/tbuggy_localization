#!/usr/bin/env python3
"""
Extract camera-IMU extrinsics from TF static transforms and IMU noise stats.
Reads directly from the ROS2 bag sqlite3 database.

Usage:
    python3 extract_tf_and_imu.py /home/udit/data/log_01_ros2/log_01_ros2_0.db3
"""

import sys
import sqlite3
import numpy as np
from rosbags.typesys import get_typestore, Stores

store = get_typestore(Stores.ROS2_HUMBLE)


def quat_to_rot(qx, qy, qz, qw):
    n = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1-2*(qy**2+qz**2), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx**2+qz**2), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)]
    ])


def rot_to_quat(R):
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return np.array([(R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s, 0.25/s])
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return np.array([0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s])
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return np.array([(R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s])
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        return np.array([(R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s])


def make_T(tx, ty, tz, qx, qy, qz, qw):
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(qx, qy, qz, qw)
    T[:3, 3] = [tx, ty, tz]
    return T


def extract_tf_static(db):
    """Read all unique /tf_static transforms."""
    rows = db.execute("SELECT data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name='/tf_static')").fetchall()
    seen = set()
    tfs = {}
    for row in rows:
        msg = store.deserialize_cdr(bytes(row[0]), 'tf2_msgs/msg/TFMessage')
        for t in msg.transforms:
            key = (t.header.frame_id, t.child_frame_id)
            if key not in seen:
                seen.add(key)
                tfs[key] = make_T(
                    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
                    t.transform.rotation.x, t.transform.rotation.y,
                    t.transform.rotation.z, t.transform.rotation.w
                )
    return tfs


def extract_imu_stats(db, n_samples=2000):
    """Extract IMU noise statistics from the first n_samples."""
    rows = db.execute(
        "SELECT data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name='/tbuggy/imu_ins') LIMIT ?",
        (n_samples,)
    ).fetchall()

    accels, gyros = [], []
    frame_id = None
    for row in rows:
        msg = store.deserialize_cdr(bytes(row[0]), 'sensor_msgs/msg/Imu')
        if frame_id is None:
            frame_id = msg.header.frame_id
        accels.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyros.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    accels = np.array(accels)
    gyros = np.array(gyros)
    return frame_id, accels, gyros


def main(db_path):
    db = sqlite3.connect(db_path)

    # --- TF Static ---
    print("=" * 60)
    print("ALL STATIC TRANSFORMS")
    print("=" * 60)
    tfs = extract_tf_static(db)
    for (parent, child), T in tfs.items():
        t = T[:3, 3]
        q = rot_to_quat(T[:3, :3])
        print(f"\n  {parent} -> {child}")
        print(f"    t: [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]")
        print(f"    q: [x={q[0]:.6f}, y={q[1]:.6f}, z={q[2]:.6f}, w={q[3]:.6f}]")

    # --- Camera-IMU Extrinsic ---
    print("\n" + "=" * 60)
    print("CAMERA-IMU EXTRINSIC (T_imu_cam)")
    print("=" * 60)

    # Chain: base_footprint -> os1/os_sensor -> os2/os_sensor -> camera_front
    T_bf_os1  = tfs[('tbuggy/base_footprint', 'tbuggy/os1/os_sensor')]
    T_os1_os2 = tfs[('tbuggy/os1/os_sensor', 'tbuggy/os2/os_sensor')]
    T_os2_cam = tfs[('tbuggy/os2/os_sensor', 'tbuggy/camera_front')]
    T_bf_cam  = T_bf_os1 @ T_os1_os2 @ T_os2_cam

    # IMU: base_footprint -> sensor_wgs84
    T_bf_imu  = tfs[('tbuggy/base_footprint', 'tbuggy/sensor_wgs84')]

    # T_imu_cam: point in cam frame -> IMU frame
    T_imu_cam = np.linalg.inv(T_bf_imu) @ T_bf_cam

    print("\n  Camera origin in base_footprint:")
    print(f"    [{T_bf_cam[0,3]:.4f}, {T_bf_cam[1,3]:.4f}, {T_bf_cam[2,3]:.4f}] m")
    print("\n  IMU origin in base_footprint:")
    print(f"    [{T_bf_imu[0,3]:.4f}, {T_bf_imu[1,3]:.4f}, {T_bf_imu[2,3]:.4f}] m")

    print("\n  T_imu_cam (4x4) for kalibr_imucam_chain.yaml:")
    for row in T_imu_cam:
        print(f"    - [{row[0]:.8f}, {row[1]:.8f}, {row[2]:.8f}, {row[3]:.8f}]")

    q = rot_to_quat(T_imu_cam[:3, :3])
    t = T_imu_cam[:3, 3]
    print(f"\n  Translation (cam origin in IMU): [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] m")
    print(f"  Quaternion [qx,qy,qz,qw]: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]")

    # Sanity check: camera optical axis direction in IMU frame
    cam_z_in_imu = T_imu_cam[:3, :3] @ np.array([0, 0, 1])
    print(f"\n  Camera +Z (optical/forward) in IMU frame: [{cam_z_in_imu[0]:.3f}, {cam_z_in_imu[1]:.3f}, {cam_z_in_imu[2]:.3f}]")
    print(f"  (should be roughly along +X of robot if camera faces forward)")

    # --- IMU Stats ---
    print("\n" + "=" * 60)
    print("IMU NOISE STATISTICS")
    print("=" * 60)

    frame_id, accels, gyros = extract_imu_stats(db, n_samples=2000)
    rate = 100.0
    print(f"\n  IMU frame_id: '{frame_id}'")
    print(f"  Sample rate: ~{rate:.0f} Hz")
    print(f"  Samples used: {len(accels)}")
    print(f"\n  Accel mean  [m/s^2]: [{accels[:,0].mean():.4f}, {accels[:,1].mean():.4f}, {accels[:,2].mean():.4f}]")
    print(f"  Accel std   [m/s^2]: [{accels[:,0].std():.4f}, {accels[:,1].std():.4f}, {accels[:,2].std():.4f}]")
    print(f"  Accel norm  (expect 9.81): {np.linalg.norm(accels.mean(axis=0)):.4f}")
    print(f"\n  Gyro mean  [rad/s]:  [{gyros[:,0].mean():.6f}, {gyros[:,1].mean():.6f}, {gyros[:,2].mean():.6f}]")
    print(f"  Gyro std   [rad/s]:  [{gyros[:,0].std():.6f}, {gyros[:,1].std():.6f}, {gyros[:,2].std():.6f}]")

    # Noise density estimates: sigma_continuous = sigma_discrete / sqrt(rate)
    accel_nd = accels.std(axis=0) / np.sqrt(rate)
    gyro_nd  = gyros.std(axis=0)  / np.sqrt(rate)
    print(f"\n  Approx accel noise density [m/s^2/sqrt(Hz)]: {accel_nd.max():.4e} (max axis)")
    print(f"  Approx gyro  noise density [rad/s/sqrt(Hz)]:  {gyro_nd.max():.4e} (max axis)")
    print("\n  Note: these are upper bounds (include motion), use as starting point for config.")

    db.close()


if __name__ == '__main__':
    db_path = sys.argv[1] if len(sys.argv) > 1 else "/home/udit/data/log_01_ros2/log_01_ros2_0.db3"
    main(db_path)
