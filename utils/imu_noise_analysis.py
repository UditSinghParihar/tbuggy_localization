#!/usr/bin/env python3
"""
Compute IMU noise parameters from the stationary period at bag start.

The 4 OpenVINS IMU parameters and how they are computed:

1. accelerometer_noise_density  [m/s^2/sqrt(Hz)]
   - White noise on each raw accel sample (high-frequency, per-sample noise)
   - From stationary data: sigma_nd = std(accel_axis) / sqrt(sample_rate)
   - Allan variance interpretation: the -1/2 slope region (short tau)

2. accelerometer_random_walk  [m/s^3/sqrt(Hz)]  (also called bias random walk)
   - How fast the accelerometer bias drifts over time (low-frequency drift)
   - Needs long Allan variance analysis for accuracy
   - Heuristic from stationary data: ~noise_density / 100 (typical ratio for MEMS)
   - For high-grade INS: often much smaller than MEMS

3. gyroscope_noise_density  [rad/s/sqrt(Hz)]
   - White noise on each raw gyro sample
   - From stationary data: sigma_nd = std(gyro_axis) / sqrt(sample_rate)

4. gyroscope_random_walk  [rad/s^2/sqrt(Hz)]  (also called bias random walk)
   - How fast the gyroscope bias drifts over time
   - Heuristic: ~noise_density / 100

Usage:
    python3 imu_noise_analysis.py
"""

import sqlite3
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rosbags.typesys import get_typestore, Stores

store = get_typestore(Stores.ROS2_HUMBLE)
DB_PATH = "/home/udit/data/log_01_ros2/log_01_ros2_0.db3"
RESULTS_DIR = "/home/udit/codes/tii_assignment/colcon_ws_tii/utils/results"
SAMPLE_RATE = 100.0  # Hz


def load_imu(db, n_max=None):
    """Load all IMU samples as arrays."""
    query = "SELECT data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name='/tbuggy/imu_ins')"
    if n_max:
        query += f" LIMIT {n_max}"
    rows = db.execute(query).fetchall()

    times, ax, ay, az, gx, gy, gz = [], [], [], [], [], [], []
    for row in rows:
        msg = store.deserialize_cdr(bytes(row[0]), 'sensor_msgs/msg/Imu')
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        times.append(t)
        ax.append(msg.linear_acceleration.x)
        ay.append(msg.linear_acceleration.y)
        az.append(msg.linear_acceleration.z)
        gx.append(msg.angular_velocity.x)
        gy.append(msg.angular_velocity.y)
        gz.append(msg.angular_velocity.z)

    return (np.array(times), np.array(ax), np.array(ay), np.array(az),
            np.array(gx), np.array(gy), np.array(gz))


def find_stationary_window(times, ax, ay, az, gx, gy, gz, window_sec=10.0):
    """
    Find the MOST STATIONARY window using minimum combined variance (sliding window).
    More robust than threshold-based detection when the vehicle is always slightly moving.
    """
    GRAVITY = 9.81
    win = max(1, int(window_sec * SAMPLE_RATE))

    accel_norm = np.sqrt(ax**2 + ay**2 + az**2)
    gyro_norm  = np.sqrt(gx**2 + gy**2 + gz**2)

    # Score = variance of (accel_norm - gravity) + 100 * variance of gyro_norm
    # (factor 100 to weight gyro similarly to accel in m/s^2 units)
    best_score = np.inf
    best_start = 0
    for i in range(len(accel_norm) - win):
        a_var = np.var(accel_norm[i:i+win] - GRAVITY)
        g_var = np.var(gyro_norm[i:i+win])
        score = a_var + 100 * g_var
        if score < best_score:
            best_score = score
            best_start = i

    s_end = best_start + win

    # Find where motion clearly starts: first index where accel norm deviates > 0.5 m/s^2
    # from mean of the stationary window, sustained for > 0.5s
    stat_mean_accel = np.mean(accel_norm[best_start:s_end])
    motion_start_idx = s_end  # default: after the stationary window
    for i in range(s_end, len(accel_norm) - 5):
        if np.abs(accel_norm[i] - stat_mean_accel) > 0.5:
            motion_start_idx = i
            break

    return best_start, s_end, motion_start_idx


def compute_noise_params(data_axis, rate):
    """
    Compute continuous-time noise density from discrete stationary samples.

    Discrete std sigma_d relates to continuous noise density sigma_c by:
        sigma_d = sigma_c * sqrt(rate)   =>   sigma_c = sigma_d / sqrt(rate)

    This is because the continuous PSD integrates over the Nyquist band [0, rate/2],
    giving: sigma_d^2 = sigma_c^2 * rate
    """
    sigma_d = np.std(data_axis)
    sigma_c = sigma_d / np.sqrt(rate)
    return sigma_d, sigma_c


def main():
    db = sqlite3.connect(DB_PATH)
    print("Loading IMU data (first 10000 samples)...")
    times, ax, ay, az, gx, gy, gz = load_imu(db, n_max=10000)
    t0 = times[0]
    times -= t0  # relative time in seconds

    # --- Find stationary window ---
    s_start, s_end, motion_idx = find_stationary_window(times, ax, ay, az, gx, gy, gz)
    stat_dur = times[s_end - 1] - times[s_start]

    print(f"\n{'='*60}")
    print(f"STATIONARY WINDOW DETECTED")
    print(f"{'='*60}")
    print(f"  Start index : {s_start}  ({times[s_start]:.2f} s)")
    print(f"  End index   : {s_end}    ({times[s_end-1]:.2f} s)")
    print(f"  Duration    : {stat_dur:.2f} s  ({s_end - s_start} samples)")
    print(f"  Motion first starts at: {times[motion_idx]:.2f} s (index {motion_idx})")

    # Slice stationary period
    ax_s = ax[s_start:s_end]
    ay_s = ay[s_start:s_end]
    az_s = az[s_start:s_end]
    gx_s = gx[s_start:s_end]
    gy_s = gy[s_start:s_end]
    gz_s = gz[s_start:s_end]

    # --- Compute noise parameters ---
    print(f"\n{'='*60}")
    print(f"NOISE PARAMETER COMPUTATION (from stationary window)")
    print(f"{'='*60}")
    print(f"\n  Formula: noise_density = std(axis) / sqrt(sample_rate)")
    print(f"  sample_rate = {SAMPLE_RATE} Hz => sqrt(rate) = {np.sqrt(SAMPLE_RATE):.4f}")

    accel_axes = {'x': ax_s, 'y': ay_s, 'z': az_s}
    gyro_axes  = {'x': gx_s, 'y': gy_s, 'z': gz_s}

    print("\n  ACCELEROMETER:")
    accel_nd_vals = []
    for name, data in accel_axes.items():
        sigma_d, sigma_c = compute_noise_params(data, SAMPLE_RATE)
        accel_nd_vals.append(sigma_c)
        print(f"    {name}-axis: std={sigma_d:.5f} m/s^2  =>  noise_density={sigma_c:.4e} m/s^2/sqrt(Hz)")

    accel_nd = np.max(accel_nd_vals)  # conservative: use max axis
    accel_rw = accel_nd * 0.1         # heuristic: random walk ~10% of noise density
    print(f"\n    => accel_noise_density (max axis): {accel_nd:.4e} m/s^2/sqrt(Hz)")
    print(f"    => accel_random_walk (heuristic x0.1): {accel_rw:.4e} m/s^3/sqrt(Hz)")
    print(f"    (For proper random_walk: Allan Variance analysis over hours is ideal)")

    print("\n  GYROSCOPE:")
    gyro_nd_vals = []
    for name, data in gyro_axes.items():
        sigma_d, sigma_c = compute_noise_params(data, SAMPLE_RATE)
        gyro_nd_vals.append(sigma_c)
        print(f"    {name}-axis: std={sigma_d:.6f} rad/s  =>  noise_density={sigma_c:.4e} rad/s/sqrt(Hz)")

    gyro_nd = np.max(gyro_nd_vals)
    gyro_rw = gyro_nd * 0.1
    print(f"\n    => gyro_noise_density (max axis): {gyro_nd:.4e} rad/s/sqrt(Hz)")
    print(f"    => gyro_random_walk (heuristic x0.1): {gyro_rw:.4e} rad/s^2/sqrt(Hz)")

    print(f"\n{'='*60}")
    print(f"RECOMMENDED VALUES FOR kalibr_imu_chain.yaml")
    print(f"{'='*60}")
    # Apply 2x safety margin on top of stationary estimates
    print(f"  (2x safety margin applied on noise_density for unmodelled effects)")
    print(f"  accelerometer_noise_density: {accel_nd*2:.4e}")
    print(f"  accelerometer_random_walk:   {accel_rw*2:.4e}")
    print(f"  gyroscope_noise_density:     {gyro_nd*2:.4e}")
    print(f"  gyroscope_random_walk:       {gyro_rw*2:.4e}")

    # --- Plots ---
    accel_norm = np.sqrt(ax**2 + ay**2 + az**2)
    gyro_norm  = np.sqrt(gx**2 + gy**2 + gz**2)

    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('IMU Data — Stationary Window Detection', fontsize=13)

    # Plot accel norm with stationary window highlighted
    axes[0].plot(times, accel_norm, 'b', lw=0.6, label='Accel norm')
    axes[0].axhline(9.81, color='g', ls='--', lw=1, label='Gravity (9.81)')
    axes[0].axvspan(times[s_start], times[s_end-1], alpha=0.2, color='green', label='Stationary window')
    axes[0].axvline(times[motion_idx], color='r', ls='--', lw=1.5, label=f'Motion start ({times[motion_idx]:.1f}s)')
    axes[0].set_ylabel('|accel| [m/s²]')
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # Plot gyro norm
    axes[1].plot(times, gyro_norm, 'r', lw=0.6, label='Gyro norm')
    axes[1].axvspan(times[s_start], times[s_end-1], alpha=0.2, color='green', label='Stationary window')
    axes[1].axvline(times[motion_idx], color='r', ls='--', lw=1.5)
    axes[1].set_ylabel('|gyro| [rad/s]')
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    # Plot stationary accel histogram
    axes[2].hist(ax_s - ax_s.mean(), bins=60, alpha=0.6, label='Accel-X (zero-mean)', density=True)
    axes[2].hist(gz_s - gz_s.mean(), bins=60, alpha=0.6, label='Gyro-Z (zero-mean, x100)', density=True)
    axes[2].set_xlabel('Value')
    axes[2].set_ylabel('Density')
    axes[2].set_title('Noise distribution in stationary window')
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    out = f"{RESULTS_DIR}/imu_noise_analysis.png"
    plt.savefig(out, dpi=120)
    print(f"\nPlot saved to: {out}")

    db.close()
    return accel_nd * 2, accel_rw * 2, gyro_nd * 2, gyro_rw * 2


if __name__ == '__main__':
    main()
