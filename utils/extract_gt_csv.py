#!/usr/bin/env python3
"""
extract_gt_csv.py — Convert /tbuggy/odom from a ROS2 bag to OpenVINS ASL ground truth CSV.

OpenVINS expects a 17-column CSV (EuRoC ASL format):
  timestamp_ns, p_x, p_y, p_z, q_w, q_x, q_y, q_z, v_x, v_y, v_z, bg_x, bg_y, bg_z, ba_x, ba_y, ba_z

/tbuggy/odom provides position + orientation + twist (linear velocity).
Biases are unknown from odom — set to zero (only affects RMSE printout, not VIO estimation).

Accepts either a bag directory (reads db3 paths from metadata.yaml) or a direct .db3 path.

Usage:
  python3 extract_gt_csv.py /data/log_01_ros2 utils/results/gt_log01.csv
  python3 extract_gt_csv.py /data/log_02_ros2 utils/results/gt_log02.csv
"""

import os
import sys
import sqlite3
import csv
import yaml
from rosbags.typesys import get_typestore, Stores


def resolve_db3_paths(input_path: str) -> list[str]:
    """Accept a bag directory or a direct .db3 file; return list of .db3 paths."""
    if input_path.endswith(".db3"):
        return [input_path]

    metadata_path = os.path.join(input_path, "metadata.yaml")
    if not os.path.exists(metadata_path):
        print(f"ERROR: {metadata_path} not found")
        sys.exit(1)

    with open(metadata_path) as f:
        meta = yaml.safe_load(f)

    rel_paths = meta["rosbag2_bagfile_information"]["relative_file_paths"]
    db3_paths = []
    for rel in rel_paths:
        full = os.path.join(input_path, rel)
        if not os.path.exists(full):
            print(f"ERROR: db3 file not found: {full}")
            sys.exit(1)
        if os.path.getsize(full) == 0:
            print(f"  Skipping empty file: {full}")
            continue
        db3_paths.append(full)

    if not db3_paths:
        print(f"ERROR: no valid db3 files found in {input_path}")
        sys.exit(1)

    return db3_paths

TOPIC_NAME = "/tbuggy/odom"
MSG_TYPE   = "nav_msgs/msg/Odometry"


def extract_gt(bag_input: str, out_csv: str):
    store = get_typestore(Stores.ROS2_HUMBLE)

    db3_paths = resolve_db3_paths(bag_input)

    rows = []
    for db3_path in db3_paths:
        con = sqlite3.connect(db3_path)
        cur = con.cursor()

        cur.execute("SELECT id FROM topics WHERE name = ?", (TOPIC_NAME,))
        row = cur.fetchone()
        if row is None:
            print(f"  No topic '{TOPIC_NAME}' in {db3_path} — skipping")
            con.close()
            continue
        topic_id = row[0]

        cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (topic_id,)
        )
        chunk = cur.fetchall()
        con.close()
        print(f"  {len(chunk)} odom messages in {os.path.basename(db3_path)}")
        rows.extend(chunk)

    if not rows:
        print(f"ERROR: no odom messages found")
        sys.exit(1)

    rows.sort(key=lambda r: r[0])
    print(f"Found {len(rows)} odom messages total")

    with open(out_csv, "w", newline="") as f:
        writer = csv.writer(f)
        # EuRoC ASL header (OpenVINS skips the first line)
        writer.writerow([
            "#timestamp [ns]",
            "p_x [m]", "p_y [m]", "p_z [m]",
            "q_w", "q_x", "q_y", "q_z",
            "v_x [m/s]", "v_y [m/s]", "v_z [m/s]",
            "bg_x", "bg_y", "bg_z",
            "ba_x", "ba_y", "ba_z",
        ])

        for timestamp_ns, raw in rows:
            msg = store.deserialize_cdr(raw, MSG_TYPE)

            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            pz = msg.pose.pose.position.z
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            # Linear velocity in body frame — ground vehicle mostly forward
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            vz = msg.twist.twist.linear.z

            writer.writerow([
                timestamp_ns,
                f"{px:.9f}", f"{py:.9f}", f"{pz:.9f}",
                f"{qw:.9f}", f"{qx:.9f}", f"{qy:.9f}", f"{qz:.9f}",
                f"{vx:.9f}", f"{vy:.9f}", f"{vz:.9f}",
                "0.0", "0.0", "0.0",  # gyro bias (unknown)
                "0.0", "0.0", "0.0",  # accel bias (unknown)
            ])

    print(f"Saved {len(rows)} GT poses → {out_csv}")
    print(f"\nFirst row timestamp (s): {rows[0][0] * 1e-9:.3f}")
    print(f"Last  row timestamp (s): {rows[-1][0] * 1e-9:.3f}")
    print(f"Duration: {(rows[-1][0] - rows[0][0]) * 1e-9:.1f} s")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    extract_gt(sys.argv[1], sys.argv[2])

