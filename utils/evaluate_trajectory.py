#!/usr/bin/env python3
"""
evaluate_trajectory.py — Convert OpenVINS output + GT to TUM format and run evo evaluation.

OpenVINS estimate format (filepath_est):
  timestamp(s) qx qy qz qw px py pz vx vy vz ... (many more cols)

GT ASL CSV format (from extract_gt_csv.py):
  timestamp_ns, px, py, pz, qw, qx, qy, qz, vx, vy, vz, ...

TUM format (for evo):
  timestamp tx ty tz qx qy qz qw

Usage:
  python3 evaluate_trajectory.py \
    --est  /home/udit/codes/tii_assignment/colcon_ws_tii/utils/results/ov_estimate_log01.txt \
    --gt   /home/udit/codes/tii_assignment/colcon_ws_tii/utils/results/gt_log01.csv \
    --out  /home/udit/codes/tii_assignment/colcon_ws_tii/utils/results/
"""

import argparse
import subprocess
import sys
import os


def convert_est_to_tum(est_path: str, tum_path: str):
    """Convert OpenVINS estimate file to TUM trajectory format."""
    rows = []
    with open(est_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            cols = line.split()
            # cols: timestamp qx qy qz qw px py pz ...
            ts   = cols[0]
            qx, qy, qz, qw = cols[1], cols[2], cols[3], cols[4]
            px, py, pz      = cols[5], cols[6], cols[7]
            rows.append(f"{ts} {px} {py} {pz} {qx} {qy} {qz} {qw}")
    with open(tum_path, "w") as f:
        f.write("\n".join(rows) + "\n")
    print(f"  EST → TUM: {len(rows)} poses → {tum_path}")
    return len(rows)


def convert_gt_to_tum(gt_csv_path: str, tum_path: str):
    """Convert ASL GT CSV to TUM trajectory format."""
    rows = []
    with open(gt_csv_path) as f:
        next(f)  # skip header
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            cols = line.split(",")
            # cols: timestamp_ns, px, py, pz, qw, qx, qy, qz, ...
            ts_s = float(cols[0]) * 1e-9   # ns → seconds
            px, py, pz = cols[1], cols[2], cols[3]
            qw, qx, qy, qz = cols[4], cols[5], cols[6], cols[7]
            rows.append(f"{ts_s:.9f} {px} {py} {pz} {qx} {qy} {qz} {qw}")
    with open(tum_path, "w") as f:
        f.write("\n".join(rows) + "\n")
    print(f"  GT  → TUM: {len(rows)} poses → {tum_path}")
    return len(rows)


def run_evo(est_tum: str, gt_tum: str):
    """Run evo_ape and evo_rpe with SE3 alignment."""
    print("\n--- ATE (Absolute Trajectory Error) ---")
    subprocess.run([
        "evo_ape", "tum", gt_tum, est_tum,
        "--align",          # SE3 alignment (rotation + translation)
        "--correct_scale",  # monocular VIO has unknown scale
        "--verbose",
    ], check=False)

    print("\n--- RPE (Relative Pose Error, segment 10m) ---")
    subprocess.run([
        "evo_rpe", "tum", gt_tum, est_tum,
        "--align",
        "--correct_scale",
        "--delta", "10",
        "--delta_unit", "m",
        "--verbose",
    ], check=False)

    print("\n--- Trajectory overlay (aligned + scale corrected) ---")
    subprocess.run([
        "evo_traj", "tum", est_tum,
        "--ref", gt_tum,
        "--align",
        "--correct_scale",
    ], check=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--est", required=True, help="OpenVINS estimate txt file")
    parser.add_argument("--gt",  required=True, help="GT ASL CSV file (from extract_gt_csv.py)")
    parser.add_argument("--out", required=True, help="Output directory for TUM files and plots")
    parser.add_argument("--label", default="", help="Label appended to TUM filenames, e.g. log01 or log02")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    suffix = f"_{args.label}" if args.label else ""
    est_tum = os.path.join(args.out, f"est_tum{suffix}.txt")
    gt_tum  = os.path.join(args.out, f"gt_tum{suffix}.txt")

    print("Converting to TUM format...")
    n_est = convert_est_to_tum(args.est, est_tum)
    n_gt  = convert_gt_to_tum(args.gt,  gt_tum)
    print(f"  Estimate: {n_est} poses,  GT: {n_gt} poses")

    if n_est < 10:
        print("ERROR: Too few estimate poses — run OpenVINS first.")
        sys.exit(1)

    run_evo(est_tum, gt_tum)


if __name__ == "__main__":
    main()
