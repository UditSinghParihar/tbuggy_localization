#!/usr/bin/env python3
"""
plot_trajectory_comparison.py — Align VIO estimate to GT via Umeyama and plot comparison.

Usage:
  python3 plot_trajectory_comparison.py \
    --est /home/udit/codes/tii_assignment/colcon_ws_tii/utils/results/est_tum.txt \
    --gt  /home/udit/codes/tii_assignment/colcon_ws_tii/utils/results/gt_tum.txt \
    --out /home/udit/codes/tii_assignment/colcon_ws_tii/utils/results/
"""

import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os


def load_tum(path):
    data = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            cols = line.split()
            ts = float(cols[0])
            px, py, pz = float(cols[1]), float(cols[2]), float(cols[3])
            data.append([ts, px, py, pz])
    return np.array(data)  # (N, 4): [ts, x, y, z]


def associate(est, gt, max_diff=0.01):
    """Match est to gt by closest timestamp."""
    matched_est, matched_gt = [], []
    for row in est:
        ts = row[0]
        diffs = np.abs(gt[:, 0] - ts)
        idx = np.argmin(diffs)
        if diffs[idx] < max_diff:
            matched_est.append(row[1:4])
            matched_gt.append(gt[idx, 1:4])
    return np.array(matched_est), np.array(matched_gt)


def umeyama_alignment(src, dst):
    """
    Umeyama similarity transform: dst ≈ s * R @ src + t
    Returns: s (scale), R (3x3), t (3,)
    """
    n = src.shape[0]
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst
    var_src = (src_c ** 2).sum() / n
    H = (src_c.T @ dst_c) / n
    U, D, Vt = np.linalg.svd(H)
    det = np.linalg.det(Vt.T @ U.T)
    S = np.diag([1, 1, np.sign(det)])
    R = Vt.T @ S @ U.T
    s = (D * S.diagonal()).sum() / var_src
    t = mu_dst - s * R @ mu_src
    return s, R, t


def compute_ate(est_aligned, gt_matched):
    errors = np.linalg.norm(est_aligned - gt_matched, axis=1)
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--est", required=True)
    parser.add_argument("--gt",  required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--label", default="", help="Label appended to output filenames, e.g. log01 or log02")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    print("Loading trajectories...")
    est = load_tum(args.est)
    gt  = load_tum(args.gt)
    print(f"  EST: {len(est)} poses   GT: {len(gt)} poses")

    print("Associating timestamps (max 10ms diff)...")
    est_matched, gt_matched = associate(est, gt)
    print(f"  Matched: {len(est_matched)} pairs")

    print("Umeyama alignment (Sim3: scale + rotation + translation)...")
    s, R, t = umeyama_alignment(est_matched, gt_matched)
    print(f"  Scale correction: {s:.4f}")
    print(f"  Rotation:\n{R}")
    print(f"  Translation: {t}")

    est_aligned = (s * (R @ est_matched.T)).T + t

    # ATE
    ate_errors = compute_ate(est_aligned, gt_matched)
    print(f"\n--- ATE (after Sim3 alignment) ---")
    print(f"  RMSE:   {np.sqrt((ate_errors**2).mean()):.3f} m")
    print(f"  Mean:   {ate_errors.mean():.3f} m")
    print(f"  Median: {np.median(ate_errors):.3f} m")
    print(f"  Max:    {ate_errors.max():.3f} m")
    print(f"  Min:    {ate_errors.min():.3f} m")

    # Path lengths
    def path_length(pts):
        return np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()
    est_len = path_length(est_matched)
    gt_len  = path_length(gt_matched)
    print(f"\n  Path length — EST: {est_len:.1f}m,  GT: {gt_len:.1f}m")
    print(f"  Scale ratio (before correction): {gt_len/est_len:.4f}")

    # ---- Plot 1: Trajectory overlay (XY top-down) ----
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(gt_matched[:, 0],     gt_matched[:, 1],     "b-",  lw=1.5, label="GT (/tbuggy/odom)")
    ax.plot(est_aligned[:, 0],    est_aligned[:, 1],    "r--", lw=1.5, label=f"OpenVINS (scale={s:.3f})")
    ax.scatter(gt_matched[0, 0],  gt_matched[0, 1],     c="g", s=80, zorder=5, label="Start")
    ax.scatter(gt_matched[-1, 0], gt_matched[-1, 1],    c="k", s=80, zorder=5, label="End")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title(f"Trajectory Comparison (Sim3 aligned)\nATE RMSE={np.sqrt((ate_errors**2).mean()):.2f}m  Scale={s:.3f}")
    ax.legend()
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    suffix = f"_{args.label}" if args.label else ""
    out_traj = os.path.join(args.out, f"trajectory_comparison{suffix}.png")
    fig.savefig(out_traj, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"\nSaved: {out_traj}")

    # ---- Plot 2: ATE error over time ----
    fig, ax = plt.subplots(figsize=(10, 4))
    timestamps = est[: len(est_matched), 0] - est[0, 0]  # relative time
    # recompute timestamps relative
    est_full = load_tum(args.est)
    ts_rel = est_full[:len(ate_errors), 0] - est_full[0, 0]
    ax.plot(ts_rel, ate_errors, "r-", lw=1, alpha=0.7)
    ax.axhline(ate_errors.mean(), color="b", linestyle="--", label=f"Mean={ate_errors.mean():.2f}m")
    ax.fill_between(ts_rel, 0, ate_errors, alpha=0.2, color="r")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("ATE [m]")
    ax.set_title("Absolute Trajectory Error over Time")
    ax.legend()
    ax.grid(True, alpha=0.3)
    out_ate = os.path.join(args.out, f"ate_over_time{suffix}.png")
    fig.savefig(out_ate, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_ate}")

    print(f"\nDone. Results in {args.out}")


if __name__ == "__main__":
    main()
