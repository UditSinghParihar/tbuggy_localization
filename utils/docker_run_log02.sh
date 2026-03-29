#!/bin/bash
# docker_run_log02.sh — Reproduce log_02 results inside the tbuggy_vio Docker container.
#
# Usage:
#   ./utils/docker_run_log02.sh /path/to/bags [output_dir]
#
# Example:
#   ./utils/docker_run_log02.sh /home/reviewer/data ./output
#
# Same config as log_01 (no re-tuning) — robustness check per assignment requirement.

set -e

IMAGE="uditsinghparihar/tbuggy_vio:latest"
BAG_DIR="${1:-}"
OUT_DIR="${2:-$(pwd)/output}"

if [ -z "$BAG_DIR" ]; then
    echo "Usage: $0 /path/to/bags [output_dir]"
    echo "  /path/to/bags must contain log_02_ros2/ subdirectory"
    exit 1
fi

BAG_PATH="$BAG_DIR/log_02_ros2"

if [ ! -d "$BAG_PATH" ]; then
    echo "ERROR: bag directory not found: $BAG_PATH"
    exit 1
fi

mkdir -p "$OUT_DIR"

echo "============================================================"
echo "  tbuggy VIO — log_02 Reproduction (robustness check)"
echo "  Image : $IMAGE"
echo "  Bags  : $BAG_DIR"
echo "  Output: $OUT_DIR"
echo "============================================================"
echo ""

# ── Step 1: Extract ground truth CSV ─────────────────────────────────────────
echo "[Step 1/4] Extracting ground truth CSV from bag..."
docker run --rm \
    -v "$BAG_DIR":/data \
    -v "$OUT_DIR":/colcon_ws/utils/results \
    "$IMAGE" \
    python3 utils/extract_gt_csv.py \
        /data/log_02_ros2/log_02_ros2_0.db3 \
        utils/results/gt_log02.csv
echo "  -> GT CSV saved to $OUT_DIR/gt_log02.csv"
echo ""

# ── Steps 2 & 3: Print commands for two terminals ────────────────────────────
echo "[Step 2/4] Open TERMINAL A and run this command to launch OpenVINS (with RViz):"
echo ""
echo "  # Allow Docker to access your X display (run once per session):"
echo "  xhost +local:docker"
echo ""
echo "  docker run --rm -it --network=host \\"
echo "    -e DISPLAY=\$DISPLAY \\"
echo "    -v /tmp/.X11-unix:/tmp/.X11-unix \\"
echo "    -v \"$BAG_DIR\":/data \\"
echo "    -v \"$OUT_DIR\":/colcon_ws/utils/results \\"
echo "    $IMAGE \\"
echo "    ros2 launch ov_msckf subscribe.launch.py \\"
echo "      config_path:=/colcon_ws/src/open_vins/config/tbuggy/estimator_config.yaml \\"
echo "      max_cameras:=1 use_stereo:=false save_total_state:=true \\"
echo "      filepath_est:=/colcon_ws/utils/results/ov_estimate_log02.txt \\"
echo "      filepath_std:=/colcon_ws/utils/results/ov_estimate_std_log02.txt \\"
echo "      path_gt:=/colcon_ws/utils/results/gt_log02.csv \\"
echo "      rviz_enable:=true"
echo ""
echo "[Step 3/4] Open TERMINAL B and run this command to play the bag:"
echo ""
echo "  docker run --rm -it --network=host \\"
echo "    -v \"$BAG_DIR\":/data \\"
echo "    $IMAGE \\"
echo "    ros2 bag play /data/log_02_ros2 --clock --rate 1.0"
echo ""
echo "  Wait for the bag to finish playing, then Ctrl+C OpenVINS in Terminal A."
echo ""

# ── Step 4: Evaluate (run after OpenVINS is done) ────────────────────────────
echo "[Step 4/4] Press ENTER when OpenVINS has finished to run evaluation..."
read -r

echo "Running evaluation and generating plots..."
docker run --rm \
    -v "$OUT_DIR":/colcon_ws/utils/results \
    "$IMAGE" \
    bash -c "
        set -e
        python3 utils/evaluate_trajectory.py \
            --est utils/results/ov_estimate_log02.txt \
            --gt  utils/results/gt_log02.csv \
            --out utils/results/ \
            --label log02
        python3 utils/plot_trajectory_comparison.py \
            --est utils/results/est_tum_log02.txt \
            --gt  utils/results/gt_tum_log02.txt \
            --out utils/results/ \
            --label log02
    "

echo ""
echo "============================================================"
echo "  Done. Results saved to: $OUT_DIR"
echo "  trajectory_comparison_log02.png"
echo "  ate_over_time_log02.png"
echo "  est_tum_log02.txt, gt_tum_log02.txt"
echo "============================================================"
