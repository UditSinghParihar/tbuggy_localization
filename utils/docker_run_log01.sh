#!/bin/bash
# docker_run_log01.sh — Reproduce log_01 results inside the tbuggy_vio Docker container.
#
# Usage (ROS1 bag file):
#   ./utils/docker_run_log01.sh /path/to/log_01.bag [output_dir]
#
# Usage (already-converted ROS2 bag directory):
#   ./utils/docker_run_log01.sh /path/to/bags [output_dir]
#
# Example:
#   ./utils/docker_run_log01.sh /home/reviewer/data/log_01.bag ./output
#
# If a ROS1 .bag file is provided the script converts it to ROS2 format
# automatically before running the pipeline.

set -e

IMAGE="uditsinghparihar/tbuggy_vio:latest"
INPUT="${1:-}"
OUT_DIR="$(realpath -m "${2:-$(pwd)/output}")"
# Resolve the utils/ directory next to this script so updated Python scripts
# are mounted into the container (no image rebuild needed after script changes).
UTILS_DIR="$(dirname "$(realpath "$0")")"

if [ -z "$INPUT" ]; then
    echo "Usage: $0 /path/to/log_01.bag [output_dir]"
    echo "       $0 /path/to/bags       [output_dir]"
    echo ""
    echo "  /path/to/log_01.bag — ROS1 bag file (auto-converted to ROS2 format)"
    echo "  /path/to/bags       — directory containing log_01_ros2/ subdirectory"
    exit 1
fi

# ── Detect input format ───────────────────────────────────────────────────────
if [[ "$INPUT" == *.bag ]]; then
    BAG_FILE="$(realpath "$INPUT")"
    BAG_DIR="$(dirname "$BAG_FILE")"
    BAG_NAME="$(basename "$BAG_FILE" .bag)"
    ROS2_BAG_DIR="$BAG_DIR/${BAG_NAME}_ros2"

    if [ ! -f "$BAG_FILE" ]; then
        echo "ERROR: ROS1 bag file not found: $BAG_FILE"
        exit 1
    fi

    if [ -d "$ROS2_BAG_DIR" ]; then
        echo "[Pre-step] ROS2 bag already exists at $ROS2_BAG_DIR — skipping conversion."
    else
        echo "[Pre-step] Converting ROS1 bag → ROS2 format..."
        echo "  Source : $BAG_FILE"
        echo "  Output : $ROS2_BAG_DIR"
        echo ""

        docker run --rm \
            -v "$BAG_DIR":/data \
            "$IMAGE" \
            rosbags-convert --src /data/${BAG_NAME}.bag --dst /data/${BAG_NAME}_ros2 &
        CONV_PID=$!

        echo -n "  Progress: "
        while kill -0 "$CONV_PID" 2>/dev/null; do
            if [ -d "$ROS2_BAG_DIR" ]; then
                SIZE=$(du -sh "$ROS2_BAG_DIR" 2>/dev/null | cut -f1)
                echo -ne "\r  Progress: ${SIZE} written   "
            else
                echo -ne "\r  Progress: starting...      "
            fi
            sleep 2
        done
        wait "$CONV_PID"
        echo -e "\r  Progress: done                "

        # Fix metadata.yaml inside Docker (directory is owned by root from conversion).
        # rosbags-convert writes offered_qos_profiles as YAML list [] but
        # ros2 bag play's yaml-cpp expects a string value.
        docker run --rm \
            -v "$BAG_DIR":/data \
            "$IMAGE" \
            sed -i "s/offered_qos_profiles: \[\]/offered_qos_profiles: ''/g" \
                /data/${BAG_NAME}_ros2/metadata.yaml

        echo "  -> ROS2 bag saved to $ROS2_BAG_DIR"
    fi

    # Patch in case bag was converted in a previous run without the fix.
    docker run --rm \
        -v "$BAG_DIR":/data \
        "$IMAGE" \
        sed -i "s/offered_qos_profiles: \[\]/offered_qos_profiles: ''/g" \
            /data/${BAG_NAME}_ros2/metadata.yaml 2>/dev/null || true
else
    BAG_DIR="$(realpath "$INPUT")"
fi

BAG_PATH="$BAG_DIR/log_01_ros2"

if [ ! -d "$BAG_PATH" ]; then
    echo "ERROR: bag directory not found: $BAG_PATH"
    exit 1
fi

mkdir -p "$OUT_DIR"

echo "============================================================"
echo "  tbuggy VIO — log_01 Reproduction"
echo "  Image : $IMAGE"
echo "  Bags  : $BAG_DIR"
echo "  Output: $OUT_DIR"
echo "============================================================"
echo ""

# ── Step 1: Extract ground truth CSV ─────────────────────────────────────────
echo "[Step 1/4] Extracting ground truth CSV from bag..."
docker run --rm \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v "$BAG_DIR":/data \
    -v "$UTILS_DIR":/colcon_ws/utils \
    -v "$OUT_DIR":/colcon_ws/utils/results \
    "$IMAGE" \
    python3 utils/extract_gt_csv.py \
        /data/log_01_ros2 \
        utils/results/gt_log01.csv
echo "  -> GT CSV saved to $OUT_DIR/gt_log01.csv"
echo ""

# ── Steps 2 & 3: Print commands for two terminals ────────────────────────────
echo "[Step 2/4] Open TERMINAL A and run this command to launch OpenVINS (with RViz):"
echo ""
echo "  # Allow Docker to access your X display (run once per session):"
echo "  xhost +local:docker"
echo ""
echo "  docker run --rm -it --network=host \\"
echo "    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \\"
echo "    -e ROS_DOMAIN_ID=0 \\"
echo "    -e DISPLAY=\$DISPLAY \\"
echo "    -v /tmp/.X11-unix:/tmp/.X11-unix \\"
echo "    -v \"$BAG_DIR\":/data \\"
echo "    -v \"$OUT_DIR\":/colcon_ws/utils/results \\"
echo "    $IMAGE \\"
echo "    ros2 launch ov_msckf subscribe.launch.py \\"
echo "      config_path:=/colcon_ws/src/open_vins/config/tbuggy/estimator_config.yaml \\"
echo "      max_cameras:=1 use_stereo:=false save_total_state:=true \\"
echo "      filepath_est:=/colcon_ws/utils/results/ov_estimate_log01.txt \\"
echo "      filepath_std:=/colcon_ws/utils/results/ov_estimate_std_log01.txt \\"
echo "      path_gt:=/colcon_ws/utils/results/gt_log01.csv \\"
echo "      rviz_enable:=true"
echo ""
echo "[Step 3/4] Open TERMINAL B and run this command to play the bag:"
echo ""
echo "  docker run --rm -it --network=host \\"
echo "    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \\"
echo "    -e ROS_DOMAIN_ID=0 \\"
echo "    -v \"$BAG_DIR\":/data \\"
echo "    $IMAGE \\"
echo "    ros2 bag play /data/log_01_ros2 --clock --rate 1.0"
echo ""
echo "  Wait for the bag to finish playing, then Ctrl+C OpenVINS in Terminal A."
echo ""

# ── Step 4: Evaluate (run after OpenVINS is done) ────────────────────────────
echo "[Step 4/4] Press ENTER when OpenVINS has finished to run evaluation..."
read -r

echo "Running evaluation and generating plots..."
docker run --rm \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v "$UTILS_DIR":/colcon_ws/utils \
    -v "$OUT_DIR":/colcon_ws/utils/results \
    "$IMAGE" \
    bash -c "
        set -e
        python3 utils/evaluate_trajectory.py \
            --est utils/results/ov_estimate_log01.txt \
            --gt  utils/results/gt_log01.csv \
            --out utils/results/ \
            --label log01
        python3 utils/plot_trajectory_comparison.py \
            --est utils/results/est_tum_log01.txt \
            --gt  utils/results/gt_tum_log01.txt \
            --out utils/results/ \
            --label log01
    "

echo ""
echo "============================================================"
echo "  Done. Results saved to: $OUT_DIR"
echo "  trajectory_comparison_log01.png"
echo "  ate_over_time_log01.png"
echo "  est_tum_log01.txt, gt_tum_log01.txt"
echo "============================================================"
