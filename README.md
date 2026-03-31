# tbuggy Visual Localization — OpenVINS Monocular VIO

Monocular Visual-Inertial Odometry pipeline for the **tbuggy** outdoor desert UGV using [OpenVINS 2.7](https://github.com/rpng/open_vins) on ROS2 Humble.

Results, evaluation plots, and full methodology are documented in [`utils/documentation.md`](utils/documentation.md).

---

## Feature Tracking — Live Demo

### log\_02: VIO initialising at hangar (rich near-field texture)

![log02 start — feature tracks at hangar](utils/results/log02_start_features.gif)

### log\_02: Feature tracks during U-turn in the desert

![log02 U-turn — feature tracks during turn](utils/results/log02_turn_features.gif)

> Green dots = KLT-tracked MSCKF features. The hangar provides dense, stable tracks that
> anchor scale and heading. Feature count drops during the featureless desert section,
> causing the scale drift visible in the quantitative results.

---

## Quickstart — Docker (Recommended)

The easiest way to reproduce results. Requires only Docker + the bag files.

```bash
# Pull the pre-built image (~3-4 GB download)
docker pull uditsinghparihar/tbuggy_vio:latest

# Clone this repo to get the helper scripts
git clone https://github.com/UditSinghParihar/tbuggy_localization.git colcon_ws_tii
cd colcon_ws_tii
chmod +x utils/docker_run_log01.sh utils/docker_run_log02.sh

# Run log_01 — pass the ROS1 .bag file directly (auto-converted inside Docker)
./utils/docker_run_log01.sh /path/to/log_01.bag ./output

# Run log_02
./utils/docker_run_log02.sh /path/to/log_02.bag ./output
```

The scripts guide you through each step interactively. Results are saved to `./output/` on your host.

> **ROS1 bags:** If you have the original `.bag` files, pass them directly — the script converts them to ROS2 format automatically using `rosbags-convert` inside the container. The converted directory (e.g. `log_01_ros2/`) is saved next to the `.bag` file and reused on subsequent runs.

> **ROS2 bags:** You can also pass a directory containing `log_01_ros2/` / `log_02_ros2/` subdirectories: `./utils/docker_run_log01.sh /path/to/bags ./output`

> **RViz (Linux only):** Before running the OpenVINS launch command printed in Step 2, run `xhost +local:docker` once in your terminal to allow the container to open a display window.

---

## Manual Setup (without Docker)

- Ubuntu 22.04
- ROS2 Humble
- OpenCV, Eigen3, Ceres (standard ROS2 desktop + VIO dependencies)

```bash
sudo apt install ros-humble-desktop libceres-dev \
  libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev
pip install rosbags evo matplotlib numpy
```

---

## Clone and Build

```bash
# Clone with submodule
git clone --recurse-submodules https://github.com/UditSinghParihar/tbuggy_localization.git colcon_ws_tii
cd colcon_ws_tii

# If already cloned without --recurse-submodules
git submodule update --init --recursive

# Build (memory-safe — use -j4 if your system allows)
source /opt/ros/humble/setup.bash
MAKEFLAGS="-j2" colcon build --symlink-install \
  --executor sequential \
  --parallel-workers 1
```

---

## Bag Data

Place ROS2 bag files at:
```
/home/udit/data/log_01_ros2/
/home/udit/data/log_02_ros2/
```

| Sequence | Duration | Description |
|----------|----------|-------------|
| log_01   | 549.8 s  | Desert straight run, sparse features |
| log_02   | 336.7 s  | Oval loop near hangar, richer features |

---

## Run OpenVINS

### Step 1 — Generate ground truth CSV (one-time per bag)

```bash
python3 utils/extract_gt_csv.py \
  /home/udit/data/log_01_ros2/log_01_ros2_0.db3 \
  utils/results/gt_log01.csv
```

### Step 2 — Terminal 1: Launch OpenVINS

```bash
source install/setup.bash
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=$(pwd)/src/open_vins/config/tbuggy/estimator_config.yaml \
  max_cameras:=1 \
  use_stereo:=false \
  save_total_state:=true \
  filepath_est:=$(pwd)/utils/results/ov_estimate_log01.txt \
  filepath_std:=$(pwd)/utils/results/ov_estimate_std_log01.txt \
  path_gt:=$(pwd)/utils/results/gt_log01.csv \
  rviz_enable:=true
```

### Step 3 — Terminal 2: Play bag

```bash
ros2 bag play /home/udit/data/log_01_ros2 --clock --rate 1.0
```

For log_02, replace `log01` → `log02` in `filepath_est`, `path_gt`, and the bag path.

---

## Evaluate Trajectory

After the run completes:

```bash
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
```

For log_02 replace `log01` → `log02` throughout.

---

## Config Files

Located in `src/open_vins/config/tbuggy/`:

| File | Purpose |
|------|---------|
| `estimator_config.yaml` | Main VIO estimator settings (monocular, ZUPT, CLAHE, dynamic init) |
| `kalibr_imu_chain.yaml` | IMU noise parameters derived from stationary window in log_01 |
| `kalibr_imucam_chain.yaml` | Camera intrinsics + camera-IMU extrinsic (T_imu_cam from TF chain) |

---

## Utility Scripts

Located in `utils/`:

| Script | Purpose |
|--------|---------|
| `extract_tf_and_imu.py` | Compute T_imu_cam from `/tf_static` TF chain in the bag |
| `imu_noise_analysis.py` | Detect stationary window and compute IMU noise parameters |
| `extract_gt_csv.py` | Convert `/tbuggy/odom` to EuRoC ASL ground truth CSV |
| `evaluate_trajectory.py` | Convert outputs to TUM format and run `evo` ATE/RPE |
| `plot_trajectory_comparison.py` | Sim3 Umeyama alignment and trajectory comparison plots |
