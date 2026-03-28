# tbuggy Visual Localization — OpenVINS Monocular VIO

Monocular Visual-Inertial Odometry pipeline for the **tbuggy** outdoor desert UGV using [OpenVINS 2.7](https://github.com/rpng/open_vins) on ROS2 Humble.

Results, evaluation plots, and full methodology are documented in [`utils/documentation.md`](utils/documentation.md).

---

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- OpenCV, Eigen3, Boost (standard ROS2 desktop dependencies)

```bash
sudo apt install ros-humble-desktop ros-humble-rviz2 \
  libopencv-dev libeigen3-dev
pip install rosbags evo
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
ros2 bag play /home/udit/data/log_01_ros2 --clock --rate 0.5
```

For log_02, replace `log01` → `log02` in `filepath_est`, `path_gt`, and the bag path.

---

## Evaluate Trajectory

After the run completes:

```bash
python3 utils/evaluate_trajectory.py \
  --est utils/results/ov_estimate_log01.txt \
  --gt  utils/results/gt_log01.csv \
  --out utils/results/

python3 utils/plot_trajectory_comparison.py \
  --est utils/results/est_tum.txt \
  --gt  utils/results/gt_tum.txt \
  --out utils/results/
```

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
