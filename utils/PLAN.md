# Plan: Visual Localization for tbuggy UGV using OpenVINS

## Context
Assignment requires building a VIO pipeline for a ground vehicle ("tbuggy") using ROS2 bag data. User has OpenVINS 2.7.0 already built in `colcon_ws_tii/`. Two bags exist at `/home/udit/data/` (log_01: ~550s, log_02: ~337s) with monocular camera (~30Hz), IMU (~100Hz), ground truth odometry, GPS, and TF data.

**Method choice**: Monocular VIO via OpenVINS (MSCKF) — justified by having camera + IMU, real-time capable, handles monocular scale via IMU, well-documented open-source.

---

## Step 1: Data Exploration Script
Create `/home/udit/codes/tii_assignment/scripts/data_exploration.py` using `rosbags` library (no ROS2 node needed).

**Extract from bag:**
- Camera intrinsics from `/tbuggy/camera_front/camera_info` (fx, fy, cx, cy, distortion, resolution)
- TF static transforms from `/tf_static` (find camera-to-IMU extrinsic)
- IMU statistics from `/tbuggy/imu_ins` (rate, accel/gyro noise estimates)
- Ground truth trajectory from `/tbuggy/odom` → save as TUM format
- Message frequency analysis per topic (detect frame drops)
- Extract ~5 sample images for visual inspection

**Generate plots:**
- 2D ground truth trajectory (x-y)
- IMU accel/gyro over time
- Message frequency histograms

**Dependencies:** `pip install rosbags matplotlib numpy`

---

## Step 2: Create OpenVINS Config
Create 3 files in `colcon_ws_tii/src/open_vins/config/tbuggy/`:

### `estimator_config.yaml`
- Base on `rs_d455` config (monocular)
- `max_cameras: 1`, `use_stereo: false`
- `init_dyn_use: true` (UGV may be moving at bag start)
- `try_zupt: true` (vehicle stops help constrain drift)
- `histogram_method: "CLAHE"` (outdoor lighting)
- `num_pts: 250`, `track_frequency: 30.0`
- `calib_cam_extrinsics: true`, `calib_cam_intrinsics: true` (online refinement)
- `feat_rep_slam: "ANCHORED_MSCKF_INVERSE_DEPTH"` (better for mono)
- `save_total_state: true` with output paths

### `kalibr_imu_chain.yaml`
- `rostopic: /tbuggy/imu_ins`
- `update_rate: 100.0`
- Start with EuRoC noise defaults, tune from data exploration results

### `kalibr_imucam_chain.yaml`
- `rostopic: /tbuggy/camera_front/image_raw`
- Fill intrinsics/distortion from Step 1's CameraInfo extraction
- Fill `T_imu_cam` from Step 1's TF static extraction
- `distortion_model: radtan` (ROS plumb_bob = radtan)

---

## Step 3: Run Pipeline
**Terminal 1 — OpenVINS:**
```bash
source install/setup.bash
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=.../config/tbuggy/estimator_config.yaml \
  max_cameras:=1 use_stereo:=false rviz_enable:=true
```

**Terminal 2 — Bag playback:**
```bash
ros2 bag play /home/udit/data/log_01_ros2 --clock --rate 0.5
```

Start at half speed; increase if OpenVINS keeps up.

---

## Step 4: Evaluation
Create `/home/udit/codes/tii_assignment/scripts/evaluate_trajectory.py`

**KPIs:**
1. **ATE (Absolute Trajectory Error)** — overall accuracy after Sim(3) alignment (mono needs scale correction)
2. **RPE (Relative Pose Error)** — drift rate over fixed-length segments
3. **Scale error** — scale factor from Sim(3) alignment (ideal = 1.0)
4. **Processing time** — must be <33ms/frame for real-time at 30Hz
5. **Initialization time** — time to first valid pose

**Tools:** `evo` library (`evo_ape tum ... -vas` for scale-corrected evaluation)

Run on log_01 (tune), then log_02 (robustness check, no re-tuning).

---

## Step 5: Dockerfile
Base on `osrf/ros:humble-desktop`, install deps, clone OpenVINS, copy config + scripts, build workspace. Mount bag data as volume at runtime.

---

## Step 6: Documentation
Technical report covering: data analysis, method justification, implementation details, KPI results with plots, failure analysis, future improvements.

---

## Execution Order
1. **Data exploration script** → extract intrinsics, extrinsics, plot ground truth
2. **Create config YAMLs** using extracted values
3. **First test run** on log_01 (debug/tune)
4. **Full run + evaluation** on log_01
5. **Robustness run** on log_02
6. **Dockerfile + report**

## Key Files to Modify/Create
- `scripts/data_exploration.py` (new)
- `config/tbuggy/estimator_config.yaml` (new)
- `config/tbuggy/kalibr_imu_chain.yaml` (new)
- `config/tbuggy/kalibr_imucam_chain.yaml` (new)
- `scripts/evaluate_trajectory.py` (new)
- `Dockerfile` (new)
- `report/report.md` (new)

## Verification
- Run data_exploration.py → check plots and extracted values make sense
- Run OpenVINS on log_01 → verify initialization, check rviz for tracked features
- Run evo_ape/evo_rpe → check KPIs are reasonable (ATE < few meters, drift < 5%)
- Run on log_02 without re-tuning → compare KPIs
