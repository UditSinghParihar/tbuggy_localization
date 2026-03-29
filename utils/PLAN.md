# VIO Tuning Plan — tbuggy Desert Dataset

## 1. Dataset Analysis

### Visual Environment

| Sequence | Scene | Features | Challenge |
|----------|-------|----------|-----------|
| log_01 | Open flat desert | Extremely sparse — sand, scattered shrubs, horizon | Almost no trackable corners; KLT degrades to sky/ground |
| log_02 | Desert + hangar/building | Moderate — building facade, vehicles, edge lines | Rich near-field, featureless far-field after turn |

**log_01 camera view:** sand-coloured flat terrain, scattered low vegetation, hazy sky, horizon at ~40% image height. The only consistent features are the horizon line, a sensor post (foreground), and sparse desert plants. This is close to the worst-case scenario for feature-based VIO.

**log_02 camera view:** starts in front of a metal hangar with strong straight edges, red door frames, vehicles — good feature density. After the loop turn, the scene transitions to open desert. This explains why log_02 should perform significantly better despite being shorter.

### Motion Profile

| Sequence | Duration | Path | Shape | Key challenge |
|----------|----------|------|-------|--------------|
| log_01 | 549.8s | ~1059m | Roughly linear, wavy lateral undulations | No loop closure; drift accumulates unbounded |
| log_02 | 336.7s | ~500m (est.) | Closed oval loop | Near-loop closure — drift can be measured from start-end gap |

### Sensor Configuration
- **Camera:** 1920×1080 @ 28Hz (not 30Hz — actual field rate). Wide-angle, forward-facing.
- **IMU:** 100Hz, high-grade GNSS/INS unit. High accel-Z std (0.37 m/s²) from engine idle vibration.
- **Extrinsic offset:** Camera is 1.65m above and 1.14m forward of IMU — large lever arm; extrinsic calibration is critical.

---

## 2. Current Performance (Baseline — log_01 partial run)

| Metric | Value | Notes |
|--------|-------|-------|
| Frames tracked | 7,927 / 16,473 (48%) | Filter lost tracking in final ~24s of 549s bag |
| ATE RMSE | **56.73m** | Over ~1072m path = **5.3%** |
| Mean ATE | 50.39m | |
| Max ATE | 121.5m | End of trajectory — accumulated drift |
| RPE mean | 3.71m / 10m | **37.1% relative error** |
| RPE RMSE | 4.56m / 10m | |
| Sim3 scale | **1.363** | 36.3% scale error — major issue |

### ATE Pattern Interpretation (from ate_over_time.png)
- **t=0–80s:** High ATE (~80m) dropping to ~3m → filter initialising and converging
- **t=80–130s:** Good tracking, low error (~3–45m)
- **t=130–230s:** ATE rises to ~72m → featureless desert section, scale/heading error accumulates
- **t=230–310s:** Good tracking again, drops to ~15m → feature-rich segment or partial loop
- **t=310–430s:** Rises to ~120m → filter diverges near end, likely total feature loss

**Root causes:**
1. **Scale = 1.363** — Featureless desert → few long feature tracks → poor scale observability in monocular VIO. Scale degrades further during featureless segments.
2. **Oscillating ATE** — Filter cycles between good tracking (near features / ZUPT) and divergence (open desert). Not a single monotonic drift — suggests intermittent tracking failure.
3. **Initialization lag** — First 80s have high ATE, suggesting slow convergence. `init_dyn_use:true` helps but may need tighter conditions.

---

## 3. State-of-the-Art VIO KPIs

### Published Benchmarks

| Dataset | Conditions | Top method ATE | OpenVINS ATE |
|---------|-----------|---------------|-------------|
| EuRoC MH01 (indoor, good features) | Rich texture, slow motion | 0.03m (ORB-SLAM3) | ~0.15m |
| EuRoC V101 (indoor, fast) | Rich texture, aggressive motion | 0.08m | ~0.25m |
| KITTI (outdoor road) | Road markings, buildings | 0.5–1% path | 1–2% path |
| TartanAir (challenging synthetic) | Varied, some featureless | 2–10% path | 4–15% path |
| **Desert UGV (no standard benchmark)** | Featureless, vibration | — | — |

### Realistic Targets for This Dataset

| Grade | ATE RMSE | % of path | Achievability |
|-------|----------|-----------|--------------|
| Excellent | < 20m | < 2% | Hard — requires near-perfect tuning + good initialization |
| Good | 20–35m | 2–3.5% | Achievable with proposed tuning |
| Acceptable | 35–55m | 3.5–5% | Current baseline is at 5.3%, just below acceptable |
| Poor | > 55m | > 5% | Current baseline |

**Key constraint:** Monocular VIO cannot recover absolute scale. The Sim3 Umeyama alignment corrects for scale in evaluation, but a 36% scale error means the filter's internal metric estimates are severely wrong, leading to poor velocity/position propagation between visual updates. Reducing scale error is the single highest-impact improvement.

**Note on RPE:** RPE of 3.71m/10m (37%) is very high. For comparison, good outdoor VIO achieves 0.5–2% RPE. The desert scene + scale error explains this. Target: < 2m/10m (< 20%).

---

## 4. Tuning Strategy — Ordered by Expected Impact

### Group A: Feature Extraction (Highest Impact)

The root cause of most issues is insufficient feature density in featureless terrain. More features = longer tracks = better scale and heading observability.

| Parameter | Current | Proposed | Reason |
|-----------|---------|----------|--------|
| `num_pts` | 300 | **500** | More tracked points; KLT handles more in flat desert |
| `fast_threshold` | 20 | **10** | Lower threshold detects weaker corners (sand, horizon edges) |
| `min_px_dist` | 15 | **8** | Allows denser packing — critical when features are sparse |
| `grid_x` / `grid_y` | 8 / 5 | **10 / 7** | Finer grid ensures spatial spread across sky/sand regions |
| `downsample_cameras` | true (960×540) | **false (1920×1080)** | Full resolution reveals subtle texture; at 28Hz the compute cost is manageable |
| `knn_ratio` | 0.70 | **0.75** | Slightly more permissive match ratio in low-texture |

**Expected effect:** More features tracked per frame, longer track lifetimes, better triangulation geometry → reduced scale error, fewer tracking gaps.

---

### Group B: SLAM Features (Critical for Monocular Scale)

SLAM features are long-lived landmarks that provide persistent scale constraints. The current `max_slam=50` is too low for a featureless scene where scale is hard to observe.

| Parameter | Current | Proposed | Reason |
|-----------|---------|----------|--------|
| `max_slam` | 50 | **150** | 3× more SLAM features → far stronger scale constraint |
| `max_slam_in_update` | 25 | **75** | Use all available SLAM features each update step |
| `dt_slam_delay` | 2 | **1** | Initialize SLAM features 1s earlier → scale observable sooner |
| `max_msckf_in_update` | 40 | **60** | More MSCKF measurements per update |

**Expected effect:** Scale error reduced from 1.363 toward 1.0–1.2. This is the most direct fix for the scale issue.

---

### Group C: Filter Window (Better Triangulation Geometry)

A longer clone window gives more viewpoints for triangulation, which is essential for scale observability, especially when the vehicle moves slowly or straight.

| Parameter | Current | Proposed | Reason |
|-----------|---------|----------|--------|
| `max_clones` | 11 | **15** | Longer history → better geometry for depth estimation |

**Expected effect:** Improved feature depth estimation, lower scale error. Cost: ~30% more memory/compute per update — acceptable at 28Hz.

---

### Group D: IMU Noise Model

Currently using 1× measured values (stationary window). For a moving desert UGV:

| Parameter | Current (1×) | Proposed (2×) | Reason |
|-----------|-------------|---------------|--------|
| `accelerometer_noise_density` | 3.7e-2 | **7.4e-2** | Engine + terrain vibration doubles during motion |
| `accelerometer_random_walk` | 4.0e-3 | **7.4e-3** | Proportional to noise_density |
| `gyroscope_noise_density` | 8.5e-4 | **1.7e-3** | Safety margin for gyro drift in featureless sections |
| `gyroscope_random_walk` | 8.5e-5 | **1.7e-4** | Proportional to noise_density |

**Why 2× in this specific scene:** In featureless segments, the filter relies on IMU-only propagation for extended periods. Under-estimated noise → filter becomes over-confident → bad visual update (from low-texture frame) → catastrophic divergence. The 2× margin keeps the uncertainty ellipsoid realistic during dead-reckoning gaps.

**Revert to 1× if:** System initializes but loses tracking within 10–20s (over-inflation can slow convergence).

---

### Group E: Pixel Noise / Update Gating

In featureless scenes, KLT tracks are noisier (low gradient → sub-pixel uncertainty is larger). Tightening the measurement model to account for this prevents bad updates from corrupting the filter state.

| Parameter | Current | Proposed | Reason |
|-----------|---------|----------|--------|
| `up_msckf_sigma_px` | 1.0 | **1.5** | KLT on sand/sky has ~1.5px reprojection noise |
| `up_slam_sigma_px` | 1.0 | **1.5** | Same for SLAM features |
| `up_msckf_chi2_multipler` | 1 | **1** | Keep; chi2 gating is already active |

---

### Group F: Gravity and Initialization

| Parameter | Current | Proposed | Reason |
|-----------|---------|----------|--------|
| `gravity_mag` | 9.81 | **9.82** | Measured gravity norm from bag = 9.82 m/s² |
| `init_dyn_min_deg` | 5.0 | **3.0** | Allow initialization with smaller rotation (UGV drives straight) |
| `init_window_time` | 2.0 | **1.5** | Faster initialization; UGV may have clear motion from t=0 |

---

### Group G: ZUPT Tuning

Zero-velocity updates are critical on a UGV — they constrain bias drift during stops. The desert UGV likely stops briefly at turns.

| Parameter | Current | Proposed | Reason |
|-----------|---------|----------|--------|
| `zupt_max_velocity` | 0.5 | **0.3** | Tighter threshold → only genuine stops trigger ZUPT |
| `zupt_max_disparity` | 0.5 | **1.5** | More permissive disparity; bumpy terrain causes apparent motion during stops |
| `zupt_noise_multiplier` | 10 | **10** | Keep — inflates measurement noise during ZUPT, softening the constraint |

---

## 4H. Feature Detection and Tracking — Deep Dive

### What OpenVINS Uses

OpenVINS uses a two-stage pipeline: **FAST detection** followed by **KLT optical flow tracking**.

#### FAST (Features from Accelerated Segment Test)

FAST tests a ring of 16 pixels around each candidate pixel `p`. A corner is declared if there exist `N` (typically 9) contiguous pixels on the ring that are all brighter or all darker than `I(p) ± threshold`:

```
corner(p) = 1  if  ∃ contiguous arc S ⊆ {1..16} : |S| ≥ 9
                    and ∀ x ∈ S : I(x) > I(p) + t   (bright arc)
                    or  ∀ x ∈ S : I(x) < I(p) - t   (dark arc)
```

**Why FAST fails in desert:** Sand and sky have very low spatial gradients. The intensity differences `I(x) - I(p)` around a candidate on a sand patch are only 2–8 intensity units, far below typical threshold `t=20`. Result: almost zero corners detected in the sandy lower half of the image. CLAHE pre-processing helps by stretching local contrast, but cannot create structure that isn't there.

#### KLT (Kanade-Lucas-Tomasi) Optical Flow

KLT tracks a feature from frame `I_1` to `I_2` by minimizing the photometric error over a patch W around point `x`:

```
E(d) = Σ_{x∈W} [I_2(x + d) - I_1(x)]²
```

Solved iteratively via the Lucas-Kanade equation:

```
A^T A · d = A^T b
where A = [∂I/∂x  ∂I/∂y]  (image Jacobian / structure tensor)
      b = [I_1(x) - I_2(x)]  (temporal difference)
```

The solution `d = (A^T A)^{-1} A^T b` fails when `A^T A` is near-singular — exactly the case in sand/sky where gradients `∂I/∂x, ∂I/∂y ≈ 0`. This makes `A^T A ≈ 0` → the aperture problem → no unique optical flow solution.

**`knn_ratio`** in OpenVINS controls the Lowe ratio test for descriptor matching (when re-detecting lost features): a match is accepted if `d_best / d_second_best < ratio`. Higher ratio = more permissive = keeps more matches in low-texture.

---

### Why FAST+KLT Is Suboptimal for Desert

| Issue | Root cause | Effect |
|-------|-----------|--------|
| Low corner count | Sand gradient ≈ 0 | Few detected features, large gaps in image |
| Aperture problem | Uniform texture | KLT diverges or produces noisy flow vectors |
| Track lifetime | Bad KLT in homogeneous regions | Short tracks → poor triangulation geometry |
| Scale observability | Needs parallax across frames | Short tracks + nearly-straight motion → degenerate geometry |

---

### Better Alternatives for Desert

#### 1. Shi-Tomasi / GFTT (Good Features to Track)
Detects corners by the minimum eigenvalue of the structure tensor `A^T A`:
```
λ_min(A^T A) > threshold
```
Slightly better than FAST in low-texture because it ranks corners by trackability directly, not by raw intensity difference. Already available in OpenVINS via `use_klt: false` (ORB mode uses GFTT internally).

**Desert gain:** ~10–15% more features found at low threshold. Still fails in pure sand.

#### 2. ORB (Oriented FAST + Rotated BRIEF)
Combines FAST detection with a 256-bit binary descriptor. Descriptor matching is used to recover lost tracks across large displacements. However, the descriptor itself is based on pixel pair comparisons — in a featureless sand patch, the descriptor is essentially random and non-repeatable.

**Desert gain:** Better re-detection after tracking failure near the vehicle (rocks, sensor posts). Poor in open sand.

#### 3. SIFT / SURF (Scale-Invariant Feature Transform)
Uses Difference-of-Gaussian blob detection, invariant to scale and rotation. Finds "blobs" (local extrema in scale space):
```
D(x, σ) = G(x, kσ) * I - G(x, σ) * I
extremum if D(x,σ) > all 26 neighbours in (x,y,σ) space
```
Better than FAST in outdoor scenes with subtle texture, but 10–50× slower. Not real-time for 1920×1080 @ 28Hz without GPU.

#### 4. SuperPoint (Learned Features) — **Best for Desert**
A self-supervised deep network that outputs keypoint heatmaps and 256-D descriptors. Trained to detect repeatable interest points in homographic pairs. Crucially, it was trained on diverse scenes including low-texture environments and learns to detect structure from statistical patterns invisible to hand-crafted detectors.

```
F_encoder → {keypoint heatmap H(u,v), descriptor map D(u,v)}
H(u,v) ∈ [0,1]: probability of keypoint at pixel (u,v)
D(u,v) ∈ R^256: normalized descriptor
```

**Desert gain:** Empirically 3–5× more repeatable detections in featureless outdoor terrain vs FAST. Runs at ~30Hz on GPU.

#### 5. SuperGlue / LightGlue (Learned Matcher)
Graph neural network that matches SuperPoint descriptors using attention over all candidate pairs simultaneously. Handles partial visibility, large viewpoint changes, and low-texture scenes far better than nearest-neighbour matching.

**Practical note for OpenVINS:** SuperPoint+LightGlue requires replacing the tracking front-end. OpenVINS does not natively support learned features — this would require a custom feature tracker node that publishes tracks to the OpenVINS subscriber. Feasible but outside the scope of this assignment.

#### 6. Sky Masking (`use_mask: true`)
Since sky has zero features but FAST still wastes budget detecting noise there, masking the top 30% of the image (where horizon sits) forces the feature budget to be spent on the slightly-richer sand/vegetation region.

```
mask(u,v) = 0  if  v < 0.30 * H   (top 30% = sky)
mask(u,v) = 1  otherwise
```

Can be passed as a PNG mask image to OpenVINS. Small but free gain: ~5% better feature quality.

---

### Summary: What We Can Realistically Improve

| Detector | Desert quality | Real-time | OpenVINS support |
|----------|---------------|-----------|-----------------|
| FAST (current) | Poor | Yes | Native |
| FAST + CLAHE (current) | Moderate | Yes | Native |
| FAST threshold=10 (proposed) | Better | Yes | Native (our change) |
| Shi-Tomasi | Marginal gain | Yes | Partial |
| SuperPoint + LightGlue | Excellent | GPU only | Requires custom node |
| Sky mask | Small gain | Yes | Native (`use_mask`) |

The proposed tuning (lower threshold, more points, finer grid, no downsampling) extracts maximum performance from FAST+KLT without changing the OpenVINS architecture. SuperPoint is the clear path forward if GPU is available.

---

## 4I. VIO Initialization — Deep Mathematics

### Why Initialization Is Critical

A VIO filter tracks the state:
```
X = {R_WI, p_WI, v_WI, b_g, b_a}
```
- `R_WI ∈ SO(3)`: rotation from IMU to world
- `p_WI ∈ R³`: position
- `v_WI ∈ R³`: velocity (in world frame)
- `b_g ∈ R³`: gyroscope bias
- `b_a ∈ R³`: accelerometer bias

**The filter cannot self-correct a bad initial state.** If velocity `v₀` is wrong by 1 m/s, position error grows as `~Δv * t²/2` — quadratic divergence. If gravity direction `g` is wrong by 1°, horizontal acceleration is contaminated by `g * sin(1°) ≈ 0.17 m/s²`, causing velocity drift of `0.17 * t m/s`. After 10s: 8.5m error in position.

**In monocular VIO:** initialization must also recover metric scale `s`, since visual-only reconstruction gives structure up to scale. Scale is entangled with velocity and gravity, making initialization a nonlinear problem.

---

### Static Initialization (vehicle stationary at t=0)

If the vehicle is completely still, the accelerometer reads pure gravity:
```
a_measured = R_WI^T * g_W + b_a + n_a
```

Taking the mean over the stationary window:
```
ā = R_WI^T * g_W + b_a
```

Gravity direction (unit vector) in body frame:
```
ĝ_B = -ā / ||ā||    (negative because gravity points down)
```

This gives roll and pitch directly:
```
roll  = atan2(ĝ_B[1], ĝ_B[2])
pitch = atan2(-ĝ_B[0], sqrt(ĝ_B[1]² + ĝ_B[2]²))
```

Yaw is unobservable from gravity alone (rotation about gravity axis leaves ā unchanged). Initial yaw = 0 (arbitrary).

Gyroscope bias from stationary window:
```
b_g = (1/N) Σᵢ ω_measured[i]    (mean gyro output = bias when ω_true = 0)
```

Initial velocity: `v₀ = 0` (known from stationary condition).

**Why this fails for our dataset:** The bag may not start stationary (or only briefly). The `init_dyn_use: true` flag tells OpenVINS to use dynamic initialization instead.

---

### Dynamic Initialization (OpenVINS `init_dyn_use: true`)

Dynamic initialization recovers `{v₀, g_W, s, b_a, b_g}` from a window of camera frames with known visual tracks and simultaneous IMU measurements.

#### Step 1: IMU Preintegration

Between consecutive camera frames at times `i` and `j`, integrate IMU:
```
ΔR_ij = Π_{k=i}^{j-1} Exp((ω_k - b_g) * δt)          [rotation preintegral]
Δv_ij = Σ_{k=i}^{j-1} ΔR_ik * (a_k - b_a) * δt       [velocity preintegral]
Δp_ij = Σ_{k=i}^{j-1} [ΔR_ik * (a_k - b_a) * δt² / 2 + Δv_ik * δt]  [position preintegral]
```

where `Exp(φ) = I + sin(||φ||)/||φ|| [φ]× + (1-cos(||φ||))/||φ||² [φ]×²` is the Rodrigues exponential map on SO(3).

These are **relative** quantities — they depend only on IMU data between frames, not on the global state. They can be precomputed once and reused.

#### Step 2: Camera Frame Constraints

From visual feature tracks, monocular SfM gives positions up to an unknown scale `s`:
```
p̃_i = s * p_true_i
```

The scale factor `s` is the same for all frames (rigid structure).

#### Step 3: Linear System for Scale, Velocity, Gravity

Stacking the IMU position preintegral equation across all frame pairs:
```
p_j - p_i = v_i * Δt_ij + (1/2) * g_W * Δt_ij² + R_i * Δp_ij
```

Substituting `p = s * p̃` and rearranging:
```
s*(p̃_j - p̃_i) - R_i * Δp_ij = v_i * Δt_ij + (1/2) * g_W * Δt_ij²
```

For N consecutive frame pairs, this forms a linear system `A * x = b`:
```
A = [Δt₁₂·I  (1/2)·Δt₁₂²·I  (p̃₂-p̃₁)]   x = [v₁; g_W; s]
    [Δt₂₃·I  (1/2)·Δt₂₃²·I  (p̃₃-p̃₂)]   b = [R₁·Δp₁₂; R₂·Δp₂₃; ...]
    [...]
```

Solved with SVD:
```
x* = V * Σ⁺ * U^T * b   (least-squares solution)
```

The condition number `κ(A)` indicates whether the system is well-determined:
- `κ → ∞`: degenerate — typically pure translation (scale unobservable) or too-short window
- `κ < 1/init_dyn_min_rec_cond`: accepted (our setting: `min_rec_cond = 1e-12`)

**`init_dyn_min_deg: 3.0`** (our tuned value): requires the camera to rotate at least 3° within the initialization window. This ensures the visual structure has sufficient parallax for reliable `p̃` estimates, making matrix `A` well-conditioned.

**`init_window_time: 1.5s`** (our tuned value): the window used to accumulate frame pairs. Shorter = faster initialization but less conditioning. 1.5s gives ~42 frames at 28Hz — sufficient for 5–6 frame pairs.

#### Step 4: MLE Refinement (`init_dyn_mle_opt_calib: false`)

The linear solution is used as initialization for a Maximum Likelihood Estimation:
```
x_MLE = argmin Σ_i ||z_i - h(x)||²_{Σ_i}
```
where `z_i` are the visual reprojection measurements and `h(x)` is the reprojection model. Solved by Gauss-Newton with `init_dyn_mle_max_iter=50` iterations.

The MLE step refines all quantities simultaneously with proper noise weighting, giving a consistent initial state that the EKF can then maintain.

#### Why Initialization Matters for Our ATE

The ATE plot shows error dropping from ~80m to ~3m over the first 80s. This is the **initialization transient** — the filter is converging from a bad initial state. Better initialization:
- Reduces ATE in the first 80s (currently wasted)
- Gives correct initial scale → less scale drift later
- Correct initial velocity → prevents early velocity-driven divergence

Our change `init_dyn_min_deg: 5.0 → 3.0` means the filter initializes even when the UGV drives nearly straight (as is common in desert traversal), avoiding the failure mode where initialization never triggers.

---

## 4J. ZUPT — Zero-Velocity Updates in VIO

### Physical Meaning

A **Zero-Velocity Update (ZUPT)** is an EKF measurement update that asserts the robot's velocity is (approximately) zero. It fires when the robot is stationary or moving very slowly.

Detection in OpenVINS uses two independent tests:
```
||v̂||₂ < zupt_max_velocity     (predicted velocity test)
disp_features < zupt_max_disparity  (visual feature disparity test)
```

Both must pass. This prevents false ZUPTs (e.g., a fast pan of the camera would have large disparity, blocking ZUPT even if IMU says slow).

---

### ZUPT as an EKF Measurement

The measurement model:
```
z_zupt = 0 ∈ R³             (true velocity = 0)
h(X) = v_WI                 (state velocity)
H_zupt = [0 | I₃ | 0 ...]   (Jacobian: selects velocity rows from state)
R_zupt = (σ_v)² · I₃        (measurement noise covariance, σ_v set by zupt_noise_multiplier)
```

EKF update equations:
```
Innovation:          ν = z_zupt - H · x̂  =  -v̂_WI
Innovation cov:      S = H · P · H^T + R_zupt
Kalman gain:         K = P · H^T · S^{-1}
State update:        x̂ ← x̂ + K · ν       (pulls velocity toward 0)
Covariance update:   P ← (I - K·H) · P    (reduces velocity uncertainty)
```

The `zupt_noise_multiplier = 10` scales `R_zupt` by 10×, making the zero-velocity constraint soft rather than hard. This is important for bumpy terrain where the vehicle nominally stops but the body still vibrates — a hard constraint would corrupt the filter state.

---

### Why ZUPT Is Critical in VIO

#### 1. Accelerometer Bias Becomes Observable at Stops

When `v = 0` and `a_true = 0` (no acceleration at stop), the accelerometer reads:
```
a_measured = R_WI^T · g_W + b_a + n_a
```

Since `R_WI` is known from the filter and `g_W` is known, we can directly estimate:
```
b_a ≈ a_measured - R_WI^T · g_W
```

Without stops, accelerometer bias `b_a` is only weakly observable during motion (requires careful excitation). With ZUPT at every stop, `b_a` is updated at each stop — dramatically reducing position drift between stops.

#### 2. Velocity Error Bounded Between Stops

IMU-only velocity integration drifts as:
```
δv(t) = ∫₀ᵗ (δb_a + n_a) dτ  ≈  δb_a · t  (dominant term)
```

Position error from velocity drift:
```
δp(t) = ∫₀ᵗ δv(τ) dτ  ≈  δb_a · t² / 2   (quadratic growth)
```

With ZUPT at time `t_stop`:
```
δv(t_stop) → ~0    (velocity corrected)
δp(t) = δb_a · (t - t_stop)² / 2   (error clock restarted)
```

Each stop resets the quadratic error clock. A UGV that stops every 30s accumulates far less drift than one that drives continuously.

#### 3. Gyroscope Bias Observability

Heading drift: `δψ(t) ≈ δb_gz · t`. Between visual updates, heading is only corrected when features reproject. In featureless desert, there may be seconds between good visual updates. ZUPT doesn't directly observe heading, but it constrains the lateral velocity to zero:
```
v_y = 0  at stop  →  lateral velocity component observed
```

In 2D planar motion, constraining `v_y` during stops plus the non-holonomic constraint (robot can't slide sideways) effectively constrains heading drift.

#### 4. Quantitative Impact for Desert UGV

In our dataset, the ATE drops to ~3m at t≈80s and again to ~15m at t≈300s — these are likely points where the vehicle stopped or slowed, triggering ZUPTs that reset the velocity error. The periods of rising ATE (t=130–230s, t=310–430s) are likely continuous driving sections with no ZUPT triggers.

Our tuning changes:
- `zupt_max_velocity: 0.5 → 0.3`: tighter detection, avoids false positives during slow crawl that isn't a true stop
- `zupt_max_disparity: 0.5 → 1.5`: more permissive visual check; bumpy desert terrain causes slight camera shake even at a genuine stop, so the old threshold of 0.5px was blocking valid ZUPTs

---

### ZUPT vs Non-Holonomic Constraint (NHC)

For a UGV that cannot slide sideways, a complementary constraint is the **Non-Holonomic Constraint**:
```
v_lateral = 0   (always, not just at stops)
v_vertical = 0
```

NHC acts like a continuous ZUPT in the lateral and vertical directions. OpenVINS doesn't natively support NHC, but it would be a powerful addition for this ground vehicle. If implemented, it would reduce lateral drift continuously — similar benefit to ZUPT but firing every frame rather than only at stops.

---

## 5. Combined Proposed Config Changes

### `estimator_config.yaml` — diff

```yaml
# Feature extraction
num_pts:           300  →  500
fast_threshold:    20   →  10
grid_x:            8    →  10
grid_y:            5    →  7
min_px_dist:       15   →  8
knn_ratio:         0.70 →  0.75
downsample_cameras: true → false

# SLAM features
max_slam:          50   →  150
max_slam_in_update: 25  →  75
dt_slam_delay:     2    →  1

# Filter window
max_clones:        11   →  15
max_msckf_in_update: 40 →  60

# Update model
up_msckf_sigma_px: 1    →  1.5
up_slam_sigma_px:  1    →  1.5

# Gravity
gravity_mag:       9.81 →  9.82

# Initialization
init_dyn_min_deg:  5.0  →  3.0
init_window_time:  2.0  →  1.5

# ZUPT
zupt_max_velocity: 0.5  →  0.3
zupt_max_disparity: 0.5 →  1.5
```

### `kalibr_imu_chain.yaml` — diff

```yaml
accelerometer_noise_density: 3.7e-2  →  7.4e-2
accelerometer_random_walk:   4.0e-3  →  7.4e-3
gyroscope_noise_density:     8.5e-4  →  1.7e-3
gyroscope_random_walk:       8.5e-5  →  1.7e-4
```

---

## 6. Expected Outcomes After Tuning

| Metric | Baseline | Expected after tuning | Mechanism |
|--------|----------|-----------------------|-----------|
| Scale | 1.363 | **1.05–1.20** | More SLAM features + larger window |
| ATE RMSE | 56.73m | **20–35m** | Better scale + fewer tracking gaps |
| Mean ATE | 50.39m | **15–28m** | |
| RPE mean | 3.71m/10m | **1.5–2.5m/10m** | Better local tracking |
| Frames tracked | 48% | **75–90%** | Lower FAST threshold catches more features |

These are estimates based on analogous challenging outdoor datasets. Actual results depend on how feature-sparse the sequence truly is and whether the UGV motion is sufficiently non-degenerate for scale observability.

---

## 7. What Cannot Be Fixed by Tuning

| Issue | Reason | Mitigation |
|-------|--------|-----------|
| Absolute scale = 1.0 | Monocular VIO is scale-ambiguous by definition | Use Sim3 evaluation (already doing this) |
| ATE < 5m on log_01 | Open desert; no loop closures; IMU drift unbounded | Would require stereo camera or GPS fusion |
| Perfect start/end alignment | Monocular drift over 550s is inherent | Log_02 (closed loop) will show much less end drift |
| Featureless sky/sand | Can't track what doesn't exist | Mask sky region (`use_mask: true`) — minor gain |

---

## 8. Incremental Testing Protocol

Run each group independently to isolate impact. Re-run full log_01 bag after each group:

1. **Group A only** (feature extraction) → measure Δ ATE and Δ scale
2. **Group B only** (SLAM features) → compare baseline
3. **Groups A+B together** → expected largest single improvement
4. **Groups A+B+C+D** → full proposed config
5. **Final config on log_02** (no re-tuning) → robustness check

---

## 10. Loop Closure — Analysis and Options for OpenVINS

### 10A. Does OpenVINS Have Loop Closure?

**No.** The core MSCKF filter has no place recognition, no pose graph, and no keyframe database.
OpenVINS intentionally omits loop closure to remain a pure VIO odometry system. This is why log_02 fails to close its oval loop — by the time the vehicle returns to the hangar (~300s later), all SLAM features from the start have been marginalized out of the 15-clone sliding window.

However, OpenVINS **does publish a secondary topic interface** (added in PR #66, 2020) specifically for downstream loop closure modules to subscribe to:

```
/ov_msckf/points_slam       — marginalized 3D feature positions
/ov_msckf/odometry          — pose estimate
/ov_msckf/loop_pose         — historical pose for loop closure
/ov_msckf/loop_point        — feature pointcloud with camera info
```

Two API hooks also exist in the codebase:
- `TrackBase::change_feat_id()` — re-assigns feature IDs when a loop is detected (merges old/new observations)
- `VioManager::retriangulate_active_tracks()` — re-triangulates all active features, useful after a loop correction

The dev roadmap explicitly lists loop closure as a planned future feature: *"Creation of a secondary graph-based thread that loosely introduces loop closures (akin to the second thread of VINS-Mono)".*

### 10B. Official RPNG Companion Repositories

**`rpng/ov_secondary`** — the official loop closure companion for OpenVINS
- Ported from VINS-Mono/VINS-Fusion (HKUST)
- Place recognition: **DBoW2 + BRIEF descriptors** — retrieves candidate keyframes by vocabulary similarity
- Loop verification: 2D-2D RANSAC → 3D-2D PnP RANSAC → geometric consistency
- Pose graph: **4-DOF optimization via Ceres** (yaw + translation; roll/pitch observable by IMU so not optimized)
- **Loosely coupled only** — corrected odometry is published as a separate topic; corrections do NOT re-enter the OpenVINS EKF state
- Architecture:

```
OpenVINS EKF  →  publishes loop topics  →  ov_secondary node
                                              ├─ DBoW2 keyframe DB
                                              ├─ pose graph (Ceres)
                                              └─ publishes corrected odometry
```

- **Limitation: ROS1 only** — catkin build, ~8 commits, no active maintenance

**`rpng/ov_maplab`** — offline loop closure via ETH maplab
- Exports OpenVINS runs to maplab's ViMap format
- Loop closure runs **offline** with `loopclosure_all_missions` + pose graph `relax`
- Useful for post-mission analysis and multi-session map merging
- Not real-time

### 10C. Community Forks with Loop Closure

| Fork | Method | ROS2 | Desert Suitability | Notes |
|------|--------|-------|--------------------|-------|
| `rpng/ov_secondary` | DBoW2 + BRIEF | No (ROS1) | Poor — BRIEF fails on featureless sand | Official, most documented |
| `Li-Jesse-Jiaze/ov_hloc` | SuperPoint + NetVLAD | No (ROS1 Noetic) | **Best** — NetVLAD is appearance-based, not feature-point-based | GPU required; ~19 commits |
| `PranavNedunghat/OpenVINS_SuperPointGlue` | SuperPoint + SuperGlue tracker | No (ROS1) | Good tracker improvement | Tracking only, not loop closure |
| `yuhaozhang7/openvins-slamfuse` | ov_secondary style | No | Moderate | Limited docs |
| `rpng/MINS` | Full multi-sensor (LiDAR+GPS+Camera+IMU) | No (ROS1) | High — GPS corrects drift | Replaces OpenVINS entirely; complex |

### 10D. Loop Closure Methods — Suitability for Desert

| Method | Mechanism | Desert Suitability | Notes |
|--------|-----------|-------------------|-------|
| DBoW2 + BRIEF/ORB | Bag-of-words on local corners | **Poor** | Sand textures sparse and repetitive → high false positive rate |
| NetVLAD | CNN global scene descriptor | **Best** | Appearance-based; handles illumination/viewpoint change; VGG16 backbone needs GPU |
| SuperPoint + SuperGlue | Learned keypoints + GNN matcher | **Good** | Works on low-texture scenes; GPU required for real-time |
| iBoW-LCD | Incremental BoW, no pre-trained vocabulary | Good | Builds vocabulary online; used in OV2SLAM (not OpenVINS) |

### 10E. Why `ov_secondary` (DBoW2) Would Still Struggle on tbuggy

Even after porting to ROS2, DBoW2 on the tbuggy desert sequences would likely produce poor loop closure results:

1. **Sparse BRIEF features** — FAST+BRIEF on sand/sky gives very few stable descriptors per frame
2. **Repetitive textures** — sandy terrain produces near-identical BoW vectors → perceptual aliasing (false positives)
3. **Large viewpoint change** — the vehicle approaches the hangar from a different angle on return, changing the apparent geometry significantly
4. **No pre-trained desert vocabulary** — the standard DBoW2 vocabularies are trained on indoor/urban scenes

`ov_hloc` with NetVLAD would be better: it queries on holistic image appearance (not individual corners), making it robust to the uniform appearance of desert terrain.

### 10F. Practical Options for This Assignment

**Option 1 — Port `ov_secondary` to ROS2** (2–3 days effort)
- Clone ov_secondary, convert catkin → ament_cmake, update ROS1 APIs to ROS2
- Subscribe to OpenVINS ROS2 loop closure topics
- Expected: loop closure fires sometimes on hangar re-observation; will likely miss loop on featureless desert section

**Option 2 — `ov_hloc` with ROS2 wrapper** (1 week effort)
- Run hloc (Python, GPU) as a separate process; write a ROS2 bridge node
- SuperPoint extracts local features per keyframe; NetVLAD provides global descriptor for retrieval
- Candidate frames verified with SuperGlue matches + PnP
- Expected: much more reliable loop detection at hangar return even with large viewpoint change

**Option 3 — Switch to VINS-Mono (ROS1)** (1 day effort)
- VINS-Mono has built-in DBoW2 loop closure with VINS-Fusion's pose graph
- Would require ROS1 + bag conversion (ros2 bag → ros1 bag)
- Not a viable path for ROS2-native assignment

**Option 4 — ORB-SLAM3 (ROS2 wrapper available)** (2–3 days)
- ORB-SLAM3 has tightly integrated loop closure (DBoW2 + graph BA)
- ROS2 wrappers exist (e.g., `zang09908/ORB_SLAM3_ROS2`)
- Monocular+IMU mode available
- Caveat: ORB features still struggle in desert; Atlas map maintenance can be fragile

**Most practical for desert + ROS2: Option 2 (ov_hloc + ROS2 bridge) or Option 4 (ORB-SLAM3 ROS2)**

---

## 9. Remaining Tasks

| Task | Status | Command / Notes |
|------|--------|----------------|
| Apply tuned config | TODO | Edit estimator_config.yaml + kalibr_imu_chain.yaml |
| Full log_01 run (tuned) | TODO | `ros2 bag play ... --rate 0.5`, ~18 min |
| Evaluate log_01 | TODO | `evaluate_trajectory.py` + `plot_trajectory_comparison.py` |
| Generate gt_log02.csv | TODO | `extract_gt_csv.py log_02 ...` |
| Full log_02 run | TODO | Same config, no re-tuning |
| Evaluate log_02 | TODO | Compare ATE with log_01 |
| Dockerfile | TODO | Base: `osrf/ros:humble-desktop` |
| Technical report | TODO | PDF or Markdown |
| Git push | TODO | `git push -u origin master` |
