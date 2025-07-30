# 🧭 Micromouse Localization & Stability Roadmap

> Goal: Build a stable and accurate micromouse using sensor fusion with magnetic encoders, VL53L1X distance sensors, and optionally a gyro.

---

## ✅ Overview

| System         | Strength                                 | Weakness                                |
|----------------|-------------------------------------------|------------------------------------------|
| **Encoders**   | Precise short-term motion, velocity control | Accumulates error (drift)               |
| **VL53L1X**    | Good for lateral correction and alignment  | Noisy, affected by wall irregularities  |
| **Gyro (opt)** | Great for stable turns                    | May drift, needs fusion/reset           |

---

## 🗺️ Step-by-Step Roadmap

---

### 🧱 Stage 1: Reliable Basic Motion

**🎯 Goal**: Consistent straight-line motion and turning

- [ ] Tune PID for each motor using encoders (velocity or position mode)
- [ ] Move exactly 1 block (e.g., 18 cm) and stop
- [ ] Implement basic 90° left/right turns
- [ ] Test brake or smooth stop to prevent overshoot
- [ ] (Optional) Add trapezoidal velocity ramping

---

### 🧱 Stage 2: Wall Following (Unfused)

**🎯 Goal**: Stay centered between walls during forward motion

- [ ] Read left and right VL53L1X distances
- [ ] Compute: `wall_error = left_dist - right_dist`
- [ ] Apply wall_error as input to steering PID (adjust wheel speeds)
- [ ] Disable correction if walls are far (>12 cm)

---

### 🧱 Stage 3: Encoder + Wall Fusion (Complementary Filter)

**🎯 Goal**: Prevent long-term heading drift using wall data

- [ ] Track heading (`theta`) from encoders
- [ ] At block center, estimate heading error:
  ```c
  heading_from_walls = atan((left - right) / wall_distance);
  ```
Use alpha ≈ 0.9 to prioritize encoder data but allow gradual correction from wall sensor input.

 Optionally reset odometry when both wall distances are approximately equal (i.e., at block center)

### 🧱 Stage 4: Gyro-Assisted Turns (Optional but Recommended)

**🎯 Goal**: Improve turn accuracy and repeatability using a low-cost gyro (e.g., MPU6050 or BNO055)

- [ ] Read and integrate angular velocity during turns
- [ ] Stop rotation precisely when integrated heading reaches desired angle (e.g., 90°)
- [ ] Calibrate or reset gyro drift before or during the run
- [ ] Combine gyro and encoder data for better heading estimate:
  ```c
  fused_theta = beta * encoder_theta + (1 - beta) * gyro_theta;
```c
fused_theta = beta * encoder_theta + (1 - beta) * gyro_theta;
```
Apply only during turn phase, or temporarily increase trust in gyro during rotation

### 🧱 Stage 5: Block-Based Re-Centering

**🎯 Goal**: Maintain long-term alignment with the maze grid despite drift

- [ ] At every block center:
  - Measure both side walls
  - If distances are similar (within 2–3 mm), assume centered and reset lateral offset
  - Use wall info to correct heading error (θ)
- [ ] Periodically apply position corrections to counter accumulated drift
- [ ] Log deviation trends over multiple blocks for post-run analysis

---

### 🧱 Stage 6: Advanced Techniques (Optional)

**🎯 Goal**: Maximize localization accuracy and autonomous control

- [ ] Implement an Extended Kalman Filter (EKF) to fuse:
  - Encoder deltas
  - Wall sensor distances
  - Gyro (if available)
- [ ] Add AI-assisted PID tuning:
  - Use Bayesian optimization or other ML-based tuning strategies
  - Run tuning algorithm on a PC with feedback from ESP32 over Wi-Fi
- [ ] Stream sensor and pose data over Wi-Fi (e.g., to a dashboard or logger) for live tuning, analysis, and plotting

---

### 📆 Suggested Week 1 Plan (Recap)

| Day | Task                                     |
|-----|------------------------------------------|
| 1   | Encoder PID tuning, test straight motion |
| 2   | Accurate 1-block move and stop           |
| 3   | VL53L1X integration and calibration       |
| 4   | Wall-following PID with wall_error       |
| 5   | Basic 90° turn implementation            |
| 6   | Add complementary filter for heading     |
| 7   | Test drift correction and block alignment|

---
