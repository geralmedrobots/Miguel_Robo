# Odometry Calibration Guide

**Purpose:** Empirically tune odometry covariance model for your specific robot hardware and operating environment.

---

## Overview

The odometry system uses a **distance-dependent covariance model** to estimate position and orientation uncertainty. Accurate covariance is critical for:

- **Sensor Fusion:** EKF/UKF uses covariance to weight odometry vs. other sensors (LIDAR, IMU, GPS)
- **SLAM:** Accurate covariance improves map quality and loop closure
- **Path Planning:** Nav2 uses covariance for obstacle avoidance margins

**Current Model (conservative defaults):**
```
σ_position = sqrt(0.001) * (1 + distance/10)  [meters]
σ_yaw = sqrt(0.03) * (1 + distance/10)        [radians]
```

---

## Required Equipment

### Mandatory:
- ✅ Measuring tape (≥10m, 1mm precision)
- ✅ Carpenter's square or laser level (for 90° angles)
- ✅ Stopwatch
- ✅ Flat, level surface (concrete floor ideal)
- ✅ Masking tape (for marking)

### Recommended:
- 🔶 Total station or laser tracker (professional calibration)
- 🔶 IMU/gyroscope (for yaw validation)
- 🔶 Motion capture system (e.g., OptiTrack, Vicon)
- 🔶 RTK-GPS (outdoor calibration)

---

## Calibration Procedure

### Test 1: Straight Line Position Error

**Goal:** Measure position drift over known distance

1. **Setup:**
   ```bash
   # Mark start/end positions on floor
   # Distance: 10m (adjust for available space)
   
   # Reset odometry
   ros2 service call /somanet_driver/reset_odometry std_srvs/srv/Empty
   ```

2. **Execute:**
   ```bash
   # Drive robot straight for 10m at constant velocity
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}" --once
   
   # Stop at 10m mark
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}}" --once
   ```

3. **Measure:**
   ```bash
   # Record odometry position
   ros2 topic echo /odom --once
   # Note: pose.pose.position.x, pose.pose.position.y
   
   # Measure actual position with tape measure
   # Error = ||measured - odometry||
   ```

4. **Repeat:**
   - Run 10 trials
   - Compute mean error (ε̄) and standard deviation (σ)

5. **Tune `VAR_POSITION`:**
   ```cpp
   // In odometry_calculator.cpp
   // Set to (3-sigma rule): VAR_POSITION = (σ / 3)²
   
   // Example: If σ = 0.05m → VAR_POSITION = (0.05 / 3)² ≈ 0.000278
   static constexpr double VAR_POSITION = 0.000278;
   ```

---

### Test 2: Rotation Yaw Error

**Goal:** Measure orientation drift during in-place rotation

1. **Setup:**
   ```bash
   # Mark reference direction (e.g., tape line on floor)
   # Place robot at center
   
   # Reset odometry
   ros2 service call /somanet_driver/reset_odometry std_srvs/srv/Empty
   ```

2. **Execute:**
   ```bash
   # Rotate 360° (full circle)
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --rate 10
   
   # Stop after ~12.6 seconds (2π rad at 0.5 rad/s)
   # Final orientation should be 0°
   ```

3. **Measure:**
   ```bash
   # Record odometry yaw
   ros2 topic echo /odom --once
   # Convert quaternion to yaw: atan2(2*(w*z + x*y), 1-2*(y² + z²))
   
   # Measure actual orientation (compass, IMU, or visual alignment)
   # Error = |measured_yaw - odometry_yaw|
   ```

4. **Repeat:**
   - Run 10 trials (mix CW and CCW rotations)
   - Compute mean error (ε̄_yaw) and standard deviation (σ_yaw)

5. **Tune `VAR_YAW`:**
   ```cpp
   // In odometry_calculator.cpp
   // Set to: VAR_YAW = (σ_yaw / 3)²
   
   // Example: If σ_yaw = 0.1 rad (5.7°) → VAR_YAW = (0.1 / 3)² ≈ 0.0011
   static constexpr double VAR_YAW = 0.0011;
   ```

---

### Test 3: Figure-8 Path (Combined Motion)

**Goal:** Validate model on realistic trajectory (mixed linear + angular)

1. **Setup:**
   ```bash
   # Mark figure-8 path on floor (scale: 5m loops)
   # Record ground truth waypoints (GPS, mocap, or manual survey)
   ```

2. **Execute:**
   ```bash
   # Drive figure-8 pattern (manual or autonomous)
   # Log odometry data:
   ros2 bag record /odom /cmd_vel
   ```

3. **Analysis:**
   ```bash
   # Play back bag and compare odometry to ground truth
   ros2 bag play <bag_file>
   
   # Plot error over distance:
   #   - X-axis: Distance traveled (from odometry)
   #   - Y-axis: Position error magnitude
   #   - Overlay: sqrt(covariance) = 1-sigma bound
   
   # Validation criteria:
   #   ✅ ~68% of samples within 1-sigma bound
   #   ✅ ~95% of samples within 2-sigma bound
   ```

4. **Adjust growth factor:**
   ```cpp
   // If error grows faster than predicted:
   double distance_factor = 1.0 + total_distance_ / 8.0;  // Faster growth
   
   // If error grows slower than predicted:
   double distance_factor = 1.0 + total_distance_ / 12.0;  // Slower growth
   ```

---

### Test 4: Maximum Distance Validation

**Goal:** Determine when odometry becomes unreliable (saturation point)

1. **Execute:**
   ```bash
   # Drive long straight path (>50m)
   # Log odometry + ground truth
   ```

2. **Find saturation distance:**
   ```
   # When error reaches 3x predicted covariance → odometry unreliable
   # Set MAX_VAR_POSITION accordingly
   
   # Example: If error exceeds 1m at 50m distance:
   const double MAX_VAR_POSITION = 1.0;  // 1 m² variance
   ```

---

## Calibration Results Template

Document your results in `config/odometry_calibration_report.yaml`:

```yaml
# Odometry Calibration Report
# Date: YYYY-MM-DD
# Robot ID: <serial_number>
# Surface: concrete/asphalt/carpet
# Temperature: <°C>
# Tire pressure: <PSI> (if applicable)

straight_line_test:
  distance: 10.0  # meters
  trials: 10
  mean_error: 0.05  # meters
  std_deviation: 0.02  # meters
  var_position_tuned: 0.000044  # (0.02/3)^2

rotation_test:
  angle: 6.283  # radians (360°)
  trials: 10
  mean_error: 0.087  # radians (5°)
  std_deviation: 0.035  # radians (2°)
  var_yaw_tuned: 0.000136  # (0.035/3)^2

figure8_test:
  total_distance: 25.0  # meters
  max_position_error: 0.45  # meters at 25m
  validation: PASS  # 95% within 2-sigma

max_reliable_distance: 80.0  # meters
max_var_position: 2.25  # m^2 (saturation at 1.5m error)

recommended_settings:
  VAR_POSITION: 0.000044
  VAR_YAW: 0.000136
  distance_factor_divisor: 10.0  # (1 + d/10)
  MAX_VAR_POSITION: 2.25
  MAX_VAR_YAW: 5.0
```

---

## Applying Calibration Results

1. **Edit source code:**
   ```bash
   nano src/odometry_calculator.cpp
   ```

2. **Update constants:**
   ```cpp
   // Line ~127
   static constexpr double VAR_POSITION = 0.000044;  // From calibration
   static constexpr double VAR_YAW = 0.000136;       // From calibration
   
   // Line ~183
   const double MAX_VAR_POSITION = 2.25;  // From saturation test
   const double MAX_VAR_YAW = 5.0;
   ```

3. **Rebuild:**
   ```bash
   colcon build --packages-select somanet
   ```

4. **Validate:**
   ```bash
   # Run validation test (long trajectory with ground truth)
   ros2 launch somanet launch.py
   
   # Log covariance vs actual error
   ros2 topic echo /odom --field pose.covariance[0]  # x variance
   ```

---

## Advanced Calibration (Optional)

### Surface-Dependent Models

Different surfaces have different slip characteristics:

```cpp
// In odometry_calculator.cpp
double getSurfaceMultiplier() {
    // Load from parameter server
    std::string surface_type = this->get_parameter("surface_type").as_string();
    
    if (surface_type == "concrete") return 1.0;      // Reference
    if (surface_type == "asphalt") return 1.2;       // Slight slip
    if (surface_type == "tile") return 1.5;          // More slip
    if (surface_type == "carpet") return 2.0;        // High slip
    return 1.0;  // Default
}

// Apply multiplier
var_position = VAR_POSITION * distance_factor * distance_factor * getSurfaceMultiplier();
```

### Load-Dependent Models

Heavier payloads may increase slip:

```cpp
double getLoadMultiplier(double payload_kg) {
    // Calibrate with different payloads: 0kg, 50kg, 100kg
    return 1.0 + (payload_kg / 100.0) * 0.2;  // +20% per 100kg
}
```

---

## Troubleshooting

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| Error grows faster than predicted | Wheel slip, poor surface | Increase `distance_factor` divisor (e.g., 8.0 instead of 10.0) |
| Error grows slower than predicted | High-traction surface | Decrease divisor (e.g., 12.0) |
| Large initial error (at origin) | Sensor noise, quantization | Increase `VAR_POSITION`, `VAR_YAW` base values |
| Asymmetric error (CW vs CCW) | Wheel diameter mismatch | Calibrate wheel diameters separately |
| Random jumps in error | Encoder glitches, loose wiring | Fix hardware, enable acceleration validation |

---

## References

- **ISO 13849-1:** Validation of safety-related odometry (§7.2)
- **ROS REP-105:** Coordinate frames for mobile platforms
- **Probabilistic Robotics** (Thrun et al., 2005): Chapter 5 - Robot Motion
- **"Characterization of the Mecanum Wheel and Odometry Error"** (Muir & Neuman, 1987)

---

**Next Steps:**
1. Run calibration tests with your robot
2. Document results in `config/odometry_calibration_report.yaml`
3. Update constants in `odometry_calculator.cpp`
4. Validate with long-duration test (>1 hour continuous operation)
5. Re-calibrate annually or after hardware changes (wheel replacement, tire pressure, etc.)
