# Changelog

All notable changes to the Ultrabot AGV Navigation System.

---

## [4.1.0] - 2025-11-06

### Added
- **SLAM integration** (SLAM Toolbox + Cartographer)
- **EKF sensor fusion** (robot_localization) for odometry filtering - integrated in all 5 launch files
- **Behavior Tree XML** files for safety-aware navigation
  - `navigate_w_safety.xml` - Enhanced single-goal navigation with AGV-specific recovery
  - `navigate_through_poses_w_safety.xml` - Multi-waypoint navigation with safety monitoring
- **launch_slam.py**, **launch_cartographer.py**, **launch_slam_nav.py**
- **save_map.py** utility script
- **slam_params.yaml**, **cartographer_config.lua**, **ekf_params.yaml**
- **SLAM_GUIDE.md** documentation
- **teleop_joy.cpp** complete rewrite with watchdog, parameters, safety integration

### Changed
- **package.xml** version 4.0.0 → 4.1.0
- Added slam_toolbox, cartographer, robot_localization dependencies
- Fixed collision_monitor routing (cmd_vel_nav_safe)
- Uniformized scan topics
- EKF node integrated in all launch files (50 Hz filtering)

---

## [4.0.0] - 2025-11-05

### Added
- **Nav2 autonomous navigation** (DWB, NavFn, AMCL, behaviors)
- **command_mux_node** (teleop vs autonomous arbitration)
- **nav2_params.yaml**, **launch_with_nav2.py**
- **README_NAV2.md** documentation

### Changed
- 5-layer command flow (added mux layer)
- **package.xml** version 3.3.0 → 4.0.0

---

## [3.1.0] - 2025-11-04

### Added
- Parameter validation with strict bounds (ISO 13849-1)
- **PARAMETER_VALIDATION.md** documentation

---

## [3.0.0] - 2025

### Added
- Modular odometry system (OdometryCalculator)
- Command arbitration with 4 priorities
- 21 unit tests, 95% coverage

---

## [2.0.0] - 2024

### Added
- Safety supervisor (ISO 13849-1 Cat 3 PL d)
- Certified parameter validation (HMAC-SHA256)
- SROS2 security support

---

## [1.0.0] - 2023

### Added
- Initial ROS2 Humble migration
- EtherCAT motor control (SOEM)
- Lifecycle-managed nodes
- Differential drive odometry

---

**Current Version:** 4.1.0 | **Last Updated:** 2025-11-05
