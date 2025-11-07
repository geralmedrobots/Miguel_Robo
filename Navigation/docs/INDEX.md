# ULTRABOT AGV Documentation Index

**Version 4.1.0** | **Complete Documentation Reference**

---

## 📖 Documentation Overview

This directory contains detailed technical documentation for the ULTRABOT AGV navigation system. For quick start and overview, see the main [README.md](../README.md).

---

## 📚 Documentation Structure

### Level 1: Quick Start (Root Directory)

**Target Audience:** Operators, new users

| File | Purpose | Read Time |
|------|---------|-----------|
| [README.md](../README.md) | System overview, quick start, launch commands | 5 min |
| [SAFETY.md](../SAFETY.md) | Safety procedures, emergency protocols | 10 min |
| [BUILD_GUIDE.md](../BUILD_GUIDE.md) | Build instructions, troubleshooting | 15 min |

### Level 2: Integration Guides (Root Directory)

**Target Audience:** System integrators, advanced users

| File | Purpose | Read Time |
|------|---------|-----------|
| [README_NAV2.md](../README_NAV2.md) | Nav2 integration, behavior trees, advanced navigation | 20 min |
| [SLAM_GUIDE.md](../SLAM_GUIDE.md) | SLAM configuration (Cartographer, SLAM Toolbox) | 15 min |
| [CALIBRATION_GUIDE.md](../CALIBRATION_GUIDE.md) | Sensor calibration procedures | 20 min |
| [BEHAVIOR_TREE_GUIDE.md](../BEHAVIOR_TREE_GUIDE.md) | Custom behavior tree creation | 15 min |
| [TIME_SYNC_GUIDE.md](../TIME_SYNC_GUIDE.md) | NTP/Chrony time synchronization | 15 min |
| [PARAMETER_VALIDATION.md](../PARAMETER_VALIDATION.md) | Safety parameter certification | 10 min |
| [SROS2_GUIDE.md](../SROS2_GUIDE.md) | DDS security configuration | 20 min |

### Level 3: Technical Reference (docs/ Directory)

**Target Audience:** Developers, maintainers

| File | Purpose | Read Time |
|------|---------|-----------|
| [ARCHITECTURE.md](ARCHITECTURE.md) | System architecture, data flow, state machines | 30 min |
| [API.md](API.md) | API reference, Doxygen setup, class documentation | 25 min |
| [SENSORS.md](SENSORS.md) | Multi-sensor configuration (LiDAR, cameras, IMU) | 30 min |
| [TESTING.md](TESTING.md) | Test suite guide, coverage, certification checklist | 25 min |

### Level 4: Meta Documentation

| File | Purpose |
|------|---------|
| [CHANGELOG.md](../CHANGELOG.md) | Version history, release notes |
| [INDEX.md](INDEX.md) | This file - documentation navigation |

---

## 🎯 Documentation by Task

### "I want to..."

#### ...get started quickly
1. Read [README.md](../README.md) - Quick Start section
2. Read [SAFETY.md](../SAFETY.md) - Safety procedures
3. Follow [BUILD_GUIDE.md](../BUILD_GUIDE.md) - Build and run

#### ...understand the system architecture
1. Read [README.md](../README.md) - System Architecture section
2. Deep dive: [ARCHITECTURE.md](ARCHITECTURE.md) - Complete architecture
3. Reference: [API.md](API.md) - Class documentation

#### ...configure sensors
1. Read [SENSORS.md](SENSORS.md) - Complete sensor guide
2. Calibrate: [CALIBRATION_GUIDE.md](../CALIBRATION_GUIDE.md)
3. Time sync: [TIME_SYNC_GUIDE.md](../TIME_SYNC_GUIDE.md)

#### ...set up autonomous navigation
1. Read [README_NAV2.md](../README_NAV2.md) - Nav2 integration
2. Create map: [SLAM_GUIDE.md](../SLAM_GUIDE.md)
3. Customize: [BEHAVIOR_TREE_GUIDE.md](../BEHAVIOR_TREE_GUIDE.md)

#### ...develop new features
1. Setup: [API.md](API.md) - Doxygen and API reference
2. Architecture: [ARCHITECTURE.md](ARCHITECTURE.md) - System design
3. Build: [BUILD_GUIDE.md](../BUILD_GUIDE.md) - Development workflow

#### ...deploy to production
1. Safety: [SAFETY.md](../SAFETY.md) - Safety certification
2. Security: [SROS2_GUIDE.md](../SROS2_GUIDE.md) - DDS encryption
3. Validation: [PARAMETER_VALIDATION.md](../PARAMETER_VALIDATION.md) - Parameter signing

---

## 📊 Documentation Statistics

- **Total Guides:** 15 files
- **Total Lines:** ~10,000 lines of documentation
- **Code Examples:** 170+ code snippets
- **Diagrams:** 25+ ASCII/text diagrams
- **Configuration Samples:** 40+ YAML examples
- **Test Cases:** 44+ automated tests

---

## 🔍 Quick Reference

### Launch Commands

```bash
# SLAM mapping
ros2 launch somanet launch_slam.py

# Navigation with map
ros2 launch somanet launch_with_nav2.py map:=my_map.yaml

# Full sensor suite
ros2 launch somanet launch_sensors.py \
    enable_lidar_2d:=true enable_lidar_3d:=true \
    enable_realsense:=true enable_imu:=true
```

### Key Topics

```bash
/scan                 # Merged 2D LiDAR (360°)
/points               # 3D LiDAR point cloud
/imu/data             # IMU measurements
/odom                 # Raw wheel odometry (200 Hz)
/odometry/filtered    # EKF-fused odometry (50 Hz)
/cmd_vel              # Final velocity command
```

### Key Nodes

```bash
somanet_driver          # EtherCAT motor control
safety_supervisor       # Safety validation
command_arbitrator      # Priority-based command selection
odometry_calculator     # Wheel odometry
ekf_filter_node         # Sensor fusion (EKF)
```

---

## 🆘 Getting Help

### Troubleshooting Order

1. **Check logs:** `ros2 launch somanet <launch_file> --log-level debug`
2. **Read relevant guide:** See "Documentation by Task" above
3. **Check diagnostics:** `ros2 topic echo /diagnostics`
4. **Verify topics:** `ros2 topic list` and `ros2 topic hz <topic>`
5. **Search issues:** Check repository issue tracker

### Common Issues

| Symptom | Guide | Section |
|---------|-------|---------|
| Robot not moving | [BUILD_GUIDE.md](../BUILD_GUIDE.md) | Troubleshooting |
| Sensors not publishing | [SENSORS.md](SENSORS.md) | Troubleshooting |
| Navigation fails | [README_NAV2.md](../README_NAV2.md) | Troubleshooting |
| EtherCAT errors | [BUILD_GUIDE.md](../BUILD_GUIDE.md) | EtherCAT Setup |
| Safety stop triggered | [SAFETY.md](../SAFETY.md) | Fault Recovery |
| Time sync issues | [TIME_SYNC_GUIDE.md](../TIME_SYNC_GUIDE.md) | Monitoring |
| Tests failing | [TESTING.md](TESTING.md) | Debugging Failed Tests |

---

## 📝 Documentation Standards

All documentation follows these conventions:

- **Markdown Format:** All files are `.md` (Markdown)
- **Code Blocks:** Use triple backticks with language tags (bash, yaml, cpp, python)
- **Section Numbering:** Use `##` for main sections, `###` for subsections
- **Examples:** Provide working code examples for all configurations
- **ASCII Diagrams:** Use text-based diagrams for portability
- **Cross-References:** Link to related documentation with relative paths

---

## 🔄 Keeping Documentation Updated

When modifying the codebase:

1. **Update relevant guide** if changing functionality
2. **Add code examples** for new features
3. **Update CHANGELOG.md** with version changes
4. **Regenerate API docs** if changing public interfaces (`doxygen Doxyfile`)
5. **Update this INDEX.md** if adding new documentation files

---

## 📞 Contact

**ULTRABOT Team** | **Version 4.1.0** | **November 2025**

For questions or contributions, see [README.md](../README.md) - Contributing section.

---

**Navigation:**
- [← Back to Main README](../README.md)
- [Architecture →](ARCHITECTURE.md)
- [API Reference →](API.md)
- [Sensor Guide →](SENSORS.md)
- [Testing Guide →](TESTING.md)
