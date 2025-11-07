#  ULTRABOT AGV - Autonomous Navigation System

**Version 4.1.0** | **ROS2 Humble** | **ISO 13849-1 Cat 3 PL d**

> Safety-critical AGV control with Nav2 autonomous navigation, SLAM mapping, and multi-sensor fusion

---

## 📚 Documentation

### Core Guides

| Topic | File | Description |
|-------|------|-------------|
| 🏗️ **Architecture** | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | System architecture, data flow, state machines |
| 🧭 **Navigation & SLAM** | [README_NAV2.md](README_NAV2.md) | Nav2 integration, behavior trees, advanced tuning |
| 📡 **Sensors** | [docs/SENSORS.md](docs/SENSORS.md) | Multi-sensor configuration (LiDAR, cameras, IMU) |
| 🛡️ **Safety** | [SAFETY.md](SAFETY.md) | ISO 13849-1 compliance, safety procedures |
| 🔧 **Build & Deploy** | [BUILD_GUIDE.md](BUILD_GUIDE.md) | Compilation, dependencies, troubleshooting |
| 👨‍💻 **API Reference** | [docs/API.md](docs/API.md) | Doxygen setup, class documentation |

### Additional Resources

| Topic | File |
|-------|------|
| 🔒 **Security (SROS2)** | [SROS2_GUIDE.md](SROS2_GUIDE.md) |
| 📏 **Calibration** | [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md) |
| 🌲 **Behavior Trees** | [BEHAVIOR_TREE_GUIDE.md](BEHAVIOR_TREE_GUIDE.md) |
| ⏱️ **Time Sync** | [TIME_SYNC_GUIDE.md](TIME_SYNC_GUIDE.md) |
| ✅ **Parameter Validation** | [PARAMETER_VALIDATION.md](PARAMETER_VALIDATION.md) |
| 📝 **Changelog** | [CHANGELOG.md](CHANGELOG.md) |

---

## 🚀 Quick Start

### Prerequisites

```bash
# Ubuntu 22.04 + ROS2 Humble
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-cartographer-ros \
    ros-humble-robot-localization \
    ros-humble-realsense2-camera \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-sick-scan-xd \
    ros-humble-ira-laser-tools \
    ros-humble-joy \
    libyaml-cpp-dev \
    libssl-dev
```

### Build

```bash
cd ~/ros2_ws/src
git clone <repo-url> navigation

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select somanet --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Verify build (run tests)
colcon test --packages-select somanet
colcon test-result --verbose
```

### Launch Options

#### 1. Create Map with SLAM
```bash
# Option A: SLAM Toolbox (recommended)
ros2 launch somanet launch_slam.py slam_method:=slam_toolbox

# Option B: Cartographer (advanced users)
ros2 launch somanet launch_cartographer.py

# Save map when done
ros2 run somanet save_map.py --name my_warehouse_map
```

#### 2. Autonomous Navigation
```bash
# Navigate with saved map
ros2 launch somanet launch_with_nav2.py map:=my_warehouse_map.yaml

# Send navigation goal via RViz or CLI
ros2 topic pub /goal_pose geometry_msgs/PoseStamped ...
```

#### 3. Manual Control Only
```bash
# Teleoperation with joystick
ros2 launch somanet launch.py enable_teleop:=true

# Or keyboard control
ros2 run somanet teleop_keyboard_safe.py
```

#### 4. Full Sensor Suite
```bash
# Launch all sensors (2x LiDAR 2D, 1x LiDAR 3D, 2x RealSense, IMU)
ros2 launch somanet launch_sensors.py \
    enable_lidar_2d:=true \
    enable_lidar_3d:=true \
    enable_realsense:=true \
    enable_imu:=true
```

---

##  System Architecture

### Layered Design

```
┌──────────────────────────────────────────────────────────────┐
│              APPLICATION LAYER                               │
│  Nav2 Stack | SLAM Toolbox | Cartographer                   │
└───────────────────────────┬──────────────────────────────────┘
                            ↓ /cmd_vel_nav
┌──────────────────────────────────────────────────────────────┐
│         COMMAND MULTIPLEXING LAYER                           │
│  command_mux (Source Select) → command_arbitrator (Priority) │
└───────────────────────────┬──────────────────────────────────┘
                            ↓ /cmd_vel
┌──────────────────────────────────────────────────────────────┐
│                  SAFETY LAYER                                │
│  safety_supervisor (ISO 13849-1 Validation)                  │
└───────────────────────────┬──────────────────────────────────┘
                            ↓ /wheel_cmd_safe
┌──────────────────────────────────────────────────────────────┐
│               ODOMETRY & FUSION LAYER                        │
│  odometry_calculator | robot_localization (EKF)              │
└───────────────────────────┬──────────────────────────────────┘
                            ↓
┌──────────────────────────────────────────────────────────────┐
│                  HARDWARE LAYER                              │
│  somanet_driver (EtherCAT 200Hz) + Sensor Drivers            │
└──────────────────────────────────────────────────────────────┘
```

**Command Priority Hierarchy:**
- 🔴 **Emergency (255)** - E-stop, safety critical
- 🟠 **Manual (200)** - Joystick/keyboard teleop
- 🟢 **Autonomous (100)** - Nav2 navigation
- 🔵 **Test (50)** - Development/testing

**For detailed architecture:** See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)

---

## 📡 Sensor Suite

The ULTRABOT integrates **5 sensor categories** for robust perception:

```
         Ouster OS1-64 (360° 3D LiDAR)
                    ┌─────┐
                    │  ○  │
                    └─────┘
                      
  RealSense D455    ╔═══════╗    RealSense D455
  (Left Camera)     ║ ROBOT ║    (Right Camera)
         ◄──────────║       ║──────────►
                    ║  IMU  ║
  LiDAR 2D Front    ║       ║    LiDAR 2D Rear
         ►──────────╚═══════╝──────────◄
        (270° FOV)                (270° FOV)
```

| Sensor | Model | Purpose | Frequency |
|--------|-------|---------|-----------|
| **LiDAR 2D** (×2) | SICK TiM571 | 360° obstacle detection | 15 Hz |
| **LiDAR 3D** | Ouster OS1-64 | 3D mapping, elevation | 10 Hz |
| **Cameras** (×2) | RealSense D455 | Depth perception | 30 Hz |
| **IMU** | MPU6050 | Inertial measurement | 100 Hz |
| **Encoders** | Incremental | Wheel odometry | 200 Hz |

**Sensor Fusion:**
- **EKF**: Fuses wheel odometry + IMU (50 Hz output)
- **LiDAR Merger**: Combines front + rear 2D LiDARs into 360° scan
- **Depth-to-LaserScan**: Converts RealSense depth to 2D scans
- **Nav2 Costmaps**: Multi-sensor obstacle detection

**For sensor configuration:** See [docs/SENSORS.md](docs/SENSORS.md)

---

##  Safety Features

| Feature | Standard | Implementation |
|---------|----------|----------------|
| Emergency Stop | ISO 13850 | Hardware STO + software |
| Velocity Limits | ISO 3691-4 | 1.0 m/s max |
| Watchdog Timer | IEC 61508 | 0.5s timeout |
| Command Validation | ISO 13849-1 | Dual-channel Cat 3 PL d |
| Parameter Auth | ISO 13849-1 | HMAC-SHA256 |

 **READ [SAFETY.md](SAFETY.md) BEFORE OPERATING**

---

## 📡 Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel_nav` | Twist | Nav2 commands |
| `/cmd_vel_teleop` | Twist | Manual control |
| `/cmd_vel` | Twist | Command arbitrator output |
| `/wheel_cmd_safe` | Twist | Safety validated commands |
| `/odom` | Odometry | Raw wheel odometry (200 Hz) |
| `/odom_filtered` | Odometry | EKF filtered odometry (50 Hz) |
| `/scan` | LaserScan | LIDAR data |

---

## 🧪 System Capabilities

-  **Autonomous Navigation** - Nav2 stack with DWB controller and NavFn planner
-  **SLAM Mapping** - SLAM Toolbox (online) + Cartographer (offline, high-quality)
-  **Multi-Sensor Fusion** - EKF (wheel encoders + IMU), LiDAR merger, depth cameras
-  **Manual Teleop** - Joystick/keyboard with priority override
-  **Safety Certified** - ISO 13849-1 Cat 3 PL d (triple-layer command filtering)
-  **Real-Time Control** - 200 Hz EtherCAT motor control with deterministic timing
-  **SROS2 Security** - Optional DDS encryption and authentication
-  **Lifecycle Management** - Managed nodes for deterministic startup/shutdown
-  **Comprehensive Testing** - 21 unit tests, 95%+ coverage, integration tests

---

## 🔧 Development

### Repository Structure

```
Navigation/
├── config/               # Configuration files
│   ├── nav2_params.yaml  # Nav2 stack parameters
│   ├── slam_params.yaml  # SLAM configuration
│   ├── ekf_params.yaml   # Sensor fusion (EKF)
│   ├── sensors_config.yaml  # Multi-sensor parameters
│   └── safety_params.yaml   # Safety-critical settings
├── launch/               # Launch files (6 files)
├── src/                  # C++ source (10 nodes)
├── include/              # Headers (6 files)
├── test/                 # Unit tests (5 test suites)
├── scripts/              # Python utilities
├── urdf/                 # Robot description (URDF/xacro)
└── docs/                 # Detailed documentation
    ├── ARCHITECTURE.md   # System architecture
    ├── API.md            # API reference
    └── SENSORS.md        # Sensor configuration
```

### Building from Source

```bash
# Clone repository
cd ~/ros2_ws/src
git clone <repo-url> navigation

# Install dependencies
cd navigation
./scripts/install_dependencies.sh

# Build
cd ~/ros2_ws
colcon build --packages-select somanet \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Run tests
colcon test --packages-select somanet
colcon test-result --all --verbose
```

### API Documentation (Doxygen)

```bash
# Install Doxygen
sudo apt install doxygen graphviz

# Generate API docs
cd ~/ros2_ws/src/navigation
doxygen Doxyfile

# View in browser
firefox docs/api/html/index.html
```

**See:** [docs/API.md](docs/API.md) for detailed API reference

---

##  Troubleshooting

### Robot Not Moving

```bash
# 1. Check command flow at each layer
ros2 topic hz /cmd_vel_nav          # Nav2 output
ros2 topic hz /cmd_vel_mux          # After mux
ros2 topic hz /cmd_vel              # After arbitrator
ros2 topic hz /wheel_cmd_safe       # After safety

# 2. Check which command source is active
ros2 topic echo /active_command_source

# 3. Check safety supervisor status
ros2 topic echo /diagnostics | grep safety

# 4. Manually send test command
ros2 topic pub /cmd_vel_manual geometry_msgs/Twist \
    "linear: {x: 0.1}" --once
```

### EtherCAT Connection Failed

```bash
# 1. Set network interface (REQUIRED before launch)
export ETHERCAT_INTERFACE=eth0

# 2. Check interface is up
ip link show $ETHERCAT_INTERFACE

# 3. Verify EtherCAT devices
sudo ethercat slaves  # Should show motor controllers

# 4. Check permissions
sudo usermod -aG ethercat $USER
sudo reboot
```

### Sensors Not Publishing

```bash
# Check all sensor topics
ros2 topic list | grep -E "scan|points|imu|camera"

# LiDAR 2D troubleshooting
ping 192.168.1.10  # Front LiDAR
ping 192.168.1.11  # Rear LiDAR

# Ouster 3D troubleshooting
ping 192.168.1.20
curl http://192.168.1.20/api/v1/sensor/metadata

# RealSense troubleshooting
realsense-viewer  # GUI tool for testing

# IMU troubleshooting
sudo i2cdetect -y 1  # Should show 0x68
```

### Navigation Not Working

```bash
# 1. Verify localization
ros2 topic echo /amcl_pose  # Should update as robot moves

# 2. Check costmaps (visualize in RViz)
ros2 topic echo /local_costmap/costmap
ros2 topic echo /global_costmap/costmap

# 3. Verify Nav2 stack is running
ros2 node list | grep nav2

# 4. Check Nav2 diagnostics
ros2 topic echo /diagnostics | grep nav2
```

**For detailed troubleshooting:** See [BUILD_GUIDE.md](BUILD_GUIDE.md)

---

##  Project Stats

- **Lines of Code:** ~15,000 (C++/Python)
- **C++ Files:** 10 nodes, 6 headers, 5 test suites
- **Launch Files:** 6 (main, nav2, slam, sensors, cartographer, slam+nav)
- **Configuration Files:** 10 YAML files
- **Test Coverage:** 95%+ (21 unit tests, integration tests)
- **Documentation:** 12 guides (3,000+ lines)
- **Safety Standards:** ISO 13849-1 Cat 3 PL d, ISO 3691-4, IEC 61508
- **Supported Sensors:** 2x LiDAR 2D, 1x LiDAR 3D, 2x RealSense, 1x IMU

---

## 🤝 Contributing

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Add tests** for new functionality
4. **Document** public APIs with Doxygen comments
5. **Run tests** (`colcon test --packages-select somanet`)
6. **Commit** changes (`git commit -m 'Add amazing feature'`)
7. **Push** to branch (`git push origin feature/amazing-feature`)
8. **Open** a Pull Request

**Coding Standards:**
- Follow [ROS2 C++ Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- Use Doxygen comments for public APIs (see [docs/API.md](docs/API.md))
- Maintain >90% test coverage
- Safety-critical code requires review by 2+ maintainers

---

##  License

Apache License 2.0 - See [LICENSE](LICENSE) for details.

---

## ⚠️ SAFETY NOTICE

**This is a safety-critical system designed for autonomous operation.**

- ⛔ **READ [SAFETY.md](SAFETY.md) BEFORE OPERATING**
- ⚠️ Emergency stop must be accessible at all times
- 🔒 Safety parameters are cryptographically signed (HMAC-SHA256)
- ✅ Velocity limits are enforced at multiple layers (defense in depth)
- 🧪 All safety functions must pass certification tests

**ISO 13849-1 Category 3, Performance Level d compliance requires:**
- Trained operators only
- Regular safety audits (see `config/maintenance_audit.yaml`)
- No modifications to safety-critical code without re-certification

---

**Version 4.1.0** | **November 2025** | **ULTRABOT Team**

**Quick Links:**
- [Architecture](docs/ARCHITECTURE.md) | [API](docs/API.md) | [Sensors](docs/SENSORS.md)
- [Navigation](README_NAV2.md) | [Safety](SAFETY.md) | [Build](BUILD_GUIDE.md)
