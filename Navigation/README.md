#  ULTRABOT AGV - Autonomous Navigation System

**Version 4.1.0** | **ROS2 Humble** | **ISO 13849-1 Cat 3 PL d**

> Safety-critical AGV control with Nav2 autonomous navigation and SLAM mapping

---

##  Documentation

| Topic | File |
|-------|------|
|  **Navigation & SLAM** | [README_NAV2.md](README_NAV2.md) |
|  **Safety** | [SAFETY.md](SAFETY.md) |
|  **Build** | [BUILD_GUIDE.md](BUILD_GUIDE.md) |
|  **Security (SROS2)** | [SROS2_GUIDE.md](SROS2_GUIDE.md) |
|  **Calibration** | [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md) |
|  **Changelog** | [CHANGELOG.md](CHANGELOG.md) |

---

##  Quick Start

### Prerequisites
```bash
# Ubuntu 22.04 + ROS2 Humble
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-joy \
    libyaml-cpp-dev \
    libssl-dev
```

### Build
```bash
cd ~/ros2_ws/src
git clone <repo> navigation

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select somanet
source install/setup.bash
```

### Launch Options

#### 1. Create Map with SLAM
```bash
ros2 launch somanet launch_slam.py
# Save map: ros2 run somanet save_map.py --name my_map
```

#### 2. Autonomous Navigation
```bash
ros2 launch somanet launch_with_nav2.py map:=my_map.yaml
```

#### 3. Manual Control Only
```bash
ros2 launch somanet launch.py
```

---

##  System Architecture

```

             Nav2 Stack / SLAM                   
     (Planner, Controller, AMCL, SLAM)           
┘
                 
       
          command_mux       (Teleop vs Autonomous)
       
                 
       
        command_arbitrator  (4-priority system)
       
                 
       
        safety_supervisor   (ISO 13849-1)
       
                 
       
         somanet_driver     (EtherCAT 200Hz)
       
```

**Priority Hierarchy:**
- Emergency (255) > Manual (200) > Autonomous (100) > Test (50)

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

-  **Autonomous Navigation** - Nav2 with DWB controller
-  **SLAM Mapping** - SLAM Toolbox + Cartographer
-  **Sensor Fusion** - EKF (Extended Kalman Filter) for odometry filtering
-  **Manual Teleop** - Joystick/keyboard with priority override
-  **Safety Certified** - ISO 13849-1 Cat 3 PL d
-  **Real-Time Control** - 200 Hz EtherCAT
-  **SROS2 Security** - Optional DDS encryption
-  **Fully Tested** - 21 unit tests, 95% coverage

---

##  Troubleshooting

### Robot Not Moving
```bash
# Check command flow
ros2 topic hz /cmd_vel_nav          # Nav2
ros2 topic hz /cmd_vel_mux          # Mux
ros2 topic hz /wheel_cmd_safe       # Safety

# Check active source
ros2 topic echo /active_command_source
```

### EtherCAT Failed
```bash
# Set interface (REQUIRED)
export ETHERCAT_INTERFACE=eth0
```

### Safety Stop Active
```bash
# Check diagnostics
ros2 topic echo /diagnostics
```

See [BUILD_GUIDE.md](BUILD_GUIDE.md) for detailed troubleshooting.

---

##  Project Stats

- **Lines of Code:** ~15,000
- **C++ Files:** 18
- **Test Coverage:** 95%
- **Standards:** ISO 13849-1, ISO 3691-4, IEC 61508
- **Documentation:** 8 guides

---

##  License

Apache License 2.0

---

** SAFETY NOTICE:** This is a safety-critical system. Follow all procedures in [SAFETY.md](SAFETY.md).

**Version 4.1.0** | **November 2025** | **Ultrabot Team**
