# NAV2 AUTONOMOUS NAVIGATION INTEGRATION
# ============================================================================
# **ULTRABOT AGV - Navigation Stack Documentation v4.0.0**
# ============================================================================

## 📋 Table of Contents
1. [Architecture Overview](#architecture-overview)
2. [Command Flow](#command-flow)
3. [Quick Start](#quick-start)
4. [Configuration](#configuration)
5. [Map Setup](#map-setup)
6. [Troubleshooting](#troubleshooting)
7. [Safety Considerations](#safety-considerations)
8. [API Reference](#api-reference)

---

## 🏗️ Architecture Overview

### System Layers (Bottom-Up)

```
┌─────────────────────────────────────────────────────────────┐
│                    NAVIGATION LAYER                          │
│  Nav2 Stack: Planner, Controller, BT Navigator, Behaviors   │
└─────────────────────────────────────────────────────────────┘
                            ↓ /cmd_vel_nav
┌─────────────────────────────────────────────────────────────┐
│               COMMAND MULTIPLEXING LAYER                     │
│  • command_mux: Consolidates autonomous sources              │
│  • command_arbitrator: Priority-based safety arbitration     │
└─────────────────────────────────────────────────────────────┘
                            ↓ /wheel_cmd_safe
┌─────────────────────────────────────────────────────────────┐
│                    SAFETY LAYER                              │
│  safety_supervisor: ISO 13849-1 compliant monitoring         │
└─────────────────────────────────────────────────────────────┘
                            ↓ /wheel_cmd_safe
┌─────────────────────────────────────────────────────────────┐
│                    HARDWARE LAYER                            │
│  somanet_driver: EtherCAT motor control + odometry           │
└─────────────────────────────────────────────────────────────┘
```

### Component Responsibilities

| Component | Purpose | Topics |
|-----------|---------|--------|
| **Nav2 Stack** | Path planning, trajectory following, behaviors | Publishes: `/cmd_vel_nav` |
| **command_mux** | Consolidates Nav2 + teleop sources | Subscribes: `/cmd_vel_nav`, `/cmd_vel_teleop` <br> Publishes: `/cmd_vel_mux` |
| **command_arbitrator** | Priority-based command selection (ISO 3691-4) | Subscribes: `/cmd_vel_mux` (as `/cmd_vel_auto`), `/cmd_vel_manual`, `/cmd_vel_emergency` <br> Publishes: `/cmd_vel` |
| **safety_supervisor** | Velocity limit enforcement, fault detection | Subscribes: `/cmd_vel` <br> Publishes: `/wheel_cmd_safe` |
| **somanet_driver** | EtherCAT motor control, odometry publishing | Subscribes: `/wheel_cmd_safe` <br> Publishes: `/odom`, `/tf (odom→base_link)` |

---

## 🔄 Command Flow

### Priority Levels (ISO 3691-4 Compliant)

```
Priority 0 (HIGHEST): Safety / Emergency Stop
         ↓
Priority 1 (HIGH):    Manual Teleop (Joystick/Keyboard)
         ↓
Priority 2 (MEDIUM):  Autonomous Navigation (Nav2)
         ↓
Priority 3 (LOW):     Other Sources
```

### Timeout Behavior (ISO 13849-1 Fail-Safe)

- **Manual timeout**: 1.0s → Releases control to lower priority
- **Nav2 timeout**: 0.5s → Publishes **ZERO velocity** (fail-safe)
- **Safety timeout**: 0.5s → **EMERGENCY STOP** (all motors disabled)

### Data Flow Diagram

```
┌─────────────┐
│   Nav2      │ Planning: 1 Hz
│  Planner    │ Control: 20 Hz
└──────┬──────┘
       │ /cmd_vel_nav
       ↓
┌─────────────┐        ┌─────────────┐
│ Teleop Joy  │────────┤             │
└─────────────┘        │             │
       │ /cmd_vel_teleop│  command_  │
┌─────────────┐        │    mux     │
│Safety Event │────────┤             │
└─────────────┘        │             │
       /cmd_vel_safety └──────┬──────┘
                              │ /cmd_vel_mux
                              ↓
                       ┌──────────────┐
                       │  command_    │
                       │  arbitrator  │ Priority-based
                       └──────┬───────┘ selection (20 Hz)
                              │ /cmd_vel
                              ↓
                       ┌──────────────┐
                       │   safety_    │ Velocity limits
                       │  supervisor  │ (50 Hz)
                       └──────┬───────┘
                              │ /wheel_cmd_safe
                              ↓
                       ┌──────────────┐
                       │  somanet_    │ EtherCAT I/O
                       │   driver     │ (200 Hz)
                       └──────┬───────┘
                              │
                         ┌────┴────┐
                         │ Motors  │
                         └─────────┘
```

---

## 🚀 Quick Start

### Prerequisites

```bash
# Install Nav2 stack (ROS2 Humble)
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install dependencies
sudo apt install ros-humble-dwb-core ros-humble-dwb-critics ros-humble-dwb-plugins

# Install map server (for localization)
sudo apt install ros-humble-nav2-map-server ros-humble-nav2-amcl
```

### Build the System

```bash
cd ~/ros2_ws
colcon build --packages-select somanet --symlink-install

source install/setup.bash
```

### Launch Navigation Stack

#### Option 1: Navigation Only (No Localization - Odom Frame)

```bash
ros2 launch somanet launch_with_nav2.py use_rviz:=True
```

**Use case**: Dead-reckoning navigation in known environments without map.

#### Option 2: Full Navigation with Localization (Map Frame)

```bash
# 1. Create/obtain a map first (using SLAM)
ros2 launch somanet launch_with_nav2.py \
  map:=/path/to/your/map.yaml \
  use_rviz:=True
```

**Use case**: Localized navigation with AMCL (Adaptive Monte Carlo Localization).

---

## ⚙️ Configuration

### Robot Parameters (`config/robot_config.yaml`)

**CRITICAL**: These MUST match your physical robot!

```yaml
# Physical dimensions
distance_wheels: 0.5    # Wheelbase (m)
wheel_diameter: 0.17    # Wheel diameter (m)

# Velocity limits (ISO 3691-4 AGV standards)
max_linear_vel: 1.0     # m/s
max_angular_vel: 1.0    # rad/s

# Safety timeouts
cmd_watchdog_timeout: 0.5  # Command timeout (s)
```

### Nav2 Parameters (`config/nav2_params.yaml`)

Key parameters to tune:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Match EtherCAT update rate
    
    FollowPath:
      max_vel_x: 1.0        # Must match robot_config.yaml
      max_vel_theta: 1.0
      acc_lim_x: 0.5        # Acceleration limit (m/s²)
      acc_lim_theta: 1.0    # Angular acceleration (rad/s²)

local_costmap:
  local_costmap:
    ros__parameters:
      width: 3              # Local costmap size (m)
      height: 3
      resolution: 0.05      # 5cm resolution
      footprint: "[ [0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25] ]"
```

### Command Mux Timeouts

```yaml
# In launch file parameters
command_mux:
  default_timeout: 0.5    # Generic timeout (s)
  teleop_timeout: 1.0     # Manual control timeout (s)
  nav2_timeout: 0.5       # Autonomous timeout (s)
```

---

## 🗺️ Map Setup

### Creating a Map (SLAM)

#### Option 1: Using slam_toolbox (Recommended)

```bash
# Install slam_toolbox
sudo apt install ros-humble-slam-toolbox

# Launch SLAM
ros2 launch slam_toolbox online_async_launch.py

# Drive the robot to map the environment (use teleop)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel_teleop

# Save the map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_warehouse
```

#### Option 2: Using Cartographer

```bash
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
# Follow Cartographer documentation for configuration
```

### Map Files Structure

```
~/maps/
├── my_warehouse.yaml   # Map metadata
└── my_warehouse.pgm    # Occupancy grid image
```

**my_warehouse.yaml** example:
```yaml
image: my_warehouse.pgm
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

---

## 🐛 Troubleshooting

### Issue 1: Nav2 Commands Not Reaching Robot

**Symptoms**: 
- `rqt_graph` shows Nav2 publishing `/cmd_vel_nav`
- Robot does not move

**Diagnosis**:
```bash
# Check topic connections
ros2 topic echo /cmd_vel_nav     # Nav2 output
ros2 topic echo /cmd_vel_mux     # command_mux output
ros2 topic echo /cmd_vel         # command_arbitrator output
ros2 topic echo /wheel_cmd_safe  # safety_supervisor output

# Check active source
ros2 topic echo /command_mux/status
ros2 topic echo /active_command_source
```

**Solutions**:
1. **Timeout Issue**: Nav2 commands timing out
   ```bash
   # Increase nav2_timeout in launch file
   nav2_timeout: 1.0  # Was 0.5
   ```

2. **Priority Preemption**: Teleop is active (higher priority)
   ```bash
   # Release joystick/keyboard
   # Or check deadman switch status
   ros2 topic echo /deadman_status
   ```

3. **Safety Limits**: Velocity exceeds safety bounds
   ```bash
   # Check safety_supervisor diagnostics
   ros2 topic echo /diagnostics | grep safety
   ```

---

### Issue 2: Robot Not Localizing (AMCL)

**Symptoms**:
- Robot position jumps erratically
- Nav2 fails to plan paths

**Diagnosis**:
```bash
# Check AMCL particle cloud
ros2 topic echo /particle_cloud

# Verify laser scan data
ros2 topic echo /scan

# Check TF tree
ros2 run tf2_tools view_frames
```

**Solutions**:
1. **Set Initial Pose**: In RVIZ, use "2D Pose Estimate" tool
2. **Increase Particles**:
   ```yaml
   amcl:
     max_particles: 5000  # Increase if large uncertainty
   ```
3. **Verify Scan Topic**: AMCL expects `/scan` by default

---

### Issue 3: Robot Gets Stuck / Won't Move

**Symptoms**: Nav2 plans path but robot oscillates or stops

**Solutions**:

1. **Reduce Footprint Inflation**:
   ```yaml
   inflation_layer:
     inflation_radius: 0.45  # Reduce from 0.55
   ```

2. **Increase Costmap Size**:
   ```yaml
   local_costmap:
     width: 5    # Increase from 3
     height: 5
   ```

3. **Tune DWB Controller**:
   ```yaml
   FollowPath:
     PathAlign.scale: 24.0     # Reduce from 32.0 (less strict)
     BaseObstacle.scale: 0.01  # Reduce from 0.02 (less conservative)
   ```

---

### Issue 4: EtherCAT / Hardware Issues

**Symptoms**: Driver fails to initialize

```bash
# Check EtherCAT interface
ip link show

# Verify permissions
sudo setcap cap_net_raw+ep install/somanet/lib/somanet/main

# Check driver logs
ros2 run somanet main --ros-args --log-level debug
```

---

## 🔒 Safety Considerations (ISO 13849-1 / ISO 3691-4)

### Emergency Stop Protocol

1. **Hardware E-Stop** (external circuit):
   - Cuts power to motors (outside ROS2)
   - Category 0 stop (uncontrolled)

2. **Software E-Stop** (command_arbitrator):
   ```bash
   ros2 topic pub /cmd_vel_emergency geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
   ```
   - Preempts all other commands
   - Category 1 stop (controlled deceleration)

### Velocity Limit Enforcement

```yaml
# config/safety_params.yaml
max_linear_velocity: 1.0   # m/s (ISO 3691-4: ≤ 2.0 m/s for indoor AGVs)
max_angular_velocity: 1.0  # rad/s
```

**Enforcement Points**:
1. `command_arbitrator`: Clamps commands to configured limits
2. `safety_supervisor`: Secondary validation (dual-channel ISO 13849-1 Cat 3)
3. `somanet_driver`: Final hardware-level limit

### Timeout Behavior (Fail-Safe)

| Component | Timeout | Fail-Safe Action |
|-----------|---------|------------------|
| command_mux | 0.5s-1.0s | Publish ZERO velocity |
| command_arbitrator | 0.5s | Switch to lower priority / ZERO |
| safety_supervisor | 0.5s | **EMERGENCY STOP** + fault event |
| somanet_driver | 0.5s | **ZERO velocity** to motors |

---

## 📡 API Reference

### Published Topics

| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| `/odom` | `nav_msgs/Odometry` | 200 Hz | Robot odometry (odom→base_link) |
| `/tf` | `tf2_msgs/TFMessage` | 200 Hz | Transform tree (odom→base_link) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | System health status |
| `/command_mux/status` | `std_msgs/String` | 20 Hz | Active command source name |
| `/active_command_source` | `std_msgs/String` | 20 Hz | Arbitrator active source |
| `/safety/fault_events` | `std_msgs/String` | Event | Safety violations/recoveries |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Nav2 autonomous commands |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | Manual teleoperation |
| `/cmd_vel_safety` | `geometry_msgs/Twist` | Safety supervisor override |
| `/cmd_vel_emergency` | `geometry_msgs/Twist` | Emergency stop commands |

### Services (Nav2)

```bash
# Set initial pose (AMCL)
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty

# Clear costmaps
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
```

### Actions (Nav2)

```bash
# Navigate to pose
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Follow waypoints
ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \
  "{poses: [{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}}}, ...]}"
```

---

## 📊 Performance Tuning

### Controller Frequency vs. Latency

```yaml
controller_server:
  controller_frequency: 20.0  # Match EtherCAT (200 Hz PDO / 10 = 20 Hz control)
```

**Trade-offs**:
- **Higher frequency (30-50 Hz)**: Smoother control, more CPU usage
- **Lower frequency (10-15 Hz)**: Less load, jerkier motion

### Planner vs. Controller Balance

```yaml
planner_server:
  expected_planner_frequency: 1.0  # 1 Hz = replan every second

controller_server:
  FollowPath:
    sim_time: 1.5  # Trajectory lookahead (seconds)
```

**Recommendation**: 
- **Planner**: 1-2 Hz (static environments), 5-10 Hz (dynamic)
- **Controller**: 10-20 Hz (differential drive), 30-50 Hz (omnidirectional)

---

## 🔧 Advanced Configuration

### Custom Behavior Trees

Nav2 uses Behavior Trees (BT) for high-level logic. Default: Navigate w/ Replanning.

**Custom BT XML**:
```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "/path/to/custom_bt.xml"
```

**Example**: Add "Wait for Human Clearance" behavior before navigation.

### Multi-Robot Coordination

For fleets, integrate with:
- **RMF (Robotics Middleware Framework)**: Traffic management
- **fleet_adapter**: Multi-robot task allocation

```bash
sudo apt install ros-humble-rmf-*
```

---

## 📚 Additional Resources

- **Nav2 Official Docs**: https://navigation.ros.org/
- **ISO 13849-1 Summary**: See `SAFETY.md`
- **ISO 3691-4 AGV Standards**: Industrial truck safety requirements
- **Differential Drive Tuning**: https://navigation.ros.org/tuning/index.html

---

## 🆘 Support

**Bug Reports**: Open issue with logs:
```bash
ros2 run somanet main --ros-args --log-level debug 2>&1 | tee debug.log
```

**Configuration Help**: Check diagnostics:
```bash
ros2 topic echo /diagnostics
```

**Community**: ROS Discourse (https://discourse.ros.org/)

---

**Version**: 4.0.0 (Nav2 Integration)  
**Last Updated**: 2025-11-05  
**Compliance**: ISO 13849-1 Cat 3 PL d, ISO 3691-4, IEC 61508 SIL 2
