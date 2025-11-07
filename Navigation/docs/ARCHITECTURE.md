# ULTRABOT AGV - System Architecture

**Version 4.1.0** | **ROS2 Humble Architecture Documentation**

> Comprehensive architectural overview of the ULTRABOT AGV navigation and control system

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Component Architecture](#component-architecture)
3. [Data Flow](#data-flow)
4. [Sensor Integration](#sensor-integration)
5. [State Machines](#state-machines)
6. [Safety Architecture](#safety-architecture)
7. [ROS2 Graph](#ros2-graph)

---

## System Overview

### Layered Architecture

The ULTRABOT AGV follows a **5-layer safety-critical architecture** designed to comply with ISO 13849-1 Category 3 Performance Level d:

```
┌──────────────────────────────────────────────────────────────────┐
│                        APPLICATION LAYER                          │
│  • Nav2 Stack (Planning, Control, Recovery)                      │
│  • SLAM (Cartographer, SLAM Toolbox)                             │
│  • Sensor Fusion (EKF, LiDAR Merger)                             │
└──────────────────────────────────────────────────────────────────┘
                                 ↓
┌──────────────────────────────────────────────────────────────────┐
│                     COMMAND MULTIPLEXING LAYER                    │
│  • command_mux: Autonomous vs Teleop source selection            │
│  • command_arbitrator: 4-level priority arbitration              │
└──────────────────────────────────────────────────────────────────┘
                                 ↓
┌──────────────────────────────────────────────────────────────────┐
│                         SAFETY LAYER                              │
│  • safety_supervisor: ISO 13849-1 compliant monitoring           │
│  • Parameter validation (HMAC-SHA256)                            │
│  • Watchdog timer (500ms timeout)                                │
└──────────────────────────────────────────────────────────────────┘
                                 ↓
┌──────────────────────────────────────────────────────────────────┐
│                        ODOMETRY LAYER                             │
│  • odometry_calculator: Differential drive kinematics            │
│  • robot_localization (EKF): Sensor fusion                       │
│  • TF Publisher: Transform tree management                       │
└──────────────────────────────────────────────────────────────────┘
                                 ↓
┌──────────────────────────────────────────────────────────────────┐
│                        HARDWARE LAYER                             │
│  • somanet_driver: EtherCAT motor control (200 Hz)               │
│  • Sensor drivers: LiDAR, Cameras, IMU                           │
└──────────────────────────────────────────────────────────────────┘
```

### Key Design Principles

1. **Defense in Depth**: Multiple safety layers with independent validation
2. **Fail-Safe Design**: System defaults to safe state on any failure
3. **Priority-Based Control**: Emergency > Manual > Autonomous > Test
4. **Sensor Redundancy**: Multi-sensor fusion with graceful degradation
5. **Real-Time Performance**: Deterministic control loops (200 Hz motor control)

---

## Component Architecture

### Core Components

#### 1. Motion Control Pipeline

```
┌────────────────────┐
│   Nav2 Planner     │  Global path planning (A*, NavFn)
└─────────┬──────────┘
          │ /plan
          ↓
┌────────────────────┐
│  Nav2 Controller   │  Local trajectory following (DWB)
└─────────┬──────────┘
          │ /cmd_vel_nav
          ↓
┌────────────────────┐
│   command_mux      │  Source selection (autonomous vs teleop)
└─────────┬──────────┘
          │ /cmd_vel_mux
          ↓
┌────────────────────┐
│ command_arbitrator │  Priority arbitration (4 levels)
└─────────┬──────────┘
          │ /cmd_vel
          ↓
┌────────────────────┐
│ safety_supervisor  │  Velocity limits, collision check
└─────────┬──────────┘
          │ /cmd_vel_safe
          ↓
┌────────────────────┐
│  somanet_driver    │  EtherCAT motor control (200 Hz)
└────────────────────┘
```

#### 2. Sensor Processing Pipeline

```
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│  LiDAR 2D    │  │  LiDAR 2D    │  │  LiDAR 3D    │
│  (Front)     │  │  (Rear)      │  │  (Ouster)    │
└──────┬───────┘  └──────┬───────┘  └──────┬───────┘
       │                 │                 │
       │ /scan_front     │ /scan_rear      │ /points
       │                 │                 │
       └─────────┬───────┘                 │
                 ↓                         │
       ┌─────────────────┐                 │
       │ laserscan_multi │                 │
       │     _merger     │                 │
       └────────┬────────┘                 │
                │ /scan                    │
                │                          │
                ├──────────────────────────┤
                ↓                          ↓
       ┌─────────────────────────────────────┐
       │        Nav2 Costmap Layers          │
       │  (Obstacle, Inflation, Voxel)       │
       └─────────────────────────────────────┘

┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│  RealSense   │  │  RealSense   │  │     IMU      │
│ (Front Left) │  │ (Front Right)│  │   (MPU6050)  │
└──────┬───────┘  └──────┬───────┘  └──────┬───────┘
       │ /depth_left     │ /depth_right     │ /imu/data
       │                 │                  │
       ↓                 ↓                  ↓
  ┌────────────────────────────────────────────┐
  │       robot_localization (EKF)             │
  │  Fuses: Wheel odometry + IMU               │
  └────────────────────────────────────────────┘
                      ↓ /odometry/filtered
```

#### 3. Transform Tree (TF)

```
map
 └─ odom
     └─ base_footprint
         └─ base_link
             ├─ wheel_left_link
             ├─ wheel_right_link
             ├─ scan_front_link (LiDAR 2D front)
             ├─ scan_rear_link (LiDAR 2D rear)
             ├─ ouster_link (LiDAR 3D)
             ├─ camera_front_left_link
             │   ├─ camera_front_left_depth_frame
             │   └─ camera_front_left_color_frame
             ├─ camera_front_right_link
             │   ├─ camera_front_right_depth_frame
             │   └─ camera_front_right_color_frame
             └─ imu_link
```

**Frame Descriptions:**
- `map`: Global reference frame (SLAM origin)
- `odom`: Odometry reference frame (continuous, drift-prone)
- `base_footprint`: Ground projection of robot center
- `base_link`: Robot geometric center (0.15m above ground)
- Sensor frames: Positioned according to physical mounting locations

---

## Data Flow

### Command Flow (Write Path)

```
┌─────────────────────────────────────────────────────────────────┐
│                         INPUT SOURCES                            │
├──────────────┬──────────────┬──────────────┬──────────────┬─────┤
│ Nav2 Stack   │ Teleop Joy   │ Teleop KB    │ Emergency    │Test │
│ /cmd_vel_nav │/cmd_vel_teleop│/cmd_vel_teleop│/cmd_vel_emer │...│
└──────┬───────┴──────┬───────┴──────┬───────┴──────┬───────┴─────┘
       │              │              │              │
       │ P100         │ P200         │ P255         │ P50
       ↓              ↓              ↓              ↓
┌──────────────────────────────────────────────────────────────────┐
│                     command_arbitrator                            │
│  Logic: Select highest priority non-zero command                 │
│  Priority: Emergency(255) > Manual(200) > Auto(100) > Test(50)   │
└─────────────────────────────┬────────────────────────────────────┘
                              │ /cmd_vel (selected command)
                              ↓
┌──────────────────────────────────────────────────────────────────┐
│                     safety_supervisor                             │
│  Checks:                                                          │
│   • Velocity limits (max 1.0 m/s linear, 1.0 rad/s angular)      │
│   • Acceleration limits                                           │
│   • Watchdog timeout (500ms)                                      │
│   • Collision prediction (if enabled)                             │
└─────────────────────────────┬────────────────────────────────────┘
                              │ /cmd_vel_safe
                              ↓
┌──────────────────────────────────────────────────────────────────┐
│                     somanet_driver                                │
│  • Converts Twist to wheel velocities (differential drive)       │
│  • Sends PDO frames via EtherCAT (200 Hz)                        │
│  • Motor control: Position/Velocity modes                        │
└──────────────────────────────────────────────────────────────────┘
```

### Sensor Flow (Read Path)

```
┌──────────────────────────────────────────────────────────────────┐
│                        SENSOR HARDWARE                            │
├──────────┬──────────┬──────────┬──────────┬──────────┬──────────┤
│ Wheel    │ LiDAR 2D │ LiDAR 3D │ RealSense│ RealSense│   IMU    │
│ Encoders │ (Front)  │ (Ouster) │  (Left)  │  (Right) │(MPU6050) │
└────┬─────┴────┬─────┴────┬─────┴────┬─────┴────┬─────┴────┬─────┘
     │          │          │          │          │          │
     │ 200Hz    │ 10Hz     │ 10Hz     │ 30Hz     │ 30Hz     │ 100Hz
     ↓          ↓          ↓          ↓          ↓          ↓
┌────┴──────────────────────────────────────────────────────┴──────┐
│                   SENSOR PREPROCESSING                            │
│  • Wheel: odometry_calculator (diff drive kinematics)            │
│  • LiDAR 2D: laserscan_multi_merger (front+rear fusion)          │
│  • LiDAR 3D: pointcloud_to_laserscan (optional 2D projection)    │
│  • RealSense: pointcloud_to_laserscan (depth to laserscan)       │
│  • IMU: Raw data (no preprocessing)                              │
└────┬──────────────────────────────────────────────────────┬──────┘
     │                                                       │
     │ /odom (200 Hz)                                       │ /imu/data
     ↓                                                       ↓
┌────────────────────────────────────────────────────────────────┐
│               robot_localization (EKF)                         │
│  Fuses: /odom (wheel encoders) + /imu/data                    │
│  Output: /odometry/filtered (50 Hz)                            │
│  Config: config/ekf_params.yaml                                │
└────────────────────────────┬───────────────────────────────────┘
                             │
                             ↓
┌────────────────────────────────────────────────────────────────┐
│                          Nav2 Stack                            │
│  Localization: AMCL (particle filter on /scan)                 │
│  Mapping: Cartographer/SLAM Toolbox                            │
│  Planning: NavFn planner on costmap                            │
│  Control: DWB local planner                                    │
└────────────────────────────────────────────────────────────────┘
```

---

## Sensor Integration

### Multi-Sensor Fusion Strategy

The ULTRABOT uses a **hybrid sensor fusion** approach:

#### 1. LiDAR Integration

**2D LiDAR Fusion (Front + Rear):**
- **Driver**: `sick_scan_xd` (or `urg_node` for Hokuyo)
- **Fusion**: `laserscan_multi_merger` node
- **Purpose**: 360° obstacle detection for costmaps
- **Configuration**: `config/sensors_config.yaml`

```yaml
# Front LiDAR: 270° FOV, 0.25° resolution
# Rear LiDAR: 270° FOV, 0.25° resolution
# Merged output: /scan topic (360° coverage)
```

**3D LiDAR (Ouster OS1-64):**
- **Driver**: `ros2_ouster` package
- **Output**: PointCloud2 on `/points` topic
- **Uses**:
  1. 3D obstacle detection (elevation mapping)
  2. Voxel layer in Nav2 costmap
  3. Optional 2D projection for redundancy

#### 2. Vision Integration

**RealSense D455 Cameras (2x Front-Facing):**
- **Driver**: `realsense2_camera` package
- **Outputs**:
  - Color images: `/camera_*/color/image_raw`
  - Depth maps: `/camera_*/depth/image_rect_raw`
  - Point clouds: `/camera_*/depth/color/points`
- **Uses**:
  1. Depth-to-LaserScan conversion for costmap
  2. Visual odometry (optional, currently disabled)
  3. Object detection (future implementation)

#### 3. Inertial Measurement

**IMU (MPU6050):**
- **Driver**: `mpu6050driver` or `ros2_mpu6050`
- **Output**: `/imu/data` (100 Hz)
- **Fused Data**:
  - Angular velocity (yaw rate)
  - Linear acceleration (x, y)
  - Orientation (quaternion)
- **Integration**: robot_localization EKF (see `config/ekf_params.yaml`)

#### 4. Time Synchronization

**Critical for Multi-Sensor Fusion:**
- **Mechanism**: NTP (network) or Chrony (hardware PPS)
- **Requirements**: <10ms skew between sensors
- **Monitoring**: `/diagnostics` topic reports sync status
- **Guide**: See `TIME_SYNC_GUIDE.md`

### Sensor Observation Sources (Nav2 Costmaps)

**Local Costmap (5m radius, high resolution):**
```yaml
observation_sources: scan scan_realsense_left scan_realsense_right ouster_pointcloud
```

**Global Costmap (50m radius, lower resolution):**
```yaml
observation_sources: scan ouster_pointcloud
```

---

## State Machines

### Lifecycle Node States (All Nodes)

ULTRABOT uses ROS2 **managed lifecycle nodes** for deterministic startup/shutdown:

```
┌─────────────┐
│  UNCONFIGURED
└──────┬──────┘
       │ configure()
       ↓
┌─────────────┐
│  INACTIVE   │ ←──────────┐
└──────┬──────┘            │
       │ activate()        │ deactivate()
       ↓                   │
┌─────────────┐            │
│   ACTIVE    │ ───────────┘
└──────┬──────┘
       │ cleanup() or shutdown()
       ↓
┌─────────────┐
│  FINALIZED  │
└─────────────┘
```

**Lifecycle-Managed Nodes:**
- `somanet_driver`
- `command_arbitrator`
- `safety_supervisor`
- `command_mux`
- `odometry_calculator`

### Command Arbitrator State

```
┌─────────────────────────────────────────────────────────┐
│                   IDLE STATE                            │
│  • No commands received                                 │
│  • Output: zero velocity                                │
└─────────────────────┬───────────────────────────────────┘
                      │ Non-zero command on any priority
                      ↓
┌─────────────────────────────────────────────────────────┐
│                 ARBITRATING STATE                       │
│  • Evaluate all active command sources                  │
│  • Select highest priority non-zero command             │
│  • Publish selected command to /cmd_vel                 │
│  • Update /active_command_source                        │
└─────────────────────┬───────────────────────────────────┘
                      │ 500ms timeout (no new commands)
                      ↓
┌─────────────────────────────────────────────────────────┐
│                  TIMEOUT STATE                          │
│  • All commands assumed zero (watchdog triggered)       │
│  • Output: zero velocity (SAFE STOP)                    │
│  • Publish timeout diagnostic                           │
└─────────────────────┬───────────────────────────────────┘
                      │ New command received
                      ↓
                   ARBITRATING
```

### Safety Supervisor State

```
┌─────────────────────────────────────────────────────────┐
│                   NORMAL MODE                           │
│  • Validate velocity limits                             │
│  • Monitor watchdog                                      │
│  • Pass through commands                                │
└─────────────────────┬───────────────────────────────────┘
                      │ Violation detected
                      ↓
┌─────────────────────────────────────────────────────────┐
│                 FAULT STATE                             │
│  • Output: zero velocity                                │
│  • Publish diagnostic error                             │
│  • Conditions:                                          │
│    - Velocity > 1.0 m/s                                 │
│    - Watchdog timeout (500ms)                           │
│    - Invalid command (NaN, Inf)                         │
└─────────────────────┬───────────────────────────────────┘
                      │ Fault cleared
                      ↓
                   NORMAL MODE
```

---

## Safety Architecture

### ISO 13849-1 Category 3, Performance Level d Compliance

#### Triple-Layer Command Filtering

```
┌──────────────────────────────────────────────────────────────┐
│                  LAYER 1: COMMAND ARBITRATOR                 │
│  Function: Priority-based source selection                   │
│  Safety: Prevents conflicting commands                       │
│  Standard: ISO 3691-4 (AGV safety)                           │
└──────────────────────────┬───────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────┐
│                  LAYER 2: SAFETY SUPERVISOR                  │
│  Function: Velocity limiting, watchdog, validation           │
│  Safety: Enforces velocity bounds (1.0 m/s max)              │
│  Standard: ISO 13849-1 Cat 3 PL d                            │
└──────────────────────────┬───────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────┐
│                  LAYER 3: MOTOR CONTROLLER                   │
│  Function: Hardware-level safety (STO, Safe Torque Off)      │
│  Safety: Emergency stop input (digital I/O)                  │
│  Standard: IEC 61800-5-2 (STO function)                      │
└──────────────────────────────────────────────────────────────┘
```

#### Parameter Authentication

**HMAC-SHA256 Validation:**
- Safety-critical parameters are hashed and signed
- Configuration file: `config/certified_safety_params.yaml`
- Hash generation: `scripts/generate_certification_hash.py`
- Validation node: `certified_params_validator`

**Protected Parameters:**
- Maximum velocities (linear, angular)
- Acceleration limits
- Watchdog timeout
- Emergency stop configuration

#### Fault Detection & Response

| Fault Type | Detection | Response | Recovery |
|------------|-----------|----------|----------|
| **Watchdog Timeout** | No command for 500ms | Zero velocity output | Auto-recover on new command |
| **Velocity Violation** | v > 1.0 m/s | Clamp to limit + diagnostic | Auto-recover |
| **Invalid Command** | NaN/Inf values | Zero velocity output | Manual reset required |
| **EtherCAT Lost** | PDO timeout | Safe stop, disable motors | Requires driver restart |
| **Parameter Tamper** | HMAC mismatch | Refuse to start node | Requires re-certification |

---

## ROS2 Graph

### Key Topics

| Topic | Type | Frequency | Publisher | Subscriber(s) |
|-------|------|-----------|-----------|---------------|
| `/cmd_vel_nav` | Twist | Variable | nav2_controller | command_mux |
| `/cmd_vel_teleop` | Twist | 10 Hz | teleop_joy/teleop_keyboard | command_mux |
| `/cmd_vel_emergency` | Twist | On-demand | E-stop button handler | command_arbitrator |
| `/cmd_vel_mux` | Twist | Variable | command_mux | command_arbitrator |
| `/cmd_vel` | Twist | Variable | command_arbitrator | safety_supervisor |
| `/cmd_vel_safe` | Twist | Variable | safety_supervisor | somanet_driver |
| `/odom` | Odometry | 200 Hz | odometry_calculator | robot_localization, nav2 |
| `/odometry/filtered` | Odometry | 50 Hz | robot_localization (EKF) | nav2_controller |
| `/scan` | LaserScan | 10 Hz | laserscan_multi_merger | nav2_costmap, slam |
| `/points` | PointCloud2 | 10 Hz | ouster_driver | nav2_costmap (voxel layer) |
| `/imu/data` | Imu | 100 Hz | imu_driver | robot_localization |
| `/diagnostics` | DiagnosticArray | 1 Hz | All nodes | diagnostics_aggregator |
| `/active_command_source` | String | On change | command_arbitrator | Monitoring tools |

### Services

| Service | Type | Provider | Purpose |
|---------|------|----------|---------|
| `/emergency_stop` | Trigger | safety_supervisor | Trigger safe stop |
| `/reset_safety` | Trigger | safety_supervisor | Clear safety faults |
| `/switch_controller` | SwitchController | controller_manager | Switch control modes |

### Parameters (Dynamic Reconfigurable)

**Safety Parameters (READ-ONLY, certified):**
- `max_velocity_linear`: 1.0 m/s
- `max_velocity_angular`: 1.0 rad/s
- `watchdog_timeout`: 0.5 s

**Tunable Parameters:**
- Nav2 controller parameters (DWB)
- Costmap inflation radius
- AMCL particle count
- EKF process noise

---

## Performance Characteristics

### Real-Time Performance

| Component | Cycle Time | Jitter | Priority |
|-----------|------------|--------|----------|
| EtherCAT PDO | 5 ms (200 Hz) | <500 µs | SCHED_FIFO 90 |
| Odometry Calc | 5 ms (200 Hz) | <1 ms | SCHED_FIFO 80 |
| Safety Supervisor | 10 ms (100 Hz) | <2 ms | SCHED_FIFO 70 |
| Command Arbitrator | 20 ms (50 Hz) | <5 ms | Normal |
| Nav2 Controller | 100 ms (10 Hz) | <10 ms | Normal |
| Sensor Drivers | Varies (10-100 Hz) | Variable | Normal |

### Resource Usage (Typical)

- **CPU**: 30-40% (4-core ARM Cortex-A72)
- **RAM**: 2-3 GB (ROS2 + Nav2 + SLAM)
- **Network**: 10-20 Mbps (sensor data + Nav2)
- **Disk I/O**: <10 MB/s (logging)

---

## References

- **Safety Standards**: ISO 13849-1, ISO 3691-4, IEC 61508, IEC 61800-5-2
- **ROS2 Documentation**: https://docs.ros.org/en/humble/
- **Nav2 Documentation**: https://navigation.ros.org/
- **EtherCAT**: IEC 61158 Type 12

**Version 4.1.0** | **Architecture Documentation** | **ULTRABOT Team**
