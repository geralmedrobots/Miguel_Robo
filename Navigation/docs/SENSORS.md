# ULTRABOT AGV - Sensor Configuration Guide

**Version 4.1.0** | **Multi-Sensor Integration Documentation**

> Complete guide for configuring and calibrating the 5-category sensor suite

---

## Table of Contents

1. [Sensor Suite Overview](#sensor-suite-overview)
2. [LiDAR 2D Configuration](#lidar-2d-configuration)
3. [LiDAR 3D Configuration](#lidar-3d-configuration)
4. [RealSense Cameras](#realsense-cameras)
5. [IMU Configuration](#imu-configuration)
6. [Sensor Fusion](#sensor-fusion)
7. [Time Synchronization](#time-synchronization)
8. [Calibration Procedures](#calibration-procedures)
9. [Troubleshooting](#troubleshooting)

---

## Sensor Suite Overview

The ULTRABOT AGV uses a **5-category sensor suite** for robust autonomous navigation:

```
                    ┌─────────────┐
                    │  Ouster 3D  │ (Top-mounted, 360°)
                    │   OS1-64    │
                    └─────────────┘
                          
     ┌────────────┐                  ┌────────────┐
     │ RealSense  │                  │ RealSense  │
     │  D455 (L)  │                  │  D455 (R)  │
     └────────────┘                  └────────────┘
           
   ┌──────────────────────────────────────────┐
   │           ULTRABOT Robot Base            │
   │                                          │
   │  ┌──────────┐    ┌──────┐    ┌─────────┐│
   │  │ LiDAR 2D │    │ IMU  │    │LiDAR 2D ││
   │  │  (Front) │    │MPU   │    │ (Rear)  ││
   │  │          │    │6050  │    │         ││
   │  └──────────┘    └──────┘    └─────────┘│
   └──────────────────────────────────────────┘
```

### Sensor Specifications

| Sensor | Model | FOV | Range | Frequency | Purpose |
|--------|-------|-----|-------|-----------|---------|
| **LiDAR 2D Front** | SICK TiM571 / Hokuyo UST-10LX | 270° | 0.1-10m | 15 Hz | Front obstacle detection |
| **LiDAR 2D Rear** | SICK TiM571 / Hokuyo UST-10LX | 270° | 0.1-10m | 15 Hz | Rear obstacle detection |
| **LiDAR 3D** | Ouster OS1-64 | 360° x 45° | 0.5-120m | 10 Hz | 3D mapping, elevation |
| **Camera Left** | Intel RealSense D455 | 87° x 58° | 0.6-6m | 30 Hz | Depth perception, left side |
| **Camera Right** | Intel RealSense D455 | 87° x 58° | 0.6-6m | 30 Hz | Depth perception, right side |
| **IMU** | MPU6050 / MPU9250 | N/A | N/A | 100 Hz | Inertial measurement |

### ROS2 Topics Published

```yaml
# LiDAR 2D
/scan_front:         sensor_msgs/msg/LaserScan  (15 Hz)
/scan_rear:          sensor_msgs/msg/LaserScan  (15 Hz)
/scan:               sensor_msgs/msg/LaserScan  (15 Hz, merged)

# LiDAR 3D
/points:             sensor_msgs/msg/PointCloud2 (10 Hz)
/scan_ouster:        sensor_msgs/msg/LaserScan   (10 Hz, 2D projection)

# RealSense (Left)
/camera_front_left/color/image_raw:        sensor_msgs/msg/Image
/camera_front_left/depth/image_rect_raw:   sensor_msgs/msg/Image
/camera_front_left/depth/color/points:     sensor_msgs/msg/PointCloud2
/scan_realsense_left:                      sensor_msgs/msg/LaserScan

# RealSense (Right)
/camera_front_right/color/image_raw:       sensor_msgs/msg/Image
/camera_front_right/depth/image_rect_raw:  sensor_msgs/msg/Image
/camera_front_right/depth/color/points:    sensor_msgs/msg/PointCloud2
/scan_realsense_right:                     sensor_msgs/msg/LaserScan

# IMU
/imu/data:           sensor_msgs/msg/Imu (100 Hz)
```

---

## LiDAR 2D Configuration

### Hardware Setup

**Front LiDAR:**
- Position: Front-center, 0.15m above ground
- Orientation: Forward-facing, 0° yaw
- Connection: Ethernet (192.168.1.10) or USB
- Frame ID: `scan_front_link`

**Rear LiDAR:**
- Position: Rear-center, 0.15m above ground
- Orientation: Rear-facing, 180° yaw
- Connection: Ethernet (192.168.1.11) or USB
- Frame ID: `scan_rear_link`

### Configuration File: `config/sensors_config.yaml`

```yaml
lidar_2d_front:
  device_type: "sick_tim571"  # or "hokuyo_ust10lx"
  ip_address: "192.168.1.10"
  port: 2111
  frame_id: "scan_front_link"
  scan_topic: "/scan_front"
  
  # Scan parameters
  angle_min: -2.35619  # -135° in radians
  angle_max: 2.35619   # +135° in radians (270° total FOV)
  angle_increment: 0.00436332  # 0.25° in radians
  range_min: 0.1       # meters
  range_max: 10.0      # meters
  
  # Filtering
  enable_range_filter: true
  range_filter_min: 0.1
  range_filter_max: 9.5
  enable_angle_filter: false

lidar_2d_rear:
  device_type: "sick_tim571"
  ip_address: "192.168.1.11"
  port: 2111
  frame_id: "scan_rear_link"
  scan_topic: "/scan_rear"
  
  # Same parameters as front
  angle_min: -2.35619
  angle_max: 2.35619
  angle_increment: 0.00436332
  range_min: 0.1
  range_max: 10.0

# Merge front + rear into single 360° scan
laserscan_merger:
  destination_frame: "base_link"
  cloud_destination_topic: "/merged_cloud"
  scan_destination_topic: "/scan"
  laserscan_topics:
    - /scan_front
    - /scan_rear
  angle_min: -3.14159  # -180° (full 360°)
  angle_max: 3.14159   # +180°
  angle_increment: 0.00436332  # 0.25°
  range_min: 0.1
  range_max: 10.0
  merge_method: "average"  # or "min", "max"
```

### Launch Configuration

```python
# In launch/launch_sensors.py
lidar_2d_front_node = Node(
    package='sick_scan_xd',  # or urg_node for Hokuyo
    executable='sick_generic_caller',
    name='lidar_2d_front',
    parameters=[sensors_config, {
        'scanner_type': 'sick_tim_5xx',
        'hostname': '192.168.1.10',
        'frame_id': 'scan_front_link'
    }],
    remappings=[('scan', '/scan_front')],
    condition=IfCondition(LaunchConfiguration('enable_lidar_2d'))
)

# Merger node
laserscan_merger_node = Node(
    package='ira_laser_tools',
    executable='laserscan_multi_merger',
    name='laserscan_merger',
    parameters=[sensors_config],
    condition=IfCondition(LaunchConfiguration('enable_lidar_2d'))
)
```

### Driver Installation

```bash
# SICK TiM571
sudo apt install ros-humble-sick-scan-xd

# Hokuyo UST-10LX
sudo apt install ros-humble-urg-node

# LaserScan Merger
sudo apt install ros-humble-ira-laser-tools
```

---

## LiDAR 3D Configuration

### Hardware Setup

**Ouster OS1-64:**
- Position: Top-center, 0.5m above ground
- Orientation: Upright (standard mounting)
- Connection: Ethernet (192.168.1.20)
- Frame ID: `ouster_link`
- Vertical Resolution: 64 beams
- Vertical FOV: 45° (+22.5° to -22.5°)
- Horizontal FOV: 360°

### Configuration File: `config/sensors_config.yaml`

```yaml
ouster_lidar:
  sensor_hostname: "192.168.1.20"
  lidar_mode: "1024x10"  # 1024 columns, 10 Hz
  frame_id: "ouster_link"
  
  # Topics
  points_topic: "/points"
  imu_topic: "/ouster/imu"  # Ouster has built-in IMU
  
  # Point cloud filtering
  enable_voxel_filter: true
  voxel_size: 0.05  # 5cm voxels (reduce density)
  
  # 2D projection (optional)
  enable_2d_projection: true
  projection_topic: "/scan_ouster"
  projection_min_height: -0.2  # meters below sensor
  projection_max_height: 0.3   # meters above sensor
  
  # Intensity normalization
  min_intensity: 0
  max_intensity: 2000
```

### Launch Configuration

```python
# In launch/launch_sensors.py
ouster_driver_node = Node(
    package='ros2_ouster',
    executable='ouster_driver',
    name='ouster_driver',
    parameters=[sensors_config, {
        'lidar_ip': '192.168.1.20',
        'computer_ip': '192.168.1.1',
        'lidar_mode': '1024x10',
        'imu_port': 7503,
        'lidar_port': 7502
    }],
    condition=IfCondition(LaunchConfiguration('enable_lidar_3d'))
)

# Optional: PointCloud to LaserScan conversion
pointcloud_to_laserscan_node = Node(
    package='pointcloud_to_laserscan',
    executable='pointcloud_to_laserscan_node',
    name='ouster_to_laserscan',
    remappings=[
        ('cloud_in', '/points'),
        ('scan', '/scan_ouster')
    ],
    parameters=[{
        'min_height': -0.2,
        'max_height': 0.3,
        'angle_min': -3.14159,
        'angle_max': 3.14159,
        'range_min': 0.5,
        'range_max': 120.0,
        'use_inf': True
    }],
    condition=IfCondition(LaunchConfiguration('enable_lidar_3d'))
)
```

### Driver Installation

```bash
# Ouster ROS2 driver
sudo apt install ros-humble-ros2-ouster

# PointCloud to LaserScan converter
sudo apt install ros-humble-pointcloud-to-laserscan

# PCL for filtering (optional)
sudo apt install ros-humble-pcl-conversions
```

### Network Configuration

Configure static IP for Ouster sensor:

```bash
# Find sensor on network
sudo nmap -sP 192.168.1.0/24

# Configure sensor IP (via web interface or API)
curl -X POST http://192.168.1.20/api/v1/network/ipv4 \
  -H "Content-Type: application/json" \
  -d '{"addr": "192.168.1.20", "mask": "255.255.255.0", "gateway": "192.168.1.1"}'
```

---

## RealSense Cameras

### Hardware Setup

**Left Camera (Front-Left):**
- Position: 0.2m left of center, 0.4m above ground
- Orientation: Forward-facing, 15° inward tilt
- Connection: USB 3.2
- Serial Number: (read from camera)
- Frame ID: `camera_front_left_link`

**Right Camera (Front-Right):**
- Position: 0.2m right of center, 0.4m above ground
- Orientation: Forward-facing, 15° inward tilt
- Connection: USB 3.2
- Serial Number: (read from camera)
- Frame ID: `camera_front_right_link`

### Configuration File: `config/sensors_config.yaml`

```yaml
realsense_front_left:
  serial_no: ""  # Auto-detect or specify: "12345678"
  usb_port_id: ""  # Auto-detect
  device_type: "d455"
  
  # Frame IDs
  base_frame_id: "camera_front_left_link"
  depth_frame_id: "camera_front_left_depth_frame"
  color_frame_id: "camera_front_left_color_frame"
  
  # Camera configuration
  enable_depth: true
  enable_color: true
  enable_infra1: false
  enable_infra2: false
  enable_pointcloud: true
  
  # Resolution and framerate
  depth_width: 640
  depth_height: 480
  depth_fps: 30
  color_width: 640
  color_height: 480
  color_fps: 30
  
  # Depth settings
  depth_module.emitter_enabled: 1  # Laser projector ON
  depth_module.exposure: 8500
  depth_module.gain: 16
  
  # Advanced depth
  depth_module.visual_preset: 3  # High accuracy
  depth_module.enable_auto_exposure: true
  
  # Filters
  filters: "spatial,temporal,hole_filling"
  
  # Align depth to color
  align_depth: true
  
  # PointCloud to LaserScan conversion
  enable_laserscan_conversion: true
  laserscan_topic: "/scan_realsense_left"
  laserscan_min_height: -0.2
  laserscan_max_height: 0.3
  laserscan_min_range: 0.6
  laserscan_max_range: 6.0

realsense_front_right:
  # Same configuration as left camera
  serial_no: ""
  device_type: "d455"
  base_frame_id: "camera_front_right_link"
  depth_frame_id: "camera_front_right_depth_frame"
  color_frame_id: "camera_front_right_color_frame"
  
  # (All other parameters identical to front_left)
  enable_depth: true
  enable_color: true
  enable_pointcloud: true
  depth_width: 640
  depth_height: 480
  depth_fps: 30
  color_width: 640
  color_height: 480
  color_fps: 30
  enable_laserscan_conversion: true
  laserscan_topic: "/scan_realsense_right"
```

### Launch Configuration

```python
# In launch/launch_sensors.py
realsense_left_node = Node(
    package='realsense2_camera',
    executable='realsense2_camera_node',
    name='realsense_front_left',
    namespace='camera_front_left',
    parameters=[sensors_config, {
        'camera_name': 'camera_front_left',
        'enable_sync': True,
        'align_depth.enable': True
    }],
    condition=IfCondition(LaunchConfiguration('enable_realsense'))
)

# PointCloud to LaserScan for left camera
realsense_left_to_scan = Node(
    package='pointcloud_to_laserscan',
    executable='pointcloud_to_laserscan_node',
    name='realsense_left_to_laserscan',
    remappings=[
        ('cloud_in', '/camera_front_left/depth/color/points'),
        ('scan', '/scan_realsense_left')
    ],
    parameters=[{
        'target_frame': 'camera_front_left_depth_frame',
        'min_height': -0.2,
        'max_height': 0.3,
        'angle_min': -0.7854,  # -45° (limited FOV)
        'angle_max': 0.7854,   # +45°
        'range_min': 0.6,
        'range_max': 6.0
    }],
    condition=IfCondition(LaunchConfiguration('enable_realsense'))
)
```

### Driver Installation

```bash
# RealSense ROS2 wrapper
sudo apt install ros-humble-realsense2-camera

# RealSense SDK (if not auto-installed)
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils

# Verify installation
realsense-viewer  # GUI tool for testing cameras
```

### USB Bandwidth Optimization

RealSense cameras use significant USB bandwidth. For 2 cameras:

```bash
# Check USB bandwidth
lsusb -t

# Reduce resolution if bandwidth issues occur
# In sensors_config.yaml:
depth_width: 424  # Reduced from 640
depth_height: 240  # Reduced from 480
```

---

## IMU Configuration

### Hardware Setup

**MPU6050 / MPU9250:**
- Position: Center of robot base
- Orientation: X-forward, Y-left, Z-up
- Connection: I2C (address 0x68)
- Frame ID: `imu_link`
- Measurement Range:
  - Gyroscope: ±250°/s
  - Accelerometer: ±2g

### Configuration File: `config/sensors_config.yaml`

```yaml
imu:
  device: "/dev/i2c-1"
  i2c_address: 0x68
  frame_id: "imu_link"
  topic: "/imu/data"
  
  # Publishing rate
  rate: 100  # Hz
  
  # Gyroscope configuration
  gyro_range: 250  # ±250°/s
  gyro_offset_x: 0.0
  gyro_offset_y: 0.0
  gyro_offset_z: 0.0
  
  # Accelerometer configuration
  accel_range: 2  # ±2g
  accel_offset_x: 0.0
  accel_offset_y: 0.0
  accel_offset_z: 0.0
  
  # Complementary filter (for orientation)
  enable_orientation_filter: true
  filter_alpha: 0.98  # 98% gyro, 2% accel
  
  # Covariance
  orientation_covariance: [0.01, 0.0, 0.0,
                           0.0, 0.01, 0.0,
                           0.0, 0.0, 0.01]
  angular_velocity_covariance: [0.001, 0.0, 0.0,
                                0.0, 0.001, 0.0,
                                0.0, 0.0, 0.001]
  linear_acceleration_covariance: [0.01, 0.0, 0.0,
                                   0.0, 0.01, 0.0,
                                   0.0, 0.0, 0.01]
```

### Launch Configuration

```python
# In launch/launch_sensors.py
imu_node = Node(
    package='mpu6050driver',  # or ros2_mpu6050
    executable='mpu6050driver_node',
    name='imu',
    parameters=[sensors_config],
    condition=IfCondition(LaunchConfiguration('enable_imu'))
)
```

### Driver Installation

```bash
# MPU6050 ROS2 driver
sudo apt install ros-humble-mpu6050driver

# Or build from source
cd ~/ros2_ws/src
git clone https://github.com/hiwad-aziz/ros2_mpu6050.git
cd ~/ros2_ws
colcon build --packages-select ros2_mpu6050

# Enable I2C
sudo raspi-config  # Interface Options > I2C > Enable
sudo reboot

# Test I2C device
sudo i2cdetect -y 1  # Should show 0x68
```

### Calibration

```bash
# Run calibration routine (robot stationary on level surface)
ros2 run mpu6050driver calibrate_imu

# Update offsets in sensors_config.yaml
```

---

## Sensor Fusion

### EKF Localization (robot_localization)

The ULTRABOT fuses wheel odometry + IMU using an Extended Kalman Filter (EKF).

**Configuration:** `config/ekf_params.yaml`

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0  # Hz
    sensor_timeout: 0.1
    two_d_mode: true
    
    # Odometry source 1: Wheel encoders
    odom0: /odom
    odom0_config: [false, false, false,   # x, y, z (position)
                   false, false, false,   # roll, pitch, yaw (orientation)
                   true,  true,  false,   # vx, vy, vz (linear velocity)
                   false, false, true,    # vroll, vpitch, vyaw (angular velocity)
                   false, false, false]   # ax, ay, az (acceleration)
    odom0_differential: false
    odom0_relative: false
    
    # IMU source 1: MPU6050
    imu0: /imu/data
    imu0_config: [false, false, false,    # x, y, z (position)
                  false, false, true,     # roll, pitch, yaw (orientation)
                  false, false, false,    # vx, vy, vz (linear velocity)
                  false, false, true,     # vroll, vpitch, vyaw (angular velocity)
                  true,  true,  false]    # ax, ay, az (acceleration)
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true
    
    # Process noise covariance
    process_noise_covariance: [0.05, 0.0,  0.0,  ...]  # Full 15x15 matrix
    
    # Output
    world_frame: odom
    odom_frame: odom
    base_link_frame: base_footprint
    map_frame: map
    
    publish_tf: true
    publish_acceleration: false
```

**Launch:**

```python
ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    parameters=[ekf_config],
    remappings=[('odometry/filtered', '/odometry/filtered')]
)
```

### Nav2 Costmap Sensor Integration

**Local Costmap (5m radius, 0.05m resolution):**

```yaml
# config/nav2_params.yaml
local_costmap:
  observation_sources: scan scan_realsense_left scan_realsense_right ouster_pointcloud
  
  scan:
    topic: /scan
    sensor_frame: base_link
    observation_persistence: 0.0
    expected_update_rate: 10.0
    data_type: "LaserScan"
    clearing: true
    marking: true
    inf_is_valid: true
  
  scan_realsense_left:
    topic: /scan_realsense_left
    sensor_frame: camera_front_left_depth_frame
    observation_persistence: 0.0
    expected_update_rate: 15.0
    data_type: "LaserScan"
    clearing: true
    marking: true
    min_obstacle_height: 0.1
    max_obstacle_height: 2.0
  
  scan_realsense_right:
    topic: /scan_realsense_right
    sensor_frame: camera_front_right_depth_frame
    observation_persistence: 0.0
    expected_update_rate: 15.0
    data_type: "LaserScan"
    clearing: true
    marking: true
    min_obstacle_height: 0.1
    max_obstacle_height: 2.0
  
  ouster_pointcloud:
    topic: /points
    sensor_frame: ouster_link
    observation_persistence: 0.0
    expected_update_rate: 10.0
    data_type: "PointCloud2"
    clearing: true
    marking: true
    min_obstacle_height: 0.2
    max_obstacle_height: 2.0
```

**Global Costmap (50m radius, 0.1m resolution):**

```yaml
global_costmap:
  observation_sources: scan ouster_pointcloud
  
  # (Reduced observation sources for performance)
```

---

## Time Synchronization

**Critical for multi-sensor fusion!**

All sensors must be time-synchronized to within **<10ms** for accurate fusion.

### NTP Synchronization (Network-based)

```bash
# Install NTP
sudo apt install ntp

# Configure NTP server
sudo nano /etc/ntp.conf
# Add:
server 192.168.1.1 prefer  # Your local NTP server

# Restart NTP
sudo systemctl restart ntp

# Check synchronization
ntpq -p
```

### Chrony (High-Precision)

For GPS/PPS-based synchronization, use Chrony.

**See:** `TIME_SYNC_GUIDE.md` for detailed setup.

### Timestamp Monitoring

Monitor time skew between sensors:

```bash
ros2 topic echo /diagnostics | grep -A 10 "time_sync"
```

---

## Calibration Procedures

### 1. IMU Calibration

**Prerequisites:** Robot stationary on level surface

```bash
# Launch IMU driver
ros2 launch somanet launch_sensors.py enable_imu:=true \
    enable_lidar_2d:=false enable_lidar_3d:=false enable_realsense:=false

# Run calibration (collect 1000 samples)
ros2 run mpu6050driver calibrate_imu --samples 1000

# Output: gyro/accel offsets
# Update config/sensors_config.yaml with offsets
```

### 2. LiDAR Extrinsic Calibration

**Calibrate LiDAR positions relative to `base_link`:**

```bash
# Method 1: Measure manually with ruler/laser
# Update urdf/ultrabot.urdf.xacro with measured positions

# Method 2: Use lidar_align package (requires known environment)
sudo apt install ros-humble-lidar-align
ros2 run lidar_align lidar_align_node
```

### 3. RealSense Extrinsic Calibration

**Calibrate camera positions:**

```bash
# Method 1: Measure manually
# Update urdf/ultrabot.urdf.xacro

# Method 2: Use ArUco markers
sudo apt install ros-humble-aruco-ros
# Place marker in known position, run calibration node
```

### 4. Odometry Calibration

**Calibrate wheel radius and wheelbase:**

```bash
# Drive robot forward 5 meters, measure actual distance
ros2 run somanet validate_odometry.py --distance 5.0

# Rotate robot 360°, measure actual rotation
ros2 run somanet validate_odometry.py --rotation 360

# Update config/robot_config.yaml with corrected values
```

**See:** `CALIBRATION_GUIDE.md` for detailed procedures.

---

## Troubleshooting

### LiDAR 2D Not Publishing

```bash
# Check network connectivity
ping 192.168.1.10

# Check topic
ros2 topic hz /scan_front

# View raw data
ros2 topic echo /scan_front --once

# Common issues:
# - Incorrect IP address
# - Firewall blocking port 2111
# - USB cable not USB 3.0 (for USB LiDARs)
```

### Ouster 3D Not Connecting

```bash
# Check sensor IP
ping 192.168.1.20

# Check sensor status (web interface)
curl http://192.168.1.20/api/v1/sensor/metadata

# Reset sensor
curl -X POST http://192.168.1.20/api/v1/system/reset

# Common issues:
# - Incorrect lidar_mode (must match sensor config)
# - Network MTU too low (increase to 9000 for jumbo frames)
```

### RealSense Not Detected

```bash
# List USB devices
realsense-viewer  # Should show connected cameras

# Check USB bandwidth
lsusb -t

# Check serial numbers
rs-enumerate-devices

# Common issues:
# - USB 2.0 port (requires USB 3.0+)
# - Insufficient USB power (use powered hub)
# - Multiple cameras on same USB controller (split across controllers)
```

### IMU Data Noisy

```bash
# Check I2C connection
sudo i2cdetect -y 1

# Recalibrate IMU (robot stationary)
ros2 run mpu6050driver calibrate_imu

# Common issues:
# - Vibration (mount IMU on damped platform)
# - Electromagnetic interference (shield cables)
# - Temperature drift (allow IMU to warm up 5 minutes)
```

### Time Synchronization Issues

```bash
# Check NTP status
ntpq -p

# Check time offset
ros2 topic echo /diagnostics | grep time_sync

# Force sync
sudo ntpdate -s 192.168.1.1

# Common issues:
# - NTP server unreachable
# - Large initial time skew (>1000s)
# - System clock drift (bad RTC battery)
```

---

## References

- **SICK TiM571 Manual**: https://cdn.sick.com/media/docs/TiM5xx_TiM7xx_UM_en.pdf
- **Ouster OS1 Guide**: https://static.ouster.dev/sensor-docs/
- **RealSense D455 Datasheet**: https://www.intelrealsense.com/depth-camera-d455/
- **MPU6050 Datasheet**: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
- **robot_localization**: http://docs.ros.org/en/melodic/api/robot_localization/
- **Nav2 Costmap 2D**: https://navigation.ros.org/configuration/packages/costmap-plugins/

**Version 4.1.0** | **Sensor Configuration Guide** | **ULTRABOT Team**
