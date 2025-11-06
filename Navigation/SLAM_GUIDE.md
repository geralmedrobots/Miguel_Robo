# SLAM Guide - Simultaneous Localization and Mapping

**Ultrabot AGV** | **Version 4.1.0** | **SLAM Integration**

---

## 📋 Overview

This guide covers **SLAM (Simultaneous Localization and Mapping)** implementation for the Ultrabot AGV. SLAM allows the robot to create a map of an unknown environment while simultaneously tracking its location within that map.

### Supported SLAM Algorithms

| Algorithm | Package | Best For | Performance |
|-----------|---------|----------|-------------|
| **SLAM Toolbox** | `slam_toolbox` | General purpose, online mapping | ⭐⭐⭐⭐⭐ |
| **Cartographer** | `cartographer_ros` | Large environments, high accuracy | ⭐⭐⭐⭐ |

---

## 🚀 Quick Start

### Option 1: SLAM Only (Map Creation)

```bash
# Install SLAM dependencies
sudo apt install ros-humble-slam-toolbox ros-humble-cartographer-ros

# Launch SLAM with teleop control
ros2 launch somanet launch_slam.py

# Drive the robot around to create map
# Use joystick or keyboard: ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save the map
ros2 run somanet save_map.py --name my_warehouse_map
```

### Option 2: SLAM + Navigation (Autonomous Exploration)

```bash
# Launch SLAM with Nav2 integration
ros2 launch somanet launch_slam_nav.py

# Set navigation goals in RVIZ while building the map
# Robot will navigate and map simultaneously
```

### Option 3: Cartographer SLAM

```bash
# Use Cartographer instead of SLAM Toolbox
ros2 launch somanet launch_cartographer.py
```

---

## 🗺️ Map Creation Workflow

### 1. Start SLAM Session

```bash
# Terminal 1: Launch SLAM system
ros2 launch somanet launch_slam.py use_rviz:=true

# Terminal 2: Verify topics
ros2 topic list | grep -E "(scan|odom|map)"
# Expected output:
#   /scan
#   /odom
#   /map
```

### 2. Drive Robot to Map Environment

**Manual Teleoperation:**
```bash
# Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=cmd_vel_teleop

# Controls:
#   i/k: forward/backward
#   j/l: turn left/right
#   q/z: increase/decrease speed
```

**Best Practices:**
- ✅ Drive slowly (0.2-0.3 m/s) for better scan matching
- ✅ Make smooth turns to avoid motion blur
- ✅ Overlap coverage (revisit areas for loop closure)
- ✅ Include distinctive features (corners, doorways)
- ❌ Avoid featureless corridors or symmetric rooms

### 3. Monitor Map Quality

**In RVIZ:**
- Check `/map` topic visualization
- Green areas = well-mapped
- Gray areas = unknown
- Black areas = obstacles

**In Terminal:**
```bash
# Check map resolution
ros2 topic echo /map --once | grep resolution

# Monitor SLAM status
ros2 topic echo /slam_toolbox/feedback
```

### 4. Save Map

```bash
# Option 1: Using save_map.py script
ros2 run somanet save_map.py --name warehouse_floor1 --dir ~/maps

# Option 2: Using ROS2 map_server
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# Option 3: SLAM Toolbox serialization (includes scan data)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/user/maps/my_map'}"
```

**Output Files:**
- `my_map.pgm` - Grayscale image (occupied/free/unknown)
- `my_map.yaml` - Map metadata (resolution, origin)
- `my_map.posegraph` - SLAM Toolbox graph (optional)

---

## ⚙️ Configuration

### SLAM Toolbox Parameters (`config/slam_params.yaml`)

```yaml
slam_toolbox:
  ros__parameters:
    # Resolution
    resolution: 0.05  # 5cm grid cells
    
    # Frames
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    
    # Loop closure
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    
    # Scan matching
    use_scan_matching: true
    minimum_travel_distance: 0.2  # meters
    minimum_travel_heading: 0.2   # radians
```

**Key Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `resolution` | 0.05 | Map cell size (meters) |
| `minimum_travel_distance` | 0.2 | Min distance before processing scan |
| `do_loop_closing` | true | Enable loop closure detection |
| `loop_search_maximum_distance` | 3.0 | Max distance for loop closure |

### Cartographer Parameters (`config/cartographer_config.lua`)

```lua
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
```

---

## 🔧 Troubleshooting

### Problem: Map drifts over time

**Symptoms:**
- Straight walls appear wavy
- Rooms don't close properly
- Odometry error accumulation

**Solutions:**
```bash
# 1. Check odometry accuracy
ros2 run somanet validate_odometry.py

# 2. Increase scan matching weight
# Edit config/slam_params.yaml:
#   correlation_search_space_dimension: 0.7  # was 0.5

# 3. Enable more aggressive loop closure
#   loop_match_minimum_response_fine: 0.35  # was 0.45
```

### Problem: SLAM fails to start

**Symptoms:**
- No `/map` topic published
- SLAM node crashes immediately

**Solutions:**
```bash
# Check if laser scan is publishing
ros2 topic hz /scan
# If not publishing, check sensor drivers

# Verify SLAM node is running
ros2 node list | grep slam

# Check logs
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args --log-level debug
```

### Problem: Poor map quality

**Symptoms:**
- Obstacles misaligned
- Duplicate walls
- Noisy map

**Solutions:**
1. **Drive slower** - Reduce velocity to 0.2 m/s
2. **Better lighting** - Lidar needs consistent returns
3. **Avoid dynamic obstacles** - People/moving objects confuse SLAM
4. **Increase overlap** - Revisit areas from different angles

```yaml
# Adjust scan matching sensitivity:
correlation_search_space_resolution: 0.005  # finer search (was 0.01)
correlation_search_space_smear_deviation: 0.05  # less smoothing (was 0.1)
```

---

## 🎯 Advanced Features

### Localization with Existing Map

After creating a map, use it for localization only:

```bash
# 1. Save map first
ros2 run somanet save_map.py --name my_map

# 2. Launch with map (AMCL localization)
ros2 launch somanet launch_with_nav2.py map:=~/maps/my_map.yaml

# 3. Set initial pose in RVIZ ("2D Pose Estimate")
```

### Continuous Mapping (Update Map)

```bash
# Launch SLAM with existing map
ros2 launch somanet launch_slam.py

# Load previous map
ros2 service call /slam_toolbox/deserialize_map \
  slam_toolbox/srv/DeserializePoseGraph \
  "{filename: '/home/user/maps/my_map', match_type: 1}"

# Drive to new areas - map will be updated
# Save updated map when done
```

### Multi-Floor Mapping

```bash
# Floor 1
ros2 launch somanet launch_slam.py
# ... map floor 1 ...
ros2 run somanet save_map.py --name floor1

# Floor 2 (new session)
ros2 launch somanet launch_slam.py
# ... map floor 2 ...
ros2 run somanet save_map.py --name floor2
```

---

## 📊 Performance Metrics

### SLAM Toolbox

| Metric | Typical Value | Target |
|--------|---------------|--------|
| **Map update rate** | 5 Hz | >1 Hz |
| **Localization accuracy** | ±5 cm | <10 cm |
| **CPU usage** | 15-25% | <40% |
| **Memory usage** | 200-500 MB | <1 GB |
| **Max map size** | 500x500 m | Limited by RAM |

### Cartographer

| Metric | Typical Value | Target |
|--------|---------------|--------|
| **Map update rate** | 1 Hz | >0.5 Hz |
| **Localization accuracy** | ±3 cm | <5 cm |
| **CPU usage** | 30-50% | <60% |
| **Memory usage** | 500-1000 MB | <2 GB |
| **Max map size** | 1000x1000 m | Limited by RAM |

---

## 🛡️ Safety Considerations

### ISO 13849-1 Compliance

SLAM operation maintains safety compliance:

| Safety Feature | Implementation |
|----------------|----------------|
| **Velocity limits** | Enforced by safety_supervisor (1.0 m/s max) |
| **Watchdog timeout** | All commands timeout to safe stop |
| **Emergency stop** | Hardware e-stop always active |
| **Obstacle detection** | Lidar scan used for collision avoidance |

### Safe Mapping Practices

1. ✅ **Clear workspace** - Remove people during initial mapping
2. ✅ **Test e-stop** - Verify emergency stop before mapping
3. ✅ **Supervised operation** - Operator present during mapping
4. ✅ **Slow speeds** - Mapping at 0.2-0.3 m/s (below 1.0 m/s limit)

---

## 📚 References

- **SLAM Toolbox**: https://github.com/SteveMacenski/slam_toolbox
- **Cartographer**: https://github.com/cartographer-project/cartographer
- **Nav2 SLAM Tutorial**: https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html
- **Map Server**: https://github.com/ros-planning/navigation2/tree/main/nav2_map_server

---

## 🔗 Related Documentation

- [README_NAV2.md](README_NAV2.md) - Nav2 navigation & SLAM guide
- [BUILD_GUIDE.md](BUILD_GUIDE.md) - Build & troubleshooting
- [SAFETY.md](SAFETY.md) - Safety compliance

---

**Version**: 4.1.0 | **Last Updated**: 2025-11-05
