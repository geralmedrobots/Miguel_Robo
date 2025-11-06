# BEHAVIOR TREE INTEGRATION - ULTRABOT AGV

## Overview

Custom Behavior Tree XML files for Nav2 navigation with AGV-specific safety enhancements.

**Why Custom BTs?**
- Default Nav2 BTs don't handle AGV-specific safety events (deadman, EtherCAT faults)
- ISO 13849-1 and ISO 3691-4 require predictable, documented recovery behaviors
- AGV industrial environments need specialized obstacle avoidance strategies

---

## Behavior Trees Included

### 1. `navigate_w_safety.xml`
**Purpose:** Single-goal navigation with safety monitoring

**Features:**
- Pre-navigation safety checks (deadman, driver operational)
- Recovery behaviors for safety stops
- AGV-specific escape maneuvers (rotate, back up)
- Operator intervention timeout (30s)
- Controlled retry limits (3 attempts)

**Use Case:** Simple point-to-point navigation with safety compliance

### 2. `navigate_through_poses_w_safety.xml`
**Purpose:** Multi-waypoint navigation with continuous safety monitoring

**Features:**
- Sequential waypoint execution
- Real-time collision detection monitoring
- Pause on deadman release, resume on re-engage
- Waypoint skip on persistent failures
- Mission abort on critical safety events

**Use Case:** Warehouse routes, predefined patrol paths, multi-stop deliveries

---

## Safety Enhancements

### Integrated Safety Checks

| Safety Event | BT Response | Recovery Time |
|--------------|-------------|---------------|
| **Deadman Released** | Pause navigation, wait for re-engage | Immediate |
| **EtherCAT Fault** | Stop, wait for driver recovery | 5s + retry |
| **Collision Detected** | Stop, back up 30cm, replan | 4s |
| **Safety Stop Active** | Halt, wait for clearance | 30s timeout |
| **Path Blocked** | Rotate 90°, clear costmap, retry | 4s |

### Recovery Sequence Hierarchy

```
1. Wait for Safety Clearance (5s)
   └─> Check deadman + driver operational

2. Rotate Recovery (90°)
   └─> Clear sensor blind spots

3. Back Up Recovery (30cm)
   └─> Escape local minimum

4. Operator Intervention (30s)
   └─> Manual rescue required
```

---

## Configuration

### Enabling Custom BTs

Edit `config/nav2_params.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "$(find-pkg-share somanet)/config/behavior_trees/navigate_w_safety.xml"
    default_nav_through_poses_bt_xml: "$(find-pkg-share somanet)/config/behavior_trees/navigate_through_poses_w_safety.xml"
```

**Note:** Custom BT plugins (`IsDeadmanActive`, `IsDriverOperational`) are optional.
If not implemented, remove those condition nodes from the XML.

### Tunable Parameters

Edit XML files directly:

```xml
<!-- Retry limits -->
<RecoveryNode number_of_retries="3">

<!-- Safety check frequency -->
<RateController hz="1.0">

<!-- Recovery timeouts -->
<Wait wait_duration="5"/>

<!-- Back-up distance (AGV-specific) -->
<BackUp backup_dist="0.30" backup_speed="0.15"/>
```

---

## Custom BT Plugins (Optional)

For full safety integration, implement these BT plugins:

### 1. `IsDeadmanActive` (Condition Node)
```cpp
// Checks if deadman button is pressed
// Returns SUCCESS if active, FAILURE otherwise
// Subscribes to: /deadman_status (std_msgs/Bool)
```

### 2. `IsDriverOperational` (Condition Node)
```cpp
// Checks if EtherCAT driver is operational
// Returns SUCCESS if OK, FAILURE on fault
// Subscribes to: /driver_status (std_msgs/String)
```

### 3. `PublishSafetyStatus` (Action Node)
```cpp
// Publishes safety status for fleet management
// Publishes to: /safety_events (std_msgs/String)
// Statuses: PAUSED, WAITING_FOR_OPERATOR, MISSION_ABORTED, etc.
```

**Implementation:** Create a ROS2 package `somanet_bt_plugins` inheriting from `nav2_behavior_tree::BtActionNode` or `BtConditionNode`.

---

## Testing

### 1. Validate XML Syntax
```bash
# Check for XML parsing errors
ros2 run nav2_bt_navigator bt_navigator --ros-args -p default_nav_to_pose_bt_xml:=/path/to/navigate_w_safety.xml
```

### 2. Test Single-Goal Navigation
```bash
# Launch Nav2 with custom BT
ros2 launch somanet launch_with_nav2.py

# Send test goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}}"
```

### 3. Test Multi-Waypoint Navigation
```bash
# Send multiple goals
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}}}, \
            {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}]}"
```

### 4. Simulate Safety Events
```bash
# Test deadman release recovery
ros2 topic pub /deadman_status std_msgs/Bool "data: false" -1

# Test EtherCAT fault
ros2 topic pub /driver_status std_msgs/String "data: 'FAULT'" -1
```

---

## Behavior Tree Visualization

### Using Groot (BT Editor)

```bash
# Install Groot
sudo apt install ros-humble-groot

# Visualize BT
groot config/behavior_trees/navigate_w_safety.xml
```

### Expected BT Structure

```
NavigateRecovery [Recovery]
├─ NavigateWithReplanning [Pipeline]
│  ├─ SafetyChecks [Sequence]
│  │  ├─ CheckDeadman
│  │  └─ CheckEtherCAT
│  ├─ ComputePath
│  ├─ SmoothPath
│  └─ FollowPath [Recovery]
│     ├─ Path Execution
│     └─ ClearAndReplan
└─ SafetyRecoverySequence [Fallback]
   ├─ WaitForSafetyClearance (5s)
   ├─ RotateRecovery (90°)
   ├─ BackUpRecovery (30cm)
   └─ OperatorIntervention (30s)
```

---

## Compliance Notes

### ISO 13849-1 (Functional Safety)
- ✅ Predictable recovery sequences (documented in XML)
- ✅ Fail-safe timeouts (30s operator intervention)
- ✅ Multiple safety checks (deadman, driver, collision)

### ISO 3691-4 (AGV Safety)
- ✅ Operator intervention mechanism
- ✅ Pause on safety events (not blind continuation)
- ✅ Status reporting for fleet management

---

## Troubleshooting

### BT Not Loading
```
ERROR: Failed to load behavior tree: /path/to/navigate_w_safety.xml
```
**Solution:** Check file permissions and path in `nav2_params.yaml`

### Custom Plugins Not Found
```
ERROR: Behavior tree node 'IsDeadmanActive' not registered
```
**Solution:** Either implement the plugin or remove the node from XML

### Navigation Stuck in Recovery Loop
```
WARN: Recovery behavior failed 3 times
```
**Solution:** Check safety status (`/deadman_status`, `/driver_status`)

---

## References

- [Nav2 Behavior Trees](https://navigation.ros.org/behavior_trees/index.html)
- [BehaviorTree.CPP](https://www.behaviortree.dev/)
- [Groot Editor](https://github.com/BehaviorTree/Groot)
- [ISO 3691-4 AGV Safety](https://www.iso.org/standard/70660.html)
