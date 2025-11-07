# ULTRABOT AGV - API Documentation

**Version 4.1.0** | **ROS2 Humble API Reference**

> Developer guide for ULTRABOT codebase with Doxygen setup and class documentation

---

## Table of Contents

1. [Overview](#overview)
2. [Doxygen Setup](#doxygen-setup)
3. [Core Classes](#core-classes)
4. [Node Interfaces](#node-interfaces)
5. [Message Definitions](#message-definitions)
6. [Code Examples](#code-examples)

---

## Overview

The ULTRABOT AGV codebase is organized into the following components:

```
Navigation/
├── src/                          # Node implementations
│   ├── main.cpp                  # Main entry point (multi-node launcher)
│   ├── ethercat_driver.cpp       # Hardware interface (200 Hz control)
│   ├── safety_supervisor_node.cpp # Safety monitoring (ISO 13849-1)
│   ├── command_arbitrator_node.cpp # Priority-based command selection
│   ├── command_mux_node.cpp      # Autonomous vs teleop switching
│   ├── odometry_calculator.cpp   # Differential drive kinematics
│   ├── certified_params_validator.cpp # HMAC-SHA256 parameter validation
│   ├── maintenance_mode_manager.cpp # Maintenance access control
│   └── teleop_joy.cpp            # Joystick teleop interface
│
├── include/                      # Header files
│   ├── ethercat_driver.hpp
│   ├── drive_interface.hpp       # Abstract hardware interface
│   ├── mock_driver.hpp           # Testing driver (no hardware)
│   ├── odometry_calculator.hpp
│   ├── certified_params_validator.hpp
│   └── maintenance_mode_manager.hpp
│
└── test/                         # Unit tests
    ├── test_ethercat_failures.cpp
    ├── test_safety_critical.cpp
    ├── test_odometry_calculator.cpp
    ├── test_mock_driver.cpp
    └── test_lifecycle_integration.cpp
```

---

## Doxygen Setup

### Installation

```bash
# Install Doxygen
sudo apt install doxygen graphviz

# Generate documentation
cd ~/ros2_ws/src/navigation
doxygen Doxyfile
```

### Doxyfile Configuration

Create `Doxyfile` in the repository root:

```doxyfile
# Project info
PROJECT_NAME           = "ULTRABOT AGV Navigation"
PROJECT_NUMBER         = "4.1.0"
PROJECT_BRIEF          = "Safety-critical AGV control with Nav2 integration"

# Input
INPUT                  = src include
FILE_PATTERNS          = *.cpp *.hpp *.h
RECURSIVE              = YES
EXCLUDE                = build/ install/ log/

# Output
OUTPUT_DIRECTORY       = docs/api
GENERATE_HTML          = YES
GENERATE_LATEX         = NO
HTML_OUTPUT            = html

# Documentation extraction
EXTRACT_ALL            = YES
EXTRACT_PRIVATE        = YES
EXTRACT_STATIC         = YES
EXTRACT_LOCAL_CLASSES  = YES

# Graphs
HAVE_DOT               = YES
CLASS_DIAGRAMS         = YES
COLLABORATION_GRAPH    = YES
CALL_GRAPH             = YES
CALLER_GRAPH           = YES

# Source browsing
SOURCE_BROWSER         = YES
INLINE_SOURCES         = NO
REFERENCED_BY_RELATION = YES
REFERENCES_RELATION    = YES
```

### Generating Documentation

```bash
# From repository root
doxygen Doxyfile

# Open in browser
xdg-open docs/api/html/index.html
```

---

## Core Classes

### 1. EtherCatDriver

**File:** `src/ethercat_driver.cpp`, `include/ethercat_driver.hpp`

**Purpose:** Hardware interface for EtherCAT motor controllers (SOMANET drives)

**Key Features:**
- Real-time EtherCAT communication (200 Hz)
- Lifecycle-managed node (configure/activate/deactivate/cleanup)
- Dual-channel motor control (left/right wheels)
- Odometry feedback from encoders
- Hardware emergency stop integration

**Class Definition:**

```cpp
/**
 * @class EtherCatDriver
 * @brief ROS2 lifecycle node for EtherCAT-based differential drive control
 * 
 * Implements DriveInterface for hardware abstraction.
 * Publishes odometry at 200 Hz from wheel encoders.
 * Subscribes to /cmd_vel_safe for velocity commands.
 * 
 * @note Requires ETHERCAT_INTERFACE environment variable
 * @note Real-time performance requires SCHED_FIFO scheduling
 */
class EtherCatDriver : public rclcpp_lifecycle::LifecycleNode, 
                       public DriveInterface
{
public:
  /**
   * @brief Constructor
   * @param options Node options for lifecycle management
   */
  explicit EtherCatDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Destructor - ensures safe EtherCAT shutdown
   */
  ~EtherCatDriver();

  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  // DriveInterface implementation
  void send_wheel_velocities(double left_vel, double right_vel) override;
  void read_encoder_positions(int32_t& left_pos, int32_t& right_pos) override;
  bool is_connected() const override;

private:
  /**
   * @brief Main control loop (200 Hz)
   */
  void control_loop();

  /**
   * @brief Velocity command callback
   * @param msg Twist message with linear.x and angular.z
   */
  void velocity_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Convert Twist to differential drive wheel velocities
   * @param linear Linear velocity (m/s)
   * @param angular Angular velocity (rad/s)
   * @return pair<left_vel, right_vel> in rad/s
   */
  std::pair<double, double> twist_to_wheels(double linear, double angular);

  // EtherCAT communication
  ec_master_t* master_;        ///< EtherCAT master handle
  ec_domain_t* domain_;        ///< Process data domain
  ec_slave_config_t* left_slave_;   ///< Left motor slave config
  ec_slave_config_t* right_slave_;  ///< Right motor slave config

  // ROS2 interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Robot parameters
  double wheel_radius_;        ///< Wheel radius (m)
  double wheel_base_;          ///< Distance between wheels (m)
  double max_wheel_vel_;       ///< Maximum wheel velocity (rad/s)
};
```

**Usage Example:**

```cpp
// Launch with lifecycle management
auto driver = std::make_shared<EtherCatDriver>();
driver->configure();
driver->activate();

// Later...
driver->deactivate();
driver->cleanup();
```

---

### 2. SafetySupervisor

**File:** `src/safety_supervisor_node.cpp`

**Purpose:** ISO 13849-1 compliant safety monitoring and command validation

**Key Features:**
- Velocity limit enforcement (1.0 m/s max)
- Watchdog timer (500ms timeout)
- Command validation (NaN/Inf detection)
- Diagnostic publishing

**Class Definition:**

```cpp
/**
 * @class SafetySupervisor
 * @brief Safety-critical velocity command validator
 * 
 * Implements ISO 13849-1 Category 3, Performance Level d safety functions.
 * All velocity commands must pass through this node before reaching hardware.
 * 
 * @note This node MUST NOT be bypassed in production systems
 * @warning Modifications require safety re-certification
 */
class SafetySupervisor : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SafetySupervisor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  /**
   * @brief Validate and limit velocity command
   * @param msg Input velocity command
   */
  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Check if command is safe
   * @param linear Linear velocity (m/s)
   * @param angular Angular velocity (rad/s)
   * @return true if within limits
   */
  bool is_safe_velocity(double linear, double angular);

  /**
   * @brief Clamp velocity to safety limits
   * @param vel Input velocity
   * @param max_vel Maximum allowed velocity
   * @return Clamped velocity
   */
  double clamp_velocity(double vel, double max_vel);

  /**
   * @brief Watchdog timeout callback
   */
  void watchdog_timeout_callback();

  /**
   * @brief Publish diagnostic status
   */
  void publish_diagnostics();

  // Safety parameters (loaded from certified_safety_params.yaml)
  double max_linear_vel_;      ///< Maximum linear velocity (m/s)
  double max_angular_vel_;     ///< Maximum angular velocity (rad/s)
  double watchdog_timeout_;    ///< Watchdog timeout (seconds)

  // ROS2 interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // State
  rclcpp::Time last_command_time_;  ///< Last valid command timestamp
  bool watchdog_active_;            ///< Watchdog timer active flag
};
```

**Documentation Example:**

```cpp
/**
 * @brief Validate and limit velocity command
 * 
 * This function implements the core safety logic:
 * 1. Check for NaN/Inf values (invalid command)
 * 2. Clamp velocities to certified limits
 * 3. Reset watchdog timer
 * 4. Publish safe command or zero velocity
 * 
 * @param msg Input velocity command from /cmd_vel
 * 
 * @note Function executes in O(1) time (no loops)
 * @note All commands are published, even if clamped
 * 
 * @warning If watchdog expires, zero velocity is published
 * 
 * Safety Analysis:
 * - Max execution time: <100 µs (tested)
 * - Fail-safe: Defaults to zero velocity on any error
 * - Coverage: 100% tested (test_safety_critical.cpp)
 */
void SafetySupervisor::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Implementation...
}
```

---

### 3. CommandArbitrator

**File:** `src/command_arbitrator_node.cpp`

**Purpose:** Priority-based command source selection (ISO 3691-4 AGV safety)

**Class Definition:**

```cpp
/**
 * @class CommandArbitrator
 * @brief Priority-based velocity command selector
 * 
 * Implements 4-level priority system:
 * - Emergency (255): E-stop, safety critical
 * - Manual (200): Teleop, joystick/keyboard
 * - Autonomous (100): Nav2, SLAM
 * - Test (50): Development/testing
 * 
 * Highest priority non-zero command is selected.
 * 
 * @note Complies with ISO 3691-4 section 5.9 (AGV safety)
 */
class CommandArbitrator : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CommandArbitrator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

private:
  /**
   * @brief Arbitration callback (called at 50 Hz)
   */
  void arbitrate();

  /**
   * @brief Emergency command callback (P255)
   */
  void emergency_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Manual command callback (P200)
   */
  void manual_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Autonomous command callback (P100)
   */
  void auto_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Test command callback (P50)
   */
  void test_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Check if command is non-zero
   */
  bool is_nonzero(const geometry_msgs::msg::Twist& cmd);

  // Command sources
  geometry_msgs::msg::Twist emergency_cmd_;  ///< Priority 255
  geometry_msgs::msg::Twist manual_cmd_;     ///< Priority 200
  geometry_msgs::msg::Twist auto_cmd_;       ///< Priority 100
  geometry_msgs::msg::Twist test_cmd_;       ///< Priority 50

  // Timestamps
  rclcpp::Time emergency_time_;
  rclcpp::Time manual_time_;
  rclcpp::Time auto_time_;
  rclcpp::Time test_time_;

  // ROS2 interfaces
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> cmd_subs_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr source_pub_;
  rclcpp::TimerBase::SharedPtr arbitration_timer_;
};
```

---

### 4. OdometryCalculator

**File:** `src/odometry_calculator.cpp`, `include/odometry_calculator.hpp`

**Purpose:** Differential drive kinematics and odometry calculation

**Class Definition:**

```cpp
/**
 * @class OdometryCalculator
 * @brief Computes odometry from wheel encoder positions
 * 
 * Implements differential drive kinematics:
 * - Forward kinematics: (left_vel, right_vel) -> (linear, angular)
 * - Odometry integration: position tracking from encoder deltas
 * 
 * @note Uses trapezoidal integration for improved accuracy
 * @note Publishes TF transform (odom -> base_footprint)
 */
class OdometryCalculator : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit OdometryCalculator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Update odometry from new encoder readings
   * @param left_encoder Left wheel encoder position (ticks)
   * @param right_encoder Right wheel encoder position (ticks)
   * @param dt Time delta since last update (seconds)
   */
  void update(int32_t left_encoder, int32_t right_encoder, double dt);

private:
  /**
   * @brief Compute wheel velocities from encoder deltas
   * @param left_delta Change in left encoder (ticks)
   * @param right_delta Change in right encoder (ticks)
   * @param dt Time interval (seconds)
   * @return pair<left_vel, right_vel> in m/s
   */
  std::pair<double, double> compute_wheel_velocities(
    int32_t left_delta, int32_t right_delta, double dt);

  /**
   * @brief Compute robot velocities from wheel velocities
   * @param left_vel Left wheel velocity (m/s)
   * @param right_vel Right wheel velocity (m/s)
   * @return pair<linear_vel, angular_vel>
   */
  std::pair<double, double> compute_robot_velocities(
    double left_vel, double right_vel);

  /**
   * @brief Integrate odometry (x, y, theta) from velocities
   * @param linear Linear velocity (m/s)
   * @param angular Angular velocity (rad/s)
   * @param dt Time interval (seconds)
   */
  void integrate_odometry(double linear, double angular, double dt);

  /**
   * @brief Publish odometry message and TF transform
   */
  void publish_odometry();

  // Robot parameters
  double wheel_radius_;        ///< Wheel radius (m)
  double wheel_base_;          ///< Distance between wheels (m)
  int32_t encoder_resolution_; ///< Encoder ticks per revolution

  // Odometry state
  double x_;      ///< X position (m)
  double y_;      ///< Y position (m)
  double theta_;  ///< Heading angle (radians)
  double vx_;     ///< Linear velocity (m/s)
  double vtheta_; ///< Angular velocity (rad/s)

  // Previous encoder values
  int32_t prev_left_encoder_;
  int32_t prev_right_encoder_;

  // ROS2 interfaces
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
```

---

### 5. CertifiedParamsValidator

**File:** `src/certified_params_validator.cpp`, `include/certified_params_validator.hpp`

**Purpose:** HMAC-SHA256 validation of safety-critical parameters

**Class Definition:**

```cpp
/**
 * @class CertifiedParamsValidator
 * @brief Validates safety-critical parameters using HMAC-SHA256
 * 
 * Prevents unauthorized modification of certified safety parameters.
 * Hash is generated offline and stored in certified_safety_params.yaml.
 * Uses ROS2 logging infrastructure for integrated diagnostics.
 * 
 * @note Required for ISO 13849-1 compliance (tamper detection)
 * @warning Node will refuse to start if hash validation fails
 */
class CertifiedParamsValidator
{
public:
  /**
   * @brief Constructor
   * @param config_path Path to certified_safety_params.yaml
   * @param secret_path Path to cert.key (HMAC secret)
   * @param logger ROS2 logger for integrated logging
   */
  explicit CertifiedParamsValidator(
      const std::string& config_path, 
      const std::string& secret_path,
      rclcpp::Logger logger = rclcpp::get_logger("certified_params_validator"));

  /**
   * @brief Validate all certified parameters
   * @return true if hash matches, false if tampered
   */
  bool validate();

private:
  /**
   * @brief Load parameters from YAML file
   * @param filename Path to certified_safety_params.yaml
   * @return map of parameter name -> value
   */
  std::map<std::string, double> load_certified_params(const std::string& filename);

  /**
   * @brief Compute HMAC-SHA256 hash
   * @param params Parameter map
   * @param secret_key Certification secret (from config/cert.key)
   * @return Hex-encoded hash string
   */
  std::string compute_hash(const std::map<std::string, double>& params, 
                          const std::string& secret_key);

  /**
   * @brief Compare computed hash with stored hash
   * @param computed_hash Hash computed from current parameters
   * @param stored_hash Hash from YAML file
   * @return true if match (parameters not tampered)
   */
  bool verify_hash(const std::string& computed_hash, const std::string& stored_hash);

  std::string cert_file_;   ///< Path to certified_safety_params.yaml
  std::string key_file_;    ///< Path to cert.key (secret)
};
```

---

## Node Interfaces

### Lifecycle State Transitions

All critical nodes implement the `rclcpp_lifecycle::LifecycleNode` interface:

```cpp
/**
 * @brief Configure lifecycle callback
 * 
 * Called when node transitions UNCONFIGURED -> INACTIVE.
 * 
 * Responsibilities:
 * - Load parameters
 * - Allocate resources (do NOT activate yet)
 * - Create subscriptions/publishers
 * 
 * @return SUCCESS or FAILURE
 */
CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

/**
 * @brief Activate lifecycle callback
 * 
 * Called when node transitions INACTIVE -> ACTIVE.
 * 
 * Responsibilities:
 * - Activate publishers
 * - Start timers
 * - Begin processing
 * 
 * @return SUCCESS or FAILURE
 */
CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

/**
 * @brief Deactivate lifecycle callback
 * 
 * Called when node transitions ACTIVE -> INACTIVE.
 * 
 * Responsibilities:
 * - Stop timers
 * - Deactivate publishers
 * - Maintain resources (do NOT deallocate)
 * 
 * @return SUCCESS or FAILURE
 */
CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

/**
 * @brief Cleanup lifecycle callback
 * 
 * Called when node transitions INACTIVE -> UNCONFIGURED.
 * 
 * Responsibilities:
 * - Deallocate resources
 * - Close hardware connections
 * - Reset state
 * 
 * @return SUCCESS or FAILURE
 */
CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
```

### DriveInterface (Abstract Class)

**File:** `include/drive_interface.hpp`

```cpp
/**
 * @class DriveInterface
 * @brief Abstract interface for differential drive hardware
 * 
 * Implemented by:
 * - EtherCatDriver (real hardware)
 * - MockDriver (testing without hardware)
 * 
 * Allows unit testing without EtherCAT hardware.
 */
class DriveInterface
{
public:
  virtual ~DriveInterface() = default;

  /**
   * @brief Send wheel velocity commands to motors
   * @param left_vel Left wheel velocity (rad/s)
   * @param right_vel Right wheel velocity (rad/s)
   */
  virtual void send_wheel_velocities(double left_vel, double right_vel) = 0;

  /**
   * @brief Read encoder positions from motors
   * @param left_pos Output: left encoder position (ticks)
   * @param right_pos Output: right encoder position (ticks)
   */
  virtual void read_encoder_positions(int32_t& left_pos, int32_t& right_pos) = 0;

  /**
   * @brief Check if hardware connection is active
   * @return true if connected, false otherwise
   */
  virtual bool is_connected() const = 0;
};
```

---

## Message Definitions

### Standard ROS2 Messages Used

| Message Type | Package | Usage |
|--------------|---------|-------|
| `geometry_msgs::msg::Twist` | geometry_msgs | Velocity commands (linear, angular) |
| `nav_msgs::msg::Odometry` | nav_msgs | Robot odometry (position, velocity) |
| `sensor_msgs::msg::LaserScan` | sensor_msgs | LiDAR scan data |
| `sensor_msgs::msg::PointCloud2` | sensor_msgs | 3D LiDAR point clouds |
| `sensor_msgs::msg::Imu` | sensor_msgs | IMU data (orientation, velocity, accel) |
| `sensor_msgs::msg::JointState` | sensor_msgs | Wheel joint positions/velocities |
| `diagnostic_msgs::msg::DiagnosticArray` | diagnostic_msgs | System diagnostics |
| `std_msgs::msg::String` | std_msgs | Active command source (arbitrator) |

### Twist Message Structure

```cpp
// geometry_msgs/msg/Twist
struct Twist {
  Vector3 linear;   // Linear velocity (x, y, z)
  Vector3 angular;  // Angular velocity (roll, pitch, yaw)
};

// For differential drive:
// - linear.x: Forward velocity (m/s)
// - angular.z: Yaw rate (rad/s)
// - All other fields: UNUSED (must be zero)
```

### Odometry Message Structure

```cpp
// nav_msgs/msg/Odometry
struct Odometry {
  Header header;
  string child_frame_id;
  PoseWithCovariance pose;      // Position (x, y, theta)
  TwistWithCovariance twist;    // Velocity (vx, vtheta)
};

// ULTRABOT configuration:
// - header.frame_id: "odom"
// - child_frame_id: "base_footprint"
// - Covariance: Diagonal matrix (encoder + IMU noise)
```

---

## Code Examples

### 1. Creating a Custom Node

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"

/**
 * @class MyCustomNode
 * @brief Example lifecycle node template
 */
class MyCustomNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MyCustomNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : rclcpp_lifecycle::LifecycleNode("my_custom_node", options)
  {
    RCLCPP_INFO(get_logger(), "MyCustomNode constructed");
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override
  {
    RCLCPP_INFO(get_logger(), "Configuring...");
    
    // Load parameters
    this->declare_parameter("my_param", 1.0);
    my_param_ = this->get_parameter("my_param").as_double();
    
    // Create subscription
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&MyCustomNode::cmd_callback, this, std::placeholders::_1));
    
    // Create publisher (inactive until activate())
    output_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/output", 10);
    
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override
  {
    RCLCPP_INFO(get_logger(), "Activating...");
    
    // Activate publisher
    output_pub_->on_activate();
    
    // Start timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MyCustomNode::timer_callback, this));
    
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    
    // Stop timer
    timer_->cancel();
    
    // Deactivate publisher
    output_pub_->on_deactivate();
    
    return CallbackReturn::SUCCESS;
  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received command: linear=%.2f, angular=%.2f",
                msg->linear.x, msg->angular.z);
  }

  void timer_callback()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = my_param_;
    output_pub_->publish(msg);
  }

  double my_param_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

### 2. Unit Testing with Mock Hardware

```cpp
#include <gtest/gtest.h>
#include "mock_driver.hpp"
#include "odometry_calculator.hpp"

/**
 * @brief Test odometry calculation without real hardware
 */
TEST(OdometryTest, ForwardMotion)
{
  // Create mock driver
  auto mock_driver = std::make_shared<MockDriver>();
  
  // Create odometry calculator
  auto odom_calc = std::make_shared<OdometryCalculator>();
  
  // Simulate forward motion: both wheels rotating forward
  mock_driver->set_encoder_positions(1000, 1000);  // Initial
  odom_calc->update(1000, 1000, 0.01);             // dt = 10ms
  
  mock_driver->set_encoder_positions(2000, 2000);  // +1000 ticks each
  odom_calc->update(2000, 2000, 0.01);
  
  // Verify robot moved forward (x > 0, y ≈ 0)
  auto pose = odom_calc->get_pose();
  EXPECT_GT(pose.x, 0.0);
  EXPECT_NEAR(pose.y, 0.0, 0.01);
  EXPECT_NEAR(pose.theta, 0.0, 0.01);
}

/**
 * @brief Test rotation (left wheel forward, right wheel backward)
 */
TEST(OdometryTest, Rotation)
{
  auto mock_driver = std::make_shared<MockDriver>();
  auto odom_calc = std::make_shared<OdometryCalculator>();
  
  mock_driver->set_encoder_positions(0, 0);
  odom_calc->update(0, 0, 0.01);
  
  // Left wheel forward, right wheel backward (rotate CCW)
  mock_driver->set_encoder_positions(1000, -1000);
  odom_calc->update(1000, -1000, 0.01);
  
  // Verify robot rotated (theta > 0, x ≈ 0, y ≈ 0)
  auto pose = odom_calc->get_pose();
  EXPECT_NEAR(pose.x, 0.0, 0.01);
  EXPECT_NEAR(pose.y, 0.0, 0.01);
  EXPECT_GT(pose.theta, 0.0);
}
```

### 3. Parameter Validation Example

```cpp
#include "certified_params_validator.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Create a node for logging context
  auto node = std::make_shared<rclcpp::Node>("param_validator_test");
  
  // Create validator with ROS2 logger integration
  auto validator = std::make_shared<CertifiedParamsValidator>(
      "/path/to/certified_safety_params.yaml",
      "/path/to/cert.key",
      node->get_logger());
  
  if (!validator->loadAndValidate())
  {
    RCLCPP_FATAL(node->get_logger(), 
                 "Safety parameter validation FAILED! Parameters have been tampered with.");
    RCLCPP_FATAL(node->get_logger(), 
                 "REFUSING TO START. Re-certify using scripts/generate_certification_hash.py");
    return 1;
  }
  
  RCLCPP_INFO(node->get_logger(), "Safety parameters validated successfully");
  
  // Continue with normal startup...
  rclcpp::shutdown();
  return 0;
}
```

---

## Building API Documentation

### Step 1: Generate Doxyfile

```bash
cd ~/ros2_ws/src/navigation
doxygen -g Doxyfile
```

### Step 2: Edit Doxyfile

Modify the following settings:

```doxyfile
PROJECT_NAME = "ULTRABOT AGV Navigation"
INPUT = src include
RECURSIVE = YES
EXTRACT_ALL = YES
HAVE_DOT = YES
CALL_GRAPH = YES
```

### Step 3: Generate HTML Documentation

```bash
doxygen Doxyfile
```

### Step 4: View Documentation

```bash
firefox docs/api/html/index.html
```

### Step 5: Add to CMakeLists.txt (Optional)

```cmake
# Generate Doxygen documentation
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
  
  add_custom_target(doc_doxygen ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM)
endif()
```

---

## Contributing

When adding new classes or modifying existing ones:

1. **Document all public interfaces** with Doxygen comments
2. **Include usage examples** for non-trivial functions
3. **Document safety implications** for critical functions
4. **Add unit tests** for all new functionality
5. **Update this API.md** with new classes/interfaces

### Doxygen Comment Style

```cpp
/**
 * @brief Short description (one line)
 * 
 * Longer description with implementation details,
 * algorithm explanation, safety considerations, etc.
 * 
 * @param param1 Description of first parameter
 * @param param2 Description of second parameter
 * @return Description of return value
 * 
 * @note Important usage notes
 * @warning Safety warnings or critical considerations
 * 
 * @see RelatedClass::related_function()
 * 
 * Example usage:
 * @code
 * MyClass obj;
 * obj.my_function(42, "hello");
 * @endcode
 */
ReturnType my_function(Type1 param1, Type2 param2);
```

---

## References

- **ROS2 Lifecycle**: https://design.ros2.org/articles/node_lifecycle.html
- **Doxygen Manual**: https://www.doxygen.nl/manual/
- **Google C++ Style Guide**: https://google.github.io/styleguide/cppguide.html
- **ISO 13849-1**: Safety of machinery (PL d requirements)

**Version 4.1.0** | **API Documentation** | **ULTRABOT Team**
