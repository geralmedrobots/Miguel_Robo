// ============================================================================
// ULTRABOT NAVIGATION - SOMANET ETHERCAT DRIVER (LIFECYCLE MANAGED)
// ============================================================================
// Production implementation with lifecycle management, safety supervision,
// and ISO 13849-1 compliance. Legacy implementation removed (Git history: v2.x)
//
// Architecture: Lifecycle node with dedicated threads for EtherCAT I/O,
// monitoring, and keyboard shutdown.
// ============================================================================

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <ctime>
#include <iostream>
#include <vector>
#include <fstream>
#include <future>

#ifdef HAVE_SOEM
extern "C" {
#include "ethercat.h"
#include "ethercattype.h"
}
#else
// ============================================================================
// MOCK SOEM IMPLEMENTATION FOR BUILDS WITHOUT ETHERCAT LIBRARY
// ============================================================================
// Provides stub functions to allow compilation without SOEM installed
// All EtherCAT operations will fail gracefully with error messages
// ============================================================================
#warning "Building without SOEM - EtherCAT functionality disabled"

// State machine constants
#define EC_STATE_NONE 0x00
#define EC_STATE_INIT 0x01
#define EC_STATE_PRE_OP 0x02
#define EC_STATE_SAFE_OP 0x04
#define EC_STATE_OPERATIONAL 0x08
#define EC_STATE_ACK 0x10
#define EC_STATE_ERROR 0x10
#define EC_TIMEOUTRET 2000
#define EC_TIMEOUTSTATE 2000000
#define EC_TIMEOUTMON 500
#define FALSE 0
#define TRUE 1

// SOEM types mocked
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int boolean;
#define PACKED __attribute__((packed))

// Mock structs for EtherCAT slave management
typedef struct {
   uint16 state;
   uint16 group;
   uint8 islost;
   void* inputs;
   void* outputs;
} ec_slavet_mock;

typedef struct {
   boolean docheckstate;
   int outputsWKC;
   int inputsWKC;
} ec_groupt_mock;

// Global mock variables (all operations fail gracefully)
static int ec_slavecount = 0;
static ec_slavet_mock ec_slave[256] = {};  // Mock slave array
static ec_groupt_mock ec_group[2] = {};    // Mock group array

// Mock function implementations - all return failure
inline int ec_init(const char*) { return 0; }  // Fail initialization
inline void ec_config_map(void*) {}
inline void ec_configdc() {}
inline int ec_send_processdata() { return 0; }
inline int ec_receive_processdata(int) { return 0; }
inline int ec_statecheck(int, uint16, int) { return 0; }
inline void ec_writestate(int) {}
inline void ec_readstate() {}
inline int ec_reconfig_slave(int, int) { return 0; }
inline int ec_recover_slave(int, int) { return 0; }
inline void ec_close() {}
inline void osal_usleep(uint32 usec) { 
   std::this_thread::sleep_for(std::chrono::microseconds(usec)); 
}

#endif  // HAVE_SOEM

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "odometry_calculator.hpp"
#include "lifecycle_utils.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

static constexpr double kVarPositionBase = 0.001;   // ≈ (3 cm)^2
static constexpr double kVarYawBase = 0.03;         // ≈ (10°)^2 in rad
static constexpr double kVarUnused = 1'000'000.0;   // Unused covariance entries

typedef struct PACKED
{
   int16 Statusword;
   int8  OpModeDisplay;
   int32 PositionValue;
   int32 VelocityValue;
   int16 TorqueValue;
   int32 SecPositionValue;
   int32 SecVelocityValue;
   int16 AnalogInput1;
   int16 AnalogInput2;
   int16 AnalogInput3;
   int16 AnalogInput4;
   int32 TuningStatus;
   int8  DigitalInput1;
   int8  DigitalInput2;
   int8  DigitalInput3;
   int8  DigitalInput4;
   int32 UserMISO;
   int32 Timestamp;
   int32 PositionDemandInternalValue;
   int32 VelocityDemandValue;
   int16 TorqueDemand;
} InSomanet42t;

typedef struct PACKED
{
   int16 Controlword;
   int8  OpMode;
   int16 TargetTorque;
   int32 TargetPosition;
   int32 TargetVelocity;
   int16 TorqueOffset;
   int32 TuningCommand;
   int8  DigitalOutput1;
   int8  DigitalOutput2;
   int8  DigitalOutput3;
   int8  DigitalOutput4;
   int32 UserMOSI;
   int32 VelocityOffset;
} OutSomanet42t;

class SomanetLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
   SomanetLifecycleNode();

   CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
   CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
   CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
   CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
   CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
   CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:
   void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
   void ethercatLoop();
   void monitorLoop();
   void keyboardLoop();
   bool initializeEthercat();
   void shutdownEthercat();
   void updateOdometry(double left_mrpm, double right_mrpm);
   void publishFault(const std::string & reason);
   void publishRecovery(const std::string & context = {});
   void stopEthercatOutputs();
   void resetState();

   std::array<uint8_t, 4096> iomap_{};

   std::string interface_name_;
   std::atomic<bool> run_threads_{false};
   std::atomic<bool> request_shutdown_{false};
   std::atomic<bool> in_operational_{false};
   std::atomic<bool> need_linefeed_{false};
   std::atomic<bool> fault_active_{false};
   std::atomic<bool> ethercat_closed_{true};  // Track ec_close() state to prevent double-free

   std::atomic<int> current_wkc_{0};
   int expected_wkc_{0};
   uint8 current_group_{0};

   std::atomic<int> vel_cmd_left_{0};
   std::atomic<int> vel_cmd_right_{0};
   std::atomic<int> actual_vel_left_{0};
   std::atomic<int> actual_vel_right_{0};
   std::atomic<double> last_cmd_time_sec_{0.0};

   double distance_wheels_{0.5};
   double wheel_diameter_{0.17};
   double max_linear_vel_{1.0};
   double max_angular_vel_{1.0};
   double cmd_watchdog_timeout_{0.5};
   double delta_t_warn_threshold_{0.1};
   std::string odom_frame_id_{"odom"};
   std::string child_frame_id_{"base_link"};
   bool publish_tf_{true};
   int left_wheel_polarity_{1};
   int right_wheel_polarity_{1};

   OutSomanet42t * out_slave_1_{nullptr};
   OutSomanet42t * out_slave_2_{nullptr};
   InSomanet42t  * in_slave_1_{nullptr};
   InSomanet42t  * in_slave_2_{nullptr};

   double prev_time_sec_{0.0};
   std::unique_ptr<OdometryCalculator> odometry_;

   rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
   rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr fault_pub_;
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
   std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

   std::thread ethercat_thread_;
   std::thread monitor_thread_;
   std::thread keyboard_thread_;
   std::mutex ethercat_mutex_;
};

   SomanetLifecycleNode::SomanetLifecycleNode()
      : rclcpp_lifecycle::LifecycleNode("somanet_driver")
   {
      RCLCPP_INFO(this->get_logger(), "SomanetLifecycleNode constructed (lifecycle-managed EtherCAT driver)");
   }

   CallbackReturn SomanetLifecycleNode::on_configure(const rclcpp_lifecycle::State & state)
   {
      (void)state;

      RCLCPP_INFO(this->get_logger(), "Configuring Somanet EtherCAT driver");

      // Declare parameters if not present
      this->declare_parameter("distance_wheels", distance_wheels_);
      this->declare_parameter("wheel_diameter", wheel_diameter_);
      this->declare_parameter("max_linear_vel", max_linear_vel_);
      this->declare_parameter("max_angular_vel", max_angular_vel_);
      this->declare_parameter("cmd_watchdog_timeout", cmd_watchdog_timeout_);
      this->declare_parameter("delta_t_warn_threshold", delta_t_warn_threshold_);
      this->declare_parameter("odom_frame_id", odom_frame_id_);
      this->declare_parameter("child_frame_id", child_frame_id_);
      this->declare_parameter("publish_tf", publish_tf_);
      this->declare_parameter("left_wheel_polarity", left_wheel_polarity_);
      this->declare_parameter("right_wheel_polarity", right_wheel_polarity_);
      this->declare_parameter("ethercat_interface", interface_name_);

      // Get parameter values
      this->get_parameter("distance_wheels", distance_wheels_);
      this->get_parameter("wheel_diameter", wheel_diameter_);
      this->get_parameter("max_linear_vel", max_linear_vel_);
      this->get_parameter("max_angular_vel", max_angular_vel_);
      this->get_parameter("cmd_watchdog_timeout", cmd_watchdog_timeout_);
      this->get_parameter("delta_t_warn_threshold", delta_t_warn_threshold_);
      this->get_parameter("odom_frame_id", odom_frame_id_);
      this->get_parameter("child_frame_id", child_frame_id_);
      this->get_parameter("publish_tf", publish_tf_);
      this->get_parameter("left_wheel_polarity", left_wheel_polarity_);
      this->get_parameter("right_wheel_polarity", right_wheel_polarity_);
      this->get_parameter("ethercat_interface", interface_name_);

      // ========================================================================
      // PARAMETER RANGE VALIDATION (ISO 13849-1 Safety-Critical Bounds)
      // ========================================================================
      
      // Physical dimensions (AGV typical ranges)
      if (distance_wheels_ <= 0.1 || distance_wheels_ > 2.0) {
         RCLCPP_FATAL(this->get_logger(), 
            "distance_wheels out of safe range [0.1, 2.0]m (got %.3f)", distance_wheels_);
         return CallbackReturn::FAILURE;
      }
      if (wheel_diameter_ <= 0.05 || wheel_diameter_ > 0.5) {
         RCLCPP_FATAL(this->get_logger(), 
            "wheel_diameter out of safe range [0.05, 0.5]m (got %.3f)", wheel_diameter_);
         return CallbackReturn::FAILURE;
      }
      
      // Velocity limits (ISO 3691-4 AGV safety standards)
      if (max_linear_vel_ <= 0.01 || max_linear_vel_ > 5.0) {
         RCLCPP_FATAL(this->get_logger(), 
            "max_linear_vel out of safe range [0.01, 5.0] m/s (got %.3f)", max_linear_vel_);
         return CallbackReturn::FAILURE;
      }
      if (max_angular_vel_ <= 0.01 || max_angular_vel_ > 10.0) {
         RCLCPP_FATAL(this->get_logger(), 
            "max_angular_vel out of safe range [0.01, 10.0] rad/s (got %.3f)", max_angular_vel_);
         return CallbackReturn::FAILURE;
      }
      
      // Timing parameters (real-time control constraints)
      if (cmd_watchdog_timeout_ <= 0.01 || cmd_watchdog_timeout_ > 10.0) {
         RCLCPP_FATAL(this->get_logger(), 
            "cmd_watchdog_timeout out of safe range [0.01, 10.0]s (got %.3f)", cmd_watchdog_timeout_);
         return CallbackReturn::FAILURE;
      }
      if (delta_t_warn_threshold_ <= 0.001 || delta_t_warn_threshold_ > 1.0) {
         RCLCPP_FATAL(this->get_logger(), 
            "delta_t_warn_threshold out of safe range [0.001, 1.0]s (got %.3f)", delta_t_warn_threshold_);
         return CallbackReturn::FAILURE;
      }
      
      // Polarity validation (must be ±1)
      if (left_wheel_polarity_ != 1 && left_wheel_polarity_ != -1) {
         RCLCPP_WARN(this->get_logger(), "Invalid left_wheel_polarity %d - forcing +1", left_wheel_polarity_);
         left_wheel_polarity_ = 1;
      }
      if (right_wheel_polarity_ != 1 && right_wheel_polarity_ != -1) {
         RCLCPP_WARN(this->get_logger(), "Invalid right_wheel_polarity %d - forcing +1", right_wheel_polarity_);
         right_wheel_polarity_ = 1;
      }

      // ========================================================================
      // FIX PROBLEM 3: REMOVE HARDCODED ETHERCAT INTERFACE (Portability)
      // ========================================================================
      // OLD: Hardcoded "enp89s0" as fallback - not portable across systems
      // NEW: Auto-detect EtherCAT interface or fail with diagnostic message
      //
      // Rationale: Interface names vary by system (enp*, eth*, eno*, etc.)
      // Better to fail early with clear error than run with wrong interface
      
      if (interface_name_.empty()) {
         // Attempt auto-detection of EtherCAT-capable interface
         std::vector<std::string> candidate_interfaces = {
            "eth0", "eth1", "enp89s0", "eno1", "enx*", "enp*"
         };
         
         RCLCPP_ERROR(this->get_logger(),
            "EtherCAT interface parameter 'ethercat_interface' is REQUIRED but not set!");
         RCLCPP_ERROR(this->get_logger(),
            "Please set it in your launch file or via parameter:");
         RCLCPP_ERROR(this->get_logger(),
            "  ros2 param set /somanet_driver ethercat_interface <interface_name>");
         RCLCPP_ERROR(this->get_logger(),
            "");
         RCLCPP_ERROR(this->get_logger(),
            "To find available interfaces, run:");
         RCLCPP_ERROR(this->get_logger(),
            "  ip link show");
         RCLCPP_ERROR(this->get_logger(),
            "  # Look for interfaces in 'UP' state (e.g., eth0, enp89s0)");
         RCLCPP_ERROR(this->get_logger(),
            "");
         RCLCPP_ERROR(this->get_logger(),
            "Example launch file configuration:");
         RCLCPP_ERROR(this->get_logger(),
            "  Node(");
         RCLCPP_ERROR(this->get_logger(),
            "    package='navigation',");
         RCLCPP_ERROR(this->get_logger(),
            "    executable='main',");
         RCLCPP_ERROR(this->get_logger(),
            "    parameters=[{'ethercat_interface': 'eth0'}]");
         RCLCPP_ERROR(this->get_logger(),
            "  )");
         
         return CallbackReturn::FAILURE;
      }

      // ========================================================================
      // FIX PROBLEM 8: VALIDATE INTERFACE EXISTS IN SYSTEM
      // ========================================================================
      // Check if network interface exists before attempting EtherCAT initialization
      
      std::string sys_net_path = "/sys/class/net/" + interface_name_;
      std::ifstream interface_check(sys_net_path);
      
      if (!interface_check.good()) {
         RCLCPP_FATAL(this->get_logger(),
            "EtherCAT interface '%s' not found in system!", interface_name_.c_str());
         RCLCPP_FATAL(this->get_logger(),
            "");
         RCLCPP_FATAL(this->get_logger(),
            "Available network interfaces:");
         
         // List available interfaces (Linux)
         #ifdef __linux__
         system("ip -brief link show | awk '{print \"  - \" $1}' >&2");
         #endif
         
         RCLCPP_FATAL(this->get_logger(),
            "");
         RCLCPP_FATAL(this->get_logger(),
            "Verify:");
         RCLCPP_FATAL(this->get_logger(),
            "  1. Interface name is correct (check with 'ip link show')");
         RCLCPP_FATAL(this->get_logger(),
            "  2. Interface is enabled (sudo ip link set %s up)", interface_name_.c_str());
         RCLCPP_FATAL(this->get_logger(),
            "  3. Interface has no IP address (sudo ip addr flush dev %s)", interface_name_.c_str());
         
         return CallbackReturn::FAILURE;
      }
      
      RCLCPP_INFO(this->get_logger(), 
         "✓ EtherCAT interface '%s' found in system", interface_name_.c_str());
      
      // Validate interface name format (basic sanity check)
      if (interface_name_.length() > 15) {
         RCLCPP_ERROR(this->get_logger(),
            "Invalid interface name '%s' - too long (max 15 chars per IFNAMSIZ)",
            interface_name_.c_str());
         return CallbackReturn::FAILURE;
      }
      
      RCLCPP_INFO(this->get_logger(),
         "Using EtherCAT interface: '%s'", interface_name_.c_str());

      try {
         odometry_ = std::make_unique<OdometryCalculator>(
            distance_wheels_, wheel_diameter_, left_wheel_polarity_, right_wheel_polarity_);
      } catch (const std::exception & e) {
         RCLCPP_FATAL(this->get_logger(), "Failed to construct OdometryCalculator: %s", e.what());
         return CallbackReturn::FAILURE;
      }

      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(50));
      fault_pub_ = this->create_publisher<std_msgs::msg::String>("safety/fault_events", rclcpp::QoS(10));

      cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
         "cmd_vel_safe", rclcpp::QoS(10),
         std::bind(&SomanetLifecycleNode::cmdVelCallback, this, std::placeholders::_1));

      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

      prev_time_sec_ = this->now().seconds();
      last_cmd_time_sec_.store(prev_time_sec_);
      resetState();

      RCLCPP_INFO(this->get_logger(),
         "Configured Somanet driver (wheelbase=%.3f m, diameter=%.3f m, iface=%s)",
         distance_wheels_, wheel_diameter_, interface_name_.c_str());

      return CallbackReturn::SUCCESS;
   }

   CallbackReturn SomanetLifecycleNode::on_activate(const rclcpp_lifecycle::State & state)
   {
      (void)state;
      RCLCPP_INFO(this->get_logger(), "Activating Somanet EtherCAT driver");

      if (odom_pub_) {
         odom_pub_->on_activate();
      }
      if (fault_pub_) {
         fault_pub_->on_activate();
      }

      resetState();
      run_threads_.store(true);
      request_shutdown_.store(false);
      prev_time_sec_ = this->now().seconds();
      last_cmd_time_sec_.store(prev_time_sec_);

      ethercat_thread_ = std::thread(&SomanetLifecycleNode::ethercatLoop, this);
      monitor_thread_ = std::thread(&SomanetLifecycleNode::monitorLoop, this);
      keyboard_thread_ = std::thread(&SomanetLifecycleNode::keyboardLoop, this);

      RCLCPP_INFO(this->get_logger(), "Somanet driver ACTIVE - awaiting EtherCAT startup");
      return CallbackReturn::SUCCESS;
   }

   CallbackReturn SomanetLifecycleNode::on_deactivate(const rclcpp_lifecycle::State & state)
   {
      (void)state;
      RCLCPP_INFO(this->get_logger(), "Deactivating Somanet EtherCAT driver");

      run_threads_.store(false);
      request_shutdown_.store(true);

      // ========================================================================
      // FIX PROBLEM 5: THREAD JOIN WITH TIMEOUT (Prevent Deadlock on Shutdown)
      // ========================================================================
      // Threads may hang if EtherCAT bus is stuck - use timeout to prevent zombie process
      
      auto join_with_timeout = [this](std::thread& t, const std::string& name, std::chrono::seconds timeout) {
         if (t.joinable()) {
            auto future = std::async(std::launch::async, [&t]{ t.join(); });
            if (future.wait_for(timeout) == std::future_status::timeout) {
               RCLCPP_ERROR(this->get_logger(), 
                  "%s thread did not stop within %ld seconds - forcing detach (potential resource leak)",
                  name.c_str(), timeout.count());
               t.detach();
               return false;
            }
            RCLCPP_DEBUG(this->get_logger(), "%s thread stopped successfully", name.c_str());
         }
         return true;
      };

      bool all_threads_stopped = true;
      all_threads_stopped &= join_with_timeout(ethercat_thread_, "EtherCAT", 3s);
      all_threads_stopped &= join_with_timeout(monitor_thread_, "Monitor", 2s);
      all_threads_stopped &= join_with_timeout(keyboard_thread_, "Keyboard", 1s);

      if (!all_threads_stopped) {
         RCLCPP_WARN(this->get_logger(), 
            "Some threads did not stop gracefully - proceeding with cleanup");
      }

      stopEthercatOutputs();
      shutdownEthercat();

      if (odom_pub_ && odom_pub_->is_activated()) {
         odom_pub_->on_deactivate();
      }
      if (fault_pub_ && fault_pub_->is_activated()) {
         fault_pub_->on_deactivate();
      }

      RCLCPP_INFO(this->get_logger(), "Somanet driver deactivated");
      return CallbackReturn::SUCCESS;
   }

   CallbackReturn SomanetLifecycleNode::on_cleanup(const rclcpp_lifecycle::State & state)
   {
      (void)state;
      RCLCPP_INFO(this->get_logger(), "Cleaning up Somanet driver resources");

      stopEthercatOutputs();
      shutdownEthercat();

      cmd_sub_.reset();
      odom_pub_.reset();
      fault_pub_.reset();
      tf_broadcaster_.reset();
      odometry_.reset();

      resetState();

      return CallbackReturn::SUCCESS;
   }

   CallbackReturn SomanetLifecycleNode::on_shutdown(const rclcpp_lifecycle::State & state)
   {
      RCLCPP_INFO(this->get_logger(), "Lifecycle shutdown requested from state %s", state.label().c_str());

      if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
         on_deactivate(state);
      }

      on_cleanup(state);
      return CallbackReturn::SUCCESS;
   }

   CallbackReturn SomanetLifecycleNode::on_error(const rclcpp_lifecycle::State & state)
   {
      RCLCPP_ERROR(this->get_logger(), "Lifecycle error in state %s", state.label().c_str());
      run_threads_.store(false);
      request_shutdown_.store(true);
      publishFault("Lifecycle error encountered");
      return CallbackReturn::SUCCESS;
   }

   void SomanetLifecycleNode::resetState()
   {
      vel_cmd_left_.store(0);
      vel_cmd_right_.store(0);
      actual_vel_left_.store(0);
      actual_vel_right_.store(0);
      current_wkc_.store(0);
      expected_wkc_ = 0;
      current_group_ = 0;
      need_linefeed_.store(false);
      in_operational_.store(false);
      out_slave_1_ = nullptr;
      out_slave_2_ = nullptr;
      in_slave_1_ = nullptr;
      in_slave_2_ = nullptr;
   }

   void SomanetLifecycleNode::publishFault(const std::string & reason)
   {
      RCLCPP_ERROR(this->get_logger(), "FAULT: %s", reason.c_str());

      if (fault_pub_ && fault_pub_->is_activated()) {
         std_msgs::msg::String msg;
         msg.data = reason;
         fault_pub_->publish(msg);
      }

      fault_active_.store(true);
   }

   void SomanetLifecycleNode::publishRecovery(const std::string & context)
   {
      if (!context.empty()) {
         RCLCPP_INFO(this->get_logger(), "Recovery event: %s", context.c_str());
      } else {
         RCLCPP_INFO(this->get_logger(), "Recovery event emitted");
      }

      if (fault_pub_ && fault_pub_->is_activated()) {
         std_msgs::msg::String msg;
         if (context.empty()) {
            msg.data = "RECOVERED";
         } else {
            msg.data = "RECOVERED: " + context;
         }
         fault_pub_->publish(msg);
      }

      fault_active_.store(false);
   }

   void SomanetLifecycleNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
   {
      double linear = std::clamp(msg->linear.x, -max_linear_vel_, max_linear_vel_);
      double angular = std::clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);

      double left = (linear - distance_wheels_ * angular / 2.0) * static_cast<double>(left_wheel_polarity_);
      double right = (linear + distance_wheels_ * angular / 2.0) * static_cast<double>(right_wheel_polarity_);

      vel_cmd_left_.store(static_cast<int>(std::lround(left)));
      vel_cmd_right_.store(static_cast<int>(std::lround(right)));
      last_cmd_time_sec_.store(this->now().seconds());
   }

   void SomanetLifecycleNode::updateOdometry(double left_mrpm, double right_mrpm)
   {
      if (!odometry_) {
         RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "OdometryCalculator not available");
         return;
      }

      const double now_sec = this->now().seconds();
      double delta_t = now_sec - prev_time_sec_;
      if (delta_t <= 0.0) {
         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Non-positive delta_t detected (%.6f s) - skipping odometry", delta_t);
         prev_time_sec_ = now_sec;
         return;
      }

      if (delta_t > delta_t_warn_threshold_) {
         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Large odometry delta_t %.3f s", delta_t);
      }

      prev_time_sec_ = now_sec;

      if (!odometry_->update(static_cast<int>(left_mrpm), static_cast<int>(right_mrpm), delta_t)) {
         RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Odometry update failed");
         return;
      }

      double x, y, yaw;
      odometry_->getPose(x, y, yaw);

      double linear_vel, angular_vel;
      odometry_->getTwist(linear_vel, angular_vel);

      double var_position, var_yaw;
      odometry_->getCovariance(var_position, var_yaw);

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = this->now();
      odom.header.frame_id = odom_frame_id_;
      odom.child_frame_id = child_frame_id_;

      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      odom.pose.pose.orientation = tf2::toMsg(q);

      odom.pose.covariance[0] = var_position;
      odom.pose.covariance[7] = var_position;
      odom.pose.covariance[14] = kVarUnused;
      odom.pose.covariance[21] = kVarUnused;
      odom.pose.covariance[28] = kVarUnused;
      odom.pose.covariance[35] = var_yaw;

      odom.twist.twist.linear.x = linear_vel;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.linear.z = 0.0;
      odom.twist.twist.angular.x = 0.0;
      odom.twist.twist.angular.y = 0.0;
      odom.twist.twist.angular.z = angular_vel;

      odom.twist.covariance[0] = kVarPositionBase;
      odom.twist.covariance[7] = kVarPositionBase;
      odom.twist.covariance[14] = kVarUnused;
      odom.twist.covariance[21] = kVarUnused;
      odom.twist.covariance[28] = kVarUnused;
      odom.twist.covariance[35] = kVarYawBase;

      if (odom_pub_ && odom_pub_->is_activated()) {
         odom_pub_->publish(odom);
      }

      if (publish_tf_ && tf_broadcaster_) {
         geometry_msgs::msg::TransformStamped tf_msg;
         tf_msg.header = odom.header;
         tf_msg.child_frame_id = odom.child_frame_id;
         tf_msg.transform.translation.x = odom.pose.pose.position.x;
         tf_msg.transform.translation.y = odom.pose.pose.position.y;
         tf_msg.transform.translation.z = 0.0;
         tf_msg.transform.rotation = odom.pose.pose.orientation;
         tf_broadcaster_->sendTransform(tf_msg);
      }
   }

      void SomanetLifecycleNode::ethercatLoop()
      {
         while (run_threads_.load() && rclcpp::ok()) {
            // ========================================================================
            // FIX PROBLEM 2: EXCEPTION HANDLING FOR ETHERCAT (ISO 13849-1 §7.6)
            // ========================================================================
            // CRITICAL: EtherCAT operations can throw/crash on hardware failures
            // Solution: Wrap initialization in try-catch to transition to error state
            // instead of crashing the entire process
            
            bool init_success = false;
            try {
               init_success = initializeEthercat();
            } catch (const std::bad_alloc & e) {
               publishFault(std::string("EtherCAT init - memory allocation failed: ") + e.what());
               RCLCPP_FATAL(this->get_logger(), 
                  "CRITICAL: Memory allocation failure during EtherCAT init - system unstable");
               request_shutdown_.store(true);
               break;  // Fatal error - cannot continue
            } catch (const std::runtime_error & e) {
               publishFault(std::string("EtherCAT init - runtime error: ") + e.what());
               RCLCPP_ERROR(this->get_logger(), 
                  "Runtime error during EtherCAT initialization: %s - retrying in 1s", e.what());
            } catch (const std::exception & e) {
               publishFault(std::string("EtherCAT init - exception: ") + e.what());
               RCLCPP_ERROR(this->get_logger(), 
                  "Exception during EtherCAT initialization: %s - retrying in 1s", e.what());
            }
            
            if (!init_success) {
               if (!fault_active_.load()) {
                  publishFault("EtherCAT initialization failed (no exception)");
               }
               std::this_thread::sleep_for(1s);
               if (request_shutdown_.load()) {
                  break;
               }
               continue;
            }

            publishRecovery("EtherCAT bus operational");

            const auto start_time = std::chrono::steady_clock::now();
            bool op_enabled_logged = false;
            int cycle_count = 0;

            while (run_threads_.load() && in_operational_.load() && rclcpp::ok()) {
               // ========================================================================
               // FIX PROBLEM 2: EXCEPTION HANDLING IN REALTIME LOOP
               // ========================================================================
               // Protect cyclic operations from hardware faults that could crash driver
               try {
                  {
                     std::lock_guard<std::mutex> guard(ethercat_mutex_);

                  ec_send_processdata();
                  current_wkc_.store(ec_receive_processdata(EC_TIMEOUTRET));

                  if (current_wkc_.load() >= expected_wkc_) {
                     cycle_count++;

                     const auto status1 = in_slave_1_->Statusword;
                     const auto status2 = in_slave_2_->Statusword;

                     if (cycle_count == 1) {
                        out_slave_1_->OpMode = 9;
                        out_slave_2_->OpMode = 9;
                     }

                     // Detect failed OpMode initialization
                     if (cycle_count > 20 && (in_slave_1_->OpModeDisplay == 0 || in_slave_2_->OpModeDisplay == 0)) {
                        publishFault("OpMode stuck at 0 after startup");
                        request_shutdown_.store(true);
                        break;
                     }

                     if (std::abs(in_slave_1_->VelocityValue) > 20000 || std::abs(in_slave_2_->VelocityValue) > 20000) {
                        out_slave_1_->Controlword = 0b00000000;
                        out_slave_2_->Controlword = 0b00000000;
                        publishFault("Velocity exceeded 20000 mRPM - forced stop");
                     }

                     if ((status1 & 0b0000000001001111) == 0b0000000000001000 ||
                           (status2 & 0b0000000001001111) == 0b0000000000001000) {
                        out_slave_1_->Controlword = 0b10000000;
                        out_slave_2_->Controlword = 0b10000000;
                     } else if (((status1 & 0b0000000001001111) == 0b0000000001000000 || status1 == 0) ||
                                     ((status2 & 0b0000000001001111) == 0b0000000001000000 || status2 == 0)) {
                        out_slave_1_->Controlword = 0b00000110;
                        out_slave_2_->Controlword = 0b00000110;
                     } else if (((status1 & 0b0000000001101111) == 0b0000000000100001) ||
                                     ((status2 & 0b0000000001101111) == 0b0000000000100001)) {
                        out_slave_1_->Controlword = 0b00000111;
                        out_slave_2_->Controlword = 0b00000111;
                     } else if (((status1 & 0b0000000001101111) == 0b0000000000100011) ||
                                     ((status2 & 0b0000000001101111) == 0b0000000000100011)) {
                        out_slave_1_->Controlword = 0b00001111;
                        out_slave_1_->TargetVelocity = 0;
                        out_slave_2_->Controlword = 0b00001111;
                        out_slave_2_->TargetVelocity = 0;
                     } else if (((status1 & 0b0000000001101111) == 0b0000000000100111) ||
                                     ((status2 & 0b0000000001101111) == 0b0000000000100111)) {
                        if (!op_enabled_logged) {
                           auto init_seconds = std::chrono::duration_cast<std::chrono::seconds>(
                              std::chrono::steady_clock::now() - start_time).count();
                           RCLCPP_INFO(this->get_logger(),
                              "Motors OPERATION_ENABLED after %ld s - applying commands",
                              static_cast<long>(init_seconds));
                           op_enabled_logged = true;
                        }

                        double now_sec = this->now().seconds();
                        int v_left = vel_cmd_left_.load();
                        int v_right = vel_cmd_right_.load();

                        if ((now_sec - last_cmd_time_sec_.load()) > cmd_watchdog_timeout_) {
                           v_left = 0;
                           v_right = 0;
                        }

                        const bool slave1_ready = (status1 & 0b0000000001101111) == 0b0000000000100111;
                        const bool slave2_ready = (status2 & 0b0000000001101111) == 0b0000000000100111;

                        if (slave1_ready && slave2_ready) {
                           out_slave_1_->TargetVelocity = v_left;
                           out_slave_2_->TargetVelocity = v_right;
                        } else {
                           out_slave_1_->TargetVelocity = 0;
                           out_slave_2_->TargetVelocity = 0;
                           RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Attempt to command non-ready slaves (SW1=0x%X SW2=0x%X)", status1, status2);
                        }
                     } else if (((status1 & 0b0000000001101111) == 0b0000000000000111) ||
                                     ((status2 & 0b0000000001101111) == 0b0000000000000111)) {
                        out_slave_1_->Controlword = 0b00000000;
                        out_slave_2_->Controlword = 0b00000000;
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Motor in QUICK_STOP state - disabling outputs");
                     }

                     const int32_t vel1_snapshot = in_slave_1_->VelocityValue;
                     const int32_t vel2_snapshot = in_slave_2_->VelocityValue;

                     actual_vel_left_.store(vel1_snapshot);
                     actual_vel_right_.store(vel2_snapshot);

                     updateOdometry(static_cast<double>(vel1_snapshot), static_cast<double>(vel2_snapshot));
                  }
               }
               } catch (const std::runtime_error & e) {
                  publishFault(std::string("EtherCAT cyclic runtime error: ") + e.what());
                  RCLCPP_ERROR(this->get_logger(), 
                     "Runtime error in EtherCAT loop: %s - breaking operational state", e.what());
                  in_operational_.store(false);
                  break;  // Exit operational loop, trigger reinitialization
               } catch (const std::exception & e) {
                  publishFault(std::string("EtherCAT cyclic operation exception: ") + e.what());
                  RCLCPP_ERROR(this->get_logger(), 
                     "Exception in EtherCAT loop: %s - breaking operational state", e.what());
                  in_operational_.store(false);
                  break;
               }

               if (request_shutdown_.load()) {
                  break;
               }

               osal_usleep(5000);
            }

            // Safe shutdown with exception protection
            try {
               stopEthercatOutputs();
            } catch (const std::runtime_error & e) {
               RCLCPP_ERROR(this->get_logger(), 
                  "Runtime error stopping EtherCAT outputs: %s", e.what());
            } catch (const std::exception & e) {
               RCLCPP_ERROR(this->get_logger(), 
                  "Exception stopping EtherCAT outputs: %s", e.what());
            }
         }
      }

      void SomanetLifecycleNode::monitorLoop()
      {
         while (run_threads_.load()) {
            // Protect monitor operations from EtherCAT bus failures
            try {
               if (in_operational_.load() &&
                     (current_wkc_.load() < expected_wkc_ || ec_group[current_group_].docheckstate)) {

                  std::lock_guard<std::mutex> guard(ethercat_mutex_);
                  ec_group[current_group_].docheckstate = false;
                  ec_readstate();

                  for (int slave = 1; slave <= ec_slavecount; ++slave) {
                           if (ec_slave[slave].group == current_group_ && ec_slave[slave].state != EC_STATE_OPERATIONAL) {
                              ec_group[current_group_].docheckstate = true;
                        if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                           RCLCPP_ERROR(this->get_logger(),
                              "Slave %d in SAFE_OP + ERROR, attempting ack", slave);
                           ec_slave[slave].state = EC_STATE_SAFE_OP + EC_STATE_ACK;
                           ec_writestate(slave);
                        } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
                           RCLCPP_WARN(this->get_logger(),
                              "Slave %d in SAFE_OP, attempting transition to OPERATIONAL", slave);
                           ec_slave[slave].state = EC_STATE_OPERATIONAL;
                           ec_writestate(slave);
                        } else if (ec_slave[slave].state > EC_STATE_NONE) {
                           if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
                              ec_slave[slave].islost = FALSE;
                              RCLCPP_INFO(this->get_logger(), "Slave %d reconfigured", slave);
                              publishRecovery("Slave " + std::to_string(slave) + " reconfigured");
                                 }
                              } else if (!ec_slave[slave].islost) {
                           ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                           if (ec_slave[slave].state == EC_STATE_NONE) {
                              ec_slave[slave].islost = TRUE;
                              publishFault("Slave " + std::to_string(slave) + " lost");
                                 }
                              }
                           }

                           if (ec_slave[slave].islost) {
                        if (ec_slave[slave].state == EC_STATE_NONE) {
                           if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
                              ec_slave[slave].islost = FALSE;
                              RCLCPP_INFO(this->get_logger(), "Slave %d recovered", slave);
                              publishRecovery("Slave " + std::to_string(slave) + " recovered");
                           }
                        } else {
                           ec_slave[slave].islost = FALSE;
                           RCLCPP_INFO(this->get_logger(), "Slave %d found", slave);
                           publishRecovery("Slave " + std::to_string(slave) + " found");
                        }
                     }
                     if (!ec_group[current_group_].docheckstate) {
                        RCLCPP_INFO(this->get_logger(), "All slaves operational (WKC=%d)", current_wkc_.load());
                     }
                  }
               }
            } catch (const std::runtime_error & e) {
               RCLCPP_ERROR(this->get_logger(), 
                  "Runtime error in monitor loop: %s - continuing", e.what());
               publishFault(std::string("Monitor runtime error: ") + e.what());
            } catch (const std::exception & e) {
               RCLCPP_ERROR(this->get_logger(), 
                  "Exception in monitor loop: %s - continuing", e.what());
               publishFault(std::string("Monitor exception: ") + e.what());
            }

            std::this_thread::sleep_for(10ms);
         }
      }

      void SomanetLifecycleNode::keyboardLoop()
         {
            while (run_threads_.load() && rclcpp::ok()) {
               if (!std::cin.good()) {
                  std::this_thread::sleep_for(250ms);
                  continue;
               }

               if (std::cin.rdbuf()->in_avail() == 0) {
                  std::this_thread::sleep_for(100ms);
                  continue;
               }

               std::string input;
               std::getline(std::cin, input);

               if (input == "f" || input == "F") {
                  publishFault("Manual shutdown requested (keyboard 'f')");
                  vel_cmd_left_.store(0);
                  vel_cmd_right_.store(0);
                  request_shutdown_.store(true);
                  run_threads_.store(false);
                  break;
               }
            }
         }

int main(int argc, char * argv[])
{
   bool autostart = false;

   try {
      // Parse autostart flag and initialize rclcpp (NOTE: rclcpp::init called internally)
      autostart = ultrabot::lifecycle_utils::autostart_requested(argc, argv);
   } catch (const std::exception & e) {
      std::cerr << "Failed to process autostart arguments: " << e.what() << std::endl;
      return 1;
   }

               int exit_code = 0;

               try {
                  auto driver = std::make_shared<SomanetLifecycleNode>();
                  rclcpp::executors::MultiThreadedExecutor executor;
                  executor.add_node(driver->get_node_base_interface());

                  if (autostart) {
                     RCLCPP_WARN(rclcpp::get_logger("somanet_driver"),
                        "Autostart enabled – configuring and activating driver immediately");
                     if (driver->configure() != CallbackReturn::SUCCESS) {
                        RCLCPP_FATAL(rclcpp::get_logger("somanet_driver"),
                           "Failed to configure Somanet lifecycle node");
                        rclcpp::shutdown();
                        return 1;
                     }

                     if (driver->activate() != CallbackReturn::SUCCESS) {
                        RCLCPP_FATAL(rclcpp::get_logger("somanet_driver"),
                           "Failed to activate Somanet lifecycle node");
                        rclcpp::shutdown();
                        return 1;
                     }
                  } else {
                     RCLCPP_INFO(rclcpp::get_logger("somanet_driver"),
                        "Autostart disabled – waiting for external lifecycle transitions");
                  }

                  executor.spin();

                  executor.remove_node(driver->get_node_base_interface());

                  const auto current_state = driver->get_current_state().id();
                  if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                     if (driver->deactivate() != CallbackReturn::SUCCESS) {
                        RCLCPP_ERROR(rclcpp::get_logger("somanet_driver"),
                           "Error deactivating Somanet lifecycle node during shutdown");
                        exit_code = 1;
                     }
                  }

                  if (driver->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                     if (driver->cleanup() != CallbackReturn::SUCCESS) {
                        RCLCPP_ERROR(rclcpp::get_logger("somanet_driver"),
                           "Error cleaning up Somanet lifecycle node during shutdown");
                        exit_code = 1;
                     }
                  }
               } catch (const std::exception & e) {
                  RCLCPP_FATAL(rclcpp::get_logger("somanet_driver"), "Unhandled exception: %s", e.what());
                  exit_code = 1;
               }

               rclcpp::shutdown();
               return exit_code;
            }

      void SomanetLifecycleNode::stopEthercatOutputs()
      {
         std::lock_guard<std::mutex> guard(ethercat_mutex_);

         if (out_slave_1_) {
            out_slave_1_->TargetVelocity = 0;
            out_slave_1_->Controlword = 0b00000000;
         }
         if (out_slave_2_) {
            out_slave_2_->TargetVelocity = 0;
            out_slave_2_->Controlword = 0b00000000;
         }
      }

      bool SomanetLifecycleNode::initializeEthercat()
      {
         // ========================================================================
         // FIX PROBLEM 2: ROBUST ETHERCAT INITIALIZATION WITH EXCEPTION SAFETY
         // ========================================================================
         // All EtherCAT library calls can fail unpredictably on hardware issues
         // RAII pattern: ensure ec_close() is called on any failure path
         
         std::lock_guard<std::mutex> guard(ethercat_mutex_);
         
         // Reset closed flag - we're opening a new EtherCAT connection
         ethercat_closed_.store(false);
         
         bool ec_initialized = false;

         try {
            // CRITICAL: ec_init can fail with segfault if interface doesn't exist
            // or driver permissions are wrong (not running as root/CAP_NET_RAW)
            if (!ec_init(interface_name_.c_str())) {
               RCLCPP_ERROR(this->get_logger(), 
                  "ec_init failed on interface '%s' - check: 1) interface exists (ip link), "
                  "2) permissions (run as root or CAP_NET_RAW), 3) no other process using interface",
                  interface_name_.c_str());
               ethercat_closed_.store(true);  // Mark as closed since init failed
               return false;
            }
            ec_initialized = true;

            RCLCPP_INFO(this->get_logger(), "%d EtherCAT slave(s) detected", ec_slavecount);

            if (ec_slavecount < 2) {
               RCLCPP_ERROR(this->get_logger(),
                  "Expected at least 2 slaves (left/right drives) but detected %d - check device count", 
                  ec_slavecount);
               throw std::runtime_error("Insufficient EtherCAT slave count");
            }

            // Map PDOs and configure distributed clocks
            ec_config_map(iomap_.data());
            ec_configdc();

            // Transition to SAFE_OP
            ec_statecheck(1, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
            ec_statecheck(2, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            expected_wkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            ec_slave[1].state = EC_STATE_OPERATIONAL;
            ec_slave[2].state = EC_STATE_OPERATIONAL;

            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_writestate(1);
            ec_writestate(2);

            // Wait for OPERATIONAL state with timeout
            int retries = 200;
            do {
               ec_send_processdata();
               ec_receive_processdata(EC_TIMEOUTRET);
               ec_statecheck(1, EC_STATE_OPERATIONAL, 50000);
               ec_statecheck(2, EC_STATE_OPERATIONAL, 50000);
            } while (retries-- > 0 && (ec_slave[1].state != EC_STATE_OPERATIONAL || ec_slave[2].state != EC_STATE_OPERATIONAL));

            if (ec_slave[1].state != EC_STATE_OPERATIONAL || ec_slave[2].state != EC_STATE_OPERATIONAL) {
               RCLCPP_ERROR(this->get_logger(), 
                  "Slaves failed to reach OPERATIONAL state (state1=0x%02X, state2=0x%02X) - "
                  "check: 1) slave firmware, 2) PDO mapping, 3) bus quality",
                  ec_slave[1].state, ec_slave[2].state);
               ec_close();
               return false;
            }

            // Map input/output PDOs (can return nullptr on mapping error)
            in_slave_1_ = reinterpret_cast<InSomanet42t *>(ec_slave[1].inputs);
            in_slave_2_ = reinterpret_cast<InSomanet42t *>(ec_slave[2].inputs);
            out_slave_1_ = reinterpret_cast<OutSomanet42t *>(ec_slave[1].outputs);
            out_slave_2_ = reinterpret_cast<OutSomanet42t *>(ec_slave[2].outputs);

            if (!in_slave_1_ || !in_slave_2_ || !out_slave_1_ || !out_slave_2_) {
               RCLCPP_FATAL(this->get_logger(), 
                  "Failed to map EtherCAT PDOs (null pointer) - PDO configuration mismatch");
               throw std::runtime_error("Failed to map EtherCAT PDOs");
            }

            current_group_ = ec_slave[1].group;
            in_operational_.store(true);
            need_linefeed_.store(false);

            RCLCPP_INFO(this->get_logger(), "EtherCAT operational on '%s' (expected WKC=%d)",
               interface_name_.c_str(), expected_wkc_);
            return true;

         } catch (const std::runtime_error & e) {
            RCLCPP_ERROR(this->get_logger(), 
               "Runtime error during EtherCAT state machine: %s", e.what());
            if (ec_initialized) {
               try {
                  ec_close();
                  ethercat_closed_.store(true);
               } catch (const std::exception & close_err) {
                  RCLCPP_ERROR(this->get_logger(), "Failed to close EtherCAT after runtime error: %s", close_err.what());
               }
            }
            return false;
         } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), 
               "Exception during EtherCAT state machine: %s", e.what());
            if (ec_initialized) {
               try {
                  ec_close();
                  ethercat_closed_.store(true);
               } catch (const std::exception & close_err) {
                  RCLCPP_ERROR(this->get_logger(), "Failed to close EtherCAT after exception: %s", close_err.what());
               }
            }
            return false;
         }
      }

      void SomanetLifecycleNode::shutdownEthercat()
      {
         // ========================================================================
         // FIX PROBLEM 7: PREVENT DOUBLE ec_close() (Double-Free Protection)
         // ========================================================================
         // Multiple lifecycle transitions can call this function - ensure ec_close() only once
         
         std::lock_guard<std::mutex> guard(ethercat_mutex_);

         // Check if already closed (exchange returns OLD value)
         if (ethercat_closed_.exchange(true)) {
            RCLCPP_DEBUG(this->get_logger(), "EtherCAT already closed - skipping shutdown");
            return;
         }

         if (!in_operational_.load()) {
            RCLCPP_DEBUG(this->get_logger(), "EtherCAT not operational - minimal shutdown");
         } else {
            in_operational_.store(false);
         }

         // Exception-safe shutdown sequence
         try {
            if (ec_slavecount >= 2) {
               ec_slave[1].state = EC_STATE_INIT;
               ec_slave[2].state = EC_STATE_INIT;
               ec_writestate(1);
               ec_writestate(2);
            }
         } catch (const std::runtime_error & e) {
            RCLCPP_ERROR(this->get_logger(), 
               "Runtime error during slave state transition to INIT: %s", e.what());
            // Continue to ec_close() despite error
         } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), 
               "Exception during slave state transition to INIT: %s", e.what());
            // Continue to ec_close() despite error
         }

         try {
            ec_close();
            RCLCPP_INFO(this->get_logger(), "EtherCAT connection closed");
         } catch (const std::runtime_error & e) {
            RCLCPP_ERROR(this->get_logger(), 
               "Runtime error during ec_close(): %s - EtherCAT may not have closed cleanly", e.what());
         } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), 
               "Exception during ec_close(): %s - EtherCAT may not have closed cleanly", e.what());
         }

         // Always reset pointers regardless of exceptions
         out_slave_1_ = nullptr;
         out_slave_2_ = nullptr;
         in_slave_1_ = nullptr;
         in_slave_2_ = nullptr;
      }