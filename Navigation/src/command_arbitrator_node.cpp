/**
 * @file command_arbitrator_node.cpp
 * @brief Command Arbitrator - Manages command sources with priority-based arbitration
 * 
 * Compliance:
 * - ISO 13849-1: Requires deterministic behavior in multi-source systems
 * - ISO 3691-4: Manual mode must have priority over autonomous
 * 
 * @version 1.0.0
 * @date 2025-10-28
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include "lifecycle_utils.hpp"
#include <algorithm>
#include <chrono>
#include <memory>
#include <map>
#include <string>
#include <functional>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals;

/**
 * @class CommandArbitrator
 * @brief Arbitrates between multiple velocity command sources with priority system
 * 
 * Priority Levels (ISO 3691-4 compliant):
 * 1. EMERGENCY (255) - Emergency stop commands
 * 2. MANUAL (200) - Human teleop (joystick, keyboard)
 * 3. SEMI_AUTO (150) - Supervised autonomous
 * 4. AUTO (100) - Fully autonomous navigation
 * 5. TEST (50) - Testing and validation scripts
 */
class CommandArbitrator : public rclcpp_lifecycle::LifecycleNode
{
public:
  CommandArbitrator();

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:
  // Command source information
  struct CommandSource {
    std::string name;
    uint8_t priority;
    rclcpp::Time last_received;
    geometry_msgs::msg::Twist last_command;
    bool active;
    double timeout;  // seconds
  };

  // Callbacks
  void emergencyCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void manualCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void semiAutoCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void autoCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void testCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  void deadmanCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void arbitrationTimerCallback();
  void diagnosticTimerCallback();
  
  // Arbitration logic
  void processCommand(const std::string& source_id, 
                     const geometry_msgs::msg::Twist::SharedPtr msg);
  void selectAndPublishCommand();
  std::string getActiveSource();
  bool isSourceActive(const CommandSource& source);
  void deactivateTimedOutSources();
  
  // Command sources (priority order)
  std::map<std::string, CommandSource> sources_;
  
  // State
  std::string active_source_;
  std::string previous_source_;
  bool deadman_active_;
  uint64_t source_switches_;
  uint64_t commands_blocked_;
  
  // Configuration
  double default_timeout_;
  bool require_deadman_for_manual_;
  
  // Publishers and subscribers
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_out_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr active_source_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr emergency_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr semi_auto_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr test_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr arbitration_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

CommandArbitrator::CommandArbitrator()
  : rclcpp_lifecycle::LifecycleNode("command_arbitrator"),
    active_source_("none"),
    previous_source_("none"),
    deadman_active_(false),
    source_switches_(0),
    commands_blocked_(0),
    default_timeout_(0.5),
    require_deadman_for_manual_(true)
{
  RCLCPP_INFO(this->get_logger(),
    "Command Arbitrator lifecycle node constructed - awaiting configuration");
}

CommandArbitrator::CallbackReturn CommandArbitrator::on_configure(const rclcpp_lifecycle::State& state)
{
  (void)state;

  RCLCPP_INFO(this->get_logger(), "Configuring Command Arbitrator lifecycle node");

  try {
    if (!this->has_parameter("default_timeout")) {
      this->declare_parameter<double>("default_timeout", default_timeout_);
    }
    if (!this->has_parameter("require_deadman_for_manual")) {
      this->declare_parameter<bool>("require_deadman_for_manual", require_deadman_for_manual_);
    }
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
    // Parameters already declared in previous lifecycle, proceed to fetch values.
  }

  this->get_parameter("default_timeout", default_timeout_);
  this->get_parameter("require_deadman_for_manual", require_deadman_for_manual_);

  auto now = this->now();
  sources_.clear();
  sources_["emergency"] = {"Emergency Stop", 255, now, {}, false, 1.0};
  sources_["manual"] = {"Manual Teleop", 200, now, {}, false, default_timeout_};
  sources_["semi_auto"] = {"Semi-Automatic", 150, now, {}, false, default_timeout_};
  sources_["auto"] = {"Autonomous Nav", 100, now, {}, false, 2.0};
  sources_["test"] = {"Test/Validation", 50, now, {}, false, default_timeout_};

  try {
    cmd_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    active_source_pub_ = this->create_publisher<std_msgs::msg::String>("active_command_source", rclcpp::QoS(10));
    diagnostic_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to create lifecycle publishers: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  // Subscriptions - remain active but callbacks guard lifecycle state.
  emergency_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_emergency", rclcpp::QoS(10),
    std::bind(&CommandArbitrator::emergencyCallback, this, std::placeholders::_1));

  manual_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_manual", rclcpp::QoS(10),
    std::bind(&CommandArbitrator::manualCallback, this, std::placeholders::_1));

  semi_auto_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_semi_auto", rclcpp::QoS(10),
    std::bind(&CommandArbitrator::semiAutoCallback, this, std::placeholders::_1));

  // ========================================================================
  // NAV2 INTEGRATION: Subscribe to command_mux output for autonomous nav
  // ========================================================================
  // command_mux consolidates Nav2 + other autonomous sources
  // Maps to "auto" priority (100) in arbitrator
  auto_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_mux", rclcpp::QoS(10),
    std::bind(&CommandArbitrator::autoCallback, this, std::placeholders::_1));

  test_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_test", rclcpp::QoS(10),
    std::bind(&CommandArbitrator::testCallback, this, std::placeholders::_1));

  deadman_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "deadman_status", rclcpp::QoS(10),
    std::bind(&CommandArbitrator::deadmanCallback, this, std::placeholders::_1));

  arbitration_timer_ = this->create_wall_timer(
    50ms,
    std::bind(&CommandArbitrator::arbitrationTimerCallback, this));
  arbitration_timer_->cancel();

  diagnostic_timer_ = this->create_wall_timer(
    1s,
    std::bind(&CommandArbitrator::diagnosticTimerCallback, this));
  diagnostic_timer_->cancel();

  active_source_ = "none";
  previous_source_ = "none";
  deadman_active_ = false;
  source_switches_ = 0;
  commands_blocked_ = 0;

  RCLCPP_INFO(this->get_logger(),
    "Configured Command Arbitrator - default timeout %.2f s, deadman required for manual: %s",
    default_timeout_, require_deadman_for_manual_ ? "YES" : "NO");

  return CallbackReturn::SUCCESS;
}

CommandArbitrator::CallbackReturn CommandArbitrator::on_activate(const rclcpp_lifecycle::State& state)
{
  (void)state;

  RCLCPP_INFO(this->get_logger(), "Activating Command Arbitrator lifecycle node");

  if (cmd_out_pub_) {
    cmd_out_pub_->on_activate();
  }
  if (active_source_pub_) {
    active_source_pub_->on_activate();
  }
  if (diagnostic_pub_) {
    diagnostic_pub_->on_activate();
  }

  if (arbitration_timer_) {
    arbitration_timer_->reset();
  }
  if (diagnostic_timer_) {
    diagnostic_timer_->reset();
  }

  geometry_msgs::msg::Twist zero_cmd;
  if (cmd_out_pub_ && cmd_out_pub_->is_activated()) {
    cmd_out_pub_->publish(zero_cmd);
  }

  if (active_source_pub_ && active_source_pub_->is_activated()) {
    std_msgs::msg::String source_msg;
    source_msg.data = active_source_;
    active_source_pub_->publish(source_msg);
  }

  RCLCPP_INFO(this->get_logger(), "Command Arbitrator active - ISO 13849-1/ISO 3691-4 compliance enforced");
  return CallbackReturn::SUCCESS;
}

CommandArbitrator::CallbackReturn CommandArbitrator::on_deactivate(const rclcpp_lifecycle::State& state)
{
  (void)state;

  RCLCPP_INFO(this->get_logger(), "Deactivating Command Arbitrator lifecycle node");

  if (arbitration_timer_) {
    arbitration_timer_->cancel();
  }
  if (diagnostic_timer_) {
    diagnostic_timer_->cancel();
  }

  geometry_msgs::msg::Twist zero_cmd;
  if (cmd_out_pub_) {
    if (cmd_out_pub_->is_activated()) {
      cmd_out_pub_->publish(zero_cmd);
      cmd_out_pub_->on_deactivate();
    }
  }
  if (active_source_pub_) {
    if (active_source_pub_->is_activated()) {
      active_source_pub_->on_deactivate();
    }
  }
  if (diagnostic_pub_) {
    if (diagnostic_pub_->is_activated()) {
      diagnostic_pub_->on_deactivate();
    }
  }

  active_source_ = "none";
  previous_source_ = "none";

  return CallbackReturn::SUCCESS;
}

CommandArbitrator::CallbackReturn CommandArbitrator::on_cleanup(const rclcpp_lifecycle::State& state)
{
  (void)state;

  RCLCPP_INFO(this->get_logger(), "Cleaning up Command Arbitrator lifecycle resources");

  if (arbitration_timer_) {
    arbitration_timer_->cancel();
    arbitration_timer_.reset();
  }
  if (diagnostic_timer_) {
    diagnostic_timer_->cancel();
    diagnostic_timer_.reset();
  }

  emergency_sub_.reset();
  manual_sub_.reset();
  semi_auto_sub_.reset();
  auto_sub_.reset();
  test_sub_.reset();
  deadman_sub_.reset();

  cmd_out_pub_.reset();
  active_source_pub_.reset();
  diagnostic_pub_.reset();

  sources_.clear();

  return CallbackReturn::SUCCESS;
}

CommandArbitrator::CallbackReturn CommandArbitrator::on_shutdown(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(this->get_logger(), "Lifecycle shutdown requested from state: %s", state.label().c_str());

  if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    if (on_deactivate(state) != CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Error while deactivating during shutdown");
      return CallbackReturn::FAILURE;
    }
  }

  if (on_cleanup(state) != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Error while cleaning up during shutdown");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CommandArbitrator::CallbackReturn CommandArbitrator::on_error(const rclcpp_lifecycle::State& state)
{
  RCLCPP_ERROR(this->get_logger(), "Lifecycle error encountered in state: %s", state.label().c_str());

  if (arbitration_timer_) {
    arbitration_timer_->cancel();
  }
  if (diagnostic_timer_) {
    diagnostic_timer_->cancel();
  }

  return CallbackReturn::SUCCESS;
}

void CommandArbitrator::emergencyCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  processCommand("emergency", msg);
}

void CommandArbitrator::manualCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  processCommand("manual", msg);
}

void CommandArbitrator::semiAutoCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  processCommand("semi_auto", msg);
}

void CommandArbitrator::autoCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  processCommand("auto", msg);
}

void CommandArbitrator::testCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  processCommand("test", msg);
}

void CommandArbitrator::deadmanCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  deadman_active_ = msg->data;
  
  if (!deadman_active_) {
    RCLCPP_WARN(this->get_logger(), "Deadman released - manual commands will be blocked");
  }
}

void CommandArbitrator::processCommand(
    const std::string& source_id, 
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    commands_blocked_++;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Ignoring command from %s while node is not ACTIVE", source_id.c_str());
    return;
  }

  auto source_it = sources_.find(source_id);
  if (source_it == sources_.end()) {
    RCLCPP_ERROR(this->get_logger(),
      "Received command from unknown source '%s'", source_id.c_str());
    commands_blocked_++;
    return;
  }

  auto& source = source_it->second;

  if (source_id == "manual" && require_deadman_for_manual_ && !deadman_active_) {
    commands_blocked_++;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Blocking manual command - deadman inactive");
    source.active = false;
    return;
  }

  source.last_received = this->now();
  source.last_command = *msg;
  source.active = true;
  
  RCLCPP_DEBUG(this->get_logger(), 
    "Received command from %s (priority %d)", 
    source.name.c_str(), source.priority);
}

void CommandArbitrator::arbitrationTimerCallback() {
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  // Deactivate timed-out sources
  deactivateTimedOutSources();
  
  // Select and publish active command
  selectAndPublishCommand();
  
  // Publish active source
  if (active_source_pub_ && active_source_pub_->is_activated()) {
    std_msgs::msg::String source_msg;
    source_msg.data = active_source_;
    active_source_pub_->publish(source_msg);
  }
}

void CommandArbitrator::deactivateTimedOutSources() {
  auto now = this->now();
  
  for (auto& [id, source] : sources_) {
    if (source.active) {
      double elapsed = (now - source.last_received).seconds();
      if (elapsed > source.timeout) {
        source.active = false;
        RCLCPP_DEBUG(this->get_logger(), 
          "Source %s timed out (%.3f s)", source.name.c_str(), elapsed);
      }
    }
  }
}

void CommandArbitrator::selectAndPublishCommand() {
  // Find highest priority active source
  std::string selected_source = "none";
  uint8_t highest_priority = 0;
  
  for (const auto& [id, source] : sources_) {
    if (!isSourceActive(source)) {
      continue;
    }
    
    if (source.priority > highest_priority) {
      highest_priority = source.priority;
      selected_source = id;
    }
  }
  
  // Check if source changed
  if (selected_source != active_source_) {
    std::string previous_name = "NONE";
    if (active_source_ != "none") {
      auto previous_it = sources_.find(active_source_);
      if (previous_it != sources_.end()) {
        previous_name = previous_it->second.name;
      } else {
        previous_name = active_source_;
      }
    }

    const std::string next_name = selected_source != "none"
      ? sources_.at(selected_source).name
      : std::string("NONE");

    RCLCPP_INFO(this->get_logger(),
      "Command source switched: %s → %s",
      previous_name.c_str(), next_name.c_str());
    
    previous_source_ = active_source_;
    active_source_ = selected_source;
    source_switches_++;
  }
  
  // ========================================================================
  // FIX PROBLEM 8: FAIL-SAFE TIMEOUT BEHAVIOR (ISO 13849-1 §5.2)
  // ========================================================================
  // CRITICAL: On timeout, must publish ZERO velocity (fail-safe)
  // NEVER publish stale/obsolete commands (fail-unsafe violation!)
  //
  // Scenario: Joystick released → timeout → robot MUST stop
  // OLD BEHAVIOR: Published last_command (robot continues moving) ❌
  // NEW BEHAVIOR: Publish zero command (robot stops) ✅
  
  if (selected_source != "none") {
    // Active source: publish its command
    if (cmd_out_pub_ && cmd_out_pub_->is_activated()) {
      cmd_out_pub_->publish(sources_.at(selected_source).last_command);
    }
  } else {
    // NO active source (all timed out) → FAIL-SAFE: ZERO velocity
    geometry_msgs::msg::Twist zero_cmd;  // All fields default to 0.0
    if (cmd_out_pub_ && cmd_out_pub_->is_activated()) {
      cmd_out_pub_->publish(zero_cmd);
    }
    
    // SAFETY: Log timeout events for diagnostics (ISO 13849-1 §7.8)
    static auto last_timeout_log = this->now();
    if ((this->now() - last_timeout_log).seconds() > 2.0) {
      RCLCPP_WARN(this->get_logger(), 
        "⚠️ FAIL-SAFE: All command sources timed out - publishing ZERO velocity");
      last_timeout_log = this->now();
    }
  }
}

bool CommandArbitrator::isSourceActive(const CommandSource& source) {
  if (!source.active) {
    return false;
  }
  
  // Special check for manual: require deadman if configured
  if (source.name == "Manual Teleop" && require_deadman_for_manual_) {
    if (!deadman_active_) {
      return false;
    }
  }
  
  return true;
}

void CommandArbitrator::diagnosticTimerCallback() {
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();
  
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "Command Arbitrator";
  status.hardware_id = "ultrabot_agv";
  
  // Determine status
  int active_count = 0;
  for (const auto& [id, source] : sources_) {
    if (isSourceActive(source)) {
      active_count++;
    }
  }
  
  if (active_source_ == "none") {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "No active command source";
  } else if (active_count > 1) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Multiple sources active - arbitrating by priority";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    try {
      status.message = "Single source active: " + sources_.at(active_source_).name;
    } catch (const std::out_of_range&) {
      status.message = "Single source active: " + active_source_;
    }
  }
  
  // Add metrics
  diagnostic_msgs::msg::KeyValue kv;
  
  kv.key = "active_source";
  kv.value = active_source_;
  status.values.push_back(kv);
  
  kv.key = "source_switches";
  kv.value = std::to_string(source_switches_);
  status.values.push_back(kv);
  
  kv.key = "commands_blocked";
  kv.value = std::to_string(commands_blocked_);
  status.values.push_back(kv);
  
  kv.key = "deadman_active";
  kv.value = deadman_active_ ? "true" : "false";
  status.values.push_back(kv);
  
  // Source statuses
  for (const auto& [id, source] : sources_) {
    kv.key = "source_" + id + "_active";
    kv.value = isSourceActive(source) ? "true" : "false";
    status.values.push_back(kv);
    
    if (source.active) {
      double age = (this->now() - source.last_received).seconds();
      kv.key = "source_" + id + "_age";
      kv.value = std::to_string(age);
      status.values.push_back(kv);
    }
  }
  
  diag_array.status.push_back(status);
  if (diagnostic_pub_ && diagnostic_pub_->is_activated()) {
    diagnostic_pub_->publish(diag_array);
  }
}

int main(int argc, char** argv)
{
  bool autostart = false;

  try {
    // Parse autostart flag and initialize rclcpp (NOTE: rclcpp::init called internally)
    autostart = ultrabot::lifecycle_utils::autostart_requested(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Failed to process autostart arguments: " << e.what() << std::endl;
    return 1;
  }
  
  try {
    auto arbitrator = std::make_shared<CommandArbitrator>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(arbitrator->get_node_base_interface());

    if (autostart) {
      RCLCPP_WARN(rclcpp::get_logger("command_arbitrator"),
        "Autostart enabled – configuring and activating Command Arbitrator immediately");

      auto configure_result = arbitrator->configure();
      if (configure_result != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
        RCLCPP_FATAL(rclcpp::get_logger("command_arbitrator"),
          "Failed to configure CommandArbitrator lifecycle node");
        rclcpp::shutdown();
        return 1;
      }

      auto activate_result = arbitrator->activate();
      if (activate_result != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
        RCLCPP_FATAL(rclcpp::get_logger("command_arbitrator"),
          "Failed to activate CommandArbitrator lifecycle node");
        rclcpp::shutdown();
        return 1;
      }
    } else {
      RCLCPP_INFO(rclcpp::get_logger("command_arbitrator"),
        "Autostart disabled – waiting for external lifecycle transitions");
    }

    executor.spin();

    executor.remove_node(arbitrator->get_node_base_interface());

    if (arbitrator->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      if (arbitrator->deactivate() != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("command_arbitrator"),
          "Error while deactivating CommandArbitrator during shutdown");
      }
    }

    if (arbitrator->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      if (arbitrator->cleanup() != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("command_arbitrator"),
          "Error while cleaning up CommandArbitrator during shutdown");
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("command_arbitrator"), 
      "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
