/**
 * @file safety_supervisor_node.cpp
 * @brief Safety Supervisor Node - Redundant safety layer for AGV control
 * 
 * Compliance:
 * - ISO 13849-1/-2: Functional safety, Category 3/PL d
 * - ISO 3691-4: AGV safety requirements (manual/automatic modes)
 * - IEC 61508: Functional safety of electrical/electronic systems
 * - EN 61800-5-2: Safety functions for drive systems
 * 
 * @version 1.0.0
 * @date 2025-10-28
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <chrono>
#include <memory>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <fstream>  // For file existence checks
#include <functional>
#include <cctype>
#include <filesystem>
#include <optional>
#include <array>
#include <atomic>
#include <system_error>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "certified_params_validator.hpp"
#include "maintenance_mode_manager.hpp"
#include "lifecycle_utils.hpp"
#include <lifecycle_msgs/msg/transition.hpp>
#include <mutex>

namespace {

std::optional<std::string> getEnvironmentVariable(const std::string& name)
{
  if (name.empty()) {
    return std::nullopt;
  }

  const char* value = std::getenv(name.c_str());
  if (value == nullptr || value[0] == '\0') {
    return std::nullopt;
  }
  return std::string(value);
}

std::string decodeBase64(const std::string& input)
{
  static const std::array<int8_t, 256> decoding_table = []() {
    std::array<int8_t, 256> table{};
    table.fill(-1);
    const std::string alphabet =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (size_t i = 0; i < alphabet.size(); ++i) {
      table[static_cast<uint8_t>(alphabet[i])] = static_cast<int8_t>(i);
    }
    return table;
  }();

  std::string output;
  output.reserve(input.size() * 3 / 4);

  int value = 0;
  int bits_collected = -8;

  for (unsigned char ch : input) {
    if (std::isspace(ch)) {
      continue;
    }
    if (ch == '=') {
      break;
    }

    const int8_t decoded = decoding_table[static_cast<size_t>(ch)];
    if (decoded < 0) {
      throw std::runtime_error("Invalid base64 character encountered");
    }

    value = (value << 6) + decoded;
    bits_collected += 6;
    if (bits_collected >= 0) {
      output.push_back(static_cast<char>((value >> bits_collected) & 0xFF));
      bits_collected -= 8;
    }
  }

  return output;
}

std::filesystem::path defaultRosHome()
{
  if (auto ros_home = getEnvironmentVariable("ROS_HOME")) {
    return std::filesystem::path(*ros_home);
  }
  if (auto home = getEnvironmentVariable("HOME")) {
    return std::filesystem::path(*home) / ".ros";
  }
  return std::filesystem::temp_directory_path();
}

}  // namespace

using namespace std::chrono_literals;

class SafetySupervisor : public rclcpp_lifecycle::LifecycleNode
{
public:
  SafetySupervisor();

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
  // Callback functions
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void deadmanCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void faultEventCallback(const std_msgs::msg::String::SharedPtr msg);
  void watchdogTimerCallback();
  void diagnosticTimerCallback();
  
  // Safety validation functions
  bool validateCommand(const geometry_msgs::msg::Twist& cmd);
  geometry_msgs::msg::Twist saturateCommand(const geometry_msgs::msg::Twist& cmd);
  geometry_msgs::msg::Twist applyRateLimiting(const geometry_msgs::msg::Twist& cmd);
  bool checkPlausibility();
  void publishSafeCommand(const geometry_msgs::msg::Twist& cmd);
  void emergencyStop(const std::string& reason);
  void requestControlledShutdown(const std::string& reason);
  std::optional<std::filesystem::path> materializeSecretToRuntimeFile(
    const std::string& secret_contents, const std::string& file_hint);
  
  // Helper functions
  double applyRateLimit(double target, double current, double max_rate, double dt);
  
  // Configuration parameters
  double max_linear_velocity_;    // m/s
  double max_angular_velocity_;   // rad/s
  double max_linear_acceleration_;  // m/s² (rate limiting)
  double max_angular_acceleration_; // rad/s² (rate limiting)
  double plausibility_threshold_; // m/s difference threshold
  double watchdog_timeout_;       // seconds
  bool require_deadman_;
  bool enable_plausibility_check_;
  bool enable_rate_limiting_;     // Enable acceleration limiting

  // State variables
  std::mutex state_mutex_;
  bool deadman_active_;
  bool safety_stop_active_;
  std::string safety_stop_reason_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_publish_time_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  geometry_msgs::msg::Twist last_published_cmd_;
  nav_msgs::msg::Odometry last_odom_;
  bool odom_received_;
  
  // Statistics for diagnostics
  uint64_t cmd_received_count_;
  uint64_t cmd_rejected_count_;
  uint64_t cmd_saturated_count_;
  uint64_t plausibility_failures_;
  uint64_t watchdog_timeouts_;

  // Certification runtime validation control
  bool runtime_cert_validation_enabled_;
  bool allow_cert_bypass_;
  double cert_validation_grace_period_;
  int cert_validation_max_retries_;
  int consecutive_cert_validation_failures_;
  bool cert_failure_window_active_;
  rclcpp::Time cert_failure_window_start_;
  std::atomic<bool> controlled_shutdown_initiated_;
  std::optional<std::filesystem::path> ephemeral_secret_path_;

  // Publishers and subscribers
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fault_event_sub_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  rclcpp::TimerBase::SharedPtr cert_validation_timer_;  // Runtime integrity check
  
  // Certified parameter validation (ISO 13849-1 §5.2.2)
  std::unique_ptr<CertifiedParamsValidator> cert_validator_;
  
  // Maintenance mode management (ISO 3691-4 §5.2.6)
  std::unique_ptr<MaintenanceModeManager> maintenance_mgr_;

  // Callback for runtime certificate validation
  void certValidationTimerCallback();

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

SafetySupervisor::SafetySupervisor()
  : rclcpp_lifecycle::LifecycleNode("safety_supervisor"),
    max_linear_velocity_(1.0),
    max_angular_velocity_(1.0),
    max_linear_acceleration_(0.5),
    max_angular_acceleration_(1.0),
    plausibility_threshold_(0.2),
    watchdog_timeout_(0.5),
    require_deadman_(true),
    enable_plausibility_check_(true),
    enable_rate_limiting_(true),
    deadman_active_(false),
    safety_stop_active_(false),
    odom_received_(false),
    cmd_received_count_(0),
    cmd_rejected_count_(0),
    cmd_saturated_count_(0),
    plausibility_failures_(0),
    watchdog_timeouts_(0),
    runtime_cert_validation_enabled_(true),
    allow_cert_bypass_(false),
    cert_validation_grace_period_(5.0),
    cert_validation_max_retries_(3),
    consecutive_cert_validation_failures_(0),
    cert_failure_window_active_(false),
    controlled_shutdown_initiated_(false),
    ephemeral_secret_path_(std::nullopt)
{
  RCLCPP_DEBUG(this->get_logger(), "SafetySupervisor lifecycle node constructed");
}

CallbackReturn SafetySupervisor::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_DEBUG(this->get_logger(), "Loading certified safety parameters (ISO 13849-1 compliance)...");

  std::string package_share_dir;
  try {
    package_share_dir = ament_index_cpp::get_package_share_directory("somanet");
  } catch (const std::exception& e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to find package 'somanet': %s", e.what());
    return CallbackReturn::FAILURE;
  }

  const std::string cert_params_path = package_share_dir + "/config/certified_safety_params.yaml";
  std::string secret_path = package_share_dir + "/config/cert.key";
  RCLCPP_DEBUG(this->get_logger(), "Certified parameters path: %s", cert_params_path.c_str());
  RCLCPP_DEBUG(this->get_logger(), "HMAC secret key path: %s", secret_path.c_str());

  this->declare_parameter("allow_certified_param_bypass", allow_cert_bypass_);
  this->declare_parameter("cert_validation_grace_period", cert_validation_grace_period_);
  this->declare_parameter("cert_validation_max_retries", cert_validation_max_retries_);
  std::string cert_key_path_override;
  std::string cert_key_env_var = "SAFETY_SUPERVISOR_CERT_KEY_B64";
  std::string cert_key_path_env_var = "SAFETY_SUPERVISOR_CERT_KEY_PATH";
  this->declare_parameter("cert_key_path_override", cert_key_path_override);
  this->declare_parameter("cert_key_env_var", cert_key_env_var);
  this->declare_parameter("cert_key_path_env_var", cert_key_path_env_var);

  this->get_parameter("allow_certified_param_bypass", allow_cert_bypass_);
  this->get_parameter("cert_validation_grace_period", cert_validation_grace_period_);
  this->get_parameter("cert_validation_max_retries", cert_validation_max_retries_);
  this->get_parameter("cert_key_path_override", cert_key_path_override);
  this->get_parameter("cert_key_env_var", cert_key_env_var);
  this->get_parameter("cert_key_path_env_var", cert_key_path_env_var);

  ephemeral_secret_path_.reset();
  controlled_shutdown_initiated_ = false;

  if (!cert_key_path_override.empty()) {
    secret_path = cert_key_path_override;
    RCLCPP_INFO(this->get_logger(),
      "Using cert.key override from parameter: %s", secret_path.c_str());
  }

  if (auto env_override = getEnvironmentVariable(cert_key_path_env_var)) {
    secret_path = *env_override;
    RCLCPP_INFO(this->get_logger(),
      "Using cert.key path from environment variable %s", cert_key_path_env_var.c_str());
  }

  if (cert_validation_grace_period_ < 0.0) {
    RCLCPP_WARN(this->get_logger(),
      "cert_validation_grace_period < 0 specified. Clamping to 0 (no grace period)");
    cert_validation_grace_period_ = 0.0;
  }

  if (cert_validation_max_retries_ < 0) {
    RCLCPP_WARN(this->get_logger(),
      "cert_validation_max_retries < 0 specified. Clamping to 0 (no retries)");
    cert_validation_max_retries_ = 0;
  }

  // ========================================================================
  // DEPLOYMENT VERIFICATION: Check if certification files exist
  // ========================================================================
  // These files are REQUIRED for ISO 13849-1 compliance in production
  // For development/testing, see scripts/generate_certification_hash.py

  std::ifstream cert_file_check(cert_params_path);
  const bool has_cert_file = cert_file_check.good();
  if (!has_cert_file) {
    if (!allow_cert_bypass_) {
      RCLCPP_FATAL(this->get_logger(), "❌ DEPLOYMENT ERROR: certified_safety_params.yaml NOT FOUND!");
      RCLCPP_FATAL(this->get_logger(), "   → Expected path: %s", cert_params_path.c_str());
      RCLCPP_FATAL(this->get_logger(), "   → This file is REQUIRED for ISO 13849-1 compliance");
      RCLCPP_FATAL(this->get_logger(), "   → Generate using: python3 scripts/generate_certification_hash.py");
      RCLCPP_FATAL(this->get_logger(), "   → See BUILD_GUIDE.md section 'Safety Parameter Certification'");
      return CallbackReturn::FAILURE;
    }
    RCLCPP_WARN(this->get_logger(),
      "⚠️  certified_safety_params.yaml missing. Falling back to built-in safe defaults (bypass enabled)");
  }

  bool has_key_file = false;
  {
    std::ifstream key_file_check(secret_path);
    has_key_file = key_file_check.good();
  }

  if (!has_key_file) {
    if (auto encoded_secret = getEnvironmentVariable(cert_key_env_var)) {
      try {
        const auto decoded_secret = decodeBase64(*encoded_secret);
        if (!decoded_secret.empty()) {
          auto runtime_secret = materializeSecretToRuntimeFile(decoded_secret, secret_path);
          if (runtime_secret) {
            secret_path = runtime_secret->string();
            has_key_file = true;
            RCLCPP_INFO(this->get_logger(),
              "Loaded cert.key from environment variable %s", cert_key_env_var.c_str());
          } else {
            RCLCPP_ERROR(this->get_logger(),
              "Failed to persist cert.key material derived from %s", cert_key_env_var.c_str());
          }
        } else {
          RCLCPP_ERROR(this->get_logger(),
            "Decoded cert.key from %s is empty", cert_key_env_var.c_str());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(),
          "Unable to decode cert.key from environment variable %s: %s",
          cert_key_env_var.c_str(), e.what());
      }
    }
  }

  if (!has_key_file) {
    if (!allow_cert_bypass_) {
      RCLCPP_FATAL(this->get_logger(), "❌ DEPLOYMENT ERROR: cert.key NOT FOUND!");
      RCLCPP_FATAL(this->get_logger(), "   → Expected path: %s", secret_path.c_str());
      RCLCPP_FATAL(this->get_logger(), "   → This HMAC secret key is REQUIRED for parameter validation");
      RCLCPP_FATAL(this->get_logger(), "   → Generate using: python3 scripts/generate_certification_hash.py");
      RCLCPP_FATAL(this->get_logger(), "   → SECURITY WARNING: Never commit cert.key to version control!");
      RCLCPP_FATAL(this->get_logger(), "   → Deploy cert.key separately (encrypted config management recommended)");
      return CallbackReturn::FAILURE;
    }
    RCLCPP_WARN(this->get_logger(),
      "⚠️  cert.key missing. Runtime certification checks disabled (bypass enabled)");
  }

  cert_file_check.close();

  if (!allow_cert_bypass_) {
    cert_validator_ = std::make_unique<CertifiedParamsValidator>(
        cert_params_path, secret_path, this->get_logger());
  } else if (has_cert_file && has_key_file) {
    cert_validator_ = std::make_unique<CertifiedParamsValidator>(
        cert_params_path, secret_path, this->get_logger());
  }

  if (cert_validator_) {
    if (!cert_validator_->loadAndValidate()) {
      if (!allow_cert_bypass_) {
        RCLCPP_FATAL(this->get_logger(), "❌ CRITICAL: Certified parameters validation FAILED!");
        RCLCPP_FATAL(this->get_logger(), "   → Possible tampering detected or certificate expired");
        RCLCPP_FATAL(this->get_logger(), "   → System CANNOT start - Safety integrity compromised");
        RCLCPP_FATAL(this->get_logger(), "   → Regenerate certificate: python3 scripts/generate_certification_hash.py");
        RCLCPP_FATAL(this->get_logger(), "   → Verify cert.key matches certified_safety_params.yaml");
        return CallbackReturn::FAILURE;
      }

      RCLCPP_WARN(this->get_logger(),
        "⚠️  Certified parameter validation failed - continuing with bypass defaults");
      cert_validator_.reset();
    }
  }

  runtime_cert_validation_enabled_ = static_cast<bool>(cert_validator_);

  if (!cert_validator_) {
    RCLCPP_WARN(this->get_logger(),
      "Runtime certification enforcement disabled. Ensure this is only used for testing! (ISO 13849-1) ");
  }

  // ========================================================================
  // FIX PROBLEM 6: EXCEPTION HANDLING FOR getParameter()
  // ========================================================================
  // CertifiedParamsValidator::getParameter() throws std::runtime_error if parameter missing
  if (cert_validator_) {
    try {
      max_linear_velocity_ = cert_validator_->getParameter("max_linear_velocity");
      max_angular_velocity_ = cert_validator_->getParameter("max_angular_velocity");
      max_linear_acceleration_ = cert_validator_->getParameter("max_linear_acceleration");
      max_angular_acceleration_ = cert_validator_->getParameter("max_angular_acceleration");
      plausibility_threshold_ = cert_validator_->getParameter("plausibility_threshold");
      watchdog_timeout_ = cert_validator_->getParameter("watchdog_timeout");
    } catch (const std::runtime_error& e) {
      RCLCPP_FATAL(this->get_logger(),
        "❌ Failed to load required certified parameter: %s", e.what());
      RCLCPP_FATAL(this->get_logger(),
        "   Check that all required parameters are present in certified_safety_params.yaml");
      return CallbackReturn::FAILURE;
    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(),
        "❌ Exception loading certified parameters: %s", e.what());
      return CallbackReturn::FAILURE;
    }

    // Get certification info for logging
    auto cert_info = cert_validator_->getCertificationInfo();

    RCLCPP_INFO(this->get_logger(), "✅ Certified parameters loaded (Cert: %s, Valid: %s)",
      cert_info.certificate_id.c_str(), cert_info.valid_until.c_str());
    RCLCPP_INFO(this->get_logger(),
      "  Hash algorithm: %s", cert_info.hash_algorithm.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(),
      "Using built-in fallback limits: linear=%.2f angular=%.2f watchdog=%.2fs",
      max_linear_velocity_, max_angular_velocity_, watchdog_timeout_);
  }

  const std::string maintenance_audit_path = package_share_dir + "/config/maintenance_audit.yaml";
  const std::string maintenance_pins_path = package_share_dir + "/config/maintenance_pins.yaml";

  maintenance_mgr_ = std::make_unique<MaintenanceModeManager>(maintenance_audit_path, maintenance_pins_path);

  RCLCPP_DEBUG(this->get_logger(), "Maintenance mode manager initialized (Mode: OPERATIONAL)");

  this->declare_parameter("max_linear_velocity", max_linear_velocity_);
  this->declare_parameter("max_angular_velocity", max_angular_velocity_);
  this->declare_parameter("max_linear_acceleration", max_linear_acceleration_);
  this->declare_parameter("max_angular_acceleration", max_angular_acceleration_);
  this->declare_parameter("plausibility_threshold", plausibility_threshold_);
  this->declare_parameter("watchdog_timeout", watchdog_timeout_);

  this->declare_parameter("require_deadman", require_deadman_);
  this->declare_parameter("enable_plausibility_check", enable_plausibility_check_);
  this->declare_parameter("enable_rate_limiting", enable_rate_limiting_);

  this->get_parameter("require_deadman", require_deadman_);
  this->get_parameter("enable_plausibility_check", enable_plausibility_check_);
  this->get_parameter("enable_rate_limiting", enable_rate_limiting_);

  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      static const std::vector<std::string> safety_params = {
        "max_linear_velocity", "max_angular_velocity",
        "max_linear_acceleration", "max_angular_acceleration",
        "plausibility_threshold", "watchdog_timeout"
      };

      for (const auto& param : params) {
        if (std::find(safety_params.begin(), safety_params.end(), param.get_name()) != safety_params.end()) {
          if (maintenance_mgr_ && maintenance_mgr_->isInMaintenanceMode()) {
            RCLCPP_WARN(this->get_logger(),
              "⚠️  MAINTENANCE MODE: Parameter '%s' modified",
              param.get_name().c_str());
            RCLCPP_WARN(this->get_logger(),
              "   Old value: %s", this->get_parameter(param.get_name()).value_to_string().c_str());
            RCLCPP_WARN(this->get_logger(),
              "   New value: %s", param.value_to_string().c_str());

            if (maintenance_mgr_) {
              maintenance_mgr_->logParameterChange(
                param.get_name(),
                this->get_parameter(param.get_name()).value_to_string(),
                param.value_to_string());
            }

            return result;
          }

          RCLCPP_ERROR(this->get_logger(),
            "🔒 DENIED: Safety parameter '%s' is READ-ONLY (certified)",
            param.get_name().c_str());
          RCLCPP_ERROR(this->get_logger(), "   → System in OPERATIONAL mode (not maintenance)");
          RCLCPP_ERROR(this->get_logger(), "   → To modify:");
          RCLCPP_ERROR(this->get_logger(), "     1. Enter MAINTENANCE mode (requires PIN)");
          RCLCPP_ERROR(this->get_logger(), "     2. Modify parameters (robot will be STOPPED)");
          RCLCPP_ERROR(this->get_logger(), "     3. Exit maintenance mode");
          RCLCPP_ERROR(this->get_logger(), "     4. Regenerate certificate hash");
          RCLCPP_ERROR(this->get_logger(), "     5. Restart node");

          result.successful = false;
          result.reason = "Safety parameters are immutable in OPERATIONAL mode (ISO 13849-1 §5.2.2). "
                           "Enter MAINTENANCE mode to modify parameters.";
          return result;
        }
      }

      return result;
    });

  if (max_linear_velocity_ <= 0.0 || max_angular_velocity_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid velocity limits. Must be > 0");
    return CallbackReturn::FAILURE;
  }

  if (max_linear_acceleration_ <= 0.0 || max_angular_acceleration_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid acceleration limits. Must be > 0");
    return CallbackReturn::FAILURE;
  }

  if (plausibility_threshold_ < 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Plausibility threshold must be >= 0");
    return CallbackReturn::FAILURE;
  }

  safe_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", rclcpp::QoS(10));
  safety_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("safety_stop", rclcpp::QoS(10));
  diagnostic_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));

  last_cmd_time_ = this->now();
  last_publish_time_ = this->now();
  last_published_cmd_ = geometry_msgs::msg::Twist();

  RCLCPP_INFO(this->get_logger(),
    "Safety Supervisor configured | Limits: v_lin=%.2f m/s v_ang=%.2f rad/s | Watchdog=%.2fs",
    max_linear_velocity_, max_angular_velocity_, watchdog_timeout_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn SafetySupervisor::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_DEBUG(this->get_logger(), "Activating Safety Supervisor");

  if (safe_cmd_pub_) {
    safe_cmd_pub_->on_activate();
  }
  if (safety_stop_pub_) {
    safety_stop_pub_->on_activate();
  }
  if (diagnostic_pub_) {
    diagnostic_pub_->on_activate();
  }

  auto qos = rclcpp::QoS(10);
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos,
    std::bind(&SafetySupervisor::cmdVelCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos,
    std::bind(&SafetySupervisor::odomCallback, this, std::placeholders::_1));

  deadman_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "deadman_status", qos,
    std::bind(&SafetySupervisor::deadmanCallback, this, std::placeholders::_1));

  fault_event_sub_ = this->create_subscription<std_msgs::msg::String>(
    "safety/fault_events", qos,
    std::bind(&SafetySupervisor::faultEventCallback, this, std::placeholders::_1));

  watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(watchdog_timeout_ * 1000 / 2)),
    std::bind(&SafetySupervisor::watchdogTimerCallback, this));

  diagnostic_timer_ = this->create_wall_timer(
    1s,
    std::bind(&SafetySupervisor::diagnosticTimerCallback, this));

  if (runtime_cert_validation_enabled_) {
    cert_validation_timer_ = this->create_wall_timer(
      1s,
      std::bind(&SafetySupervisor::certValidationTimerCallback, this));
  }

  last_cmd_time_ = this->now();
  last_publish_time_ = this->now();
  deadman_active_ = false;
  safety_stop_active_ = false;

  RCLCPP_INFO(this->get_logger(), "✅ Safety Supervisor active");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SafetySupervisor::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_DEBUG(this->get_logger(), "Deactivating Safety Supervisor");

  if (watchdog_timer_) {
    watchdog_timer_->cancel();
    watchdog_timer_.reset();
  }
  if (diagnostic_timer_) {
    diagnostic_timer_->cancel();
    diagnostic_timer_.reset();
  }
  if (cert_validation_timer_) {
    cert_validation_timer_->cancel();
    cert_validation_timer_.reset();
  }

  cmd_vel_sub_.reset();
  odom_sub_.reset();
  deadman_sub_.reset();
  fault_event_sub_.reset();

  auto publish_zero = [this]() {
    geometry_msgs::msg::Twist stop_cmd;
    if (safe_cmd_pub_ && safe_cmd_pub_->is_activated()) {
      safe_cmd_pub_->publish(stop_cmd);
    }
    if (safety_stop_pub_ && safety_stop_pub_->is_activated()) {
      std_msgs::msg::Bool stop_msg;
      stop_msg.data = true;
      safety_stop_pub_->publish(stop_msg);
    }
  };
  publish_zero();

  if (safe_cmd_pub_ && safe_cmd_pub_->is_activated()) {
    safe_cmd_pub_->on_deactivate();
  }
  if (safety_stop_pub_ && safety_stop_pub_->is_activated()) {
    safety_stop_pub_->on_deactivate();
  }
  if (diagnostic_pub_ && diagnostic_pub_->is_activated()) {
    diagnostic_pub_->on_deactivate();
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn SafetySupervisor::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_DEBUG(this->get_logger(), "Cleaning up Safety Supervisor");

  cmd_vel_sub_.reset();
  odom_sub_.reset();
  deadman_sub_.reset();
  fault_event_sub_.reset();
  watchdog_timer_.reset();
  diagnostic_timer_.reset();
  cert_validation_timer_.reset();

  safe_cmd_pub_.reset();
  safety_stop_pub_.reset();
  diagnostic_pub_.reset();

  cert_validator_.reset();
  maintenance_mgr_.reset();

  param_callback_handle_.reset();

  if (ephemeral_secret_path_) {
    std::error_code ec;
    std::filesystem::remove(*ephemeral_secret_path_, ec);
    if (ec) {
      RCLCPP_WARN(this->get_logger(),
        "Failed to remove runtime cert.key %s: %s",
        ephemeral_secret_path_->string().c_str(), ec.message().c_str());
    }
    ephemeral_secret_path_.reset();
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn SafetySupervisor::on_shutdown(const rclcpp_lifecycle::State& state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "Shutting down Safety Supervisor");
  if (safe_cmd_pub_ && safe_cmd_pub_->is_activated()) {
    safe_cmd_pub_->on_deactivate();
  }
  if (safety_stop_pub_ && safety_stop_pub_->is_activated()) {
    safety_stop_pub_->on_deactivate();
  }
  if (diagnostic_pub_ && diagnostic_pub_->is_activated()) {
    diagnostic_pub_->on_deactivate();
  }

  if (ephemeral_secret_path_) {
    std::error_code ec;
    std::filesystem::remove(*ephemeral_secret_path_, ec);
    if (ec) {
      RCLCPP_WARN(this->get_logger(),
        "Failed to remove runtime cert.key %s during shutdown: %s",
        ephemeral_secret_path_->string().c_str(), ec.message().c_str());
    }
    ephemeral_secret_path_.reset();
  }

  return CallbackReturn::SUCCESS;
}

void SafetySupervisor::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  cmd_received_count_++;
  last_cmd_time_ = this->now();
  last_cmd_vel_ = *msg;
  
  // ========================================================================
  // MAINTENANCE MODE CHECK (ISO 3691-4 §5.2.6)
  // ========================================================================
  // If in maintenance mode, robot MUST be immobilized
  if (maintenance_mgr_ && maintenance_mgr_->isInMaintenanceMode()) {
    cmd_rejected_count_++;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "⚠️  Command rejected: System in MAINTENANCE mode (robot immobilized)");
    emergencyStop("Maintenance mode active - robot cannot move");
    return;
  } else if (!maintenance_mgr_) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Maintenance manager unavailable - rejecting command for safety");
    cmd_rejected_count_++;
    emergencyStop("Maintenance manager unavailable");
    return;
  }
  
  // Check all safety conditions
  if (safety_stop_active_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Safety stop active: %s", safety_stop_reason_.c_str());
    emergencyStop(safety_stop_reason_);
    return;
  }
  
  if (require_deadman_ && !deadman_active_) {
    cmd_rejected_count_++;
    RCLCPP_DEBUG(this->get_logger(), "Command rejected: deadman not active");
    emergencyStop("Deadman button not pressed");
    return;
  }
  
  // Validate command (check for non-planar motion)
  if (!validateCommand(*msg)) {
    cmd_rejected_count_++;
    RCLCPP_WARN(this->get_logger(), 
      "Command rejected: non-planar motion (only 2D supported)");
    emergencyStop("Invalid command format");
    return;
  }
  
  // Check plausibility (commanded vs actual velocity)
  if (enable_plausibility_check_ && odom_received_ && !checkPlausibility()) {
    cmd_rejected_count_++;
    plausibility_failures_++;
    RCLCPP_ERROR(this->get_logger(), 
      "Plausibility check failed: commanded vs actual velocity mismatch");
    emergencyStop("Plausibility check failed");
    return;
  }
  
  // Saturate command to safety limits (instead of rejecting)
  geometry_msgs::msg::Twist saturated_cmd = saturateCommand(*msg);
  
  // Apply rate limiting if enabled (ISO 3691-4 Section 5.2.5)
  geometry_msgs::msg::Twist final_cmd = enable_rate_limiting_ ? 
    applyRateLimiting(saturated_cmd) : saturated_cmd;
  
  // All checks passed - publish safe command
  publishSafeCommand(final_cmd);
}

void SafetySupervisor::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_odom_ = *msg;
  odom_received_ = true;
}

void SafetySupervisor::deadmanCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  bool previous_state = deadman_active_;
  deadman_active_ = msg->data;
  
  if (previous_state != deadman_active_) {
    RCLCPP_INFO(this->get_logger(), "Deadman status changed: %s",
      deadman_active_ ? "ACTIVE" : "INACTIVE");
    
    if (!deadman_active_) {
      emergencyStop("Deadman button released");
    }
  }
}

void SafetySupervisor::faultEventCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null fault event message");
    return;
  }

  std::string payload = msg->data;
  if (payload.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty fault event message");
    return;
  }

  auto trim = [](std::string& text) {
    const auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
    text.erase(text.begin(), std::find_if(text.begin(), text.end(), [&](unsigned char c) { return !is_space(c); }));
    text.erase(std::find_if(text.rbegin(), text.rend(), [&](unsigned char c) { return !is_space(c); }).base(), text.end());
  };

  trim(payload);
  if (payload.empty()) {
    RCLCPP_WARN(this->get_logger(), "Fault event message contained only whitespace");
    return;
  }

  std::string normalized = payload;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

  auto matchesToken = [&](const std::string& token, std::string& detail_out) {
    const std::string token_with_colon = token + ":";
    if (normalized == token) {
      detail_out.clear();
      return true;
    }
    if (normalized.rfind(token_with_colon, 0) == 0 && payload.size() >= token_with_colon.size()) {
      detail_out = payload.substr(token_with_colon.size());
      trim(detail_out);
      return true;
    }
    return false;
  };

  std::string detail;
  if (matchesToken("CLEAR", detail) || matchesToken("RESET", detail) || matchesToken("RECOVERED", detail)) {
    if (safety_stop_active_) {
      safety_stop_active_ = false;
      safety_stop_reason_.clear();

      std_msgs::msg::Bool stop_msg;
      stop_msg.data = false;
      if (safety_stop_pub_ && safety_stop_pub_->is_activated()) {
        safety_stop_pub_->publish(stop_msg);
      }

      RCLCPP_INFO(this->get_logger(), "✅ Safety stop cleared after driver recovery%s%s", 
        detail.empty() ? "" : ": ", detail.empty() ? "" : detail.c_str());
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Driver recovery event received (no active safety stop)%s%s",
        detail.empty() ? "" : ": ", detail.empty() ? "" : detail.c_str());
    }
    return;
  }

  RCLCPP_ERROR(this->get_logger(), "Driver fault event: %s", payload.c_str());
  emergencyStop("Driver fault: " + payload);
}

/**
 * @brief Independent watchdog timer callback (ISO 13849-1 §7.2)
 * 
 * CRITICAL SAFETY FUNCTION:
 * Runs in SEPARATE thread from main control loop. If main.cpp EtherCAT loop
 * freezes/crashes, this watchdog STILL executes and stops the robot.
 * 
 * This is INDEPENDENT MONITORING required by ISO 13849-1 §7.2:
 * "Monitoring function shall be independent of the control function"
 * 
 * Execution: ROS2 timer (separate thread)
 * Frequency: Check every watchdog_timeout_/2 (e.g., every 250ms if timeout=500ms)
 * Action on timeout:
 *   1. Publish cmd_vel=0 to /cmd_vel_safe
 *   2. Trigger emergency stop
 *   3. Log event for audit trail
 *   4. If repeated (>3x) → Shutdown system
 */
void SafetySupervisor::watchdogTimerCallback()
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  auto now = this->now();
  auto elapsed = (now - last_cmd_time_).seconds();
  
  if (elapsed > watchdog_timeout_) {
    // ========================================================================
    // WATCHDOG EXPIRED - INDEPENDENT SAFETY ACTION
    // ========================================================================
    
    if (!safety_stop_active_) {
      watchdog_timeouts_++;
      
      RCLCPP_ERROR(this->get_logger(), 
        "🚨 WATCHDOG TIMEOUT (%.3fs) - INDEPENDENT SAFETY STOP | No cmd_vel for %.3fs", 
        watchdog_timeout_, elapsed);
      RCLCPP_ERROR(this->get_logger(), 
        "   Possible causes: Operator stopped | Node crashed | Network lost | CPU starvation");
      
      // 1. Publish zero velocity command (fail-safe)
      geometry_msgs::msg::Twist zero_cmd;
      zero_cmd.linear.x = 0.0;
      zero_cmd.linear.y = 0.0;
      zero_cmd.linear.z = 0.0;
      zero_cmd.angular.x = 0.0;
      zero_cmd.angular.y = 0.0;
      zero_cmd.angular.z = 0.0;
      
      safe_cmd_pub_->publish(zero_cmd);
      last_published_cmd_ = zero_cmd;
      
      RCLCPP_DEBUG(this->get_logger(), "Published zero velocity to /cmd_vel_safe");
      
      // 2. Trigger emergency stop flag
      emergencyStop("WATCHDOG TIMEOUT - Independent monitoring");
      
      // 3. Check for repeated timeouts (system malfunction)
      if (watchdog_timeouts_ >= 3) {
        RCLCPP_FATAL(this->get_logger(),
          "💥 CRITICAL: %lu watchdog timeouts - SYSTEM UNSTABLE - INITIATING SHUTDOWN",
          watchdog_timeouts_);

        // Additional fatal log for external monitoring systems
        RCLCPP_FATAL(this->get_logger(),
          "ULTRABOT_SAFETY_EVENT: Multiple watchdog timeouts detected - system unstable");

        requestControlledShutdown("Multiple watchdog timeouts");
      }
    }
  } else {
    // Reset timeout counter when commands are being received
    if (watchdog_timeouts_ > 0 && elapsed < (watchdog_timeout_ / 2.0)) {
      RCLCPP_INFO(this->get_logger(), 
        "✅ Command stream restored - Watchdog timeout counter reset");
      watchdog_timeouts_ = 0;
    }
  }
}

/**
 * @brief Runtime certificate validation timer callback
 * 
 * CRITICAL SAFETY FUNCTION (ISO 13849-1 §5.2.2):
 * Continuously validates that safety parameters in memory match the certified hash.
 * 
 * This prevents:
 * - Memory corruption (bit flips, cosmic rays)
 * - Malicious runtime modification (debugger, memory injection)
 * - Accidental parameter changes (software bugs)
 * 
 * If tampering is detected, system immediately enters EMERGENCY STOP and shuts down.
 * 
 * Execution frequency: 1 Hz (every 1 second)
 * Validation method: SHA-256 hash comparison
 * Action on failure: FATAL error + emergency stop + shutdown
 * 
 * Compliance:
 * - ISO 13849-1 §5.2.2: Parameters shall be protected during entire operation
 * - IEC 62304 §5.1.1: Configuration management and integrity
 * - IEC 61508 §7.4.2.5: Protection against common cause failures
 */
void SafetySupervisor::certValidationTimerCallback()
{
  if (!runtime_cert_validation_enabled_) {
    return;
  }

  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  if (!cert_validator_) {
    RCLCPP_ERROR(this->get_logger(), "Certificate validator unavailable during runtime validation");
    return;
  }

  std::string failure_reason;
  bool reloaded = false;
  if (!cert_validator_->validateRuntime(&failure_reason, &reloaded)) {
    if (!cert_failure_window_active_) {
      cert_failure_window_active_ = true;
      cert_failure_window_start_ = this->now();
    }

    consecutive_cert_validation_failures_++;

    const double failure_duration =
      (this->now() - cert_failure_window_start_).seconds();

    const bool time_limit_configured = cert_validation_grace_period_ > 0.0;
    const bool retry_limit_configured = cert_validation_max_retries_ > 0;

    const bool exceeded_time =
      time_limit_configured && failure_duration >= cert_validation_grace_period_;

    const bool exceeded_retries =
      retry_limit_configured &&
      consecutive_cert_validation_failures_ >= cert_validation_max_retries_;

    const bool immediate_abort = !time_limit_configured && !retry_limit_configured;

    RCLCPP_WARN(this->get_logger(),
      "Runtime certification validation failed (%s). Attempt %d within %.2fs",
      failure_reason.c_str(), consecutive_cert_validation_failures_, failure_duration);

    if (immediate_abort || exceeded_time || exceeded_retries) {
      RCLCPP_FATAL(this->get_logger(),
        "🚨 CERTIFICATION VALIDATION FAILED PERSISTENTLY - EMERGENCY SHUTDOWN!");
      RCLCPP_FATAL(this->get_logger(),
        "   Failure reason: %s", failure_reason.c_str());
      emergencyStop("RUNTIME CERTIFICATE VALIDATION FAILURE");
      requestControlledShutdown("Persistent certification validation failure");
    }

    return;
  }

  consecutive_cert_validation_failures_ = 0;
  cert_failure_window_active_ = false;

  // Verify that in-memory parameters match certified values
  // This catches modifications via direct memory access (not via ROS2 params)
  double cert_max_linear_vel, cert_max_angular_vel, cert_max_linear_acc;
  double cert_max_angular_acc, cert_plausibility, cert_watchdog;

  try {
    if (reloaded) {
      max_linear_velocity_ = cert_validator_->getParameter("max_linear_velocity");
      max_angular_velocity_ = cert_validator_->getParameter("max_angular_velocity");
      max_linear_acceleration_ = cert_validator_->getParameter("max_linear_acceleration");
      max_angular_acceleration_ = cert_validator_->getParameter("max_angular_acceleration");
      plausibility_threshold_ = cert_validator_->getParameter("plausibility_threshold");
      watchdog_timeout_ = cert_validator_->getParameter("watchdog_timeout");
      RCLCPP_INFO(this->get_logger(),
        "Certified parameters reloaded at runtime after file change");
    }

    cert_max_linear_vel = cert_validator_->getParameter("max_linear_velocity");
    cert_max_angular_vel = cert_validator_->getParameter("max_angular_velocity");
    cert_max_linear_acc = cert_validator_->getParameter("max_linear_acceleration");
    cert_max_angular_acc = cert_validator_->getParameter("max_angular_acceleration");
    cert_plausibility = cert_validator_->getParameter("plausibility_threshold");
    cert_watchdog = cert_validator_->getParameter("watchdog_timeout");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), 
      "Failed to read certified parameter during verification: %s", e.what());
    // If we can't read certified params, assume tampering
    emergencyStop("Certificate verification failed - parameter read error");
    return;
  }
  
  const double epsilon = 1e-6;  // Floating point comparison tolerance
  
  bool mismatch_detected = false;
  std::string mismatch_param;
  
  if (std::abs(max_linear_velocity_ - cert_max_linear_vel) > epsilon) {
    mismatch_detected = true;
    mismatch_param = "max_linear_velocity";
  } else if (std::abs(max_angular_velocity_ - cert_max_angular_vel) > epsilon) {
    mismatch_detected = true;
    mismatch_param = "max_angular_velocity";
  } else if (std::abs(max_linear_acceleration_ - cert_max_linear_acc) > epsilon) {
    mismatch_detected = true;
    mismatch_param = "max_linear_acceleration";
  } else if (std::abs(max_angular_acceleration_ - cert_max_angular_acc) > epsilon) {
    mismatch_detected = true;
    mismatch_param = "max_angular_acceleration";
  } else if (std::abs(plausibility_threshold_ - cert_plausibility) > epsilon) {
    mismatch_detected = true;
    mismatch_param = "plausibility_threshold";
  } else if (std::abs(watchdog_timeout_ - cert_watchdog) > epsilon) {
    mismatch_detected = true;
    mismatch_param = "watchdog_timeout";
  }
  
  if (mismatch_detected) {
    // CRITICAL: In-memory parameter differs from certified value!
    RCLCPP_FATAL(this->get_logger(), 
      "🚨 MEMORY CORRUPTION: Parameter '%s' modified (impossible with param locking)", 
      mismatch_param.c_str());
    RCLCPP_FATAL(this->get_logger(), 
      "   Hardware fault or malicious attack - Emergency shutdown");
    
    emergencyStop("PARAMETER MEMORY CORRUPTION");
    requestControlledShutdown("Certified parameter memory mismatch");
  }
  
  // Validation passed - log periodically to avoid spam
  static int validation_count = 0;
  validation_count++;
  if (validation_count % 300 == 1) {  // Every 5 minutes (first time + every 300s)
    RCLCPP_INFO(this->get_logger(), 
      "✅ Runtime certificate validation OK (checks: %d)", validation_count);
  }
}

void SafetySupervisor::diagnosticTimerCallback()
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  if (!cert_validator_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Diagnostic timer running without certificate validator");
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();
  
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "Safety Supervisor";
  status.hardware_id = "ultrabot_agv";
  
  // Determine overall status
  if (safety_stop_active_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "SAFETY STOP: " + safety_stop_reason_;
  } else if (!deadman_active_ && require_deadman_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Deadman inactive - robot stopped";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "All safety checks passed";
  }
  
  // Add diagnostic values
  diagnostic_msgs::msg::KeyValue kv;
  
  kv.key = "deadman_active";
  kv.value = deadman_active_ ? "true" : "false";
  status.values.push_back(kv);
  
  kv.key = "safety_stop_active";
  kv.value = safety_stop_active_ ? "true" : "false";
  status.values.push_back(kv);
  
  kv.key = "commands_received";
  kv.value = std::to_string(cmd_received_count_);
  status.values.push_back(kv);
  
  kv.key = "commands_rejected";
  kv.value = std::to_string(cmd_rejected_count_);
  status.values.push_back(kv);
  
  kv.key = "commands_saturated";
  kv.value = std::to_string(cmd_saturated_count_);
  status.values.push_back(kv);
  
  kv.key = "plausibility_failures";
  kv.value = std::to_string(plausibility_failures_);
  status.values.push_back(kv);
  
  kv.key = "watchdog_timeouts";
  kv.value = std::to_string(watchdog_timeouts_);
  status.values.push_back(kv);
  
  double acceptance_rate = cmd_received_count_ > 0 ? 
    100.0 * (cmd_received_count_ - cmd_rejected_count_) / cmd_received_count_ : 100.0;
  kv.key = "command_acceptance_rate";
  kv.value = std::to_string(acceptance_rate) + "%";
  status.values.push_back(kv);
  
  auto elapsed_since_cmd = (this->now() - last_cmd_time_).seconds();
  kv.key = "time_since_last_cmd";
  kv.value = std::to_string(elapsed_since_cmd) + "s";
  status.values.push_back(kv);
  
  // ========================================================================
  // CERTIFICATE DIAGNOSTICS (ISO 13849-1 §5.2.2)
  // ========================================================================
  // Add certification information for audit trail and monitoring
  auto cert_info = cert_validator_->getCertificationInfo();
  
  kv.key = "cert_certificate_id";
  kv.value = cert_info.certificate_id;
  status.values.push_back(kv);
  
  kv.key = "cert_hash_algorithm";
  kv.value = cert_info.hash_algorithm;
  status.values.push_back(kv);
  
  kv.key = "cert_hash";
  kv.value = cert_info.hash.substr(0, 16) + "...";  // First 16 chars only
  status.values.push_back(kv);
  
  kv.key = "cert_valid_until";
  kv.value = cert_info.valid_until;
  status.values.push_back(kv);
  
  kv.key = "cert_certified_by";
  kv.value = cert_info.certified_by;
  status.values.push_back(kv);
  
  // Check if certificate is still valid
  bool cert_valid = cert_validator_->isCertificationValid();
  kv.key = "cert_is_valid";
  kv.value = cert_valid ? "true" : "EXPIRED";
  status.values.push_back(kv);
  
  // Upgrade status to WARN if certificate expired
  if (!cert_valid && status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Certificate EXPIRED - recertification required";
  }
  
  // ========================================================================
  // MAINTENANCE MODE DIAGNOSTICS (ISO 3691-4 §5.2.6)
  // ========================================================================
  kv.key = "maintenance_mode";
  bool maintenance_active = maintenance_mgr_ && maintenance_mgr_->isInMaintenanceMode();
  kv.value = maintenance_active ? "ACTIVE" : "operational";
  status.values.push_back(kv);
  
  if (maintenance_active) {
    const auto* session = maintenance_mgr_ ? maintenance_mgr_->getCurrentSession() : nullptr;
    if (session) {
      kv.key = "maintenance_user";
      kv.value = session->user;
      status.values.push_back(kv);
      
      kv.key = "maintenance_duration_s";
      double duration = maintenance_mgr_ ? maintenance_mgr_->getMaintenanceSessionDuration() : 0.0;
      kv.value = std::to_string(static_cast<int>(duration));
      status.values.push_back(kv);
      
      kv.key = "maintenance_changes";
      kv.value = std::to_string(session->changes_made.size());
      status.values.push_back(kv);
    }
    
    // Upgrade status to WARN if in maintenance mode
    if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "MAINTENANCE MODE - Robot immobilized";
    }
  }
  
  diag_array.status.push_back(status);
  diagnostic_pub_->publish(diag_array);
}

bool SafetySupervisor::validateCommand(const geometry_msgs::msg::Twist& cmd)
{
  // Only validate command format (not velocity limits - saturation handles that)
  // All other Twist components should be zero (we only support 2D motion)
  if (cmd.linear.y != 0.0 || cmd.linear.z != 0.0 ||
      cmd.angular.x != 0.0 || cmd.angular.y != 0.0) {
    RCLCPP_WARN(this->get_logger(), "Non-planar motion commanded - rejecting");
    return false;
  }
  
  return true;
}

geometry_msgs::msg::Twist SafetySupervisor::saturateCommand(const geometry_msgs::msg::Twist& cmd)
{
  geometry_msgs::msg::Twist saturated_cmd = cmd;
  bool was_saturated = false;
  
  // Saturate linear velocity
  if (std::abs(cmd.linear.x) > max_linear_velocity_) {
    saturated_cmd.linear.x = std::copysign(max_linear_velocity_, cmd.linear.x);
    was_saturated = true;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Linear velocity saturated: %.3f → %.3f m/s",
      cmd.linear.x, saturated_cmd.linear.x);
  }
  
  // Saturate angular velocity
  if (std::abs(cmd.angular.z) > max_angular_velocity_) {
    saturated_cmd.angular.z = std::copysign(max_angular_velocity_, cmd.angular.z);
    was_saturated = true;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Angular velocity saturated: %.3f → %.3f rad/s",
      cmd.angular.z, saturated_cmd.angular.z);
  }
  
  if (was_saturated) {
    cmd_saturated_count_++;
  }
  
  return saturated_cmd;
}

double SafetySupervisor::applyRateLimit(double target, double current, double max_rate, double dt)
{
  if (dt <= 0.0) {
    return current;  // Avoid division by zero
  }
  
  double max_change = max_rate * dt;
  double diff = target - current;
  
  if (std::abs(diff) > max_change) {
    return current + std::copysign(max_change, diff);
  }
  
  return target;
}

geometry_msgs::msg::Twist SafetySupervisor::applyRateLimiting(const geometry_msgs::msg::Twist& cmd)
{
  auto now = this->now();
  double dt = (now - last_publish_time_).seconds();
  
  // First command or time went backwards - don't apply rate limiting
  if (dt <= 0.0 || dt > 1.0) {
    last_publish_time_ = now;
    last_published_cmd_ = cmd;
    return cmd;
  }
  
  geometry_msgs::msg::Twist limited_cmd;
  
  // Apply rate limiting to linear velocity
  limited_cmd.linear.x = applyRateLimit(
    cmd.linear.x, 
    last_published_cmd_.linear.x, 
    max_linear_acceleration_, 
    dt);
  
  // Apply rate limiting to angular velocity
  limited_cmd.angular.z = applyRateLimit(
    cmd.angular.z, 
    last_published_cmd_.angular.z, 
    max_angular_acceleration_, 
    dt);
  
  // Log if rate limiting occurred
  if (std::abs(limited_cmd.linear.x - cmd.linear.x) > 0.001 ||
      std::abs(limited_cmd.angular.z - cmd.angular.z) > 0.001) {
    RCLCPP_DEBUG(this->get_logger(),
      "Rate limiting: lin(%.3f→%.3f) ang(%.3f→%.3f) dt=%.3fs",
      cmd.linear.x, limited_cmd.linear.x,
      cmd.angular.z, limited_cmd.angular.z, dt);
  }
  
  last_publish_time_ = now;
  last_published_cmd_ = limited_cmd;
  
  return limited_cmd;
}


bool SafetySupervisor::checkPlausibility()
{
  if (!odom_received_) {
    return true; // Cannot check yet, allow command
  }
  
  // Get actual velocity from odometry
  double actual_linear = last_odom_.twist.twist.linear.x;
  double actual_angular = last_odom_.twist.twist.angular.z;
  
  // Get commanded velocity
  double cmd_linear = last_cmd_vel_.linear.x;
  double cmd_angular = last_cmd_vel_.angular.z;
  
  // Check linear velocity plausibility
  double linear_error = std::abs(actual_linear - cmd_linear);
  if (linear_error > plausibility_threshold_) {
    RCLCPP_ERROR(this->get_logger(),
      "Linear velocity mismatch: cmd=%.3f m/s, actual=%.3f m/s, error=%.3f m/s",
      cmd_linear, actual_linear, linear_error);
    return false;
  }
  
  // Check angular velocity plausibility (using same threshold converted to rad/s)
  double angular_threshold = plausibility_threshold_ * 2.0; // More tolerance for rotation
  double angular_error = std::abs(actual_angular - cmd_angular);
  if (angular_error > angular_threshold) {
    RCLCPP_ERROR(this->get_logger(),
      "Angular velocity mismatch: cmd=%.3f rad/s, actual=%.3f rad/s, error=%.3f rad/s",
      cmd_angular, actual_angular, angular_error);
    return false;
  }
  
  return true;
}

void SafetySupervisor::publishSafeCommand(const geometry_msgs::msg::Twist& cmd)
{
  if (safe_cmd_pub_ && safe_cmd_pub_->is_activated()) {
    safe_cmd_pub_->publish(cmd);
  }
  
  // Ensure safety stop is cleared if we're publishing valid commands
  if (safety_stop_active_) {
    safety_stop_active_ = false;
    safety_stop_reason_.clear();
    
    std_msgs::msg::Bool stop_msg;
    stop_msg.data = false;
    if (safety_stop_pub_ && safety_stop_pub_->is_activated()) {
      safety_stop_pub_->publish(stop_msg);
    }
  }
}

void SafetySupervisor::emergencyStop(const std::string& reason)
{
  // Note: state_mutex_ is already locked by calling functions
  safety_stop_active_ = true;
  safety_stop_reason_ = reason;
  
  // Publish zero velocity command
  geometry_msgs::msg::Twist stop_cmd;
  stop_cmd.linear.x = 0.0;
  stop_cmd.linear.y = 0.0;
  stop_cmd.linear.z = 0.0;
  stop_cmd.angular.x = 0.0;
  stop_cmd.angular.y = 0.0;
  stop_cmd.angular.z = 0.0;
  if (safe_cmd_pub_ && safe_cmd_pub_->is_activated()) {
    safe_cmd_pub_->publish(stop_cmd);
  }
  
  // Publish safety stop flag
  std_msgs::msg::Bool stop_msg;
  stop_msg.data = true;
  if (safety_stop_pub_ && safety_stop_pub_->is_activated()) {
    safety_stop_pub_->publish(stop_msg);
  }
  
  RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "EMERGENCY STOP: %s", reason.c_str());
}

void SafetySupervisor::requestControlledShutdown(const std::string& reason)
{
  const bool already_requested = controlled_shutdown_initiated_.exchange(true);
  if (already_requested) {
    RCLCPP_DEBUG(this->get_logger(),
      "Controlled shutdown already in progress (reason: %s)", reason.c_str());
    return;
  }

  RCLCPP_FATAL(this->get_logger(),
    "Initiating controlled lifecycle shutdown due to: %s", reason.c_str());
  auto current_state = this->get_current_state().id();

  if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    if (!this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to deactivate during controlled shutdown request");
    }
    current_state = this->get_current_state().id();
  }

  if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    if (!this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to cleanup during controlled shutdown request");
    }
    current_state = this->get_current_state().id();
  }

  if (current_state != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
    if (!this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_SHUTDOWN)) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to finalize SafetySupervisor during controlled shutdown request");
    }
  }
}

std::optional<std::filesystem::path> SafetySupervisor::materializeSecretToRuntimeFile(
  const std::string& secret_contents, const std::string& file_hint)
{
  try {
    const std::filesystem::path hint_path(file_hint);
    const std::filesystem::path runtime_dir = defaultRosHome() / "somanet";
    std::error_code ec;
    std::filesystem::create_directories(runtime_dir, ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to create runtime secret directory %s: %s",
        runtime_dir.string().c_str(), ec.message().c_str());
      return std::nullopt;
    }

    std::filesystem::path target = runtime_dir / hint_path.filename();
    {
      std::ofstream secret_file(target, std::ios::binary | std::ios::trunc);
      if (!secret_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(),
          "Unable to open runtime secret file for writing: %s", target.string().c_str());
        return std::nullopt;
      }
      secret_file.write(secret_contents.data(), static_cast<std::streamsize>(secret_contents.size()));
      secret_file.flush();
      if (!secret_file.good()) {
        RCLCPP_ERROR(this->get_logger(),
          "Failed while writing cert.key material to %s", target.string().c_str());
        return std::nullopt;
      }
    }

    std::error_code perm_ec;
    std::filesystem::permissions(target,
      std::filesystem::perms::owner_read | std::filesystem::perms::owner_write,
      std::filesystem::perm_options::replace, perm_ec);
    if (perm_ec) {
      RCLCPP_WARN(this->get_logger(),
        "Unable to set strict permissions on %s: %s",
        target.string().c_str(), perm_ec.message().c_str());
    }

    ephemeral_secret_path_ = target;
    return target;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
      "Failed to prepare runtime cert.key material: %s", e.what());
    return std::nullopt;
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
    auto safety_supervisor = std::make_shared<SafetySupervisor>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(safety_supervisor->get_node_base_interface());

    if (autostart) {
      RCLCPP_WARN(rclcpp::get_logger("safety_supervisor"),
        "Autostart enabled – configuring and activating Safety Supervisor immediately");

      if (safety_supervisor->configure() != CallbackReturn::SUCCESS) {
        RCLCPP_FATAL(rclcpp::get_logger("safety_supervisor"),
          "Failed to configure SafetySupervisor lifecycle node");
        rclcpp::shutdown();
        return 1;
      }

      if (safety_supervisor->activate() != CallbackReturn::SUCCESS) {
        RCLCPP_FATAL(rclcpp::get_logger("safety_supervisor"),
          "Failed to activate SafetySupervisor lifecycle node");
        rclcpp::shutdown();
        return 1;
      }
    } else {
      RCLCPP_INFO(rclcpp::get_logger("safety_supervisor"),
        "Autostart disabled – waiting for external lifecycle transitions");
    }

    executor.spin();

    if (safety_supervisor->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      if (safety_supervisor->deactivate() != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("safety_supervisor"),
          "Error while deactivating SafetySupervisor during shutdown");
      }
    }
    if (safety_supervisor->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      if (safety_supervisor->cleanup() != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("safety_supervisor"),
          "Error while cleaning up SafetySupervisor during shutdown");
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("safety_supervisor"), 
      "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
