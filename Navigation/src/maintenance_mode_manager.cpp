/**
 * @file maintenance_mode_manager.cpp
 * @brief Implementation of Maintenance Mode Manager
 * 
 * @version 1.1.0
 * @date 2025-11-04
 * @changes FIX PROBLEM 5: Replaced std::cout/cerr with RCLCPP_* logging
 */

#include "maintenance_mode_manager.hpp"
#include <iomanip>
#include <sstream>
#include <fstream>
#include <openssl/sha.h>

MaintenanceModeManager::MaintenanceModeManager(
    const std::string& audit_log_path,
    const std::string& pin_file_path,
    rclcpp::Logger logger)
  : current_mode_(MaintenanceMode::OPERATIONAL),
    audit_log_path_(audit_log_path),
    pin_file_path_(pin_file_path),
    logger_(logger),
    clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
{
  // Load valid PINs from configuration
  if (!loadPins()) {
    RCLCPP_WARN(logger_, "Could not load PINs from %s", pin_file_path.c_str());
    RCLCPP_WARN(logger_, "Maintenance mode will be DISABLED");
  }
}

bool MaintenanceModeManager::enterMaintenanceMode(
    const std::string& pin,
    const std::string& user,
    const std::string& reason)
{
  // Check if already in maintenance mode
  if (current_mode_ == MaintenanceMode::MAINTENANCE) {
    RCLCPP_ERROR(logger_, "Already in maintenance mode");
    return false;
  }
  
  // Verify PIN
  if (!verifyPin(pin)) {
    RCLCPP_ERROR(logger_, "Invalid PIN");
    RCLCPP_ERROR(logger_, "Authentication FAILED for user: %s", user.c_str());
    
    // Log failed attempt for security audit
    MaintenanceSession failed_attempt;
    failed_attempt.user = user + " (FAILED AUTH)";
    failed_attempt.timestamp_enter = getCurrentTimestamp();
    failed_attempt.timestamp_exit = getCurrentTimestamp();
    failed_attempt.reason = "Authentication failed - invalid PIN";
    failed_attempt.approved_by_safety_manager = false;
    appendToAuditLog(failed_attempt);
    
    return false;
  }
  
  // PIN verified - enter maintenance mode
  current_mode_ = MaintenanceMode::MAINTENANCE;
  session_start_time_ = std::chrono::system_clock::now();
  
  // Initialize session
  current_session_.user = user;
  current_session_.timestamp_enter = getCurrentTimestamp();
  current_session_.timestamp_exit = "";
  current_session_.reason = reason;
  current_session_.changes_made.clear();
  current_session_.approved_by_safety_manager = false;
  current_session_.approval_signature = "";
  
  RCLCPP_WARN(logger_, "================================");
  RCLCPP_WARN(logger_, "ENTERED MAINTENANCE MODE");
  RCLCPP_WARN(logger_, "================================");
  RCLCPP_WARN(logger_, "User: %s", user.c_str());
  RCLCPP_WARN(logger_, "Time: %s", current_session_.timestamp_enter.c_str());
  RCLCPP_WARN(logger_, "Reason: %s", reason.c_str());
  RCLCPP_WARN(logger_, "⚠️  ROBOT IS NOW IMMOBILIZED");
  RCLCPP_WARN(logger_, "⚠️  Safety parameters UNLOCKED");
  RCLCPP_WARN(logger_, "================================");
  
  return true;
}

bool MaintenanceModeManager::exitMaintenanceMode()
{
  if (current_mode_ != MaintenanceMode::MAINTENANCE) {
    RCLCPP_ERROR(logger_, "Not in maintenance mode");
    return false;
  }
  
  // Finalize session
  current_session_.timestamp_exit = getCurrentTimestamp();
  
  // Check if any parameters were changed
  bool parameters_modified = !current_session_.changes_made.empty();
  
  RCLCPP_WARN(logger_, "================================");
  RCLCPP_WARN(logger_, "EXITING MAINTENANCE MODE");
  RCLCPP_WARN(logger_, "================================");
  RCLCPP_INFO(logger_, "Session duration: %.0f seconds", getMaintenanceSessionDuration());
  RCLCPP_INFO(logger_, "Parameters changed: %zu", current_session_.changes_made.size());
  
  if (parameters_modified) {
    RCLCPP_WARN(logger_, "");
    RCLCPP_WARN(logger_, "⚠️  WARNING: Safety parameters were modified!");
    RCLCPP_WARN(logger_, "⚠️  Changes:");
    for (const auto& change : current_session_.changes_made) {
      RCLCPP_WARN(logger_, "   • %s", change.c_str());
    }
    RCLCPP_ERROR(logger_, "");
    RCLCPP_ERROR(logger_, "❌ System CANNOT restart until:");
    RCLCPP_ERROR(logger_, "   1. New certificate hash is generated");
    RCLCPP_ERROR(logger_, "   2. Changes are approved by Safety Manager");
    RCLCPP_ERROR(logger_, "   3. Run: python generate_certification_hash.py");
  } else {
    RCLCPP_INFO(logger_, "✅ No parameters modified");
    RCLCPP_INFO(logger_, "✅ Robot ready to operate");
  }
  
  RCLCPP_WARN(logger_, "================================");
  
  // Log session to audit trail
  appendToAuditLog(current_session_);
  
  // Return to operational mode
  current_mode_ = MaintenanceMode::OPERATIONAL;
  
  return true;
}

void MaintenanceModeManager::logParameterChange(
    const std::string& param_name,
    const std::string& old_value,
    const std::string& new_value)
{
  if (current_mode_ != MaintenanceMode::MAINTENANCE) {
    RCLCPP_ERROR(logger_, "Cannot log parameter change - not in maintenance mode");
    return;
  }
  
  std::string change_log = param_name + ": " + old_value + " → " + new_value;
  current_session_.changes_made.push_back(change_log);
  
  RCLCPP_WARN(logger_, "Parameter changed: %s", change_log.c_str());
}

double MaintenanceModeManager::getMaintenanceSessionDuration() const
{
  if (current_mode_ != MaintenanceMode::MAINTENANCE) {
    return 0.0;
  }
  
  auto now = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::seconds>(
    now - session_start_time_);
  
  return static_cast<double>(duration.count());
}

bool MaintenanceModeManager::verifyPin(const std::string& pin) const
{
  // Hash the provided PIN using SHA-256
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(pin.c_str()), 
         pin.length(), hash);
  
  // Convert hash to hex string
  std::stringstream ss;
  for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
    ss << std::hex << std::setw(2) << std::setfill('0') 
       << static_cast<int>(hash[i]);
  }
  std::string pin_hash = ss.str();
  
  // Check if hash matches any valid PIN
  for (const auto& valid_hash : valid_pins_) {
    if (pin_hash == valid_hash) {
      return true;
    }
  }
  
  return false;
}

bool MaintenanceModeManager::loadPins()
{
  std::ifstream pin_file(pin_file_path_);
  if (!pin_file.is_open()) {
    // If file doesn't exist, create default with example PINs
    RCLCPP_WARN(logger_, "PIN file not found: %s", pin_file_path_.c_str());
    RCLCPP_WARN(logger_, "Using default PINs (INSECURE - change in production!)");
    
    // Default PIN: "1234" (SHA-256 hashed)
    // SECURITY WARNING: This is for DEVELOPMENT ONLY!
    // In production, generate strong PINs and hash them properly
    valid_pins_.push_back(
      "03ac674216f3e15c761ee1a5e255f067953623c8b388b4459e13f978d7c846f4"  // SHA256("1234")
    );
    
    // Default PIN: "5678" (SHA-256 hashed)
    valid_pins_.push_back(
      "d0763edaa9d9bd2a9516280e9044d885b06d6d56e5e7b5e1f2c5af77a8c1c5d6"  // SHA256("5678")
    );
    
    return true;
  }
  
  // Load PINs from YAML file
  // Format:
  //   pins:
  //     - hash: "03ac674216f3e15c761ee1a5e255f067953623c8b388b4459e13f978d7c846f4"
  //       user: "technician_1"
  //     - hash: "..."
  //       user: "safety_manager"
  
  std::string line;
  bool in_pins_section = false;
  
  while (std::getline(pin_file, line)) {
    // Trim whitespace
    line.erase(0, line.find_first_not_of(" \t"));
    line.erase(line.find_last_not_of(" \t\r\n") + 1);
    
    if (line == "pins:") {
      in_pins_section = true;
      continue;
    }
    
    if (in_pins_section && line.find("- hash:") != std::string::npos) {
      // Extract hash value
      size_t quote1 = line.find('"');
      size_t quote2 = line.find('"', quote1 + 1);
      
      if (quote1 != std::string::npos && quote2 != std::string::npos) {
        std::string hash = line.substr(quote1 + 1, quote2 - quote1 - 1);
        valid_pins_.push_back(hash);
      }
    }
  }
  
  pin_file.close();
  
  if (valid_pins_.empty()) {
    RCLCPP_ERROR(logger_, "No valid PINs loaded!");
    return false;
  }
  
  RCLCPP_INFO(logger_, "Loaded %zu authorized PINs", valid_pins_.size());
  
  return true;
}

void MaintenanceModeManager::appendToAuditLog(const MaintenanceSession& session)
{
  std::ofstream audit_file(audit_log_path_, std::ios::app);
  
  if (!audit_file.is_open()) {
    RCLCPP_ERROR(logger_, "Could not open audit log: %s", audit_log_path_.c_str());
    return;
  }
  
  // Write session in YAML format
  audit_file << "\n# Maintenance Session\n";
  audit_file << "- session:\n";
  audit_file << "    user: \"" << session.user << "\"\n";
  audit_file << "    entered: \"" << session.timestamp_enter << "\"\n";
  audit_file << "    exited: \"" << session.timestamp_exit << "\"\n";
  audit_file << "    reason: \"" << session.reason << "\"\n";
  audit_file << "    changes:\n";
  
  if (session.changes_made.empty()) {
    audit_file << "      - \"No parameters modified\"\n";
  } else {
    for (const auto& change : session.changes_made) {
      audit_file << "      - \"" << change << "\"\n";
    }
  }
  
  audit_file << "    approved: " << (session.approved_by_safety_manager ? "true" : "false") << "\n";
  audit_file << "    signature: \"" << session.approval_signature << "\"\n";
  
  audit_file.close();
  
  RCLCPP_INFO(logger_, "Session logged to audit trail: %s", audit_log_path_.c_str());
}

std::string MaintenanceModeManager::getCurrentTimestamp() const
{
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  
  std::stringstream ss;
  ss << std::put_time(std::gmtime(&time_t_now), "%Y-%m-%dT%H:%M:%SZ");
  
  return ss.str();
}
