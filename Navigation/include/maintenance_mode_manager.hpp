/**
 * @file maintenance_mode_manager.hpp
 * @brief Maintenance Mode Manager for safe parameter modification
 * 
 * Implements ISO 3691-4 §5.2.6 requirement for separate operational/maintenance modes.
 * 
 * SAFETY RATIONALE:
 * - Operational Mode: Parameters LOCKED, robot can operate normally
 * - Maintenance Mode: Parameters UNLOCKED, robot IMMOBILIZED (safety stop)
 * 
 * This prevents:
 * - Unauthorized parameter changes during operation
 * - Accidental robot movement during configuration
 * - Parameter changes without proper authentication/authorization
 * 
 * Compliance:
 * - ISO 3691-4 §5.2.6: Separation of operational and maintenance modes
 * - ISO 13849-1 §5.2.2: Protection of safety parameters
 * - IEC 62443-3-3: Security levels and access control
 * 
 * @version 1.0.0
 * @date 2025-10-29
 */

#ifndef MAINTENANCE_MODE_MANAGER_HPP
#define MAINTENANCE_MODE_MANAGER_HPP

#include <string>
#include <chrono>
#include <fstream>
#include <ctime>
#include <rclcpp/rclcpp.hpp>

/**
 * @enum MaintenanceMode
 * @brief Operating modes of the system
 */
enum class MaintenanceMode {
  OPERATIONAL,  ///< Normal operation - parameters locked, robot can move
  MAINTENANCE   ///< Maintenance mode - parameters unlocked, robot stopped
};

/**
 * @struct MaintenanceSession
 * @brief Information about a maintenance session (for audit trail)
 */
struct MaintenanceSession {
  std::string user;                      ///< Username who entered maintenance mode
  std::string timestamp_enter;           ///< When maintenance mode was entered
  std::string timestamp_exit;            ///< When maintenance mode was exited
  std::string reason;                    ///< Reason for entering maintenance mode
  std::vector<std::string> changes_made; ///< List of parameter changes made
  bool approved_by_safety_manager;       ///< Whether changes were approved
  std::string approval_signature;        ///< Digital signature of approver
};

/**
 * @class MaintenanceModeManager
 * @brief Manages transitions between operational and maintenance modes
 * 
 * USAGE:
 * ```cpp
 * MaintenanceModeManager mgr("/path/to/audit_log.yaml");
 * 
 * // Try to enter maintenance mode
 * if (mgr.enterMaintenanceMode("1234", "technician_jose", "Update velocity limits")) {
 *     // Robot is now STOPPED, parameters can be modified
 *     // Modify parameters...
 *     mgr.logParameterChange("max_linear_velocity", "1.0", "1.5");
 *     
 *     // Exit maintenance mode
 *     mgr.exitMaintenanceMode();
 *     // Robot can now operate again
 * }
 * ```
 */
class MaintenanceModeManager {
public:
  /**
   * @brief Constructor
   * @param audit_log_path Path to YAML file for audit trail logging
   * @param pin_file_path Path to file containing valid PINs (default: /etc/ultrabot/pins.yaml)
   * @param logger Optional ROS2 logger (if nullptr, creates default logger)
   */
  explicit MaintenanceModeManager(
    const std::string& audit_log_path,
    const std::string& pin_file_path = "/etc/ultrabot/maintenance_pins.yaml",
    rclcpp::Logger logger = rclcpp::get_logger("maintenance_mode_manager"));
  
  /**
   * @brief Attempt to enter maintenance mode
   * @param pin 4-digit PIN for authentication
   * @param user Username of person entering maintenance mode
   * @param reason Reason for entering maintenance (logged for audit)
   * @return true if authentication successful and mode changed, false otherwise
   * 
   * Side effects:
   * - Triggers safety stop (robot immobilized)
   * - Unlocks safety parameters
   * - Logs entry to audit trail
   */
  bool enterMaintenanceMode(
    const std::string& pin,
    const std::string& user,
    const std::string& reason);
  
  /**
   * @brief Exit maintenance mode and return to operational mode
   * @return true if mode changed successfully
   * 
   * Side effects:
   * - Locks safety parameters
   * - Releases safety stop (robot can move again)
   * - Logs exit to audit trail
   * - Generates warning if parameters were modified
   */
  bool exitMaintenanceMode();
  
  /**
   * @brief Get current operating mode
   * @return Current mode (OPERATIONAL or MAINTENANCE)
   */
  MaintenanceMode getCurrentMode() const { return current_mode_; }
  
  /**
   * @brief Check if system is in maintenance mode
   * @return true if in MAINTENANCE mode, false if OPERATIONAL
   */
  bool isInMaintenanceMode() const { 
    return current_mode_ == MaintenanceMode::MAINTENANCE; 
  }
  
  /**
   * @brief Log a parameter change made during maintenance
   * @param param_name Name of parameter changed
   * @param old_value Previous value (as string)
   * @param new_value New value (as string)
   * 
   * This information is added to the audit trail for traceability.
   * Must only be called when in MAINTENANCE mode.
   */
  void logParameterChange(
    const std::string& param_name,
    const std::string& old_value,
    const std::string& new_value);
  
  /**
   * @brief Get time elapsed in current maintenance session
   * @return Duration in seconds, or 0 if not in maintenance mode
   */
  double getMaintenanceSessionDuration() const;
  
  /**
   * @brief Verify if a PIN is valid
   * @param pin PIN to verify (4-digit string)
   * @return true if PIN is valid, false otherwise
   */
  bool verifyPin(const std::string& pin) const;
  
  /**
   * @brief Get current maintenance session info
   * @return Pointer to current session, or nullptr if not in maintenance mode
   */
  const MaintenanceSession* getCurrentSession() const {
    return current_mode_ == MaintenanceMode::MAINTENANCE ? &current_session_ : nullptr;
  }

private:
  MaintenanceMode current_mode_;         ///< Current operating mode
  MaintenanceSession current_session_;   ///< Information about current session
  std::string audit_log_path_;           ///< Path to audit trail YAML file
  std::string pin_file_path_;            ///< Path to PIN database
  
  std::chrono::time_point<std::chrono::system_clock> session_start_time_;
  
  rclcpp::Logger logger_;                ///< ROS2 logger for consistent logging
  std::shared_ptr<rclcpp::Clock> clock_; ///< ROS2 clock for throttled logging
  
  /**
   * @brief Load valid PINs from configuration file
   * @return true if PINs loaded successfully
   */
  bool loadPins();
  
  /**
   * @brief Append session to audit log
   * @param session Session to log
   */
  void appendToAuditLog(const MaintenanceSession& session);
  
  /**
   * @brief Get current timestamp as ISO 8601 string
   * @return Timestamp string (e.g., "2025-10-29T14:30:00Z")
   */
  std::string getCurrentTimestamp() const;
  
  std::vector<std::string> valid_pins_;  ///< List of valid PINs (hashed)
};

#endif // MAINTENANCE_MODE_MANAGER_HPP
