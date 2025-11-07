/**
 * @file lifecycle_utils.hpp
 * @brief Common utilities for lifecycle node initialization
 * 
 * Centralizes autostart parsing logic to avoid duplication across nodes.
 * 
 * @version 1.0.0
 * @date 2025-11-07
 */

#ifndef LIFECYCLE_UTILS_HPP
#define LIFECYCLE_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <cstdlib>

namespace ultrabot {
namespace lifecycle_utils {

/**
 * @brief Convert string to boolean
 * 
 * Supports: "1", "true", "yes", "on" (case-insensitive) → true
 *           All other values → false
 * 
 * @param value String to convert
 * @return true if string represents true, false otherwise
 */
inline bool string_to_bool(const std::string& value) {
    std::string lowered = value;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), 
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    
    return lowered == "1" || lowered == "true" || lowered == "yes" || lowered == "on";
}

/**
 * @brief Parse command-line arguments and environment for autostart flag
 * 
 * Checks (in order of priority):
 * 1. Command-line flags: --autostart or --no-autostart
 * 2. Environment variable: ULTRABOT_AUTOSTART
 * 
 * Also initializes rclcpp with filtered arguments (removes autostart flags).
 * 
 * @param argc Argument count
 * @param argv Argument vector
 * @return true if autostart requested, false otherwise
 * 
 * @note This function calls rclcpp::init() internally.
 *       Do NOT call rclcpp::init() again after using this function.
 */
inline bool autostart_requested(int argc, char** argv) {
    bool cli_override = false;
    bool autostart = false;
    
    // Filter arguments, removing autostart flags
    std::vector<const char*> filtered_args;
    filtered_args.reserve(static_cast<size_t>(argc) + 1);
    filtered_args.push_back(argv[0]);  // Program name
    
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--autostart") == 0) {
            autostart = true;
            cli_override = true;
            continue;  // Skip this argument
        }
        if (std::strcmp(argv[i], "--no-autostart") == 0) {
            autostart = false;
            cli_override = true;
            continue;  // Skip this argument
        }
        filtered_args.push_back(argv[i]);
    }
    
    filtered_args.push_back(nullptr);  // Null terminator
    int filtered_argc = static_cast<int>(filtered_args.size()) - 1;
    
    // Initialize rclcpp with filtered arguments
    rclcpp::init(filtered_argc, const_cast<char**>(filtered_args.data()));
    
    // Check environment variable if no CLI override
    if (!cli_override) {
        if (const char* env = std::getenv("ULTRABOT_AUTOSTART")) {
            autostart = string_to_bool(env);
        }
    }
    
    return autostart;
}

}  // namespace lifecycle_utils
}  // namespace ultrabot

#endif  // LIFECYCLE_UTILS_HPP
