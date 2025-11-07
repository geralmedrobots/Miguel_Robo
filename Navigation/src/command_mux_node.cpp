// ============================================================================
// COMMAND MULTIPLEXER NODE - NAV2 INTEGRATION
// ============================================================================
// Priority-based command arbitration for autonomous navigation integration:
//   Priority 0 (HIGHEST): Safety supervisor / Emergency stop
//   Priority 1 (HIGH):    Teleop (keyboard/joystick) - manual override
//   Priority 2 (MEDIUM):  Nav2 autonomous navigation
//   Priority 3 (LOW):     Other sources
//
// Safety features:
//   - Timeout-based preemption (inactive sources release control)
//   - Explicit priority ordering (higher priority preempts lower)
//   - Zero-velocity timeout enforcement
//   - Diagnostic publishing for monitoring
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

class CommandMuxNode : public rclcpp::Node
{
public:
   CommandMuxNode();

private:
   enum class Priority : int {
      TELEOP = 1,      // Manual control (keyboard/joystick) - HIGHEST PRIORITY
      NAV2 = 2,        // Autonomous navigation
      OTHER = 3,       // Other sources
      NONE = 99        // No active source
   };
   
   // NOTE: Safety/Emergency handled by command_arbitrator (Priority 255)
   // command_mux focuses on teleop vs autonomous arbitration

   struct CommandSource {
      geometry_msgs::msg::Twist cmd;
      rclcpp::Time last_received;
      std::chrono::milliseconds timeout;
      bool active;
      std::string name;
   };

   void teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
   void nav2Callback(const geometry_msgs::msg::Twist::SharedPtr msg);
   void otherCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
   
   void timerCallback();
   void updateActiveSource();
   void publishDiagnostics();
   
   bool isSourceActive(const CommandSource & source) const;
   Priority getHighestActivePriority() const;

   // Subscriptions for different command sources
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_sub_;
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr other_sub_;

   // Output publisher (to command_arbitrator or directly to driver)
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
   rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

   // Timer for periodic arbitration
   rclcpp::TimerBase::SharedPtr timer_;

   // Command sources with priorities
   std::map<Priority, CommandSource> sources_;
   std::mutex sources_mutex_;

   Priority active_priority_;

   // Configuration parameters
   double default_timeout_sec_;
   double teleop_timeout_sec_;
   double nav2_timeout_sec_;
};

CommandMuxNode::CommandMuxNode()
   : Node("command_mux"),
     active_priority_(Priority::NONE)
{
   // Declare parameters
   this->declare_parameter("default_timeout", 0.5);
   this->declare_parameter("teleop_timeout", 1.0);
   this->declare_parameter("nav2_timeout", 0.5);

   default_timeout_sec_ = this->get_parameter("default_timeout").as_double();
   teleop_timeout_sec_ = this->get_parameter("teleop_timeout").as_double();
   nav2_timeout_sec_ = this->get_parameter("nav2_timeout").as_double();

   // Initialize command sources
   // NOTE: SAFETY priority removed - handled by command_arbitrator emergency channel
   
   sources_[Priority::TELEOP] = {
      geometry_msgs::msg::Twist(),
      this->now(),
      std::chrono::milliseconds(static_cast<int64_t>(teleop_timeout_sec_ * 1000)),
      false,
      "teleop"
   };

   sources_[Priority::NAV2] = {
      geometry_msgs::msg::Twist(),
      this->now(),
      std::chrono::milliseconds(static_cast<int64_t>(nav2_timeout_sec_ * 1000)),
      false,
      "nav2"
   };

   sources_[Priority::OTHER] = {
      geometry_msgs::msg::Twist(),
      this->now(),
      std::chrono::milliseconds(static_cast<int64_t>(default_timeout_sec_ * 1000)),
      false,
      "other"
   };

   // Create subscriptions
   // NOTE: cmd_vel_safety removed - safety is handled by command_arbitrator's emergency input
   // Safety supervisor operates on output validation, not input commands
   
   teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_teleop", rclcpp::QoS(10),
      std::bind(&CommandMuxNode::teleopCallback, this, std::placeholders::_1));

   // SAFETY FIX: Listen to cmd_vel_nav_safe (output from collision_monitor)
   // This ensures collision avoidance safety layer is enforced
   // Chain: controller → cmd_vel_nav → collision_monitor → cmd_vel_nav_safe → mux
   nav2_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nav_safe", rclcpp::QoS(10),
      std::bind(&CommandMuxNode::nav2Callback, this, std::placeholders::_1));

   other_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_other", rclcpp::QoS(10),
      std::bind(&CommandMuxNode::otherCallback, this, std::placeholders::_1));

   // Create publishers
   cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel_mux", rclcpp::QoS(10));

   status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "command_mux/status", rclcpp::QoS(10).transient_local());

   diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics", rclcpp::QoS(10));

   // Timer for periodic arbitration (20 Hz)
   timer_ = this->create_wall_timer(
      50ms,
      std::bind(&CommandMuxNode::timerCallback, this));

   RCLCPP_INFO(this->get_logger(), 
      "Command Multiplexer initialized (timeouts: teleop=%.1fs, nav2=%.1fs, default=%.1fs)",
      teleop_timeout_sec_, nav2_timeout_sec_, default_timeout_sec_);
}

void CommandMuxNode::teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
   std::lock_guard<std::mutex> lock(sources_mutex_);
   sources_[Priority::TELEOP].cmd = *msg;
   sources_[Priority::TELEOP].last_received = this->now();
   sources_[Priority::TELEOP].active = true;
}

void CommandMuxNode::nav2Callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
   std::lock_guard<std::mutex> lock(sources_mutex_);
   sources_[Priority::NAV2].cmd = *msg;
   sources_[Priority::NAV2].last_received = this->now();
   sources_[Priority::NAV2].active = true;
}

void CommandMuxNode::otherCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
   std::lock_guard<std::mutex> lock(sources_mutex_);
   sources_[Priority::OTHER].cmd = *msg;
   sources_[Priority::OTHER].last_received = this->now();
   sources_[Priority::OTHER].active = true;
}

bool CommandMuxNode::isSourceActive(const CommandSource & source) const
{
   if (!source.active) {
      return false;
   }

   auto elapsed = this->now() - source.last_received;
   return elapsed.seconds() < (source.timeout.count() / 1000.0);
}

Priority CommandMuxNode::getHighestActivePriority() const
{
   // Check priorities in order (0 = highest priority)
   for (const auto & [priority, source] : sources_) {
      if (isSourceActive(source)) {
         return priority;
      }
   }
   return Priority::NONE;
}

void CommandMuxNode::updateActiveSource()
{
   Priority new_priority = getHighestActivePriority();

   // Priority changed - log transition
   if (new_priority != active_priority_) {
      if (new_priority == Priority::NONE) {
         RCLCPP_INFO(this->get_logger(), "No active command source - sending zero velocity");
      } else {
         const std::string & new_name = sources_[new_priority].name;
         if (active_priority_ != Priority::NONE) {
            const std::string & old_name = sources_[active_priority_].name;
            RCLCPP_INFO(this->get_logger(), 
               "Command source priority change: %s -> %s", 
               old_name.c_str(), new_name.c_str());
         } else {
            RCLCPP_INFO(this->get_logger(), 
               "Command source activated: %s", new_name.c_str());
         }
      }
      active_priority_ = new_priority;

      // Publish status change
      std_msgs::msg::String status_msg;
      if (new_priority == Priority::NONE) {
         status_msg.data = "INACTIVE";
      } else {
         status_msg.data = sources_[new_priority].name;
      }
      status_pub_->publish(status_msg);
   }

   // Publish command from active source (or zero if none)
   geometry_msgs::msg::Twist cmd_out;
   if (new_priority != Priority::NONE) {
      cmd_out = sources_[new_priority].cmd;
   }
   // else: cmd_out is zero-initialized

   cmd_pub_->publish(cmd_out);
}

void CommandMuxNode::publishDiagnostics()
{
   diagnostic_msgs::msg::DiagnosticArray diag_array;
   diag_array.header.stamp = this->now();

   diagnostic_msgs::msg::DiagnosticStatus diag;
   diag.name = "command_mux";
   diag.hardware_id = "mux";

   std::lock_guard<std::mutex> lock(sources_mutex_);

   // Determine overall status
   if (active_priority_ == Priority::NONE) {
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diag.message = "No active command source";
   } else {
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      diag.message = "Active source: " + sources_[active_priority_].name;
   }

   // Add details for each source
   for (const auto & [priority, source] : sources_) {
      diagnostic_msgs::msg::KeyValue kv_active;
      kv_active.key = source.name + "_active";
      kv_active.value = isSourceActive(source) ? "true" : "false";
      diag.values.push_back(kv_active);

      diagnostic_msgs::msg::KeyValue kv_age;
      kv_age.key = source.name + "_age_sec";
      auto age = this->now() - source.last_received;
      kv_age.value = std::to_string(age.seconds());
      diag.values.push_back(kv_age);

      diagnostic_msgs::msg::KeyValue kv_timeout;
      kv_timeout.key = source.name + "_timeout_sec";
      kv_timeout.value = std::to_string(source.timeout.count() / 1000.0);
      diag.values.push_back(kv_timeout);
   }

   diag_array.status.push_back(diag);
   diag_pub_->publish(diag_array);
}

void CommandMuxNode::timerCallback()
{
   std::lock_guard<std::mutex> lock(sources_mutex_);
   updateActiveSource();
   publishDiagnostics();
}

int main(int argc, char ** argv)
{
   rclcpp::init(argc, argv);

   try {
      auto node = std::make_shared<CommandMuxNode>();
      rclcpp::spin(node);
   } catch (const std::exception & e) {
      RCLCPP_FATAL(rclcpp::get_logger("command_mux"), 
         "Unhandled exception: %s", e.what());
      rclcpp::shutdown();
      return 1;
   }

   rclcpp::shutdown();
   return 0;
}
