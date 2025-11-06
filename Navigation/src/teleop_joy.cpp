// Teleop joystick with safety and system integration
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

class TeleopJoy : public rclcpp::Node {
public:
  TeleopJoy() : Node("teleop_joy"), last_joy_time_(this->now()), deadman_active_(false) {
    // Declare parameters
    this->declare_parameter("axis_linear", 1);
    this->declare_parameter("axis_angular", 0);
    this->declare_parameter("scale_linear", 0.5);
    this->declare_parameter("scale_angular", 1.0);
    this->declare_parameter("deadman_button", 4);
    this->declare_parameter("max_linear_vel", 1.0);
    this->declare_parameter("max_angular_vel", 1.5);
    this->declare_parameter("watchdog_timeout", 0.5);
    
    // Get parameters
    axis_linear_ = this->get_parameter("axis_linear").as_int();
    axis_angular_ = this->get_parameter("axis_angular").as_int();
    scale_linear_ = this->get_parameter("scale_linear").as_double();
    scale_angular_ = this->get_parameter("scale_angular").as_double();
    deadman_button_ = this->get_parameter("deadman_button").as_int();
    max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
    watchdog_timeout_ = this->get_parameter("watchdog_timeout").as_double();
    
    // Publishers - CORRECTED TOPIC FOR COMMAND_MUX INTEGRATION
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_teleop", 10);
    deadman_pub_ = this->create_publisher<std_msgs::msg::Bool>("deadman_status", 10);
    
    // Subscribers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&TeleopJoy::joyCallback, this, std::placeholders::_1));
    
    // Watchdog timer
    watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TeleopJoy::watchdogCallback, this));
    
    RCLCPP_INFO(this->get_logger(), 
      "Teleop Joy initialized - Publishing to cmd_vel_teleop (command_mux input)");
    RCLCPP_INFO(this->get_logger(),
      "Config: linear_axis=%d, angular_axis=%d, deadman_button=%d",
      axis_linear_, axis_angular_, deadman_button_);
    RCLCPP_INFO(this->get_logger(),
      "Limits: max_linear=%.2f m/s, max_angular=%.2f rad/s, watchdog=%.2fs",
      max_linear_vel_, max_angular_vel_, watchdog_timeout_);
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
    last_joy_time_ = this->now();
    
    // Validate array sizes
    if (joy->axes.size() <= static_cast<size_t>(std::max(axis_linear_, axis_angular_)) ||
        joy->buttons.size() <= static_cast<size_t>(deadman_button_)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Joystick has insufficient axes or buttons");
      return;
    }
    
    geometry_msgs::msg::Twist twist;
    bool deadman_pressed = (joy->buttons[deadman_button_] == 1);
    
    // Publish deadman status
    std_msgs::msg::Bool deadman_msg;
    deadman_msg.data = deadman_pressed;
    deadman_pub_->publish(deadman_msg);
    
    if (deadman_pressed) {
      if (!deadman_active_) {
        RCLCPP_INFO(this->get_logger(), "Deadman ACTIVE - manual control enabled");
        deadman_active_ = true;
      }
      
      // Apply scaling and clamp to safety limits
      double linear_raw = std::clamp(joy->axes[axis_linear_], -1.0, 1.0);
      double angular_raw = std::clamp(joy->axes[axis_angular_], -1.0, 1.0);
      
      twist.linear.x = std::clamp(linear_raw * scale_linear_, 
                                   -max_linear_vel_, max_linear_vel_);
      twist.angular.z = std::clamp(angular_raw * scale_angular_,
                                    -max_angular_vel_, max_angular_vel_);
      
      // Log significant commands
      if (std::abs(twist.linear.x) > 0.01 || std::abs(twist.angular.z) > 0.01) {
        RCLCPP_DEBUG(this->get_logger(), "CMD: linear=%.2f, angular=%.2f",
                     twist.linear.x, twist.angular.z);
      }
    } else {
      if (deadman_active_) {
        RCLCPP_INFO(this->get_logger(), "Deadman RELEASED - robot stopped");
        deadman_active_ = false;
      }
      // twist already initialized to zero
    }
    
    vel_pub_->publish(twist);
  }
  
  void watchdogCallback() {
    auto elapsed = (this->now() - last_joy_time_).seconds();
    
    if (elapsed > watchdog_timeout_) {
      if (deadman_active_) {
        RCLCPP_WARN(this->get_logger(), 
          "Joystick watchdog timeout (%.2fs) - sending zero velocity", elapsed);
        deadman_active_ = false;
      }
      
      // Send zero velocity
      geometry_msgs::msg::Twist zero;
      vel_pub_->publish(zero);
      
      // Update deadman status
      std_msgs::msg::Bool deadman_msg;
      deadman_msg.data = false;
      deadman_pub_->publish(deadman_msg);
    }
  }

  // Publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deadman_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  
  // Configuration parameters
  int axis_linear_;
  int axis_angular_;
  int deadman_button_;
  double scale_linear_;
  double scale_angular_;
  double max_linear_vel_;
  double max_angular_vel_;
  double watchdog_timeout_;
  
  // State
  rclcpp::Time last_joy_time_;
  bool deadman_active_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopJoy>());
  rclcpp::shutdown();
  return 0;
}
