#!/usr/bin/env python3
"""
Keyboard teleoperation with safety features

Safety features:
- Deadman key requirement (Space bar)
- Command validation and saturation
- Watchdog timeout detection
- Operator action logging
- Parameter validation

Compliance: ISO 3691-4 (Manual mode operation)

@version 1.1.0
@date 2025-10-28
"""

from __future__ import print_function

import threading
import time
import hashlib
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

import sys

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
╔══════════════════════════════════════════════════════════════════╗
║      SAFE KEYBOARD TELEOPERATION - Ultrabot AGV (ISO 3691-4)     ║
╚══════════════════════════════════════════════════════════════════╝

⚠️  SAFETY: Hold SPACE BAR (deadman) while moving!

Movement Commands:
   u    i    o
   j    k    l      i/k = forward/backward
   m    ,    .      j/l = rotate left/right
                    u/o = diagonals

Speed Control:
   q/z : increase/decrease max speeds by 10%
   w/x : increase/decrease only linear speed by 10%
   e/c : increase/decrease only angular speed by 10%

Safety:
   SPACE : Deadman button (MUST hold to move)
   CTRL-C: Quit safely

Status: Waiting for deadman activation...
"""

moveBindings = {
    'i': (1, 0, 0),
    'o': (1, 0, -1),
    'j': (0, 0, 1),
    'l': (0, 0, -1),
    'u': (1, 0, 1),
    ',': (-1, 0, 0),
    '.': (-1, 0, 1),
    'm': (-1, 0, -1),
    'k': (0, 0, 0),  # stop
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


class SafeTeleopKeyboard(Node):
    """
    Keyboard teleoperation node with comprehensive safety features
    
    Features:
    - Deadman key enforcement (Space bar)
    - Velocity scaling and saturation
    - Watchdog timeout monitoring
    - Operator action logging for audit trail
    """
    
    def __init__(self):
        super().__init__('safe_teleop_keyboard')
        
        # Declare parameters
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('turn', 0.5)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('require_deadman', True)
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('operator_id', 'unknown')
        
        # Get parameters
        self.speed = self.get_parameter('speed').value
        self.turn = self.get_parameter('turn').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.require_deadman = self.get_parameter('require_deadman').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self.operator_id = self.get_parameter('operator_id').value
        
        # Validate parameters
        if not self._validate_parameters():
            raise ValueError("Invalid parameters")
        
        # Publishers
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.deadman_pub = self.create_publisher(Bool, 'deadman_status', 10)
        self.log_pub = self.create_publisher(String, 'operator_log', 10)
        
        # State tracking
        self.deadman_active = False
        self.last_key_time = self.get_clock().now()
        self.command_count = 0
        self.watchdog_triggered = False
        
        # Movement state
        self.x = 0
        self.y = 0
        self.th = 0
        
        # Watchdog timer
        self.watchdog_timer = self.create_timer(
            self.watchdog_timeout / 2.0,
            self._watchdog_callback
        )
        
        self.get_logger().info(
            f"Safe Teleop Keyboard initialized - Operator: {self.operator_id}"
        )
        self.get_logger().info(
            f"Limits: max_linear={self.max_linear_speed:.2f} m/s, "
            f"max_angular={self.max_angular_speed:.2f} rad/s, "
            f"watchdog={self.watchdog_timeout:.2f}s"
        )
        
        self._log_operator_action("TELEOP_STARTED", 0.0, 0.0)
    
    def _validate_parameters(self):
        """Validate all parameters"""
        valid = True
        
        if self.max_linear_speed <= 0.0:
            self.get_logger().error("max_linear_speed must be > 0")
            valid = False
        
        if self.max_angular_speed <= 0.0:
            self.get_logger().error("max_angular_speed must be > 0")
            valid = False
        
        if self.watchdog_timeout <= 0.0:
            self.get_logger().error("watchdog_timeout must be > 0")
            valid = False
        
        return valid
    
    def _watchdog_callback(self):
        """Check for watchdog timeout"""
        elapsed = (self.get_clock().now() - self.last_key_time).nanoseconds / 1e9
        
        if elapsed > self.watchdog_timeout and not self.watchdog_triggered:
            self.watchdog_triggered = True
            self.get_logger().warn(
                f"Keyboard watchdog timeout: {elapsed:.2f}s since last key"
            )
            self._log_operator_action("WATCHDOG_TIMEOUT", 0.0, 0.0)
            
            # Publish stop command
            self._publish_twist(0.0, 0.0, 0.0)
            
            # Publish deadman inactive
            deadman_msg = Bool()
            deadman_msg.data = False
            self.deadman_pub.publish(deadman_msg)
    
    def _log_operator_action(self, action, linear, angular):
        """Log operator action for audit trail"""
        log_msg = String()
        timestamp = self.get_clock().now().seconds_nanoseconds()[0]
        
        log_msg.data = (
            f"{timestamp} | {self.operator_id} | {action} | "
            f"lin={linear:.3f} | ang={angular:.3f}"
        )
        
        self.log_pub.publish(log_msg)
        self.get_logger().info(f"Operator log: {log_msg.data}")
    
    def _publish_twist(self, x, y, th):
        """Publish velocity command with safety checks"""
        twist = Twist()
        
        # Calculate velocities
        linear_vel = x * self.speed
        angular_vel = th * self.turn
        
        # Clamp to safety limits
        linear_vel = max(min(linear_vel, self.max_linear_speed), -self.max_linear_speed)
        angular_vel = max(min(angular_vel, self.max_angular_speed), -self.max_angular_speed)
        
        # Apply deadman logic
        if self.require_deadman and not self.deadman_active:
            linear_vel = 0.0
            angular_vel = 0.0
        
        twist.linear.x = linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_vel
        
        self.vel_pub.publish(twist)
        
        # Log significant commands
        if abs(linear_vel) > 0.01 or abs(angular_vel) > 0.01:
            self.command_count += 1
            if self.command_count % 20 == 0:  # Log every 20 commands
                self._log_operator_action("COMMAND", linear_vel, angular_vel)
    
    def process_key(self, key):
        """Process keyboard input with safety checks"""
        self.last_key_time = self.get_clock().now()
        self.watchdog_triggered = False
        
        # Check for deadman key (space bar)
        if key == ' ':
            self.deadman_active = True
            deadman_msg = Bool()
            deadman_msg.data = True
            self.deadman_pub.publish(deadman_msg)
            self.get_logger().debug("Deadman activated")
            return True
        else:
            self.deadman_active = False
            deadman_msg = Bool()
            deadman_msg.data = False
            self.deadman_pub.publish(deadman_msg)
        
        # Movement keys
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.y = moveBindings[key][1]
            self.th = moveBindings[key][2]
            
        # Speed adjustment keys
        elif key in speedBindings.keys():
            self.speed = self.speed * speedBindings[key][0]
            self.turn = self.turn * speedBindings[key][1]
            
            # Clamp speeds to maximum limits
            self.speed = min(self.speed, self.max_linear_speed)
            self.turn = min(self.turn, self.max_angular_speed)
            
            print(f"Speed: {self.speed:.2f} m/s, Turn: {self.turn:.2f} rad/s")
            
        # Stop on any other key
        else:
            if key == '\x03':  # CTRL-C
                return False
            self.x = 0
            self.y = 0
            self.th = 0
        
        # Publish the command
        self._publish_twist(self.x, self.y, self.th)
        
        return True


def getKey(settings):
    """Get a single keypress from the keyboard"""
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    """Save terminal settings for later restoration"""
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    """Restore terminal settings"""
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    """Main function"""
    settings = saveTerminalSettings()
    
    rclpy.init()
    node = SafeTeleopKeyboard()
    
    print(msg)
    
    try:
        while rclpy.ok():
            key = getKey(settings)
            
            if not node.process_key(key):
                break
            
            # Spin to process callbacks
            rclpy.spin_once(node, timeout_sec=0.0)
    
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        # Send stop command
        node._publish_twist(0.0, 0.0, 0.0)
        node._log_operator_action("TELEOP_STOPPED", 0.0, 0.0)
        
        restoreTerminalSettings(settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
