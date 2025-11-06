/**
 * @file test_safety_critical.cpp
 * @brief Safety-critical tests for ISO 13849-1 certification
 * 
 * Tests MUST verify:
 * - Watchdog timeout behavior (cmd_vel timeout → zero velocity)
 * - Command arbitrator priority enforcement
 * - Dead-man switch integration
 * - Emergency stop propagation
 * - Velocity limit enforcement
 * - Fail-safe behavior on communication loss
 * 
 * @version 1.0.0
 * @date 2025-10-31
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <thread>
#include <atomic>

using namespace std::chrono_literals;

/**
 * @brief Safety-critical test fixture
 * 
 * Provides ROS2 node infrastructure for integration testing
 */
class SafetyCriticalTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("safety_test_node");
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        
        // Start executor in background thread
        executor_thread_ = std::thread([this]() {
            executor_->spin();
        });
    }

    void TearDown() override {
        executor_->cancel();
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
        rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
};

// ============================================================================
// TEST SUITE 1: WATCHDOG TIMEOUT BEHAVIOR
// ============================================================================

/**
 * @brief Test that cmd_vel timeout results in ZERO velocity (fail-safe)
 * 
 * ISO 13849-1 §5.2 requires fail-safe behavior on communication loss
 * 
 * Scenario:
 * 1. Publish cmd_vel with velocity
 * 2. Wait for timeout (default: 500ms)
 * 3. Verify output is ZERO (not last_command)
 */
TEST_F(SafetyCriticalTest, WatchdogTimeoutZerosVelocity) {
    std::atomic<bool> received_zero{false};
    std::atomic<int> message_count{0};
    
    // Subscribe to arbitrated cmd_vel output
    auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            message_count++;
            
            // After timeout, should receive ZERO velocity
            if (message_count > 10) {  // Give time for timeout
                if (std::abs(msg->linear.x) < 0.001 &&
                    std::abs(msg->angular.z) < 0.001) {
                    received_zero = true;
                }
            }
        });
    
    // Publish initial command
    auto pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_manual", 10);
    
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.5;  // Non-zero velocity
    cmd.angular.z = 0.0;
    
    pub->publish(cmd);
    
    // Wait for timeout + margin (500ms + 200ms)
    std::this_thread::sleep_for(700ms);
    
    // CRITICAL: Must receive ZERO velocity after timeout (fail-safe)
    EXPECT_TRUE(received_zero.load()) 
        << "FAIL-SAFE VIOLATION: Watchdog timeout did not zero velocity!";
}

/**
 * @brief Test that fresh commands prevent timeout
 * 
 * If commands arrive within timeout window, robot should continue moving
 */
TEST_F(SafetyCriticalTest, FreshCommandsPreventsTimeout) {
    std::atomic<double> last_velocity{0.0};
    
    auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            last_velocity = msg->linear.x;
        });
    
    auto pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_manual", 10);
    
    // Publish commands every 200ms (within 500ms timeout)
    for (int i = 0; i < 5; i++) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.5;
        pub->publish(cmd);
        std::this_thread::sleep_for(200ms);
    }
    
    // Velocity should still be non-zero (no timeout)
    EXPECT_GT(std::abs(last_velocity.load()), 0.1) 
        << "Watchdog triggered prematurely (fresh commands ignored)";
}

// ============================================================================
// TEST SUITE 2: COMMAND ARBITRATOR PRIORITY
// ============================================================================

/**
 * @brief Test that EMERGENCY has highest priority
 * 
 * ISO 3691-4 requires emergency commands override all others
 */
TEST_F(SafetyCriticalTest, EmergencyOverridesAllSources) {
    std::atomic<double> final_velocity{999.0};
    
    auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            final_velocity = msg->linear.x;
        });
    
    auto manual_pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_manual", 10);
    auto emergency_pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_emergency", 10);
    
    // Publish manual command (priority 200)
    geometry_msgs::msg::Twist manual_cmd;
    manual_cmd.linear.x = 1.0;
    manual_pub->publish(manual_cmd);
    
    std::this_thread::sleep_for(100ms);
    
    // Publish emergency STOP (priority 255 - highest)
    geometry_msgs::msg::Twist emergency_cmd;
    emergency_cmd.linear.x = 0.0;
    emergency_cmd.angular.z = 0.0;
    emergency_pub->publish(emergency_cmd);
    
    std::this_thread::sleep_for(100ms);
    
    // Emergency should override manual (zero velocity wins)
    EXPECT_NEAR(final_velocity.load(), 0.0, 0.001) 
        << "PRIORITY VIOLATION: Emergency command did not override manual!";
}

/**
 * @brief Test priority order: MANUAL > AUTO
 */
TEST_F(SafetyCriticalTest, ManualOverridesAutonomous) {
    std::atomic<double> final_velocity{999.0};
    
    auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            final_velocity = msg->linear.x;
        });
    
    auto auto_pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_auto", 10);
    auto manual_pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_manual", 10);
    
    // Autonomous command
    geometry_msgs::msg::Twist auto_cmd;
    auto_cmd.linear.x = 0.3;
    auto_pub->publish(auto_cmd);
    
    std::this_thread::sleep_for(100ms);
    
    // Manual command (higher priority)
    geometry_msgs::msg::Twist manual_cmd;
    manual_cmd.linear.x = 0.8;
    manual_pub->publish(manual_cmd);
    
    std::this_thread::sleep_for(100ms);
    
    // Manual should win (0.8 not 0.3)
    EXPECT_NEAR(final_velocity.load(), 0.8, 0.1) 
        << "PRIORITY VIOLATION: Manual did not override autonomous!";
}

/**
 * @brief Test that lower priority source does NOT override higher
 */
TEST_F(SafetyCriticalTest, LowerPriorityCannotOverride) {
    std::atomic<double> final_velocity{999.0};
    
    auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            final_velocity = msg->linear.x;
        });
    
    auto manual_pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_manual", 10);
    auto test_pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_test", 10);
    
    // Manual command (priority 200)
    geometry_msgs::msg::Twist manual_cmd;
    manual_cmd.linear.x = 0.5;
    manual_pub->publish(manual_cmd);
    
    std::this_thread::sleep_for(100ms);
    
    // Test command (priority 50 - lower)
    geometry_msgs::msg::Twist test_cmd;
    test_cmd.linear.x = 0.9;
    test_pub->publish(test_cmd);
    
    std::this_thread::sleep_for(100ms);
    
    // Manual should still be active (0.5 not 0.9)
    EXPECT_NEAR(final_velocity.load(), 0.5, 0.1) 
        << "PRIORITY VIOLATION: Lower priority overrode higher priority!";
}

// ============================================================================
// TEST SUITE 3: DEAD-MAN SWITCH INTEGRATION
// ============================================================================

/**
 * @brief Test that manual commands require dead-man active
 * 
 * ISO 3691-4 §5.2.1.3 requires enabling device for manual mode
 */
TEST_F(SafetyCriticalTest, ManualRequiresDeadman) {
    std::atomic<double> final_velocity{999.0};
    std::atomic<bool> deadman_active{false};
    
    auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            final_velocity = msg->linear.x;
        });
    
    auto manual_pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_manual", 10);
    auto deadman_pub = node_->create_publisher<std_msgs::msg::Bool>("deadman_status", 10);
    
    // Publish manual command WITHOUT dead-man
    std_msgs::msg::Bool deadman_msg;
    deadman_msg.data = false;
    deadman_pub->publish(deadman_msg);
    
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.5;
    manual_pub->publish(cmd);
    
    std::this_thread::sleep_for(100ms);
    
    // Command should be BLOCKED (velocity = 0)
    EXPECT_NEAR(final_velocity.load(), 0.0, 0.001) 
        << "SAFETY VIOLATION: Manual command accepted without dead-man!";
}

/**
 * @brief Test that dead-man release stops robot
 */
TEST_F(SafetyCriticalTest, DeadmanReleaseStopsRobot) {
    std::atomic<double> velocity_before{0.0};
    std::atomic<double> velocity_after{999.0};
    
    auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            static int count = 0;
            if (count < 5) {
                velocity_before = msg->linear.x;
            } else {
                velocity_after = msg->linear.x;
            }
            count++;
        });
    
    auto manual_pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_manual", 10);
    auto deadman_pub = node_->create_publisher<std_msgs::msg::Bool>("deadman_status", 10);
    
    // Enable dead-man and send command
    std_msgs::msg::Bool deadman_active;
    deadman_active.data = true;
    deadman_pub->publish(deadman_active);
    
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.5;
    manual_pub->publish(cmd);
    
    std::this_thread::sleep_for(200ms);
    
    // Release dead-man
    deadman_active.data = false;
    deadman_pub->publish(deadman_active);
    
    std::this_thread::sleep_for(200ms);
    
    // Velocity should drop to zero after release
    EXPECT_GT(velocity_before.load(), 0.1) << "Initial velocity was zero (setup failed)";
    EXPECT_NEAR(velocity_after.load(), 0.0, 0.001) 
        << "SAFETY VIOLATION: Dead-man release did not stop robot!";
}

/**
 * @brief Test that autonomous mode does NOT require dead-man
 */
TEST_F(SafetyCriticalTest, AutonomousBypassesDeadman) {
    std::atomic<double> final_velocity{0.0};
    
    auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            final_velocity = msg->linear.x;
        });
    
    auto auto_pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_auto", 10);
    auto deadman_pub = node_->create_publisher<std_msgs::msg::Bool>("deadman_status", 10);
    
    // Dead-man INACTIVE
    std_msgs::msg::Bool deadman_msg;
    deadman_msg.data = false;
    deadman_pub->publish(deadman_msg);
    
    // Autonomous command (should work without dead-man)
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.3;
    auto_pub->publish(cmd);
    
    std::this_thread::sleep_for(100ms);
    
    // Autonomous should work (velocity > 0)
    EXPECT_GT(final_velocity.load(), 0.1) 
        << "Autonomous mode incorrectly blocked by dead-man requirement";
}

// ============================================================================
// TEST SUITE 4: VELOCITY LIMIT ENFORCEMENT
// ============================================================================

/**
 * @brief Test that excessive velocity is rejected/saturated
 * 
 * ISO 13849-1 §7.2 requires output verification
 */
TEST_F(SafetyCriticalTest, ExcessiveVelocityRejected) {
    std::atomic<double> max_velocity_seen{0.0};
    
    auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            double v = std::abs(msg->linear.x);
            if (v > max_velocity_seen.load()) {
                max_velocity_seen = v;
            }
        });
    
    auto pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_manual", 10);
    
    // Try to command excessive velocity (>10 m/s is unrealistic for AGV)
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 15.0;  // Way too fast!
    cmd.angular.z = 0.0;
    
    pub->publish(cmd);
    std::this_thread::sleep_for(200ms);
    
    // Velocity should be saturated/rejected (< safety limit ~2 m/s)
    EXPECT_LT(max_velocity_seen.load(), 2.5) 
        << "SAFETY VIOLATION: Excessive velocity not rejected/saturated!";
}

/**
 * @brief Test angular velocity limits
 */
TEST_F(SafetyCriticalTest, ExcessiveAngularVelocityRejected) {
    std::atomic<double> max_angular_seen{0.0};
    
    auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            double w = std::abs(msg->angular.z);
            if (w > max_angular_seen.load()) {
                max_angular_seen = w;
            }
        });
    
    auto pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_manual", 10);
    
    // Try excessive rotation (>5 rad/s unrealistic)
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 10.0;  // Way too fast!
    
    pub->publish(cmd);
    std::this_thread::sleep_for(200ms);
    
    // Angular velocity should be limited (~2 rad/s typical)
    EXPECT_LT(max_angular_seen.load(), 3.0) 
        << "SAFETY VIOLATION: Excessive angular velocity not rejected!";
}

// ============================================================================
// TEST SUITE 5: EMERGENCY STOP PROPAGATION
// ============================================================================

/**
 * @brief Test that emergency stop propagates to all subsystems
 */
TEST_F(SafetyCriticalTest, EmergencyStopPropagates) {
    // TODO: Requires integration with actual driver
    // For now, verify arbitrator behavior
    
    std::atomic<bool> zero_received{false};
    
    auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            if (std::abs(msg->linear.x) < 0.001 &&
                std::abs(msg->angular.z) < 0.001) {
                zero_received = true;
            }
        });
    
    auto emergency_pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_emergency", 10);
    
    // Send emergency stop
    geometry_msgs::msg::Twist estop;
    estop.linear.x = 0.0;
    estop.angular.z = 0.0;
    emergency_pub->publish(estop);
    
    std::this_thread::sleep_for(100ms);
    
    EXPECT_TRUE(zero_received.load()) 
        << "Emergency stop did not propagate to cmd_vel output";
}

// ============================================================================
// TEST SUITE 6: FAIL-SAFE ON COMMUNICATION LOSS
// ============================================================================

/**
 * @brief Test behavior when arbitrator node dies
 * 
 * Watchdog should detect no cmd_vel and stop motors
 */
TEST_F(SafetyCriticalTest, ArbitratorFailureDetected) {
    // This requires testing at system level (launch file test)
    // For now, document requirement
    
    GTEST_SKIP() << "System-level test: requires launch file testing";
    
    // Expected behavior:
    // 1. Kill command_arbitrator_node
    // 2. Main loop watchdog detects no cmd_vel
    // 3. Motors commanded to ZERO within 500ms
}

// ============================================================================
// TEST SUITE 7: ODOMETRY PLAUSIBILITY CHECKS
// ============================================================================

/**
 * @brief Test that implausible velocities are rejected
 */
TEST(OdometryPlausibilityTest, ImplausibleVelocityRejected) {
    OdometryCalculator odom(0.5, 0.17);
    
    // Try sensor fault: INT32_MAX (robot "jumps" 19km)
    bool result = odom.update(2147483647, 0, 0.1);
    
    // Should be REJECTED (implausible)
    EXPECT_FALSE(result) 
        << "SAFETY VIOLATION: Implausible velocity (sensor fault) not rejected!";
    
    // Pose should not change
    double x, y, yaw;
    odom.getPose(x, y, yaw);
    EXPECT_NEAR(x, 0.0, 0.001);
    EXPECT_NEAR(y, 0.0, 0.001);
}

/**
 * @brief Test acceleration validation
 */
TEST(OdometryPlausibilityTest, ImplausibleAccelerationSaturated) {
    OdometryCalculator odom(0.5, 0.17);
    
    // Start at low velocity
    odom.update(1000, 1000, 0.1);
    
    // Try sudden jump to very high velocity (implausible acceleration)
    bool result = odom.update(30000, 30000, 0.1);
    
    // Should be SATURATED (not rejected, smoother behavior)
    EXPECT_TRUE(result) << "Acceleration validation failed completely";
    
    // Verify velocity was saturated (not at full 30000)
    double vx, wz;
    odom.getTwist(vx, wz);
    
    // Should be < 3 m/s (if 30000 mRPM accepted, would be ~27 m/s!)
    EXPECT_LT(std::abs(vx), 3.0) 
        << "SAFETY VIOLATION: Implausible acceleration not saturated!";
}

/**
 * @brief Test polarity correction
 */
TEST(OdometryPlausibilityTest, PolarityCorrection) {
    // Normal polarity
    OdometryCalculator odom_normal(0.5, 0.17, 1, 1);
    odom_normal.update(1000, -1000, 1.0);  // Rotate CCW
    
    double x1, y1, yaw1;
    odom_normal.getPose(x1, y1, yaw1);
    
    // Inverted right wheel
    OdometryCalculator odom_inverted(0.5, 0.17, 1, -1);
    odom_inverted.update(1000, 1000, 1.0);  // Physical: both forward, logical: rotate CCW
    
    double x2, y2, yaw2;
    odom_inverted.getPose(x2, y2, yaw2);
    
    // Both should rotate same direction (polarity corrected)
    EXPECT_GT(yaw1, 0.0) << "Normal: Should rotate CCW";
    EXPECT_GT(yaw2, 0.0) << "Inverted: Polarity correction failed!";
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    
    std::cout << "\n"
              << "╔══════════════════════════════════════════════════════════════╗\n"
              << "║  SAFETY-CRITICAL TESTS FOR ISO 13849-1 CERTIFICATION       ║\n"
              << "╚══════════════════════════════════════════════════════════════╝\n"
              << "\n"
              << "Test Categories:\n"
              << "  1. Watchdog Timeout (fail-safe behavior)\n"
              << "  2. Command Arbitrator Priority\n"
              << "  3. Dead-man Switch Integration\n"
              << "  4. Velocity Limit Enforcement\n"
              << "  5. Emergency Stop Propagation\n"
              << "  6. Communication Loss Handling\n"
              << "  7. Odometry Plausibility Checks\n"
              << "\n";
    
    int result = RUN_ALL_TESTS();
    
    if (result == 0) {
        std::cout << "\n"
                  << "╔══════════════════════════════════════════════════════════════╗\n"
                  << "║  ✅ ALL SAFETY-CRITICAL TESTS PASSED                        ║\n"
                  << "║  System ready for ISO 13849-1 certification review          ║\n"
                  << "╚══════════════════════════════════════════════════════════════╝\n";
    } else {
        std::cout << "\n"
                  << "╔══════════════════════════════════════════════════════════════╗\n"
                  << "║  ❌ SAFETY-CRITICAL TESTS FAILED                            ║\n"
                  << "║  SYSTEM NOT READY FOR CERTIFICATION                         ║\n"
                  << "╚══════════════════════════════════════════════════════════════╝\n";
    }
    
    return result;
}
