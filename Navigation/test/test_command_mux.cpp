/**
 * @file test_command_mux.cpp
 * @brief Unit tests for CommandMuxNode (multi-source command prioritization)
 * 
 * Tests verify:
 * - Priority-based command selection (teleop > nav2 > other)
 * - Source timeout handling (inactive source detection)
 * - Zero-velocity fallback (no active sources)
 * - Diagnostic publishing (status monitoring)
 * - Active source switching (priority preemption)
 * 
 * @version 1.0.0
 * @date 2025-11-07
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>

using namespace std::chrono_literals;

// Forward declaration of CommandMuxNode (defined in command_mux_node.cpp)
// We'll use process_shared to run the actual node in the same process
class CommandMuxNode : public rclcpp::Node {
public:
    CommandMuxNode();
};

/**
 * @brief Test fixture for CommandMuxNode unit tests
 * 
 * Provides isolated ROS2 environment with actual CommandMuxNode instance
 */
class CommandMuxTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        // Create the actual CommandMuxNode under test
        mux_node_ = std::make_shared<CommandMuxNode>();
        
        // Create test node for publishers/subscribers
        test_node_ = std::make_shared<rclcpp::Node>("mux_test_node");
        
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(mux_node_);
        executor_->add_node(test_node_);
        
        // Start executor in background
        executor_thread_ = std::thread([this]() {
            executor_->spin();
        });
        
        // Allow time for discovery and node initialization
        std::this_thread::sleep_for(300ms);
    }

    void TearDown() override {
        executor_->cancel();
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
        mux_node_.reset();
        test_node_.reset();
        rclcpp::shutdown();
    }
    
    /**
     * @brief Helper to create Twist message with specified velocities
     */
    geometry_msgs::msg::Twist makeTwist(double vx, double wz) {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = vx;
        msg.angular.z = wz;
        return msg;
    }
    
    /**
     * @brief Wait for condition with timeout
     */
    template<typename Predicate>
    bool waitFor(Predicate pred, std::chrono::milliseconds timeout = 1000ms) {
        auto start = std::chrono::steady_clock::now();
        while (!pred()) {
            if (std::chrono::steady_clock::now() - start > timeout) {
                return false;
            }
            std::this_thread::sleep_for(10ms);
        }
        return true;
    }

    std::shared_ptr<CommandMuxNode> mux_node_;  // The node under test
    std::shared_ptr<rclcpp::Node> test_node_;    // Helper node for test publishers/subscribers
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
};

// ============================================================================
// TEST SUITE 1: PRIORITY-BASED COMMAND SELECTION
// ============================================================================

/**
 * @brief Test that TELEOP priority (1) overrides NAV2 (2)
 * 
 * Manual control must always override autonomous navigation for safety
 */
TEST_F(CommandMuxTest, TeleopPriorityOverridesNav2) {
    // Create publishers for different command sources
    auto teleop_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_teleop", rclcpp::QoS(10));
    auto nav2_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_nav_safe", rclcpp::QoS(10));
    
    // Create subscriber to monitor mux output
    std::atomic<bool> received{false};
    geometry_msgs::msg::Twist last_cmd;
    auto output_sub = test_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_mux", rclcpp::QoS(10),
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            last_cmd = *msg;
            received = true;
        });
    
    // Publish NAV2 command first
    nav2_pub->publish(makeTwist(0.5, 0.1));
    std::this_thread::sleep_for(100ms);
    
    // Should see NAV2 command on output
    ASSERT_TRUE(waitFor([&]() { return received.load(); }));
    EXPECT_NEAR(last_cmd.linear.x, 0.5, 0.01);
    EXPECT_NEAR(last_cmd.angular.z, 0.1, 0.01);
    
    // Publish TELEOP command (higher priority)
    received = false;
    teleop_pub->publish(makeTwist(1.0, 0.5));
    std::this_thread::sleep_for(100ms);
    
    // Should now see TELEOP command (overrides NAV2)
    ASSERT_TRUE(waitFor([&]() { return received.load(); }));
    EXPECT_NEAR(last_cmd.linear.x, 1.0, 0.01);
    EXPECT_NEAR(last_cmd.angular.z, 0.5, 0.01);
}

/**
 * @brief Test that TELEOP priority (1) overrides OTHER (3)
 */
TEST_F(CommandMuxTest, TeleopPriorityOverridesOther) {
    auto teleop_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_teleop", rclcpp::QoS(10));
    auto other_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_other", rclcpp::QoS(10));
    
    std::atomic<bool> received{false};
    geometry_msgs::msg::Twist last_cmd;
    auto output_sub = test_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_mux", rclcpp::QoS(10),
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            last_cmd = *msg;
            received = true;
        });
    
    // Publish OTHER command first
    other_pub->publish(makeTwist(0.3, 0.05));
    std::this_thread::sleep_for(100ms);
    
    // Publish TELEOP command (higher priority)
    received = false;
    teleop_pub->publish(makeTwist(0.8, 0.3));
    std::this_thread::sleep_for(100ms);
    
    // Should see TELEOP command
    ASSERT_TRUE(waitFor([&]() { return received.load(); }));
    EXPECT_NEAR(last_cmd.linear.x, 0.8, 0.01);
    EXPECT_NEAR(last_cmd.angular.z, 0.3, 0.01);
}

/**
 * @brief Test that NAV2 priority (2) overrides OTHER (3)
 */
TEST_F(CommandMuxTest, Nav2PriorityOverridesOther) {
    auto nav2_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_nav_safe", rclcpp::QoS(10));
    auto other_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_other", rclcpp::QoS(10));
    
    std::atomic<bool> received{false};
    geometry_msgs::msg::Twist last_cmd;
    auto output_sub = test_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_mux", rclcpp::QoS(10),
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            last_cmd = *msg;
            received = true;
        });
    
    // Publish OTHER command first
    other_pub->publish(makeTwist(0.2, 0.02));
    std::this_thread::sleep_for(100ms);
    
    // Publish NAV2 command (higher priority)
    received = false;
    nav2_pub->publish(makeTwist(0.6, 0.15));
    std::this_thread::sleep_for(100ms);
    
    // Should see NAV2 command
    ASSERT_TRUE(waitFor([&]() { return received.load(); }));
    EXPECT_NEAR(last_cmd.linear.x, 0.6, 0.01);
    EXPECT_NEAR(last_cmd.angular.z, 0.15, 0.01);
}

// ============================================================================
// TEST SUITE 2: TIMEOUT HANDLING
// ============================================================================

/**
 * @brief Test that sources timeout after configured duration
 * 
 * Stale commands must be rejected to prevent unsafe autonomous operation
 */
TEST_F(CommandMuxTest, SourceTimeoutDeactivation) {
    // NOTE: CommandMuxNode has default timeout of 0.5s (500ms) for default_timeout
    // We'll publish once, wait for timeout, and verify zero-velocity output
    
    auto teleop_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_teleop", rclcpp::QoS(10));
    
    std::atomic<int> msg_count{0};
    geometry_msgs::msg::Twist last_cmd;
    auto output_sub = test_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_mux", rclcpp::QoS(10),
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            last_cmd = *msg;
            msg_count++;
        });
    
    // Publish teleop command
    teleop_pub->publish(makeTwist(0.5, 0.1));
    std::this_thread::sleep_for(100ms);
    
    // Should see command
    ASSERT_TRUE(waitFor([&]() { return msg_count.load() > 0; }));
    EXPECT_NEAR(last_cmd.linear.x, 0.5, 0.01);
    
    // Wait for teleop timeout (1.0s) + processing margin
    std::this_thread::sleep_for(1200ms);
    
    // Should now receive zero-velocity (timeout)
    EXPECT_NEAR(last_cmd.linear.x, 0.0, 0.01);
    EXPECT_NEAR(last_cmd.angular.z, 0.0, 0.01);
}

/**
 * @brief Test that lower priority becomes active when higher priority times out
 */
TEST_F(CommandMuxTest, PriorityRevertOnTimeout) {
    // NOTE: Default timeouts - teleop: 1.0s, nav2: 0.5s
    // We'll publish nav2 continuously and teleop once, then wait for teleop timeout
    
    auto teleop_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_teleop", rclcpp::QoS(10));
    auto nav2_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_nav_safe", rclcpp::QoS(10));
    
    std::atomic<int> msg_count{0};
    geometry_msgs::msg::Twist last_cmd;
    auto output_sub = test_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_mux", rclcpp::QoS(10),
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            last_cmd = *msg;
            msg_count++;
        });
    
    // Publish both sources
    nav2_pub->publish(makeTwist(0.3, 0.05));
    std::this_thread::sleep_for(50ms);
    teleop_pub->publish(makeTwist(0.8, 0.2));
    std::this_thread::sleep_for(100ms);
    
    // Should see TELEOP (higher priority)
    EXPECT_NEAR(last_cmd.linear.x, 0.8, 0.01);
    
    // Keep publishing nav2 to prevent its timeout, wait for teleop timeout (1.0s)
    for (int i = 0; i < 12; ++i) {  // 12 * 100ms = 1200ms total
        nav2_pub->publish(makeTwist(0.3, 0.05));
        std::this_thread::sleep_for(100ms);
    }
    
    // Should revert to NAV2 command (teleop timed out after 1.0s)
    EXPECT_NEAR(last_cmd.linear.x, 0.3, 0.01);
    EXPECT_NEAR(last_cmd.angular.z, 0.05, 0.01);
}

// ============================================================================
// TEST SUITE 3: ZERO-VELOCITY FALLBACK
// ============================================================================

/**
 * @brief Test that zero velocity is published when no sources are active
 * 
 * Safety requirement: robot must stop when no valid commands exist
 */
TEST_F(CommandMuxTest, ZeroVelocityWhenNoActiveSources) {
    std::atomic<bool> received{false};
    geometry_msgs::msg::Twist last_cmd;
    auto output_sub = test_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_mux", rclcpp::QoS(10),
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            last_cmd = *msg;
            received = true;
        });
    
    // Wait for mux to publish (should be zero-velocity)
    std::this_thread::sleep_for(200ms);
    
    ASSERT_TRUE(waitFor([&]() { return received.load(); }));
    EXPECT_NEAR(last_cmd.linear.x, 0.0, 0.01);
    EXPECT_NEAR(last_cmd.linear.y, 0.0, 0.01);
    EXPECT_NEAR(last_cmd.linear.z, 0.0, 0.01);
    EXPECT_NEAR(last_cmd.angular.x, 0.0, 0.01);
    EXPECT_NEAR(last_cmd.angular.y, 0.0, 0.01);
    EXPECT_NEAR(last_cmd.angular.z, 0.0, 0.01);
}

/**
 * @brief Test that zero velocity is published after all sources timeout
 */
TEST_F(CommandMuxTest, ZeroVelocityAfterAllSourcesTimeout) {
    // Use default timeouts: teleop 1.0s, nav2 0.5s
    
    auto teleop_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_teleop", rclcpp::QoS(10));
    auto nav2_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_nav_safe", rclcpp::QoS(10));
    
    std::atomic<int> msg_count{0};
    geometry_msgs::msg::Twist last_cmd;
    auto output_sub = test_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_mux", rclcpp::QoS(10),
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            last_cmd = *msg;
            msg_count++;
        });
    
    // Publish both sources
    teleop_pub->publish(makeTwist(0.5, 0.1));
    nav2_pub->publish(makeTwist(0.3, 0.05));
    std::this_thread::sleep_for(100ms);
    
    // Should see non-zero command (teleop has priority)
    EXPECT_GT(std::abs(last_cmd.linear.x), 0.01);
    
    // Wait for both to timeout (teleop: 1.0s, nav2: 0.5s)
    std::this_thread::sleep_for(1200ms);
    
    // Should now be zero-velocity
    EXPECT_NEAR(last_cmd.linear.x, 0.0, 0.01);
    EXPECT_NEAR(last_cmd.angular.z, 0.0, 0.01);
}

// ============================================================================
// TEST SUITE 4: STATUS AND DIAGNOSTIC PUBLISHING
// ============================================================================

/**
 * @brief Test that status topic publishes active source name
 */
TEST_F(CommandMuxTest, StatusPublishesActiveSource) {
    auto teleop_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_teleop", rclcpp::QoS(10));
    
    std::atomic<bool> received{false};
    std::string last_status;
    auto status_sub = test_node_->create_subscription<std_msgs::msg::String>(
        "command_mux/status", rclcpp::QoS(10).transient_local(),
        [&](const std_msgs::msg::String::SharedPtr msg) {
            last_status = msg->data;
            received = true;
        });
    
    // Wait for initial status (should be INACTIVE)
    std::this_thread::sleep_for(200ms);
    ASSERT_TRUE(waitFor([&]() { return received.load(); }));
    EXPECT_EQ(last_status, "INACTIVE");
    
    // Publish teleop command
    received = false;
    teleop_pub->publish(makeTwist(0.5, 0.1));
    std::this_thread::sleep_for(100ms);
    
    // Should see "teleop" status
    ASSERT_TRUE(waitFor([&]() { return received.load(); }));
    EXPECT_EQ(last_status, "teleop");
}

/**
 * @brief Test that diagnostics publish source activity details
 */
TEST_F(CommandMuxTest, DiagnosticsPublishSourceDetails) {
    auto teleop_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_teleop", rclcpp::QoS(10));
    
    std::atomic<bool> received{false};
    diagnostic_msgs::msg::DiagnosticArray last_diag;
    auto diag_sub = test_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "diagnostics", rclcpp::QoS(10),
        [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
            last_diag = *msg;
            received = true;
        });
    
    // Publish teleop command
    teleop_pub->publish(makeTwist(0.5, 0.1));
    std::this_thread::sleep_for(100ms);
    
    // Should receive diagnostics
    ASSERT_TRUE(waitFor([&]() { return received.load(); }));
    ASSERT_EQ(last_diag.status.size(), 1u);
    
    const auto& status = last_diag.status[0];
    EXPECT_EQ(status.name, "command_mux");
    EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_EQ(status.message, "Active source: teleop");
    
    // Check that values include source details
    bool found_teleop_active = false;
    for (const auto& kv : status.values) {
        if (kv.key == "teleop_active") {
            EXPECT_EQ(kv.value, "true");
            found_teleop_active = true;
        }
    }
    EXPECT_TRUE(found_teleop_active);
}

/**
 * @brief Test that diagnostics show WARN when no active sources
 */
TEST_F(CommandMuxTest, DiagnosticsWarnWhenNoActiveSources) {
    std::atomic<bool> received{false};
    diagnostic_msgs::msg::DiagnosticArray last_diag;
    auto diag_sub = test_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "diagnostics", rclcpp::QoS(10),
        [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
            last_diag = *msg;
            received = true;
        });
    
    // Wait for initial diagnostics (no active sources)
    std::this_thread::sleep_for(200ms);
    
    ASSERT_TRUE(waitFor([&]() { return received.load(); }));
    ASSERT_EQ(last_diag.status.size(), 1u);
    
    const auto& status = last_diag.status[0];
    EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    EXPECT_EQ(status.message, "No active command source");
}

// ============================================================================
// TEST SUITE 5: ACTIVE SOURCE SWITCHING
// ============================================================================

/**
 * @brief Test rapid switching between teleop and nav2
 */
TEST_F(CommandMuxTest, RapidSourceSwitching) {
    auto teleop_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_teleop", rclcpp::QoS(10));
    auto nav2_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_nav_safe", rclcpp::QoS(10));
    
    std::atomic<int> msg_count{0};
    geometry_msgs::msg::Twist last_cmd;
    auto output_sub = test_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_mux", rclcpp::QoS(10),
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            last_cmd = *msg;
            msg_count++;
        });
    
    // Switch between sources rapidly
    for (int i = 0; i < 5; ++i) {
        teleop_pub->publish(makeTwist(0.5, 0.1));
        std::this_thread::sleep_for(30ms);
        nav2_pub->publish(makeTwist(0.3, 0.05));
        std::this_thread::sleep_for(30ms);
    }
    
    // Should see multiple messages (mux is responsive)
    EXPECT_GT(msg_count.load(), 5);
    
    // Last message should be from teleop (higher priority)
    std::this_thread::sleep_for(100ms);
    EXPECT_NEAR(last_cmd.linear.x, 0.5, 0.01);
}

/**
 * @brief Test that lower priority source doesn't interfere while higher priority is active
 */
TEST_F(CommandMuxTest, LowerPriorityNoInterference) {
    auto teleop_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_teleop", rclcpp::QoS(10));
    auto other_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_other", rclcpp::QoS(10));
    
    std::atomic<int> msg_count{0};
    geometry_msgs::msg::Twist last_cmd;
    auto output_sub = test_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_mux", rclcpp::QoS(10),
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            last_cmd = *msg;
            msg_count++;
        });
    
    // Activate teleop
    teleop_pub->publish(makeTwist(0.8, 0.2));
    std::this_thread::sleep_for(100ms);
    
    EXPECT_NEAR(last_cmd.linear.x, 0.8, 0.01);
    
    // Spam other source (should be ignored)
    for (int i = 0; i < 10; ++i) {
        other_pub->publish(makeTwist(0.1, 0.01));
        std::this_thread::sleep_for(20ms);
    }
    
    // Should still see teleop command
    EXPECT_NEAR(last_cmd.linear.x, 0.8, 0.01);
    EXPECT_NEAR(last_cmd.angular.z, 0.2, 0.01);
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
