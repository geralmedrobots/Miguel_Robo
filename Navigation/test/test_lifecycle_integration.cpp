#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <launch/launch_description.h>
#include <launch_testing/launch_testing.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <thread>
#include <memory>

using namespace std::chrono_literals;

// Test fixture to manage node and executor
class LifecycleIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("lifecycle_test_client");
        executor_.add_node(node_);
    }

    void TearDown() override {
        executor_.cancel();
        executor_.remove_node(node_);
        node_.reset();
        rclcpp::shutdown();
    }

    // Helper to wait for a node to reach a specific state
    bool waitForState(const std::string& node_name, uint8_t expected_state, std::chrono::seconds timeout) {
        auto topic_name = "/" + node_name + "/transition_event";
        auto sub = node_->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
            topic_name, 10,
            [&](const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
                if (msg->goal_state.id == expected_state) {
                    state_reached_ = true;
                }
            });

        state_reached_ = false;
        auto start_time = std::chrono::steady_clock::now();
        while (!state_reached_ && (std::chrono::steady_clock::now() - start_time) < timeout) {
            executor_.spin_some(100ms);
            std::this_thread::sleep_for(100ms);
        }
        return state_reached_;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::atomic<bool> state_reached_{false};
};

// Define the launch description for the test
launch::LaunchDescription generate_test_description() {
    auto package_share_dir = ament_index_cpp::get_package_share_directory("somanet");
    auto launch_file_path = package_share_dir + "/launch/launch.py";

    return launch::LaunchDescription({
        launch::actions::IncludeLaunchDescription(
            launch::actions::PythonLaunchDescriptionSource(launch_file_path)
        ),
        // This is a required action for launch_testing
        launch_testing::actions::ReadyToTest()
    });
}

// Test case: Verify that critical nodes transition to ACTIVE state
TEST_F(LifecycleIntegrationTest, TestNodesActivateSuccessfully) {
    // Wait for the safety_supervisor to become active
    ASSERT_TRUE(waitForState("safety_supervisor", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, 30s))
        << "Timed out waiting for safety_supervisor to become active.";

    RCLCPP_INFO(node_->get_logger(), "✅ safety_supervisor reached ACTIVE state.");

    // Wait for the somanet_driver to become active
    ASSERT_TRUE(waitForState("somanet_driver", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, 30s))
        << "Timed out waiting for somanet_driver to become active.";
    
    RCLCPP_INFO(node_->get_logger(), "✅ somanet_driver reached ACTIVE state.");
}
