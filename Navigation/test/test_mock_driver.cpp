#include <gtest/gtest.h>
#include "mock_driver.hpp"
#include <thread>
#include <chrono>

/**
 * @brief Unit tests for MockDriver
 * 
 * Validates mock implementation behavior
 */

class MockDriverTest : public ::testing::Test {
protected:
    void SetUp() override {
        driver_ = std::make_unique<MockDriver>();
        driver_->initialize("mock");
    }

    std::unique_ptr<MockDriver> driver_;
};

TEST_F(MockDriverTest, InitializationSucceeds) {
    EXPECT_TRUE(driver_->isOperational());
}

TEST_F(MockDriverTest, InitialVelocityIsZero) {
    auto [left, right] = driver_->getVelocity();
    EXPECT_EQ(left, 0);
    EXPECT_EQ(right, 0);
}

TEST_F(MockDriverTest, SetVelocityAccepted) {
    bool result = driver_->setVelocity(1000, 1000);
    EXPECT_TRUE(result);
}

TEST_F(MockDriverTest, VelocityRampsGradually) {
    // Set target velocity
    driver_->setVelocity(1000, 1000);
    
    // Simulate multiple cycles
    for (int i = 0; i < 10; i++) {
        driver_->simulateCycle();
    }
    
    // Velocity should approach target gradually
    auto [left, right] = driver_->getVelocity();
    EXPECT_GT(left, 0);
    EXPECT_LT(left, 1000); // Not instant
}

TEST_F(MockDriverTest, EmergencyStopWorks) {
    driver_->setVelocity(5000, 5000);
    driver_->emergencyStop("Test stop");
    
    auto [left, right] = driver_->getVelocity();
    EXPECT_EQ(left, 0);
    EXPECT_EQ(right, 0);
    EXPECT_FALSE(driver_->isOperational());
}

TEST_F(MockDriverTest, CannotSetVelocityAfterEStop) {
    driver_->emergencyStop("Test");
    bool result = driver_->setVelocity(1000, 1000);
    EXPECT_FALSE(result);
}

TEST_F(MockDriverTest, ResetClearsEmergencyStop) {
    driver_->emergencyStop("Test");
    driver_->reset();
    
    bool result = driver_->setVelocity(1000, 1000);
    EXPECT_TRUE(result);
}

TEST_F(MockDriverTest, CallbackInvoked) {
    int callback_count = 0;
    int last_left = 0, last_right = 0;
    
    driver_->registerUpdateCallback([&](int left, int right) {
        callback_count++;
        last_left = left;
        last_right = right;
    });
    
    driver_->setVelocity(1000, 1000);
    driver_->simulateCycle();
    
    EXPECT_GT(callback_count, 0);
}

TEST_F(MockDriverTest, StatusStringNotEmpty) {
    std::string status = driver_->getStatusString();
    EXPECT_FALSE(status.empty());
    EXPECT_NE(status.find("MockDriver"), std::string::npos);
}

TEST_F(MockDriverTest, ShutdownStopsOperation) {
    driver_->shutdown();
    EXPECT_FALSE(driver_->isOperational());
    
    bool result = driver_->setVelocity(1000, 1000);
    EXPECT_FALSE(result);
}

// Run all tests
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
