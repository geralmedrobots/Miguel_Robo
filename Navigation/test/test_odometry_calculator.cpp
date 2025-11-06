#include <gtest/gtest.h>
#include "odometry_calculator.hpp"
#include <cmath>

/**
 * @brief Unit tests for OdometryCalculator
 * 
 * Tests differential drive kinematics calculations
 */

class OdometryCalculatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Standard robot configuration
        odom_ = std::make_unique<OdometryCalculator>(0.5, 0.17);
    }

    std::unique_ptr<OdometryCalculator> odom_;
};

TEST_F(OdometryCalculatorTest, InitialPoseIsZero) {
    double x, y, yaw;
    odom_->getPose(x, y, yaw);
    
    EXPECT_DOUBLE_EQ(x, 0.0);
    EXPECT_DOUBLE_EQ(y, 0.0);
    EXPECT_DOUBLE_EQ(yaw, 0.0);
}

TEST_F(OdometryCalculatorTest, StraightLineMotion) {
    // Both wheels at 1000 mRPM for 1 second
    // wheel_diameter = 0.17m → circumference = 0.534m
    // 1 RPM → 0.534 m/min → 1000 mRPM = 1 RPM = 0.00891 m/s
    
    odom_->update(1000, 1000, 1.0);
    
    double x, y, yaw;
    odom_->getPose(x, y, yaw);
    
    EXPECT_GT(x, 0.0);          // Moved forward
    EXPECT_NEAR(y, 0.0, 0.001); // No lateral motion
    EXPECT_NEAR(yaw, 0.0, 0.01); // No rotation
}

TEST_F(OdometryCalculatorTest, RotationInPlace) {
    // Left wheel backward, right wheel forward at same speed
    // wheel_base = 0.5m
    odom_->update(-1000, 1000, 1.0);
    
    double x, y, yaw;
    odom_->getPose(x, y, yaw);
    
    EXPECT_NEAR(x, 0.0, 0.001); // No forward motion
    EXPECT_NEAR(y, 0.0, 0.001); // No lateral motion
    EXPECT_GT(yaw, 0.0);        // Rotated counterclockwise
}

TEST_F(OdometryCalculatorTest, ZeroVelocityNoMotion) {
    odom_->update(0, 0, 1.0);
    
    double x, y, yaw;
    odom_->getPose(x, y, yaw);
    
    EXPECT_DOUBLE_EQ(x, 0.0);
    EXPECT_DOUBLE_EQ(y, 0.0);
    EXPECT_DOUBLE_EQ(yaw, 0.0);
}

TEST_F(OdometryCalculatorTest, ResetClearsPose) {
    odom_->update(1000, 1000, 1.0);
    odom_->reset();
    
    double x, y, yaw;
    odom_->getPose(x, y, yaw);
    
    EXPECT_DOUBLE_EQ(x, 0.0);
    EXPECT_DOUBLE_EQ(y, 0.0);
    EXPECT_DOUBLE_EQ(yaw, 0.0);
}

TEST_F(OdometryCalculatorTest, SetPoseWorks) {
    odom_->setPose(1.5, 2.0, M_PI / 2);
    
    double x, y, yaw;
    odom_->getPose(x, y, yaw);
    
    EXPECT_DOUBLE_EQ(x, 1.5);
    EXPECT_DOUBLE_EQ(y, 2.0);
    EXPECT_DOUBLE_EQ(yaw, M_PI / 2);
}

TEST_F(OdometryCalculatorTest, AngleNormalization) {
    // Set angle > pi
    odom_->setPose(0, 0, 4.0 * M_PI);
    
    double x, y, yaw;
    odom_->getPose(x, y, yaw);
    
    // Should be normalized to [-pi, pi]
    EXPECT_GE(yaw, -M_PI);
    EXPECT_LE(yaw, M_PI);
}

TEST_F(OdometryCalculatorTest, DistanceTraveled) {
    // Move forward multiple times
    for (int i = 0; i < 10; i++) {
        odom_->update(1000, 1000, 0.1);
    }
    
    double distance = odom_->getDistanceTraveled();
    EXPECT_GT(distance, 0.0);
}

TEST_F(OdometryCalculatorTest, InvalidDeltaTimeRejected) {
    bool result1 = odom_->update(1000, 1000, 0.0);  // Zero time
    bool result2 = odom_->update(1000, 1000, -0.1); // Negative time
    
    EXPECT_FALSE(result1);
    EXPECT_FALSE(result2);
}

TEST_F(OdometryCalculatorTest, LargeDeltaTimeSubdivided) {
    // v3.0 accepts large delta_t and subdivides internally
    double x_before, y_before, yaw_before;
    odom_->getPose(x_before, y_before, yaw_before);
    
    bool result = odom_->update(1000, 1000, 2.5);  // Large delta_t
    
    EXPECT_TRUE(result);  // Should accept and subdivide
    
    // Verify pose changed correctly (forward motion occurred)
    double x_after, y_after, yaw_after;
    odom_->getPose(x_after, y_after, yaw_after);
    EXPECT_GT(x_after, x_before);  // Moved forward
    EXPECT_NEAR(y_after, y_before, 0.001);  // No lateral motion
    EXPECT_NEAR(yaw_after, yaw_before, 0.01);  // No rotation
}

TEST_F(OdometryCalculatorTest, TwistVelocity) {
    odom_->update(1000, 1000, 0.1);
    
    double vx, wz;
    odom_->getTwist(vx, wz);
    
    EXPECT_GT(vx, 0.0);          // Forward velocity
    EXPECT_NEAR(wz, 0.0, 0.01);  // No angular velocity
}

TEST_F(OdometryCalculatorTest, CovarianceGrowsWithDistance) {
    double var_pos1, var_yaw1, var_pos2, var_yaw2;
    
    // Initial covariance
    odom_->getCovariance(var_pos1, var_yaw1);
    
    // Move robot 10 meters
    for (int i = 0; i < 100; i++) {
        odom_->update(5000, 5000, 0.1);
    }
    
    // Final covariance
    odom_->getCovariance(var_pos2, var_yaw2);
    
    // Covariance should increase with distance
    EXPECT_GT(var_pos2, var_pos1);
    EXPECT_GT(var_yaw2, var_yaw1);
}

// Run all tests
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
