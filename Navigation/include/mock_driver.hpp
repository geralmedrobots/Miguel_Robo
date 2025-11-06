#include "drive_interface.hpp"
#include <iostream>
#include <cmath>

/**
 * @brief Mock drive implementation for testing without hardware
 * 
 * Simulates differential drive behavior with:
 * - Realistic acceleration limits
 * - Velocity tracking
 * - Error injection for testing
 * 
 * Perfect for unit tests and CI/CD pipelines
 */
class MockDriver : public DriveInterface {
public:
    MockDriver() : operational_(false), left_vel_(0), right_vel_(0), 
                   cycle_count_(0), emergency_stopped_(false) {}

    bool initialize(const std::string& interface_name) override {
        std::cout << "[MockDriver] Initializing (simulation mode)" << std::endl;
        operational_ = true;
        return true;
    }

    bool setVelocity(int left_mrpm, int right_mrpm) override {
        if (!operational_ || emergency_stopped_) {
            return false;
        }

        // Simulate acceleration limits (20% change per cycle)
        int delta_left = static_cast<int>((left_mrpm - left_vel_) * 0.2);
        int delta_right = static_cast<int>((right_mrpm - right_vel_) * 0.2);

        left_vel_ += delta_left;
        right_vel_ += delta_right;

        return true;
    }

    std::pair<int, int> getVelocity() const override {
        return {left_vel_, right_vel_};
    }

    bool isOperational() const override {
        return operational_ && !emergency_stopped_;
    }

    void emergencyStop(const std::string& reason) override {
        std::cout << "[MockDriver] EMERGENCY STOP: " << reason << std::endl;
        left_vel_ = 0;
        right_vel_ = 0;
        emergency_stopped_ = true;
    }

    void shutdown() override {
        operational_ = false;
        left_vel_ = 0;
        right_vel_ = 0;
    }

    void registerUpdateCallback(
        std::function<void(int left_mrpm, int right_mrpm)> callback) override {
        callback_ = callback;
    }

    std::string getStatusString() const override {
        return "MockDriver: L=" + std::to_string(left_vel_) + 
               " R=" + std::to_string(right_vel_) + " mRPM (simulation)";
    }

    double getCycleTimeUs() const override {
        return 5000.0; // 5ms nominal
    }

    // Mock-specific methods for testing
    void simulateCycle() {
        if (callback_ && operational_) {
            callback_(left_vel_, right_vel_);
        }
        cycle_count_++;
    }

    void reset() {
        emergency_stopped_ = false;
        left_vel_ = 0;
        right_vel_ = 0;
    }

private:
    bool operational_;
    bool emergency_stopped_;
    int left_vel_;
    int right_vel_;
    uint64_t cycle_count_;
    std::function<void(int, int)> callback_;
};
