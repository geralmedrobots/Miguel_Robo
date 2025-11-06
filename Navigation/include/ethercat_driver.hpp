#ifndef ETHERCAT_DRIVER_HPP
#define ETHERCAT_DRIVER_HPP

#include "drive_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <mutex>
#include <memory>
#include <thread>
#include <functional>

// Forward declarations for SOEM types
extern "C" {
    #include "ethercat.h"
    #include "ethercattype.h"
}

/**
 * @brief EtherCAT SOMANET drive implementation
 * 
 * Implements DriveInterface for SOMANET motor controllers using SOEM library.
 * Handles all low-level EtherCAT communication, state machine transitions,
 * and safety monitoring.
 * 
 * Thread Safety: All public methods are thread-safe
 * Real-time: Cyclic task runs at 200 Hz (5ms cycle time)
 */
class EthercatDriver : public DriveInterface {
public:
    EthercatDriver();
    explicit EthercatDriver(rclcpp::Logger logger);
    virtual ~EthercatDriver();

    // DriveInterface implementation
    bool initialize(const std::string& interface_name) override;
    bool setVelocity(int left_mrpm, int right_mrpm) override;
    std::pair<int, int> getVelocity() const override;
    bool isOperational() const override;
    void emergencyStop(const std::string& reason) override;
    void shutdown() override;
    void registerUpdateCallback(
        std::function<void(int left_mrpm, int right_mrpm)> callback) override;
    std::string getStatusString() const override;
    double getCycleTimeUs() const override;

private:
    // SOMANET PDO structures (from ESI file)
    struct PACKED InputPDO {
        int16_t statusword;
        int8_t op_mode_display;
        int32_t position_value;
        int32_t velocity_value;
        int16_t torque_value;
        int32_t sec_position_value;
        int32_t sec_velocity_value;
        int16_t analog_input_1;
        int16_t analog_input_2;
        int16_t analog_input_3;
        int16_t analog_input_4;
        int32_t tuning_status;
        int8_t digital_input_1;
        int8_t digital_input_2;
        int8_t digital_input_3;
        int8_t digital_input_4;
        int32_t user_miso;
        int32_t timestamp;
        int32_t position_demand_internal;
        int32_t velocity_demand;
        int16_t torque_demand;
    };

    struct PACKED OutputPDO {
        int16_t controlword;
        int8_t op_mode;
        int16_t target_torque;
        int32_t target_position;
        int32_t target_velocity;
        int16_t torque_offset;
        int32_t tuning_command;
        int8_t digital_output_1;
        int8_t digital_output_2;
        int8_t digital_output_3;
        int8_t digital_output_4;
        int32_t user_mosi;
        int32_t velocity_offset;
    };

    // Internal methods
    void cyclicTask();
    void stateCheckTask();
    bool configureSlaves();
    void handleStateMachine(int slave_id);
    bool isInFaultState(uint16_t statusword) const;
    bool needsShutdown(uint16_t statusword) const;
    bool needsSwitchOn(uint16_t statusword) const;
    bool needsEnableOperation(uint16_t statusword) const;
    bool isOperationEnabled(uint16_t statusword) const;

    // Thread management
    std::atomic<bool> running_{false};
    std::atomic<bool> operational_{false};
    std::thread cyclic_thread_;
    std::thread state_check_thread_;

    // Command synchronization
    mutable std::mutex cmd_mutex_;
    std::atomic<int> target_left_mrpm_{0};
    std::atomic<int> target_right_mrpm_{0};
    std::atomic<int> actual_left_mrpm_{0};
    std::atomic<int> actual_right_mrpm_{0};

    // EtherCAT state
    char io_map_[4096];
    int expected_wkc_{0};
    std::atomic<int> current_wkc_{0};
    InputPDO* slave1_input_{nullptr};
    InputPDO* slave2_input_{nullptr};
    OutputPDO* slave1_output_{nullptr};
    OutputPDO* slave2_output_{nullptr};

    // Statistics
    std::atomic<uint64_t> cycle_count_{0};
    std::atomic<double> avg_cycle_time_us_{5000.0};

    // Callback (FIX PROBLEM 6: NO MUTEX IN RT LOOP!)
    // SAFETY: Must be lock-free (atomic pointer swap)
    std::atomic<std::function<void(int, int)>*> update_callback_{nullptr};

    // ROS2 Logging (FIX PROBLEM 6: Replace std::cout with RCLCPP_*)
    rclcpp::Logger logger_;
    std::shared_ptr<rclcpp::Clock> clock_;

    // Safety limits
    static constexpr int MAX_VELOCITY_MRPM = 20000;
    static constexpr int SLAVE_1_ID = 1;
    static constexpr int SLAVE_2_ID = 2;
};

#endif // ETHERCAT_DRIVER_HPP
