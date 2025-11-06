#ifndef DRIVE_INTERFACE_HPP
#define DRIVE_INTERFACE_HPP

#include <string>
#include <utility>
#include <functional>

/**
 * @brief Abstract interface for motor drive control
 * 
 * This interface decouples the high-level robot control from the specific
 * hardware implementation (EtherCAT, CAN, simulation, etc.)
 * 
 * Standards Compliance:
 * - IEC 61508: Software modularity for testability
 * - ISO 13849-1: Separation of safety-critical layers
 */
class DriveInterface {
public:
    virtual ~DriveInterface() = default;

    /**
     * @brief Initialize the drive hardware/simulation
     * @param interface_name Hardware interface identifier (e.g., "enp89s0" for EtherCAT)
     * @return true if initialization successful, false otherwise
     */
    virtual bool initialize(const std::string& interface_name) = 0;

    /**
     * @brief Set target velocities for left and right wheels
     * @param left_mrpm Left wheel velocity in milli-RPM
     * @param right_mrpm Right wheel velocity in milli-RPM
     * @return true if command accepted, false if rejected (e.g., safety limits)
     */
    virtual bool setVelocity(int left_mrpm, int right_mrpm) = 0;

    /**
     * @brief Get actual velocities from motor encoders
     * @return std::pair<int, int> Left and right wheel velocities in milli-RPM
     */
    virtual std::pair<int, int> getVelocity() const = 0;

    /**
     * @brief Check if drive system is operational
     * @return true if ready to accept commands
     */
    virtual bool isOperational() const = 0;

    /**
     * @brief Emergency stop - immediately halt all motors
     * @param reason Human-readable reason for emergency stop
     */
    virtual void emergencyStop(const std::string& reason) = 0;

    /**
     * @brief Shutdown drive system gracefully
     */
    virtual void shutdown() = 0;

    /**
     * @brief Register callback for cyclic updates (typically 200 Hz)
     * @param callback Function called every cycle with actual velocities
     */
    virtual void registerUpdateCallback(
        std::function<void(int left_mrpm, int right_mrpm)> callback) = 0;

    /**
     * @brief Get drive system status information
     * @return String with diagnostic information
     */
    virtual std::string getStatusString() const = 0;

    /**
     * @brief Get cycle time statistics
     * @return Cycle time in microseconds (average)
     */
    virtual double getCycleTimeUs() const = 0;
};

#endif // DRIVE_INTERFACE_HPP
