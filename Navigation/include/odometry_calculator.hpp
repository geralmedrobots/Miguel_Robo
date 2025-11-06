#ifndef ODOMETRY_CALCULATOR_HPP
#define ODOMETRY_CALCULATOR_HPP

#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Differential drive odometry calculator
 * 
 * Calculates robot pose (x, y, yaw) from wheel velocities using
 * differential drive kinematics with arc integration.
 * 
 * Thread Safety: Not thread-safe (caller must synchronize)
 * Coordinate System: ROS standard (x forward, y left, z up)
 */
class OdometryCalculator {
public:
    /**
     * @brief Constructor
     * @param wheel_base Distance between wheels [meters]
     * @param wheel_diameter Diameter of drive wheels [meters]
     * @param left_wheel_polarity Motor polarity: +1 normal, -1 inverted (default: +1)
     * @param right_wheel_polarity Motor polarity: +1 normal, -1 inverted (default: +1)
     */
    OdometryCalculator(double wheel_base, double wheel_diameter,
                      int left_wheel_polarity = 1, int right_wheel_polarity = 1);
    
    /**
     * @brief Constructor with ROS2 logger
     * @param logger ROS2 logger for output
     * @param wheel_base Distance between wheels [meters]
     * @param wheel_diameter Diameter of drive wheels [meters]
     * @param left_wheel_polarity Motor polarity: +1 normal, -1 inverted (default: +1)
     * @param right_wheel_polarity Motor polarity: +1 normal, -1 inverted (default: +1)
     */
    OdometryCalculator(rclcpp::Logger logger, double wheel_base, double wheel_diameter,
                      int left_wheel_polarity = 1, int right_wheel_polarity = 1);

    /**
     * @brief Update odometry from wheel velocities
     * @param left_mrpm Left wheel velocity in milli-RPM
     * @param right_mrpm Right wheel velocity in milli-RPM
     * @param delta_t Time since last update [seconds]
     * @return true if update successful, false if invalid parameters
     */
    bool update(int left_mrpm, int right_mrpm, double delta_t);

    /**
     * @brief Get current pose
     * @param x Output: X position [meters]
     * @param y Output: Y position [meters]
     * @param yaw Output: Yaw angle [radians, -pi to pi]
     */
    void getPose(double& x, double& y, double& yaw) const;

    /**
     * @brief Get current twist (velocities)
     * @param linear_x Output: Linear velocity [m/s]
     * @param angular_z Output: Angular velocity [rad/s]
     */
    void getTwist(double& linear_x, double& angular_z) const;

    /**
     * @brief Reset odometry to origin
     */
    void reset();

    /**
     * @brief Set pose (for initialization or correction)
     * @param x X position [meters]
     * @param y Y position [meters]
     * @param yaw Yaw angle [radians]
     */
    void setPose(double x, double y, double yaw);

    /**
     * @brief Get covariance estimates
     * @param var_position Output: Position variance [m^2]
     * @param var_yaw Output: Yaw variance [rad^2]
     */
    void getCovariance(double& var_position, double& var_yaw) const;

    /**
     * @brief Get distance traveled since last reset
     * @return Total distance [meters]
     */
    double getDistanceTraveled() const;

private:
    // Convert mRPM to linear velocity [m/s]
    double mrpmToMetersPerSecond(int mrpm) const;

    // Normalize angle to [-pi, pi]
    double normalizeAngle(double angle) const;

    // Integrate odometry for a single time step (internal)
    void integrateStep(double delta_t);

    // Robot geometry
    double wheel_base_;      // Distance between wheels [m]
    double wheel_diameter_;  // Wheel diameter [m]
    double wheel_radius_;    // Wheel radius [m] (computed)
    int left_wheel_polarity_;   // +1 or -1 (motor polarity)
    int right_wheel_polarity_;  // +1 or -1 (motor polarity)

    // Current state
    double x_;              // Position X [m]
    double y_;              // Position Y [m]
    double yaw_;            // Orientation [rad]
    double linear_vel_;     // Linear velocity [m/s]
    double angular_vel_;    // Angular velocity [rad/s]

    // Statistics
    double total_distance_; // Total distance traveled [m]

    // FIX PROBLEM 11: Acceleration validation state
    int prev_left_mrpm_;    // Previous left wheel velocity [mRPM]
    int prev_right_mrpm_;   // Previous right wheel velocity [mRPM]
    double prev_sample_time_;  // Previous delta_t [s]

    // ROS2 Logging (FIX PROBLEM 6: Replace std::cout with RCLCPP_*)
    rclcpp::Logger logger_;
    std::shared_ptr<rclcpp::Clock> clock_;

    // Covariance constants (tuned based on experiments)
    static constexpr double VAR_POSITION = 0.001;   // ≈ (3cm)^2
    static constexpr double VAR_YAW = 0.03;         // ≈ (10°)^2
    static constexpr double PI = 3.14159265358979323846;
};

#endif // ODOMETRY_CALCULATOR_HPP
