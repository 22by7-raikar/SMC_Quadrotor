/**
 * @file trajectory_generator.h
 * @brief Trajectory Generator for Crazyflie 2.0 Quadrotor
 * 
 * Generates quintic (5th order) polynomial trajectories for waypoint navigation
 */

#ifndef SMC_QUADROTOR_TRAJECTORY_GENERATOR_H
#define SMC_QUADROTOR_TRAJECTORY_GENERATOR_H

#include <vector>
#include <Eigen/Dense>

namespace smc_quadrotor {

/**
 * @brief Generate quintic polynomial trajectories for smooth waypoint navigation
 */
class TrajectoryGenerator {
public:
    /**
     * @brief Constructor - initializes with default trajectory or empty
     * 
     * Default waypoint sequence:
     * - p0 = (0, 0, 0) to p1 = (0, 0, 1) in 5 seconds
     * - p1 = (0, 0, 1) to p2 = (1, 0, 1) in 15 seconds
     * - p2 = (1, 0, 1) to p3 = (1, 1, 1) in 15 seconds
     * - p3 = (1, 1, 1) to p4 = (0, 1, 1) in 15 seconds
     * - p4 = (0, 1, 1) to p5 = (0, 0, 1) in 15 seconds
     * 
     * @param use_default_trajectory If true, loads default trajectory. If false, trajectory must be set later.
     */
    TrajectoryGenerator(bool use_default_trajectory = true);

    /**
     * @brief Set custom trajectory waypoints and time segments
     * 
     * @param waypoints Vector of 3D waypoint positions [x, y, z] (meters)
     * @param time_segments Vector of time durations for each segment (seconds)
     *                      Must have length = waypoints.size() - 1
     * @return true if trajectory was set successfully, false if invalid parameters
     * 
     * Example:
     *   waypoints = {(0,0,0), (1,0,1), (1,1,1)}
     *   time_segments = {5.0, 10.0}
     *   This creates 2 segments: (0,0,0)->(1,0,1) in 5s, (1,0,1)->(1,1,1) in 10s
     */
    bool setTrajectory(const std::vector<Eigen::Vector3d>& waypoints,
                      const std::vector<double>& time_segments);

    /**
     * @brief Load trajectory from CSV or YAML file
     * 
     * CSV Format (with header):
     *   x, y, z, time
     *   0.0, 0.0, 0.0, 5.0
     *   1.0, 0.0, 1.0, 10.0
     *   ...
     * 
     * YAML Format:
     *   waypoints:
     *     - {x: 0.0, y: 0.0, z: 0.0, time: 5.0}
     *     - {x: 1.0, y: 0.0, z: 1.0, time: 10.0}
     *     ...
     * 
     * @param filepath Path to trajectory file (.csv or .yaml)
     * @return true if loaded successfully, false on error
     */
    bool loadFromFile(const std::string& filepath);

    /**
     * @brief Get desired position, velocity, and acceleration at time t
     * 
     * @param t Current time (seconds)
     * @param position Output: [x, y, z] (m)
     * @param velocity Output: [vx, vy, vz] (m/s)
     * @param acceleration Output: [ax, ay, az] (m/s^2)
     */
    void getDesiredState(double t, 
                        Eigen::Vector3d& position,
                        Eigen::Vector3d& velocity,
                        Eigen::Vector3d& acceleration) const;

    /**
     * @brief Get total mission time
     * @return Total time in seconds
     */
    double getTotalTime() const { return total_time_; }

    /**
     * @brief Get waypoints
     * @return Vector of waypoint positions
     */
    const std::vector<Eigen::Vector3d>& getWaypoints() const { return waypoints_; }

private:
    /**
     * @brief Compute quintic polynomial coefficients for each trajectory segment
     * 
     * Quintic polynomial: s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
     * 
     * Boundary conditions:
     * - Position at start and end
     * - Velocity = 0 at start and end
     * - Acceleration = 0 at start and end
     */
    void computeQuinticCoefficients();

    // Waypoints [x, y, z]
    std::vector<Eigen::Vector3d> waypoints_;
    
    // Time duration for each segment (seconds)
    std::vector<double> time_segments_;
    
    // Cumulative time at each waypoint
    std::vector<double> cumulative_time_;
    
    // Total mission time
    double total_time_;
    
    // Quintic polynomial coefficients for each segment
    // Each segment has a 6x3 matrix (6 coefficients for x, y, z)
    std::vector<Eigen::Matrix<double, 6, 3>> coefficients_;
};

} // namespace smc_quadrotor

#endif // SMC_QUADROTOR_TRAJECTORY_GENERATOR_H
