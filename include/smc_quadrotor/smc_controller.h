/**
 * @file smc_controller.h
 * @brief Sliding Mode Controller for Crazyflie 2.0 Quadrotor
 * 
 * Implements boundary layer-based sliding mode control for altitude and attitude control
 */

#ifndef SMC_QUADROTOR_SMC_CONTROLLER_H
#define SMC_QUADROTOR_SMC_CONTROLLER_H

#include <Eigen/Dense>
#include <map>
#include <string>

namespace smc_quadrotor {

/**
 * @brief Sliding Mode Controller for quadrotor altitude and attitude control
 * Controls: z (altitude), phi (roll), theta (pitch), psi (yaw)
 */
class SlidingModeController {
public:
    /**
     * @brief Constructor with parameters
     * @param params Map of controller parameters
     */
    explicit SlidingModeController(const std::map<std::string, double>& params);

    /**
     * @brief Main control computation function
     * 
     * @param position Current position [x, y, z]
     * @param velocity Current velocity [vx, vy, vz]
     * @param attitude Current attitude [phi, theta, psi]
     * @param angular_velocity Current angular velocity [phi_dot, theta_dot, psi_dot]
     * @param desired_position Desired position [x_d, y_d, z_d]
     * @param desired_velocity Desired velocity [vx_d, vy_d, vz_d]
     * @param desired_acceleration Desired acceleration [ax_d, ay_d, az_d]
     * @param desired_yaw Desired yaw angle (default 0)
     * @return Rotor speeds [omega1, omega2, omega3, omega4] in rad/s
     */
    Eigen::Vector4d computeControl(
        const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& attitude,
        const Eigen::Vector3d& angular_velocity,
        const Eigen::Vector3d& desired_position,
        const Eigen::Vector3d& desired_velocity,
        const Eigen::Vector3d& desired_acceleration,
        double desired_yaw = 0.0);

private:
    /**
     * @brief Wrap angle to [-pi, pi]
     * @param angle Angle in radians
     * @return Wrapped angle
     */
    double wrapAngle(double angle) const;

    /**
     * @brief Saturation function for boundary layer SMC
     * @param s Sliding surface value
     * @param phi Boundary layer thickness
     * @return Saturated value
     */
    double satFunction(double s, double phi) const;

    /**
     * @brief Compute desired roll and pitch angles from desired x, y trajectories
     * 
     * Using PD control as specified in the project document
     * 
     * @param x, y Current x, y positions
     * @param x_dot, y_dot Current x, y velocities
     * @param x_d, y_d Desired x, y positions
     * @param x_d_dot, y_d_dot Desired x, y velocities
     * @param x_d_ddot, y_d_ddot Desired x, y accelerations
     * @param u1 Total thrust force
     * @param phi_d Output: desired roll angle
     * @param theta_d Output: desired pitch angle
     */
    void computeDesiredAngles(
        double x, double y,
        double x_dot, double y_dot,
        double x_d, double y_d,
        double x_d_dot, double y_d_dot,
        double x_d_ddot, double y_d_ddot,
        double u1,
        double& phi_d, double& theta_d) const;

    /**
     * @brief SMC for altitude (z) control
     * 
     * Sliding surface: s_z = (z_dot - z_d_dot) + lambda_z * (z - z_d)
     * Control law: u1 = m * (g + z_d_ddot - lambda_z * e_dot_z - eta_z * sat(s_z / phi_z)) / (cos(phi)*cos(theta))
     * 
     * @param z, z_dot Current altitude and velocity
     * @param z_d, z_d_dot, z_d_ddot Desired altitude, velocity, acceleration
     * @param phi, theta Current roll and pitch angles (for thrust compensation)
     * @return u1: Total thrust force
     */
    double altitudeControl(
        double z, double z_dot,
        double z_d, double z_d_dot, double z_d_ddot,
        double phi, double theta) const;

    /**
     * @brief SMC for roll (phi) control
     * 
     * @param phi, phi_dot Current roll angle and angular velocity
     * @param theta, theta_dot Current pitch angle and angular velocity
     * @param psi_dot Current yaw angular velocity
     * @param phi_d Desired roll angle
     * @return u2: Roll moment
     */
    double rollControl(
        double phi, double phi_dot,
        double theta, double theta_dot, double psi_dot,
        double phi_d) const;

    /**
     * @brief SMC for pitch (theta) control
     * 
     * @param theta, theta_dot Current pitch angle and angular velocity
     * @param phi, phi_dot Current roll angle and angular velocity
     * @param psi_dot Current yaw angular velocity
     * @param theta_d Desired pitch angle
     * @return u3: Pitch moment
     */
    double pitchControl(
        double theta, double theta_dot,
        double phi, double phi_dot, double psi_dot,
        double theta_d) const;

    /**
     * @brief SMC for yaw (psi) control
     * 
     * @param psi, psi_dot Current yaw angle and angular velocity
     * @param phi, phi_dot Current roll angle and angular velocity
     * @param theta_dot Current pitch angular velocity
     * @param psi_d Desired yaw angle
     * @return u4: Yaw moment
     */
    double yawControl(
        double psi, double psi_dot,
        double phi, double phi_dot, double theta_dot,
        double psi_d) const;

    /**
     * @brief Convert control inputs [u1, u2, u3, u4] to rotor speeds
     * 
     * @param u Control input vector [u1, u2, u3, u4]
     * @return Rotor speeds [omega1, omega2, omega3, omega4] in rad/s
     */
    Eigen::Vector4d controlAllocation(const Eigen::Vector4d& u);

    // Physical parameters (Crazyflie 2.0)
    double m_;         // Mass (kg)
    double g_;         // Gravity (m/s^2)
    double Ix_;        // Moment of inertia x-axis (kg*m^2)
    double Iy_;        // Moment of inertia y-axis (kg*m^2)
    double Iz_;        // Moment of inertia z-axis (kg*m^2)
    double Ip_;        // Propeller moment of inertia (kg*m^2)

    // Propeller parameters
    double kF_;        // Thrust coefficient (N*s^2)
    double kM_;        // Moment coefficient (m)
    double l_;         // Arm length (m)

    // Rotor speed limits
    double omega_min_; // Minimum rotor speed (rad/s)
    double omega_max_; // Maximum rotor speed (rad/s)

    // PD gains for x, y position control
    double kp_xy_;     // Proportional gain
    double kd_xy_;     // Derivative gain

    // SMC parameters for z (altitude)
    double lambda_z_;  // Sliding surface slope
    double eta_z_;     // Switching gain

    // SMC parameters for phi (roll)
    double lambda_phi_;
    double eta_phi_;

    // SMC parameters for theta (pitch)
    double lambda_theta_;
    double eta_theta_;

    // SMC parameters for psi (yaw)
    double lambda_psi_;
    double eta_psi_;

    // Store last rotor angular velocity (for Omega term)
    mutable Eigen::Vector4d omega_rotors_;
};

} // namespace smc_quadrotor

#endif // SMC_QUADROTOR_SMC_CONTROLLER_H
