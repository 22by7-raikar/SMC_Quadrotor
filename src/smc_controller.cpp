/**
 * @file smc_controller.cpp
 * @brief Implementation of Sliding Mode Controller for Crazyflie 2.0 Quadrotor
 */

#include "smc_quadrotor/smc_controller.h"
#include <cmath>
#include <algorithm>

namespace smc_quadrotor {

SlidingModeController::SlidingModeController(const std::map<std::string, double>& params) 
    : omega_rotors_(Eigen::Vector4d::Zero()) {
    
    // Physical parameters (Crazyflie 2.0)
    m_ = params.count("mass") ? params.at("mass") : 0.027;
    g_ = params.count("gravity") ? params.at("gravity") : 9.81;
    Ix_ = params.count("Ix") ? params.at("Ix") : 16.571710e-6;
    Iy_ = params.count("Iy") ? params.at("Iy") : 16.571710e-6;
    Iz_ = params.count("Iz") ? params.at("Iz") : 29.261652e-6;
    Ip_ = params.count("Ip") ? params.at("Ip") : 12.65625e-8;

    // Propeller parameters
    kF_ = params.count("kF") ? params.at("kF") : 1.28192e-8;
    kM_ = params.count("kM") ? params.at("kM") : 5.964552e-3;
    l_ = params.count("arm_length") ? params.at("arm_length") : 0.046;

    // Rotor speed limits
    omega_min_ = params.count("omega_min") ? params.at("omega_min") : 0.0;
    omega_max_ = params.count("omega_max") ? params.at("omega_max") : 2618.0;

    // PD gains for x, y position control
    kp_xy_ = params.count("kp_xy") ? params.at("kp_xy") : 1.0;
    kd_xy_ = params.count("kd_xy") ? params.at("kd_xy") : 0.5;

    // SMC parameters for z (altitude)
    lambda_z_ = params.count("lambda_z") ? params.at("lambda_z") : 1.0;
    eta_z_ = params.count("eta_z") ? params.at("eta_z") : 0.5;

    // SMC parameters for phi (roll)
    lambda_phi_ = params.count("lambda_phi") ? params.at("lambda_phi") : 1.0;
    eta_phi_ = params.count("eta_phi") ? params.at("eta_phi") : 0.5;

    // SMC parameters for theta (pitch)
    lambda_theta_ = params.count("lambda_theta") ? params.at("lambda_theta") : 1.0;
    eta_theta_ = params.count("eta_theta") ? params.at("eta_theta") : 0.5;

    // SMC parameters for psi (yaw)
    lambda_psi_ = params.count("lambda_psi") ? params.at("lambda_psi") : 1.0;
    eta_psi_ = params.count("eta_psi") ? params.at("eta_psi") : 0.5;
}

double SlidingModeController::wrapAngle(double angle) const {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double SlidingModeController::satFunction(double s, double phi) const {
    if (std::abs(s) > phi) {
        return (s > 0) ? 1.0 : -1.0;
    } else {
        return s / phi;
    }
}

void SlidingModeController::computeDesiredAngles(
    double x, double y,
    double x_dot, double y_dot,
    double x_d, double y_d,
    double x_d_dot, double y_d_dot,
    double x_d_ddot, double y_d_ddot,
    double u1,
    double& phi_d, double& theta_d) const {
    
    // Compute desired forces using PD control (Eq. 1 and 2)
    double Fx = m_ * (-kp_xy_ * (x - x_d) - kd_xy_ * (x_dot - x_d_dot) + x_d_ddot);
    double Fy = m_ * (-kp_xy_ * (y - y_d) - kd_xy_ * (y_dot - y_d_dot) + y_d_ddot);

    // Convert to desired angles (Eq. 3 and 4)
    // Avoid division by zero
    if (std::abs(u1) < 1e-6) {
        u1 = 1e-6;
    }

    // theta_d = arcsin(Fx / u1)
    theta_d = std::asin(std::max(-1.0, std::min(1.0, Fx / u1)));

    // phi_d = arcsin(-Fy / u1)
    phi_d = std::asin(std::max(-1.0, std::min(1.0, -Fy / u1)));
}

double SlidingModeController::altitudeControl(
    double z, double z_dot,
    double z_d, double z_d_dot, double z_d_ddot,
    double phi, double theta) const {
    
    // Position and velocity errors
    double e_z = z - z_d;
    double e_dot_z = z_dot - z_d_dot;

    // Sliding surface
    double s_z = e_dot_z + lambda_z_ * e_z;

    // Control law with boundary layer and thrust direction compensation
    // Reference uses phi=1.0 for boundary layer
    double u1_base = m_ * (g_ + z_d_ddot - lambda_z_ * e_dot_z 
                          - eta_z_ * satFunction(s_z, 1.0));

    // Compensate for quadrotor tilt (thrust acts along body z-axis)
    double cos_phi_theta = std::cos(phi) * std::cos(theta);
    if (std::abs(cos_phi_theta) < 0.1) {  // Avoid division by zero for extreme angles
        cos_phi_theta = 0.1;
    }

    double u1 = u1_base / cos_phi_theta;

    // Ensure positive thrust
    u1 = std::max(u1, 0.0);

    return u1;
}

double SlidingModeController::rollControl(
    double phi, double phi_dot,
    double theta, double theta_dot, double psi_dot,
    double phi_d) const {
    
    // Angle error
    double phi_err = wrapAngle(phi - phi_d);

    // Sliding surface (use phi_dot directly as in reference)
    double s_phi = phi_dot + lambda_phi_ * phi_err;

    // Compute Omega = omega1 - omega2 + omega3 - omega4
    double Omega = omega_rotors_(0) - omega_rotors_(1) + omega_rotors_(2) - omega_rotors_(3);

    // Control law (reference structure)
    // Reference uses phi=1.0 for boundary layer
    double sign_s_phi = satFunction(s_phi, 1.0);
    double u2 = (Ip_ * Omega * theta_dot) - 
                (Ix_ * lambda_phi_ * phi_dot) - 
                (theta_dot * psi_dot * (Iy_ - Iz_)) - 
                (eta_phi_ * sign_s_phi * Ix_);

    return u2;
}

double SlidingModeController::pitchControl(
    double theta, double theta_dot,
    double phi, double phi_dot, double psi_dot,
    double theta_d) const {
    
    // Angle error
    double theta_err = wrapAngle(theta - theta_d);

    // Sliding surface (use theta_dot directly as in reference)
    double s_theta = theta_dot + lambda_theta_ * theta_err;

    // Compute Omega
    double Omega = omega_rotors_(0) - omega_rotors_(1) + omega_rotors_(2) - omega_rotors_(3);

    // Control law (reference structure - note the negative sign and psi_dot coupling)
    // Reference uses phi=1.0 for boundary layer
    double sign_s_theta = satFunction(s_theta, 1.0);
    double u3 = -((lambda_theta_ * Iy_ * theta_dot) + 
                  (Ip_ * Omega * phi_dot) + 
                  (phi_dot * psi_dot * (Iz_ - Ix_)) + 
                  (eta_theta_ * Iy_ * sign_s_theta));

    return u3;
}

double SlidingModeController::yawControl(
    double psi, double psi_dot,
    double phi, double phi_dot, double theta_dot,
    double psi_d) const {
    
    // Angle error
    double psi_err = wrapAngle(psi - psi_d);

    // Sliding surface (use psi_dot directly as in reference)
    double s_psi = psi_dot + lambda_psi_ * psi_err;

    // Control law (reference structure - note negative sign)
    // Reference uses phi=1.0 for boundary layer
    double sign_s_psi = satFunction(s_psi, 1.0);
    double u4 = -((phi_dot * theta_dot * (Ix_ - Iy_)) + 
                  (Iz_ * lambda_psi_ * psi_dot) + 
                  (eta_psi_ * sign_s_psi * Iz_));

    return u4;
}

Eigen::Vector4d SlidingModeController::controlAllocation(const Eigen::Vector4d& u) {
    const double sqrt2 = std::sqrt(2.0);

    // Allocation matrix
    Eigen::Matrix4d A;
    A << 1.0/(4.0*kF_), -sqrt2/(4.0*kF_*l_), -sqrt2/(4.0*kF_*l_), -1.0/(4.0*kM_*kF_),
         1.0/(4.0*kF_), -sqrt2/(4.0*kF_*l_),  sqrt2/(4.0*kF_*l_),  1.0/(4.0*kM_*kF_),
         1.0/(4.0*kF_),  sqrt2/(4.0*kF_*l_),  sqrt2/(4.0*kF_*l_), -1.0/(4.0*kM_*kF_),
         1.0/(4.0*kF_),  sqrt2/(4.0*kF_*l_), -sqrt2/(4.0*kF_*l_),  1.0/(4.0*kM_*kF_);

    // Compute omega^2
    Eigen::Vector4d omega_squared = A * u;

    // Take square root and handle negative values
    Eigen::Vector4d omega;
    for (int i = 0; i < 4; ++i) {
        omega(i) = std::sqrt(std::max(0.0, omega_squared(i)));
    }

    // Saturate to valid range
    for (int i = 0; i < 4; ++i) {
        omega(i) = std::max(omega_min_, std::min(omega_max_, omega(i)));
    }

    // Store for next iteration (for Omega computation)
    omega_rotors_ = omega;

    return omega;
}

Eigen::Vector4d SlidingModeController::computeControl(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& attitude,
    const Eigen::Vector3d& angular_velocity,
    const Eigen::Vector3d& desired_position,
    const Eigen::Vector3d& desired_velocity,
    const Eigen::Vector3d& desired_acceleration,
    double desired_yaw) {
    
    // Extract state
    double x = position(0), y = position(1), z = position(2);
    double vx = velocity(0), vy = velocity(1), vz = velocity(2);
    double phi = attitude(0), theta = attitude(1), psi = attitude(2);
    double phi_dot = angular_velocity(0);
    double theta_dot = angular_velocity(1);
    double psi_dot = angular_velocity(2);

    // Extract desired values
    double x_d = desired_position(0), y_d = desired_position(1), z_d = desired_position(2);
    double vx_d = desired_velocity(0), vy_d = desired_velocity(1), vz_d = desired_velocity(2);
    double ax_d = desired_acceleration(0), ay_d = desired_acceleration(1), az_d = desired_acceleration(2);

    // Altitude control (compute u1) - pass current angles for thrust compensation
    double u1 = altitudeControl(z, vz, z_d, vz_d, az_d, phi, theta);

    // Compute desired roll and pitch angles
    double phi_d, theta_d;
    computeDesiredAngles(x, y, vx, vy, x_d, y_d, vx_d, vy_d, ax_d, ay_d, u1, phi_d, theta_d);

    // Desired yaw
    double psi_d = desired_yaw;

    // Attitude control (compute u2, u3, u4)
    double u2 = rollControl(phi, phi_dot, theta, theta_dot, psi_dot, phi_d);
    double u3 = pitchControl(theta, theta_dot, phi, phi_dot, psi_dot, theta_d);
    double u4 = yawControl(psi, psi_dot, phi, phi_dot, theta_dot, psi_d);

    // Control vector
    Eigen::Vector4d u;
    u << u1, u2, u3, u4;

    // Allocate to rotor speeds
    Eigen::Vector4d omega = controlAllocation(u);

    return omega;
}

} // namespace smc_quadrotor
