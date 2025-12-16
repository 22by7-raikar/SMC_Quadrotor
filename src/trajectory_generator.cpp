/**
 * @file trajectory_generator.cpp
 * @brief Implementation of Trajectory Generator for Crazyflie 2.0 Quadrotor
 */

#include "smc_quadrotor/trajectory_generator.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>

namespace smc_quadrotor {

TrajectoryGenerator::TrajectoryGenerator(bool use_default_trajectory) {
    if (use_default_trajectory) {
        // Initialize waypoints [x, y, z]
        waypoints_ = {
            Eigen::Vector3d(0.0, 0.0, 0.0),  // p0
            Eigen::Vector3d(0.0, 0.0, 1.0),  // p1
            Eigen::Vector3d(1.0, 0.0, 1.0),  // p2
            Eigen::Vector3d(1.0, 1.0, 1.0),  // p3
            Eigen::Vector3d(0.0, 1.0, 1.0),  // p4
            Eigen::Vector3d(0.0, 0.0, 1.0)   // p5
        };

        // Time duration for each segment (seconds)
        time_segments_ = {5.0, 15.0, 15.0, 15.0, 15.0};

        // Compute cumulative time at each waypoint
        cumulative_time_.push_back(0.0);
        for (double dt : time_segments_) {
            cumulative_time_.push_back(cumulative_time_.back() + dt);
        }

        // Total mission time
        total_time_ = cumulative_time_.back();

        // Precompute quintic polynomial coefficients
        computeQuinticCoefficients();
    } else {
        total_time_ = 0.0;
    }
}

bool TrajectoryGenerator::setTrajectory(const std::vector<Eigen::Vector3d>& waypoints,
                                       const std::vector<double>& time_segments) {
    // Validate input
    if (waypoints.size() < 2) {
        return false;  // Need at least 2 waypoints
    }
    if (time_segments.size() != waypoints.size() - 1) {
        return false;  // Time segments must be waypoints - 1
    }
    for (double dt : time_segments) {
        if (dt <= 0.0) {
            return false;  // All time segments must be positive
        }
    }

    // Set waypoints and time segments
    waypoints_ = waypoints;
    time_segments_ = time_segments;

    // Compute cumulative time at each waypoint
    cumulative_time_.clear();
    cumulative_time_.push_back(0.0);
    for (double dt : time_segments_) {
        cumulative_time_.push_back(cumulative_time_.back() + dt);
    }

    // Total mission time
    total_time_ = cumulative_time_.back();

    // Precompute quintic polynomial coefficients
    computeQuinticCoefficients();

    return true;
}

void TrajectoryGenerator::computeQuinticCoefficients() {
    coefficients_.clear();

    for (size_t i = 0; i < time_segments_.size(); ++i) {
        const Eigen::Vector3d& p0 = waypoints_[i];      // Start position
        const Eigen::Vector3d& p1 = waypoints_[i + 1];  // End position
        double T = time_segments_[i];                    // Duration

        // Boundary conditions: [p0, v0, a0, p1, v1, a1]
        // v0 = v1 = 0, a0 = a1 = 0

        // Quintic polynomial coefficient matrix
        // [1, 0, 0, 0, 0, 0] [a0]   [p0]
        // [0, 1, 0, 0, 0, 0] [a1]   [0 ]
        // [0, 0, 2, 0, 0, 0] [a2] = [0 ]
        // [1, T, T^2, T^3, T^4, T^5] [a3]   [p1]
        // [0, 1, 2T, 3T^2, 4T^3, 5T^4] [a4]   [0 ]
        // [0, 0, 2, 6T, 12T^2, 20T^3] [a5]   [0 ]

        Eigen::Matrix<double, 6, 6> A;
        A << 1, 0,   0,        0,          0,           0,
             0, 1,   0,        0,          0,           0,
             0, 0,   2,        0,          0,           0,
             1, T,   T*T,      T*T*T,      T*T*T*T,     T*T*T*T*T,
             0, 1,   2*T,      3*T*T,      4*T*T*T,     5*T*T*T*T,
             0, 0,   2,        6*T,        12*T*T,      20*T*T*T;

        // Solve for coefficients for each axis
        Eigen::Matrix<double, 6, 3> coeff_segment;
        for (int axis = 0; axis < 3; ++axis) {  // x, y, z
            Eigen::VectorXd b(6);
            b << p0(axis), 0, 0, p1(axis), 0, 0;
            coeff_segment.col(axis) = A.fullPivLu().solve(b);
        }

        coefficients_.push_back(coeff_segment);
    }
}

void TrajectoryGenerator::getDesiredState(double t,
                                         Eigen::Vector3d& position,
                                         Eigen::Vector3d& velocity,
                                         Eigen::Vector3d& acceleration) const {
    // Clamp time to valid range
    t = std::max(0.0, std::min(t, total_time_));

    // If at or past the end, return final waypoint with zero velocity/acceleration
    if (t >= total_time_) {
        position = waypoints_.back();
        velocity.setZero();
        acceleration.setZero();
        return;
    }

    // Find which segment we're in
    size_t segment_idx = 0;
    for (size_t i = 0; i < cumulative_time_.size() - 1; ++i) {
        if (t >= cumulative_time_[i] && t < cumulative_time_[i + 1]) {
            segment_idx = i;
            break;
        }
    }
    segment_idx = std::min(segment_idx, time_segments_.size() - 1);

    // Local time within the segment
    double t_local = t - cumulative_time_[segment_idx];

    // Get coefficients for this segment
    const auto& coeff = coefficients_[segment_idx];

    // Compute position, velocity, and acceleration for each axis
    position.setZero();
    velocity.setZero();
    acceleration.setZero();

    for (int axis = 0; axis < 3; ++axis) {
        // Coefficients for this axis
        double a0 = coeff(0, axis);
        double a1 = coeff(1, axis);
        double a2 = coeff(2, axis);
        double a3 = coeff(3, axis);
        double a4 = coeff(4, axis);
        double a5 = coeff(5, axis);

        // Position: s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        position(axis) = a0 + a1*t_local + a2*t_local*t_local + 
                        a3*t_local*t_local*t_local + a4*t_local*t_local*t_local*t_local + 
                        a5*t_local*t_local*t_local*t_local*t_local;

        // Velocity: ds/dt = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
        velocity(axis) = a1 + 2*a2*t_local + 3*a3*t_local*t_local + 
                        4*a4*t_local*t_local*t_local + 5*a5*t_local*t_local*t_local*t_local;

        // Acceleration: d^2s/dt^2 = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3
        acceleration(axis) = 2*a2 + 6*a3*t_local + 12*a4*t_local*t_local + 
                            20*a5*t_local*t_local*t_local;
    }
}

bool TrajectoryGenerator::loadFromFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "ERROR: Could not open trajectory file: " << filepath << std::endl;
        return false;
    }

    std::vector<Eigen::Vector3d> waypoints;
    std::vector<double> time_segments;
    
    // Detect file format
    std::string ext = filepath.substr(filepath.find_last_of('.'));
    
    if (ext == ".csv") {
        // Parse CSV format
        std::string line;
        bool first_line = true;
        
        while (std::getline(file, line)) {
            // Skip empty lines
            if (line.empty() || line[0] == '#') continue;
            
            // Skip header line
            if (first_line) {
                first_line = false;
                // Check if it's a header (contains letters)
                if (line.find_first_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ") != std::string::npos) {
                    continue;
                }
            }
            
            // Parse: x, y, z, time
            double x, y, z, time;
            char comma;
            std::istringstream iss(line);
            
            if (iss >> x >> comma >> y >> comma >> z >> comma >> time) {
                waypoints.push_back(Eigen::Vector3d(x, y, z));
                time_segments.push_back(time);
            } else {
                std::cerr << "ERROR: Invalid CSV line: " << line << std::endl;
                return false;
            }
        }
        
    } else if (ext == ".yaml" || ext == ".yml") {
        // Parse YAML format (simple parser)
        std::string line;
        bool in_waypoints = false;
        
        while (std::getline(file, line)) {
            // Skip empty lines and comments
            if (line.empty() || line[0] == '#') continue;
            
            // Trim whitespace
            size_t start = line.find_first_not_of(" \t");
            if (start == std::string::npos) continue;
            line = line.substr(start);
            
            if (line.find("waypoints:") == 0) {
                in_waypoints = true;
                continue;
            }
            
            if (in_waypoints && line[0] == '-') {
                // Parse: - {x: 0.0, y: 0.0, z: 0.0, time: 5.0}
                double x, y, z, time;
                
                size_t x_pos = line.find("x:");
                size_t y_pos = line.find("y:");
                size_t z_pos = line.find("z:");
                size_t t_pos = line.find("time:");
                
                if (x_pos != std::string::npos && y_pos != std::string::npos && 
                    z_pos != std::string::npos && t_pos != std::string::npos) {
                    
                    sscanf(line.substr(x_pos).c_str(), "x: %lf", &x);
                    sscanf(line.substr(y_pos).c_str(), "y: %lf", &y);
                    sscanf(line.substr(z_pos).c_str(), "z: %lf", &z);
                    sscanf(line.substr(t_pos).c_str(), "time: %lf", &time);
                    
                    waypoints.push_back(Eigen::Vector3d(x, y, z));
                    time_segments.push_back(time);
                } else {
                    std::cerr << "ERROR: Invalid YAML line: " << line << std::endl;
                    return false;
                }
            }
        }
    } else {
        std::cerr << "ERROR: Unsupported file format: " << ext << " (use .csv or .yaml)" << std::endl;
        return false;
    }
    
    file.close();
    
    // Validate
    if (waypoints.empty()) {
        std::cerr << "ERROR: No waypoints found in file" << std::endl;
        return false;
    }
    
    std::cout << "Loaded " << waypoints.size() << " waypoints from " << filepath << std::endl;
    
    // Call setTrajectory with loaded data
    return setTrajectory(waypoints, time_segments);
}

} // namespace smc_quadrotor
