/**
 * @file quadrotor_control.cpp
 * @brief Main ROS Node for Crazyflie 2.0 Quadrotor Control
 * 
 * Integrates trajectory generation and sliding mode control
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/Actuators.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <vector>
#include <fstream>

#include "smc_quadrotor/trajectory_generator.h"
#include "smc_quadrotor/smc_controller.h"

class QuadrotorControl {
public:
    QuadrotorControl(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
        : nh_(nh), nh_private_(nh_private), t0_(0.0), t_(0.0), initialized_(false) {
        
        // Initialize trajectory generator
        traj_gen_ = std::make_shared<smc_quadrotor::TrajectoryGenerator>(false);
        
        // Try loading trajectory from file first, then parameters, then default
        std::string trajectory_file;
        if (nh_private_.getParam("trajectory_file", trajectory_file)) {
            ROS_INFO("Loading trajectory from file: %s", trajectory_file.c_str());
            if (traj_gen_->loadFromFile(trajectory_file)) {
                ROS_INFO("Successfully loaded trajectory from file");
            } else {
                ROS_ERROR("Failed to load trajectory from file, using default");
                traj_gen_ = std::make_shared<smc_quadrotor::TrajectoryGenerator>(true);
            }
        } else if (!loadTrajectoryFromParams()) {
            ROS_WARN("No custom trajectory specified, using default trajectory");
            traj_gen_ = std::make_shared<smc_quadrotor::TrajectoryGenerator>(true);
        }
        ROS_INFO("Trajectory generator initialized");
        
        // Get controller parameters
        std::map<std::string, double> params = getControllerParams();
        
        // Initialize SMC controller
        controller_ = std::make_shared<smc_quadrotor::SlidingModeController>(params);
        ROS_INFO("Sliding mode controller initialized");
        
        ROS_INFO("Total mission time: %.1f seconds", traj_gen_->getTotalTime());
        
        // Publisher for rotor speeds
        motor_speed_pub_ = nh_.advertise<mav_msgs::Actuators>(
            "/crazyflie2/command/motor_speed", 10);
        
        // Subscribe to Odometry topic (MUST be last - callbacks start immediately!)
        odom_sub_ = nh_.subscribe(
            "/crazyflie2/ground_truth/odometry", 10,
            &QuadrotorControl::odomCallback, this);
        
        ROS_INFO("=== Quadrotor Control Node Started ===");
        ROS_INFO("Press Ctrl + C to terminate");
    }
    
    ~QuadrotorControl() {
        saveData();
    }

private:
    std::map<std::string, double> getControllerParams() {
        std::map<std::string, double> params;
        
        // Physical parameters (Crazyflie 2.0)
        nh_private_.param("mass", params["mass"], 0.027);
        nh_private_.param("gravity", params["gravity"], 9.81);
        nh_private_.param("Ix", params["Ix"], 16.571710e-6);
        nh_private_.param("Iy", params["Iy"], 16.571710e-6);
        nh_private_.param("Iz", params["Iz"], 29.261652e-6);
        nh_private_.param("Ip", params["Ip"], 12.65625e-8);
        nh_private_.param("kF", params["kF"], 1.28192e-8);
        nh_private_.param("kM", params["kM"], 5.964552e-3);
        nh_private_.param("arm_length", params["arm_length"], 0.046);
        nh_private_.param("omega_min", params["omega_min"], 0.0);
        nh_private_.param("omega_max", params["omega_max"], 2618.0);
        
        // PD gains for x, y position control
        nh_private_.param("kp_xy", params["kp_xy"], 90.0);
        nh_private_.param("kd_xy", params["kd_xy"], 10.0);
        
        // SMC parameters for z (altitude)
        nh_private_.param("lambda_z", params["lambda_z"], 7.0);
        nh_private_.param("eta_z", params["eta_z"], 10.0);
        
        // SMC parameters for phi (roll)
        nh_private_.param("lambda_phi", params["lambda_phi"], 12.0);
        nh_private_.param("eta_phi", params["eta_phi"], 120.0);
        
        // SMC parameters for theta (pitch)
        nh_private_.param("lambda_theta", params["lambda_theta"], 12.0);
        nh_private_.param("eta_theta", params["eta_theta"], 120.0);
        
        // SMC parameters for psi (yaw)
        nh_private_.param("lambda_psi", params["lambda_psi"], 8.0);
        nh_private_.param("eta_psi", params["eta_psi"], 10.0);
        
        return params;
    }
    
    bool loadTrajectoryFromParams() {
        // Check if custom trajectory is specified
        bool has_x = nh_private_.hasParam("trajectory/waypoints_x");
        bool has_y = nh_private_.hasParam("trajectory/waypoints_y");
        bool has_z = nh_private_.hasParam("trajectory/waypoints_z");
        bool has_t = nh_private_.hasParam("trajectory/time_segments");
        
        std::cerr << "DEBUG: Parameter check: x=" << has_x << ", y=" << has_y << ", z=" << has_z << ", t=" << has_t << std::endl;
        
        if (!has_x || !has_y || !has_z || !has_t) {
            std::cerr << "DEBUG: No custom trajectory - using default" << std::endl;
            return false;
        }
        
        // Load waypoints from parameters
        std::vector<double> waypoints_x, waypoints_y, waypoints_z, time_segments;
        bool got_x = nh_private_.getParam("trajectory/waypoints_x", waypoints_x);
        bool got_y = nh_private_.getParam("trajectory/waypoints_y", waypoints_y);
        bool got_z = nh_private_.getParam("trajectory/waypoints_z", waypoints_z);
        bool got_t = nh_private_.getParam("trajectory/time_segments", time_segments);
        
        ROS_INFO("Parameter retrieval: x=%d, y=%d, z=%d, t=%d", got_x, got_y, got_z, got_t);
        
        ROS_INFO("Loaded trajectory params: %zu x, %zu y, %zu z waypoints, %zu time segments",
                 waypoints_x.size(), waypoints_y.size(), waypoints_z.size(), time_segments.size());
        
        // Validate
        if (waypoints_x.size() != waypoints_y.size() || 
            waypoints_x.size() != waypoints_z.size() ||
            waypoints_x.size() < 2) {
            ROS_ERROR("Invalid waypoint dimensions: x=%zu, y=%zu, z=%zu (must be equal and >=2)",
                     waypoints_x.size(), waypoints_y.size(), waypoints_z.size());
            return false;
        }
        
        // Convert to Eigen vectors
        std::vector<Eigen::Vector3d> waypoints;
        for (size_t i = 0; i < waypoints_x.size(); ++i) {
            waypoints.push_back(Eigen::Vector3d(waypoints_x[i], waypoints_y[i], waypoints_z[i]));
        }
        
        // Set trajectory
        if (!traj_gen_->setTrajectory(waypoints, time_segments)) {
            ROS_ERROR("Failed to set custom trajectory: setTrajectory() returned false");
            return false;
        }
        
        ROS_INFO("Loaded custom trajectory with %zu waypoints, total time: %.1fs", 
                 waypoints.size(), traj_gen_->getTotalTime());
        return true;
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Initialize time
        if (!initialized_) {
            t0_ = msg->header.stamp.toSec();
            initialized_ = true;
        }
        t_ = msg->header.stamp.toSec() - t0_;
        
        // Extract position
        Eigen::Vector3d position(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        
        // Extract orientation (quaternion to Euler)
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        Eigen::Vector3d attitude(roll, pitch, yaw);
        
        // Extract body-frame velocities
        Eigen::Vector3d v_b(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z
        );
        
        // Convert to world frame
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::Vector3d velocity = R * v_b;
        
        // Extract body-frame angular velocities
        Eigen::Vector3d w_b(
            msg->twist.twist.angular.x,
            msg->twist.twist.angular.y,
            msg->twist.twist.angular.z
        );
        
        // Convert to Euler rates
        Eigen::Matrix3d W;
        W << 1, std::sin(roll)*std::tan(pitch), std::cos(roll)*std::tan(pitch),
             0, std::cos(roll), -std::sin(roll),
             0, std::sin(roll)/std::cos(pitch), std::cos(roll)/std::cos(pitch);
        Eigen::Vector3d angular_velocity = W * w_b;
        
        // Store trajectory data
        t_series_.push_back(t_);
        x_series_.push_back(position(0));
        y_series_.push_back(position(1));
        z_series_.push_back(position(2));
        
        // Get desired trajectories
        Eigen::Vector3d desired_position, desired_velocity, desired_acceleration;
        traj_gen_->getDesiredState(t_, desired_position, desired_velocity, desired_acceleration);
        
        // Compute control using SMC
        Eigen::Vector4d motor_vel = controller_->computeControl(
            position, velocity, attitude, angular_velocity,
            desired_position, desired_velocity, desired_acceleration,
            0.0  // desired_yaw = 0
        );
        
        // Publish motor velocities
        mav_msgs::Actuators motor_speed_msg;
        motor_speed_msg.angular_velocities.resize(4);
        motor_speed_msg.angular_velocities[0] = motor_vel(0);
        motor_speed_msg.angular_velocities[1] = motor_vel(1);
        motor_speed_msg.angular_velocities[2] = motor_vel(2);
        motor_speed_msg.angular_velocities[3] = motor_vel(3);
        motor_speed_pub_.publish(motor_speed_msg);
        
        // Log control details every 1 second
        static double last_log_time = 0.0;
        if (t_ - last_log_time >= 1.0) {
            double error = (desired_position - position).norm();
            ROS_INFO("t=%.1fs | Pos: [%.2f,%.2f,%.2f] | Des: [%.2f,%.2f,%.2f] | Err: %.3fm | Motors: [%.0f,%.0f,%.0f,%.0f]",
                     t_,
                     position(0), position(1), position(2),
                     desired_position(0), desired_position(1), desired_position(2),
                     error,
                     motor_vel(0), motor_vel(1), motor_vel(2), motor_vel(3));
            last_log_time = t_;
        }
    }
    
    void saveData() {
        std::string filename = "log_cpp.txt";
        std::ofstream file(filename);
        
        if (file.is_open()) {
            file << "# Time(s) X(m) Y(m) Z(m)\n";
            for (size_t i = 0; i < t_series_.size(); ++i) {
                file << t_series_[i] << " " 
                     << x_series_[i] << " " 
                     << y_series_[i] << " " 
                     << z_series_[i] << "\n";
            }
            file.close();
            ROS_INFO("Trajectory data saved to %s", filename.c_str());
        } else {
            ROS_ERROR("Failed to save trajectory data");
        }
    }

    // ROS handles
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    // ROS communication
    ros::Subscriber odom_sub_;
    ros::Publisher motor_speed_pub_;
    
    // Controllers
    std::shared_ptr<smc_quadrotor::TrajectoryGenerator> traj_gen_;
    std::shared_ptr<smc_quadrotor::SlidingModeController> controller_;
    
    // Time tracking
    double t0_;
    double t_;
    bool initialized_;
    
    // Trajectory logging
    std::vector<double> t_series_;
    std::vector<double> x_series_;
    std::vector<double> y_series_;
    std::vector<double> z_series_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "quadrotor_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    try {
        QuadrotorControl quadrotor(nh, nh_private);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Error in quadrotor control node: %s", e.what());
        return 1;
    }
    
    return 0;
}
