#include "encoderodometry.h"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

EncoderOdometry::EncoderOdometry(const encoder_odometry_config_t &config)
    : last_left_angle_(0.0), last_right_angle_(0.0), last_timestamp_(-1.0),
      x_(0.0), y_(0.0), theta_(0.0), v_x_(0.0), omega_z_(0.0),
      linear_accumulator_(config.rolling_window_size),
      angular_accumulator_(config.rolling_window_size),
      rolling_window_size_(config.rolling_window_size),
      use_exact_integration_(config.use_exact_integration)
{
    left_wheel_radius_ = config.left_wheel_radius;
    right_wheel_radius_ = config.right_wheel_radius;
    wheel_base_ = config.wheel_base;
    pose_covariance_ = config.pose_covariance;
    twist_covariance_ = config.twist_covariance;
}

void EncoderOdometry::update(std::pair<timestamp_t, std::vector<odometry_t>> odometry)
{
    float &left_angle = odometry.second.at(0).angle;
    float &right_angle = odometry.second.at(1).angle;
    uint64_t timestamp = odometry.first * 1000; // Convert to nanoseconds

    if (last_timestamp_ < 0.0)
    {
        last_left_angle_ = left_angle;
        last_right_angle_ = right_angle;
        last_timestamp_ = timestamp;
        return;
    }

    double dt = (timestamp - last_timestamp_) * 1e-9;
    if (dt < 0.0001)
        return;

    double d_left = (left_angle - last_left_angle_) * left_wheel_radius_;
    double d_right = (right_angle - last_right_angle_) * right_wheel_radius_;

    double linear = (d_left + d_right) * 0.5;
    double angular = (d_right - d_left) / wheel_base_;

    // Integrate using selected method
    if (use_exact_integration_)
    {
        integrateExact(linear, angular);
    }
    else
    {
        integrateRungeKutta2(linear, angular);
    }

    linear_accumulator_.accumulate(linear / dt);
    angular_accumulator_.accumulate(angular / dt);

    v_x_ = linear_accumulator_.getRollingMean();
    omega_z_ = angular_accumulator_.getRollingMean();

    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    last_left_angle_ = left_angle;
    last_right_angle_ = right_angle;
    last_timestamp_ = timestamp;
}

void EncoderOdometry::integrateExact(double linear, double angular)
{
    if (fabs(angular) < 1e-6)
    {
        integrateRungeKutta2(linear, angular);
        return;
    }
    double heading_old = theta_;
    double r = linear / angular;
    theta_ += angular;
    x_ += r * (std::sin(theta_) - std::sin(heading_old));
    y_ += -r * (std::cos(theta_) - std::cos(heading_old));
}

void EncoderOdometry::integrateRungeKutta2(double linear, double angular)
{
    double direction = theta_ + angular * 0.5;
    x_ += linear * std::cos(direction);
    y_ += linear * std::sin(direction);
    theta_ += angular;
}

double EncoderOdometry::getX() const { return x_; }
double EncoderOdometry::getY() const { return y_; }
double EncoderOdometry::getTheta() const { return theta_; }

double EncoderOdometry::getLinearVelocity() const { return v_x_; }
double EncoderOdometry::getAngularVelocity() const { return omega_z_; }

void EncoderOdometry::resetPose(double x, double y, double theta)
{
    x_ = x;
    y_ = y;
    theta_ = theta;
}

nav_msgs::msg::Odometry EncoderOdometry::getOdometryMsg() const
{

    nav_msgs::msg::Odometry odom;

    odom.header.frame_id = "";
    odom.child_frame_id = "";
    odom.header.stamp = rclcpp::Time(last_timestamp_ + time_delta_ns_);

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.pose.covariance = std::move(pose_covariance_);

    odom.twist.twist.linear.x = v_x_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.z = omega_z_;

    odom.twist.covariance = std::move(twist_covariance_);

    return odom;
}

void EncoderOdometry::updateTimeDelta(int64_t delta_ns)
{
    time_delta_ns_ = delta_ns;
}

void EncoderOdometry::resetAccumulators()
{
    linear_accumulator_.reset();
    angular_accumulator_.reset();
}

void EncoderOdometry::updateFromVelocity(double linear, double angular, int64_t timestamp_ns)
{
    double dt = (timestamp_ns - last_timestamp_) * 1e-9;
    if (dt < 0.0001)
        return;

    if (use_exact_integration_)
        integrateExact(linear * dt, angular * dt);
    else
        integrateRungeKutta2(linear * dt, angular * dt);

    v_x_ = linear;
    omega_z_ = angular;
    last_timestamp_ = timestamp_ns;
}
