#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>
#include <array>
#include <vector>
#include "structs.h"
#include "RollingMeanAccumulator.h"

struct encoder_odometry_config_t
{
    double left_wheel_radius;
    double right_wheel_radius;
    double wheel_base;
    bool use_exact_integration;
    int rolling_window_size;

    std::array<double, 36> pose_covariance;
    std::array<double, 36> twist_covariance;
};

class EncoderOdometry
{
public:
    EncoderOdometry(const encoder_odometry_config_t &config);

    void update(std::pair<timestamp_t, std::vector<odometry_t>> odometry);

    void updateTimeDelta(int64_t delta_ns);

    double getX() const;
    double getY() const;
    double getTheta() const;

    double getLinearVelocity() const;
    double getAngularVelocity() const;

    void resetPose(double x, double y, double theta);
    void resetAccumulators();
    void updateFromVelocity(double linear, double angular, int64_t timestamp_ns);

    nav_msgs::msg::Odometry getOdometryMsg() const;

private:
    void integrateExact(double linear, double angular);
    void integrateRungeKutta2(double linear, double angular);

    double wheel_base_;

    double left_wheel_radius_;
    double right_wheel_radius_;
    RollingMeanAccumulator linear_accumulator_;
    RollingMeanAccumulator angular_accumulator_;
    size_t rolling_window_size_ = 10;
    bool use_exact_integration_ = true;

    int64_t time_delta_ns_; // Time difference between this system and MCU

    std::array<double, 36> pose_covariance_;
    std::array<double, 36> twist_covariance_;

    double last_left_angle_;
    double last_right_angle_;
    double last_timestamp_;

    double x_, y_, theta_;
    double v_x_, omega_z_;
};
