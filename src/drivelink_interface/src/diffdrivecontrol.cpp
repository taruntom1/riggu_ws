#include "diffdrive.h"

DiffDriveControl::DiffDriveControl(diff_drive_control_config_t config,
                                   QObject *parent)
    : QObject(parent), wheel_radius_(config.wheel_radius), wheel_base_(config.wheel_base)

{
}

void DiffDriveControl::onVelocityCommand(geometry_msgs::msg::TwistStamped cmd_vel)
{
    // Extract forward linear velocity (v) and yaw rate (ω)
    const float v = static_cast<float>(cmd_vel.twist.linear.x);  // [m/s]
    const float w = static_cast<float>(cmd_vel.twist.angular.z); // [rad/s]

    // Half of the track width:
    const float half_base = wheel_base_ * 0.5;

    // Differential‐drive inverse kinematics:
    // Calculate angular velocities (rad/s) for each wheel
    const float omega_left = (v - half_base * w) / wheel_radius_;
    const float omega_right = (v + half_base * w) / wheel_radius_;

    emit sendSetpoints({omega_left, omega_right});
}
