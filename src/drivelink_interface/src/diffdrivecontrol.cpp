#include "diffdrive.h"

DiffDriveControl::DiffDriveControl(diff_drive_control_config_t config,
                                   QObject *parent)
    : QObject(parent), wheel_radius_(config.wheel_radius), wheel_base_(config.wheel_base)

{
}

void DiffDriveControl::onVelocityCommand(geometry_msgs::msg::Twist cmd_vel)
{
    // Extract forward linear velocity (v) and yaw rate (ω)
    const float v = static_cast<float>(cmd_vel.linear.x);  // [m/s]
    const float w = static_cast<float>(cmd_vel.angular.z); // [rad/s]

    // Half of the track width:
    const float half_base = wheel_base_ * 0.5;

    // Differential‐drive inverse kinematics:
    const float v_left = (v - half_base * w) / wheel_radius_;
    const float v_right = (v + half_base * w) / wheel_radius_;

    emit sendSetpoints({v_left, v_right});
}
