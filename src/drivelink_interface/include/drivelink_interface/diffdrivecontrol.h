#pragma once

#include <QObject>
#include <geometry_msgs/msg/twist.hpp>

#include "structs.h"

struct diff_drive_control_config_t
{
    float wheel_radius = 1;
    float wheel_base = 1;
};

class DiffDriveControl : public QObject
{
    Q_OBJECT
public:
    /**
     * @brief DiffDriveControl
     * @param wheel_radius  Radius of each wheel [m]
     * @param wheel_base    Distance between the two wheels (track width) [m]
     * @param parent        Optional parent QObject
     */
    explicit DiffDriveControl(diff_drive_control_config_t config,
                              QObject *parent = nullptr);

public slots:
    /**
     * @brief onVelocityCommand
     *        Slot to be connected to a Qt signal that carries a
     *        geometry_msgs::msg::Twist::SharedPtr. Upon receiving a new
     *        cmd_vel, computes left and right wheel angular velocities.
     *
     * @param cmd_vel  Shared pointer to the incoming Twist message
     *                 (linear.x = forward velocity [m/s],
     *                  angular.z = yaw rate [rad/s])
     */
    void onVelocityCommand(geometry_msgs::msg::Twist cmd_vel);

signals:
    /**
     * @brief wheelVelocitiesReady
     *        Emitted after computing wheel angular velocities.
     *
     * @param left_wheel_vel   Angular velocity for left wheel [rad/s]
     * @param right_wheel_vel  Angular velocity for right wheel [rad/s]
     */
    void sendSetpoints(std::vector<float> velocities);

private:
    float wheel_radius_; ///< Radius of one wheel [m]
    float wheel_base_;   ///< Distance between left and right wheels [m]
};
