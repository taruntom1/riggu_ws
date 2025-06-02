#pragma once
#include <QObject>

#include <geometry_msgs/msg/twist.hpp>

#include "structs.h"
#include "odometry.h"
#include "diffdrivecontrol.h"
#include "communicationinterface.h"
#include "rosworker.h"
#include "timesyncclient.h"

struct diff_drive_config_t
{
    odometry_config_t odometry_config;
    diff_drive_control_config_t diff_drive_control_config;

    void setWheelRadius(float wheel_radius)
    {
        odometry_config.encoder_odometry_config.right_wheel_radius = wheel_radius;
        odometry_config.encoder_odometry_config.left_wheel_radius = wheel_radius;
        diff_drive_control_config.wheel_radius = wheel_radius;
    }
    void setWheelBase(float wheel_base)
    {
        odometry_config.encoder_odometry_config.wheel_base = wheel_base;
        diff_drive_control_config.wheel_base = wheel_base;
    }
};

class DiffDrive : public QObject
{
    Q_OBJECT
public:
    explicit DiffDrive(CommunicationInterface *communication_interface = nullptr,
                       RosWorker *ros_worker = nullptr,
                       TimeSyncClient *time_sync_client = nullptr,
                       diff_drive_config_t config = {},
                       QObject *parent = nullptr);
public slots:
    void updateTimeDelta(int64_t delta_time_ns);

private:
    diff_drive_config_t config_;

    CommunicationInterface *communication_interface_;
    RosWorker *ros_worker_;
    TimeSyncClient *time_sync_client_;

    Odometry odometry_;
    DiffDriveControl control_;

    void makeConnections();
};