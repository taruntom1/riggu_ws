#include "diffdrive.h"

DiffDrive::DiffDrive(CommunicationInterface *communication_interface,
                     RosWorker *ros_worker,
                     TimeSyncClient *time_sync_client,
                     diff_drive_config_t config,
                     QObject *parent)
    : QObject(parent),
      communication_interface_(communication_interface),
      ros_worker_(ros_worker),
      time_sync_client_(time_sync_client),
      config_(config),
      control_(config.diff_drive_control_config),
      odometry_(config.odometry_config)

{
    makeConnections();
}

void DiffDrive::updateTimeDelta(int64_t delta_time_ns)
{
    odometry_.updateTimeDelta(delta_time_ns);
}

void DiffDrive::makeConnections()
{
    connect(communication_interface_, &CommunicationInterface::odometryDataReceived, &odometry_, &Odometry::updateEncoderOdometry);
    connect(&odometry_, &Odometry::publishEncoderOdometry, ros_worker_, &RosWorker::publishOdometry);

    connect(ros_worker_, &RosWorker::commandVelReceived, &control_, &DiffDriveControl::onVelocityCommand); // not implemented yet
    connect(&control_, &DiffDriveControl::sendSetpoints, communication_interface_, &CommunicationInterface::sendSetpoints);

    connect(time_sync_client_, &TimeSyncClient::syncCompleted, this, &DiffDrive::updateTimeDelta);
}
