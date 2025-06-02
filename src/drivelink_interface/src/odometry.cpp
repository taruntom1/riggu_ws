#include "odometry.h"

Odometry::Odometry(odometry_config_t config, QObject *parent)
    : QObject(parent),
      encoderOdometry_(config.encoder_odometry_config)
{
}

void Odometry::updateEncoderOdometry(std::pair<timestamp_t, std::vector<odometry_t>> odometry)
{
    encoderOdometry_.update(odometry);
    emit publishEncoderOdometry(encoderOdometry_.getOdometryMsg());
}

void Odometry::resetPose(double x, double y, double theta)
{
    encoderOdometry_.resetPose(x, y, theta);
}

void Odometry::updateTimeDelta(int64_t delta_ns)
{
    encoderOdometry_.updateTimeDelta(delta_ns);
}