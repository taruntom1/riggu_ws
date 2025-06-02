#pragma once

#include <memory>
#include <string>
#include <vector>
#include <QObject>

#include "structs.h"
#include "encoderodometry.h"
#include "communicationinterface.h"
#include "rosworker.h"

struct odometry_config_t
{
    encoder_odometry_config_t encoder_odometry_config;
};

class Odometry : public QObject
{
    Q_OBJECT
public:
    explicit Odometry(odometry_config_t config = {},
                      QObject *parent = nullptr);

    void resetPose(double x, double y, double theta);

    void updateTimeDelta(int64_t delta_ns);

public slots:
    void updateEncoderOdometry(std::pair<timestamp_t, std::vector<odometry_t>> odometry);

private:
    // Differential drive odometry object
    encoder_odometry_config_t encoder_odometry_config;
    EncoderOdometry encoderOdometry_;

signals:
    void publishEncoderOdometry(nav_msgs::msg::Odometry odom_msg);
};
