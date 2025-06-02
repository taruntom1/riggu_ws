#pragma once

#include <QObject>
#include <QElapsedTimer>
#include <QSerialPortInfo>

#include <vector>
#include <cstring>

#include "serialhandler.h"
#include "structs.h"
#include "commands.h"

class CommunicationInterface : public QObject
{
    Q_OBJECT

public:
    explicit CommunicationInterface(QObject *parent = nullptr, SerialHandler *serialHandler = nullptr);

public slots:
    void listAvailablePorts(QList<QSerialPortInfo> *ports);
    void toggleConnect(const QString &port_name, int baud_rate);
    void connectSerial(const QString &port_name, int baud_rate);
    void disconnectSerial();
    void commandReceived(char command);

    void ping();

    void sendControllerProperties(controller_properties_t properties);
    void sendWheelData(int id, const wheel_data_t wheel_data);
    void sendPIDConstants(int motor_id, int pid_type, pid_constants_t pid_constants);
    void sendOdoBroadcastStatus(int motor_id, odo_broadcast_flags_t flags);

    void sendControlMode(int motor_id, ControlMode mode);

    void sendSetpoints(std::vector<float> setpoints);

signals:
    void connectionStatusChange(int status, const QString &error_string);
    void propertySetUpdate(bool status, const QString &log_string);

    void controllerPropertiesReceived();
    void motorDataReceived(int motor_id);

    void odometryDataReceived(std::pair<timestamp_t, std::vector<odometry_t>>);

    void timeSyncReplyReceived();

private:
    SerialHandler *serialHandler;

    int num_wheels = 0;
    bool connected_flag = false;

    void stopAllBroadcast();
    void restoreAllBroadcast();
    void sendProperty(Command command, const std::vector<uint8_t> &data, const QString &property_name);

    void receiveOdometry();
};
