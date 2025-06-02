#include "communicationinterface.h"

CommunicationInterface::CommunicationInterface(QObject *parent, SerialHandler *serialHandler)
    : QObject{parent}, serialHandler(serialHandler)
{
    connect(serialHandler, &SerialHandler::commandReceived, this, &CommunicationInterface::commandReceived, Qt::DirectConnection);
}

void CommunicationInterface::listAvailablePorts(QList<QSerialPortInfo> *ports)
{
    ports->append(serialHandler->getAvailablePorts());
}

void CommunicationInterface::toggleConnect(const QString &port_name, int baud_rate)
{
    QString error_string;
    int status;
    if (serialHandler->checkConnection())
    {
        serialHandler->disconnectSerial();
        status = -1;
    }
    else
    {
        status = (serialHandler->connectSerial(port_name, baud_rate, &error_string));
    }
    emit connectionStatusChange(status, error_string);
}

void CommunicationInterface::connectSerial(const QString &port_name, int baud_rate)
{
    QString error_string;
    int status;
    status = serialHandler->connectSerial(port_name, baud_rate, &error_string);
    emit connectionStatusChange(status, error_string);
}

void CommunicationInterface::disconnectSerial()
{
    serialHandler->disconnectSerial();
}

void CommunicationInterface::ping()
{
    serialHandler->setAutoReadCommandFlag(false);
    stopAllBroadcast();

    // emit propertySetUpdate("Ping Send");
    qint64 elapsed_time = serialHandler->ping(Command::PING);

    // emit propertySetUpdate("Ping received in " + QString::number(elapsed_time) + " ms");
    serialHandler->setAutoReadCommandFlag(true);
    restoreAllBroadcast();
}

void CommunicationInterface::stopAllBroadcast()
{
    serialHandler->sendCommand(Command::STOP_ALL_BROADCAST);
}

void CommunicationInterface::restoreAllBroadcast()
{
    serialHandler->sendCommand(Command::RESTORE_ALL_BROADCAST);
}

void CommunicationInterface::sendProperty(Command command,
                                          const std::vector<uint8_t> &data,
                                          const QString &property_name)
{
    serialHandler->setAutoReadCommandFlag(false);

    if (serialHandler->sendCommand(command))
    {
        if (!serialHandler->sendData(data))
        {
            emit propertySetUpdate(false, "Failed to send" + property_name);
        }
    }
    else
        emit propertySetUpdate(false, "Failed to send" + property_name + " command");

    serialHandler->setAutoReadCommandFlag(true);
    serialHandler->handleReadyRead();
}

void CommunicationInterface::sendControllerProperties(
    controller_properties_t controller_properties)
{
    sendProperty(Command::SET_CONTROLLER_PROPERTIES,
                 controller_properties.to_bytes(), "Controller Properties");
    num_wheels = controller_properties.numMotors;
}
void CommunicationInterface::sendWheelData(int id, const wheel_data_t wheel_data)
{
    std::vector<uint8_t> data;
    data.reserve(wheel_data_t::size + 1);
    data.push_back(static_cast<uint8_t>(id)); // Motor ID
    auto motor_data_vec = wheel_data.to_bytes();
    data.insert(data.end(), motor_data_vec.begin(),
                motor_data_vec.end()); // Motor Data

    sendProperty(Command::SET_MOTOR_DATA, data,
                 "Motor Data (Motor " + QString::number(id + 1) + ")");
}

void CommunicationInterface::sendPIDConstants(int motor_id, int pid_type,
                                              pid_constants_t pid_constants)
{
    std::vector<uint8_t> data;
    data.reserve(sizeof(pid_constants_t) + 2);
    data.push_back(static_cast<uint8_t>(motor_id)); // Motor ID
    data.push_back(static_cast<uint8_t>(pid_type)); // PID Type

    auto pid_data = pid_constants.to_bytes();
    std::copy(pid_data.begin(), pid_data.end(), std::back_inserter(data));

    sendProperty(Command::SET_PID_CONSTANTS, data,
                 "PID Constants (Motor " + QString::number(motor_id) +
                     (pid_type ? " Speed" : " Angle") + ")");
}

void CommunicationInterface::sendOdoBroadcastStatus(
    int motor_id, odo_broadcast_flags_t flags)
{
    std::vector<uint8_t> data;
    data.reserve(sizeof(odo_broadcast_flags_t) + 1);
    data.push_back(static_cast<uint8_t>(motor_id));

    auto odo_broadcast_flags = flags.to_bytes();
    std::copy(odo_broadcast_flags.begin(), odo_broadcast_flags.end(),
              std::back_inserter(data));

    sendProperty(Command::SET_ODO_BROADCAST_STATUS, data,
                 "Odometry broadcast status for motor " +
                     QString::number(motor_id));
}

void CommunicationInterface::sendSetpoints(std::vector<float> setpoints)
{
    std::vector<uint8_t> data(sizeof(float) * setpoints.size() + 4);
    constexpr uint8_t header_and_command[] = {
        0xAA, 0xAA, 0xAA, static_cast<uint8_t>(Command::SET_WHEEL_SETPOINT)};
    memcpy(data.data(), header_and_command, sizeof(header_and_command));

    memcpy(data.data() + 4, setpoints.data(), setpoints.size() * sizeof(float));
    serialHandler->sendData(data);
}

void CommunicationInterface::sendControlMode(int motor_id,
                                             ControlMode control_mode)
{
    std::vector<uint8_t> data;
    data.reserve(1 + sizeof(ControlMode));
    data.push_back(static_cast<char>(motor_id));
    data.push_back(static_cast<char>(control_mode));

    sendProperty(Command::SET_MOTOR_CONTROL_MODES, data,
                 "Control Mode (Motor " + QString::number(motor_id) + ")");
}

void CommunicationInterface::receiveOdometry()
{
    auto flag = serialHandler->readData(1);
    if (flag.empty())
    {
        return;
    }
    const uint8_t flags = flag[0];
    const bool angleBroadcast = flags & (1 << 0);
    const bool speedBroadcast = flags & (1 << 1);
    const bool pwmBroadcast = flags & (1 << 2);

    const size_t total_size =
        (num_wheels * sizeof(float) *
         (static_cast<int>(angleBroadcast) + static_cast<int>(speedBroadcast) +
          static_cast<int>(pwmBroadcast))) +
        sizeof(timestamp_t);

    std::pair<timestamp_t, std::vector<odometry_t>> odometry;
    odometry.second.reserve(num_wheels);

    auto data = serialHandler->readData(total_size);
    if (data.empty())
    {
        return;
    }

    size_t offset = 0;

    odometry.first = detail::readLE<timestamp_t>(data, offset);

    for (int i = 0; i < num_wheels; ++i)
    {
        odometry_t odo;
        if (angleBroadcast)
        {
            odo.angle = detail::readLE<angle_t>(data, offset);
        }
        if (speedBroadcast)
        {
            odo.angular_velocity = detail::readLE<angularvelocity_t>(data, offset);
        }
        if (pwmBroadcast)
        {
            odo.pwm_value = detail::readLE<pwmvalue_t>(data, offset);
        }
        odometry.second.push_back(odo);
    }

    emit odometryDataReceived(odometry);
}

void CommunicationInterface::commandReceived(char command)
{
    switch (static_cast<Command>(command))
    {
    case Command::SYNC_TIME:
        emit timeSyncReplyReceived();
        break;

    case Command::SEND_ODOMETRY:
        receiveOdometry();
        break;
    case Command::READ_SUCCESS:
        emit propertySetUpdate(true, "Properties set successfully");
        break;
    case Command::READ_FAILURE:
        emit propertySetUpdate(false, "Properties set failed");
        break;

    default:
        break;
    }
}
