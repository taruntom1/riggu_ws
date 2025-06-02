#include "controllermanager.h"

ControllerManager::ControllerManager(CommunicationInterface *commInterface,
                                     TimeSyncClient *timeSyncClient,
                                     SerialConfig serialConfig,
                                     QObject *parent)
    : QObject(parent), commInterface(commInterface), timeSyncClient(timeSyncClient),
      port_name(QString::fromStdString(serialConfig.port)), baud_rate(serialConfig.baud)
{
    manageQtConnections();

    if (!readConfigurationJson())
        return;

    if (!retryOperation([&]()
                        { return connectController(); }, 10, 1000))
        return;

    setTimeSync(true);

    if (!retryOperation([&]()
                        { return setControllerProperties(); }, 10, 1000))
        return;
    if (!retryOperation([&]()
                        { return setWheelData(); }, 10, 1000))
        return;
}

ControllerManager::~ControllerManager()
{
    // some code to gracefull shutdown of controller
}

void ControllerManager::manageQtConnections()
{
    // Communication Interface Connections
    QObject::connect(this, &ControllerManager::connectToController, commInterface, &CommunicationInterface::connectSerial);
    QObject::connect(this, &ControllerManager::disconnectFromController, commInterface, &CommunicationInterface::disconnectSerial);

    QObject::connect(this, &ControllerManager::sendControllerProperties, commInterface, &CommunicationInterface::sendControllerProperties);
    QObject::connect(this, &ControllerManager::sendWheelData, commInterface, &CommunicationInterface::sendWheelData);

    // TimeSyncClient Connections
    QObject::connect(this, &ControllerManager::startTimesync, timeSyncClient, &TimeSyncClient::startSync, Qt::QueuedConnection);
    QObject::connect(this, &ControllerManager::stopTimesync, timeSyncClient, &TimeSyncClient::stopSync, Qt::QueuedConnection);
}

bool ControllerManager::readConfigurationJson()
{
    std::string package_path = ament_index_cpp::get_package_share_directory("drivelink_interface");
    std::string path = package_path + "/config/controller_config.json";
    // std::string path = "/home/tarun/ros2_ws/install/drivelink_interface/share/drivelink_interface/config/controller_config.json";
    QString json_path = QString::fromStdString(path);

    RCLCPP_INFO(logger, "Reading configuration JSON from: %s", json_path.toStdString().c_str());

    if (json_path.isEmpty())
    {
        RCLCPP_WARN(logger, "Configuration path is empty. Aborting read.");
        return false; // user cancelled or invalid path
    }

    QFile file(json_path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        RCLCPP_ERROR(logger, "Could not open configuration file for reading: %s", json_path.toStdString().c_str());
        return false;
    }

    QByteArray jsonData = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData, &parseError);

    if (parseError.error != QJsonParseError::NoError || !jsonDoc.isObject())
    {
        RCLCPP_ERROR(logger, "Failed to parse JSON config: %s (error: %s)",
                     json_path.toStdString().c_str(),
                     parseError.errorString().toStdString().c_str());
        return false;
    }

    QJsonObject json = jsonDoc.object();

    controller_data_t data = StructSerialiser::fromJson_controller_data(json);

    controller_data.controllerProperties = data.controllerProperties;
    controller_data.wheelData = data.wheelData;

    RCLCPP_INFO(logger, "Configuration loaded successfully.");

    return true;
}

bool ControllerManager::connectController()
{
    bool connected = false;
    QEventLoop loop;

    RCLCPP_INFO(logger, "Attempting to connect to controller on port '%s' with baud rate %d",
                port_name.toStdString().c_str(), baud_rate);

    auto onStatusChanged = [&](bool status, const QString &message)
    {
        connected = status;
        if (connected)
        {
            RCLCPP_INFO(logger, "Successfully connected to controller: %s", message.toStdString().c_str());
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to connect to controller: %s", message.toStdString().c_str());
        }
        loop.quit();
    };

    QObject::connect(commInterface, &CommunicationInterface::connectionStatusChange, &loop, onStatusChanged);

    emit connectToController(port_name, baud_rate);
    RCLCPP_DEBUG(logger, "Waiting for connection status response...");
    loop.exec();

    QObject::disconnect(commInterface, &CommunicationInterface::connectionStatusChange, &loop, nullptr);

    if (connected)
    {
        RCLCPP_DEBUG(logger, "Controller connection process completed successfully.");
    }
    else
    {
        RCLCPP_WARN(logger, "Controller connection process completed with failure.");
    }

    return connected;
}

void ControllerManager::disconnectController()
{
    RCLCPP_INFO(logger, "Emitting disconnect request to controller");
    emit disconnectFromController();
}

void ControllerManager::setTimeSync(bool status)
{
    if (status)
    {
        RCLCPP_INFO(logger, "Enabling time synchronization");
        emit startTimesync(); // time syncronization period is 3 seconds
    }
    else
    {
        RCLCPP_INFO(logger, "Disabling time synchronization");
        emit stopTimesync();
    }
}
bool ControllerManager::setControllerProperties()
{
    bool success = false;
    QEventLoop loop;

    RCLCPP_INFO(logger, "Sending controller properties...");

    auto onStatusChanged = [&](bool status, const QString &message)
    {
        success = status;
        if (success)
        {
            RCLCPP_INFO(logger, "Controller properties set successfully: %s", message.toStdString().c_str());
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to set controller properties: %s", message.toStdString().c_str());
        }
        loop.quit();
    };

    QObject::connect(commInterface, &CommunicationInterface::propertySetUpdate, &loop, onStatusChanged);

    emit sendControllerProperties(controller_data.controllerProperties);
    RCLCPP_DEBUG(logger, "Waiting for property set confirmation...");

    loop.exec();

    QObject::disconnect(commInterface, &CommunicationInterface::propertySetUpdate, &loop, nullptr);

    return success;
}

bool ControllerManager::setWheelData()
{
    bool success = true;
    QEventLoop loop;

    RCLCPP_INFO(logger, "Starting to send wheel data for %zu wheels", controller_data.wheelData.size());

    auto onStatusChanged = [&](bool status, const QString &message)
    {
        success &= status;
        if (status)
        {
            RCLCPP_INFO(logger, "Wheel data set successfully: %s", message.toStdString().c_str());
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to set wheel data: %s", message.toStdString().c_str());
        }
        loop.quit();
    };

    QObject::connect(commInterface, &CommunicationInterface::propertySetUpdate, &loop, onStatusChanged);

    for (const wheel_data_t &wheel_data : controller_data.wheelData)
    {
        RCLCPP_DEBUG(logger, "Sending wheel data for motor ID %d", wheel_data.motor_id);
        emit sendWheelData(wheel_data.motor_id, wheel_data);
        loop.exec();
    }

    QObject::disconnect(commInterface, &CommunicationInterface::propertySetUpdate, &loop, nullptr);

    if (success)
    {
        RCLCPP_INFO(logger, "All wheel data sent successfully.");
    }
    else
    {
        RCLCPP_WARN(logger, "Some wheel data failed to send.");
    }

    return success;
}

bool ControllerManager::retryOperation(const std::function<bool()> &operation, int maxRetries, int delayMs)
{
    int retries = 0;

    RCLCPP_INFO(logger, "Starting retry operation: maxRetries=%d, delayMs=%d", maxRetries, delayMs);

    while (retries < maxRetries)
    {
        RCLCPP_DEBUG(logger, "Attempt %d of %d", retries + 1, maxRetries);

        if (operation())
        {
            RCLCPP_INFO(logger, "Operation succeeded on attempt %d", retries + 1);
            return true;
        }

        RCLCPP_WARN(logger, "Operation failed on attempt %d, retrying after %d ms...", retries + 1, delayMs);
        QThread::msleep(delayMs);
        ++retries;
    }

    RCLCPP_ERROR(logger, "Operation failed after %d attempts", maxRetries);
    return false;
}
