#include <QCoreApplication>
#include <QThread>
#include <rclcpp/rclcpp.hpp>

// Project headers
#include "serialhandler.h"
#include "controllermanager.h"
#include "communicationinterface.h"
#include "timesyncclient.h"
#include "rosworker.h"
#include "nodeconfigparser.h"
#include "diffdrive.h"

int main(int argc, char *argv[])
{
    // Initialize ROS2 and Qt core application
    rclcpp::init(argc, argv);
    QCoreApplication app(argc, argv);

    // --- ROS Worker Thread ---
    RosWorker *rosWorker = new RosWorker;
    rosWorker->start();

    // --- Communication Thread and Components ---
    QThread *commThread = new QThread;

    SerialHandler *serialHandler = new SerialHandler;
    CommunicationInterface *commInterface = new CommunicationInterface(nullptr, serialHandler);

    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    TimeSyncClient *timeSyncClient = new TimeSyncClient(serialHandler, commInterface, &clock);

    // Move components to the communication thread
    serialHandler->moveToThread(commThread);
    commInterface->moveToThread(commThread);
    timeSyncClient->moveToThread(commThread);
    commThread->start();

    // --- Application Modules ---
    // Reading configurations from yaml file using ROS2 parameters
    NodeConfigParser parser(rosWorker->getNode());

    if (!parser.isValid())
    {
        std::cerr << "Failed to load config file.\n";
        return 1;
    }

    SerialConfig serial = parser.getSerialConfig();
    ControllerManager *controllerManager = new ControllerManager(commInterface, timeSyncClient, serial, rosWorker->getNode());
    auto diff_config = parser.getDiffDriveConfig();
    diff_drive_config_t diff_drive_config;
    diff_drive_config.setWheelRadius(diff_config.wheel_radius);
    diff_drive_config.setWheelBase(diff_config.wheel_base);
    DiffDrive *diffDrive = new DiffDrive(commInterface, rosWorker, timeSyncClient, diff_drive_config);

    // --- Graceful Shutdown Handling ---
    rclcpp::on_shutdown([&app]()
                        { app.quit(); });

    QObject::connect(&app, &QCoreApplication::aboutToQuit, []()
                     {
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        } });

    // --- Execute Application Loop ---
    int ret = app.exec();

    // --- Cleanup ---

    delete controllerManager;
    rosWorker->quit();
    rosWorker->wait();

    commThread->quit();
    commThread->wait();
    delete timeSyncClient;
    delete commInterface;
    delete serialHandler;
    delete commThread;
    delete rosWorker;
    delete diffDrive;

    return ret;
}
