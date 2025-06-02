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
    // Reading configurations from yaml file
    std::string package_path = ament_index_cpp::get_package_share_directory("drivelink_interface");
    std::string path = package_path + "/config/node_config.yaml";
    //std::string path = "/home/tarun/ros2_ws/install/drivelink_interface/share/drivelink_interface/config/node_config.yaml";
    NodeConfigParser parser(path);

    if (!parser.isValid())
    {
        std::cerr << "Failed to load config file.\n";
        return 1;
    }

    SerialConfig serial = parser.getSerialConfig();
    ControllerManager *controllerManager = new ControllerManager(commInterface, timeSyncClient, serial);
    diff_drive_config_t diff_drive_config;
    diff_drive_config.setWheelRadius(1);
    diff_drive_config.setWheelBase(1);
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
    rosWorker->quit();
    rosWorker->wait();

    commThread->quit();
    commThread->wait();

    delete controllerManager;
    delete timeSyncClient;
    delete commInterface;
    delete serialHandler;
    delete commThread;
    delete rosWorker;
    delete diffDrive;

    return ret;
}
