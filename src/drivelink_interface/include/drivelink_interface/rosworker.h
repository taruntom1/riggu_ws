#pragma once

#include <QThread>
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp> // Include Odometry message
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"

class RosWorker : public QThread
{
    Q_OBJECT

public:
    explicit RosWorker(QObject *parent = nullptr);
    std::shared_ptr<rclcpp::Node> getNode() const;

public slots:

    void publishOdometry(nav_msgs::msg::Odometry odom_msg);

signals:
    void commandVelReceived(geometry_msgs::msg::TwistStamped cmd_vel);

protected:
    void run() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr control_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; // Odometry publisher
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // TF broadcaster

    void makeOdoPublisher();
    void makeControlSubscriber();

    void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
};
