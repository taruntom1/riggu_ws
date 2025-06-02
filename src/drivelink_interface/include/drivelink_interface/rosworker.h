#pragma once

#include <QThread>
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp> // Include Odometry message

class RosWorker : public QThread
{
    Q_OBJECT

public:
    explicit RosWorker(QObject *parent = nullptr);
    std::shared_ptr<rclcpp::Node> getNode() const;

public slots:

    void publishOdometry(nav_msgs::msg::Odometry odom_msg);

signals:
    void commandVelReceived(geometry_msgs::msg::Twist cmd_vel);

protected:
    void run() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; // Odometry publisher

    void makeOdoPublisher();
    void makeControlSubscriber();

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
};
