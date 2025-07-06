#include "rosworker.h"

RosWorker::RosWorker(QObject *parent)
    : QThread(parent),
      node_(std::make_shared<rclcpp::Node>("MotorController"))
{
    makeOdoPublisher();
    makeControlSubscriber();

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
}

std::shared_ptr<rclcpp::Node> RosWorker::getNode() const
{
    return node_;
}

void RosWorker::run()
{
    rclcpp::spin(node_);
}

void RosWorker::makeOdoPublisher()
{
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
}

void RosWorker::makeControlSubscriber()
{
    control_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/diff_drive/cmd_vel", 10,
        std::bind(&RosWorker::cmdVelCallback, this, std::placeholders::_1));
}

void RosWorker::publishOdometry(nav_msgs::msg::Odometry odom_msg)
{

    if (odom_pub_ && rclcpp::ok())
    {
        odom_pub_->publish(odom_msg);

        // Broadcast the transform
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = odom_msg.header.stamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_link";

        // Copy position from odometry message
        transform_stamped.transform.translation.x = odom_msg.pose.pose.position.x;
        transform_stamped.transform.translation.y = odom_msg.pose.pose.position.y;
        transform_stamped.transform.translation.z = odom_msg.pose.pose.position.z;

        // Copy rotation from odometry message
        transform_stamped.transform.rotation = odom_msg.pose.pose.orientation;

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_stamped);
    }
}

void RosWorker::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    emit commandVelReceived(*msg);
}
