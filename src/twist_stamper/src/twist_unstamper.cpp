#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistUnstamper : public rclcpp::Node
{
public:
    TwistUnstamper()
    : Node("twist_unstamper")
    {
        // Create publisher for Twist messages
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel_out", 10);

        // Create subscription for TwistStamped messages
        subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "cmd_vel_in", 10,
            std::bind(&TwistUnstamper::listener_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Started twist_unstamper!");
    }

private:
    void listener_callback(const geometry_msgs::msg::TwistStamped::SharedPtr in_msg)
    {
        // Extract and publish the twist part of the TwistStamped message
        publisher_->publish(in_msg->twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TwistUnstamper>();
    
    try {
        rclcpp::spin(node);
    } catch (const rclcpp::exceptions::RCLError & e) {
        RCLCPP_ERROR(node->get_logger(), "RCL error: %s", e.what());
    } catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }

    RCLCPP_INFO(node->get_logger(), "Exiting...");
    
    rclcpp::shutdown();
    return 0;
}
