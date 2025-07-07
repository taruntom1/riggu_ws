#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/header.hpp"

class TwistStamper : public rclcpp::Node
{
public:
    TwistStamper()
    : Node("twist_stamper")
    {
        // Declare and get frame_id parameter
        this->declare_parameter<std::string>("frame_id", "");
        this->get_parameter("frame_id", frame_id_);

        // Create publisher for TwistStamped messages
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "cmd_vel_out", 10);

        // Create subscription for Twist messages
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_in", 10,
            std::bind(&TwistStamper::listener_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Started twist_stamper!");
    }

private:
    void listener_callback(const geometry_msgs::msg::Twist::SharedPtr in_msg)
    {
        auto out_msg = geometry_msgs::msg::TwistStamped();
        
        // Set header
        out_msg.header.stamp = this->get_clock()->now();
        out_msg.header.frame_id = frame_id_;
        
        // Copy twist data
        out_msg.twist = *in_msg;

        publisher_->publish(out_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std::string frame_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TwistStamper>();
    
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
