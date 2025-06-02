#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyToCmdVel : public rclcpp::Node
{
public:
    JoyToCmdVel() : Node("joy_to_cmdvel")
    {
        // Declare parameters with default values
        // linear_axis: Index of joystick axis for linear velocity (default: 1, usually left stick vertical)
        this->declare_parameter("linear_axis", 1);  
        // angular_axis: Index of joystick axis for angular velocity (default: 0, usually left stick horizontal)
        this->declare_parameter("angular_axis", 0); 
        // linear_scale: Scaling factor for linear velocity (default: 1.0)
        this->declare_parameter("linear_scale", 1.0);
        // angular_scale: Scaling factor for angular velocity (default: 1.0)
        this->declare_parameter("angular_scale", 1.0);

        // Get parameters (can be overridden via launch file or command line)
        this->get_parameter("linear_axis", linear_axis_);
        this->get_parameter("angular_axis", angular_axis_);
        this->get_parameter("linear_scale", linear_scale_);
        this->get_parameter("angular_scale", angular_scale_);

        // Publisher and Subscriber
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&JoyToCmdVel::joy_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Joystick to cmd_vel node started.");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        geometry_msgs::msg::Twist twist_msg;

        // Check if the required axes are present in the joystick message
        if (msg->axes.size() > std::max(linear_axis_, angular_axis_)) {
            twist_msg.linear.x = linear_scale_ * msg->axes[linear_axis_];
            twist_msg.angular.z = angular_scale_ * msg->axes[angular_axis_];
            cmd_vel_pub_->publish(twist_msg);
        } else {
            // Log a warning if axes are missing or out of range
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Joystick axes out of range: received %zu axes, expected at least %d. No cmd_vel published.",
                msg->axes.size(), std::max(linear_axis_, angular_axis_) + 1);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    int linear_axis_;
    int angular_axis_;
    double linear_scale_;
    double angular_scale_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVel>());
    rclcpp::shutdown();
    return 0;
}
