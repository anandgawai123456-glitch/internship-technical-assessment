#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistRelay : public rclcpp::Node
{
public:
    TwistRelay() : Node("twist_relay")
    {
        /// Twist -> TwistStamped
        controller_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/bumperbot_controller/cmd_vel_out",
                10,
                std::bind(
                    &TwistRelay::controllerTwistCallback,
                    this,
                    std::placeholders::_1));

        controller_pub_ =
            this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/bumperbot_controller/cmd_vel",
                10);

        // TwistStamped -> Twist
        joy_sub_ =
            this->create_subscription<geometry_msgs::msg::TwistStamped>(
                "/input_joy/cmd_vel_stamped",
                10,
                std::bind(
                    &TwistRelay::joyTwistCallback,
                    this,
                    std::placeholders::_1));

        joy_pub_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "/input_joy/cmd_vel",
                10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr controller_pub_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joy_pub_;

    void controllerTwistCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::TwistStamped out;

        out.header.stamp = this->get_clock()->now();
        out.header.frame_id = "base_link";
        out.twist = *msg;

        controller_pub_->publish(out);
    }

    void joyTwistCallback(
        const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        geometry_msgs::msg::Twist out;

        out = msg->twist;

        joy_pub_->publish(out);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<TwistRelay>());

    rclcpp::shutdown();

    return 0;
}