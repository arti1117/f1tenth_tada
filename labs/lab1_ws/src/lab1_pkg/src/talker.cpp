#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Talker : public rclcpp::Node {
public:
    Talker() : Node("talker") {
        // Declare and get parameters
        this->declare_parameter("v", 0.0);
        this->declare_parameter("d", 0.0);

        v_ = this->get_parameter("v").as_double();
        d_ = this->get_parameter("d").as_double();

        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

        while (rclcpp::ok()) {
            publish_ackermann_drive();
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

private:
    void publish_ackermann_drive() {
        // Get latest parameter values
        v_ = this->get_parameter("v").as_double();
        d_ = this->get_parameter("d").as_double();

        // Create and publish message
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.drive.speed = v_;
        msg.drive.steering_angle = d_;
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published: speed=%.2f, steeering_angle=%.2f", v_, d_);
    }

    double v_;
    double d_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}
