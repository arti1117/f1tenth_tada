#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class Relay : public rclcpp::Node
{
public:
  Relay() : Node("relay")
  {
    // Create publisher
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_relay", 10);

    // Create subscriber
    drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", 10, std::bind(&Relay::callback, this, std::placeholders::_1));
  }

private:
  void callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    // Create new message with modified values
    auto new_msg = ackermann_msgs::msg::AckermannDriveStamped();
    new_msg.drive.speed = msg->drive.speed * 3.0;
    new_msg.drive.steering_angle = msg->drive.steering_angle * 3.0;

    // Publish modified message
    drive_pub_->publish(new_msg);
    RCLCPP_INFO(this->get_logger(), "Relay: speed=%.2f, steering=%.2f", 
                new_msg.drive.speed, new_msg.drive.steering_angle);
  }

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Relay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 