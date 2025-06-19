#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
  Talker() : Node("talker")
  {
    // Declare parameters
    this->declare_parameter("v", 1.0);
    this->declare_parameter("d", 0.0);

    // Create publisher
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    // Create timer
    timer_ = this->create_wall_timer(100ms, std::bind(&Talker::publish_drive, this));
  }

private:
  void publish_drive()
  {
    // Get parameters
    double v = this->get_parameter("v").as_double();
    double d = this->get_parameter("d").as_double();

    // Create message
    auto msg = ackermann_msgs::msg::AckermannDriveStamped();
    msg.drive.speed = v;
    msg.drive.steering_angle = d;

    // Publish message
    drive_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: speed=%.2f, steering=%.2f", v, d);
  }

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Talker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 