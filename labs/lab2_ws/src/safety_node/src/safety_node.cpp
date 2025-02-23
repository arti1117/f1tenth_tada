#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <vector>
#include <algorithm>
#include <cmath>


class Safety : public rclcpp::Node {
    // The class that handles emergency braking

public:
    Safety() : Node("Safety_node"), speed_(0.0), closest_obstacle_distance_(std::numeric_limits<double>::infinity()), collision_threshold_(0.5) 
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */
       drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
       scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));
       odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&Safety::odom_callback, this, std::placeholders::_1));

       RCLCPP_INFO(this->get_logger(), "Safety node has been started");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        speed_ = msg->twist.twist.linear.x;
        RCLCPP_INFO(this->get_logger(), "Current Speed: %.2f m/s", speed_);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {
        std::vector<float> ranges = msg->ranges;
        std::replace_if(ranges.begin(), ranges.end(), [](float r) { return !std::isfinite(r); }, std::numeric_limits<float>::infinity());

        auto min_element_it = std::min_element(ranges.begin(), ranges.end());
        closest_obstacle_distance_ = (min_element_it != ranges.end()) ? *min_element_it : std::numeric_limits<double>::infinity();
        RCLCPP_INFO(this->get_logger(), "Closest Obstacle Distance: %.2f meters", closest_obstacle_distance_);

        std::vector<double> angles;
        for (size_t i = 0; i < ranges.size(); ++i) {
            angles.push_back(msg->angle_min + i * msg->angle_increment);
        }

        std::vector<double> range_rates;
        for (double angle : angles) {
            range_rates.push_back(speed_ * std::cos(angle));
        }

        std::vector<double> ittc_values;
        for (size_t i = 0; i < ranges.size(); ++i) {
            if (range_rates[i] < 0) {
                ittc_values.push_back(ranges[i] / std::max(-range_rates[i], 1e-6));
            } else {
                ittc_values.push_back(std::numeric_limits<double>::infinity());
            }
        }

        if (std::any_of(ittc_values.begin(), ittc_values.end(), [this](double ttc) { return ttc < collision_threshold_; })) {
            RCLCPP_WARN(this->get_logger(), "COLLISION WARNING: iTTC below threshold! Braking...");
            publish_drive_command(0.0, 0.0);
        }
    }

    void publish_drive_command(double speed, double steering_angle) {
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = steering_angle;
        drive_publisher_->publish(drive_msg);

        RCLCPP_INFO(this->get_logger(), "Published: Speed=%.2f, Steering Angle=%.2f", speed, steering_angle);
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    double speed_;
    double closest_obstacle_distance_;
    double collision_threshold_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Safety>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}