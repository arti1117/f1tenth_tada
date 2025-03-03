#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <chrono>
#include <cmath>
#include <vector>

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers
        publisher_ = this->create_publisherd publishers::msg::AckermannDriveStamped(lidarscan_topic_, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(drive_topic_, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));

        // PID gains
        kp_ = 0.7;
        ki_ = 0.01;
        kd_ = 0.1;

        // Initialize error values
        prev_error_ = 0.0;
        integral_error_ = 0.0;
        prev_time_ = this->now();

        // Car parameters
        desired_distance_ = 1.0;
        angle_a_ = 45;
        angle_b_ = 90;
        max_speed_ = 2.0;
        min_speed_ = 0.5;

        angle_min_ = 0.0;
        angle_increment_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "Wall Follow node has been started");
    }

private:
    // Topics
    std::string lidarscan_topic_ = "/scan";
    std::string drive_topic_ = "/drive";

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    double kp_, ki_, kd_;
    double prev_error_, integral_error_;

    rclcpp::Time prev_time_;
    double desired_distance_;
    double angle_a_, angle_b_;
    double max_speed_, min_speed_;
    double angle_min_, angle_increment_;

    double get_radians(double degrees)
    {
        return degrees * M_PI / 180;
    }

    double get_range(float* range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */
        if (angle_increment_ == 0.0)
        {
            return 0.0;
        }

        int angle_index = static_cast<int>((get_radians(angle) - angle_min_) / angle_increment_);
        if 0 <= angle_index and angle_index < (sizeof(range_data) / sizeof(float))
        {
            double range_value = range_data[angle_index];
            return std::isfinite(range_value) ? range_value : std::numeric_limits<double>::infinity();
        }

        return 0.0;
    }

    double get_error(float* range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */
        double dist_a = get_range(range_data, angle_a_);
        double dist_b = get_range(range_data, angle_b_);
        double theta = get_radians(angle_b_ - angle_a_);

        if (dist_b > 0)
        {
            double alpha = atan((dist_a * cos(theta) - dist_b) / (dist_a * sin(theta)));
            double Dt = dist_a * cos(alpha);
            return dist - Dt;
        }

        return 0.0;
    }

    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        auto current_time = this->now();
        double dt = (current_time - prev_time_).seconds();
        prev_time_ = current_time;

        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        double angle = kp * error + ki * integral_ + kd * derivative;
        angle = clamp(angle, -0.3, 0.3);

        double speed = max(min_speed_, min(velocity, max_speed_ - abs(angle) * 5))

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = speed;
        publisher_->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        angle_min_ = scan_msg->angle_min;
        angle_increment_ = scan_msg->angle_increment;

        double error = get_error(scan_msg, desired_distance_);
        double velocity = max(min_speed_, max_speed_ - 0.1 * abs(error) * 2);
        pid_control(error, velocity);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}