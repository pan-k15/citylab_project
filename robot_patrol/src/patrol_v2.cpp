#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("robot_patrol"), direction_(0.0f) {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::QoS(10),
        std::bind(&Patrol::laserCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        200ms, std::bind(&Patrol::controlLoop, this));

    // Parameters
    this->declare_parameter("obstacle_threshold", 0.35); // meters
    this->declare_parameter("linear_velocity", 0.15);    // m/s
    this->declare_parameter("angular_velocity", 0.4);    // rad/s
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_ = msg; // store the scan for control loop
  }

  void controlLoop() {
    if (!last_scan_)
      return;

    float obstacle_threshold = this->get_parameter("obstacle_threshold").as_double();
    float lin_vel = this->get_parameter("linear_velocity").as_double();
    float ang_vel = this->get_parameter("angular_velocity").as_double();

    auto msg = last_scan_;

    // Index for front ray
    int front_idx = (0.0 - msg->angle_min) / msg->angle_increment;
    if (front_idx < 0 || front_idx >= (int)msg->ranges.size())
      return;

    float front_dist = msg->ranges[front_idx];
    if (std::isnan(front_dist) || std::isinf(front_dist)) {
      front_dist = std::numeric_limits<float>::max();
    }

    geometry_msgs::msg::Twist cmd;

    if (front_dist > obstacle_threshold) {
      // clear path → go forward
      cmd.linear.x = lin_vel;
      cmd.angular.z = 0.0;
    } else {
      // obstacle ahead → pick safest direction
      int start_idx = (-M_PI / 2 - msg->angle_min) / msg->angle_increment;
      int end_idx   = (M_PI / 2 - msg->angle_min) / msg->angle_increment;

      start_idx = std::max(0, start_idx);
      end_idx   = std::min((int)msg->ranges.size() - 1, end_idx);

      float max_dist = 0.0;
      int best_idx = front_idx;

      for (int i = start_idx; i <= end_idx; i++) {
        float dist = msg->ranges[i];
        if (!std::isinf(dist) && !std::isnan(dist)) {
          if (dist > max_dist) {
            max_dist = dist;
            best_idx = i;
          }
        }
      }

      float best_angle = msg->angle_min + best_idx * msg->angle_increment;

      // Store in class variable
      direction_ = best_angle;

      // turn proportionally toward safe direction
      cmd.linear.x = 0.0;
      cmd.angular.z = (direction_ > 0 ? ang_vel : -ang_vel);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "Obstacle ahead! Turning to %.2f rad (%.2f m free space)", 
          direction_, max_dist);
    }

    publisher_->publish(cmd);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  float direction_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto patrol_node = std::make_shared<Patrol>();
  rclcpp::spin(patrol_node);
  rclcpp::shutdown();
  return 0;
}
