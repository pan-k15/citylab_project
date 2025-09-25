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
        500ms, std::bind(&Patrol::timer_callback, this));

    // Parameters
    this->declare_parameter("obstacle_threshold", 0.35); // meters
    this->declare_parameter("angular_velocity", 0.2);    // rad/s
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float obstacle_threshold = this->get_parameter("obstacle_threshold").as_double();

    // Index for front ray
    int front_idx = (0.0 - msg->angle_min) / msg->angle_increment;
    if (front_idx < 0 || front_idx >= (int)msg->ranges.size())
      return;

    float front_dist = msg->ranges[front_idx];
    if (std::isnan(front_dist) || std::isinf(front_dist)) {
      front_dist = std::numeric_limits<float>::max();
    }

    if (front_dist > obstacle_threshold) {
      // Safe → keep moving
      move_robot();
      return;
    }

    // --- Obstacle detected: find safest direction ---
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

    // Index → angle
    float best_angle = msg->angle_min + best_idx * msg->angle_increment;

    // Clamp to [-π/2, π/2]
    if (best_angle < -M_PI_2) best_angle = -M_PI_2;
    if (best_angle >  M_PI_2) best_angle =  M_PI_2;

    // Store in class variable
    direction_ = best_angle;

    RCLCPP_INFO(get_logger(), 
                "Obstacle detected. Safest direction: %.3f rad, distance: %.2f m",
                direction_, max_dist);

    // Rotate toward safest direction (simple left/right decision)
    stop_robot();
    turn(direction_ > 0 ? 0.3 : -0.3);
    rclcpp::sleep_for(std::chrono::milliseconds(300));
    stop_robot();
  }

  void timer_callback() {
    std::cout << "Patrol: Robot is moving..." << std::endl;
  }

  void move_robot() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.1;
    message.angular.z = 0.0;
    publisher_->publish(message);
  }

  void stop_robot() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher_->publish(message);
  }

  void turn(float angular_velocity) {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = angular_velocity;
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;

  // Safest direction (rad, -π/2 to π/2)
  float direction_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto patrol_node = std::make_shared<Patrol>();
  rclcpp::spin(patrol_node);
  rclcpp::shutdown();
  return 0;
}
