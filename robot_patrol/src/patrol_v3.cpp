#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <limits>
#include <memory>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("robot_patrol") {
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
    last_scan_ = msg; // store scan for control loop
  }

  void controlLoop() {
    if (!last_scan_)
      return;

    float obstacle_threshold = this->get_parameter("obstacle_threshold").as_double();
    float lin_vel = this->get_parameter("linear_velocity").as_double();
    float ang_vel = this->get_parameter("angular_velocity").as_double();

    auto msg = last_scan_;

    // --- Get indexes for front, left, right ---
    int front_idx = (0.0 - msg->angle_min) / msg->angle_increment;
    int left_idx  = (M_PI/2 - msg->angle_min) / msg->angle_increment;
    int right_idx = (-M_PI/2 - msg->angle_min) / msg->angle_increment;

    front_idx = clampIndex(front_idx, msg->ranges.size());
    left_idx  = clampIndex(left_idx, msg->ranges.size());
    right_idx = clampIndex(right_idx, msg->ranges.size());

    float front_dist = sanitize(msg->ranges[front_idx]);
    float left_dist  = sanitize(msg->ranges[left_idx]);
    float right_dist = sanitize(msg->ranges[right_idx]);

    geometry_msgs::msg::Twist cmd;

    // --- Only move forward if front distance > obstacle_threshold ---
    if (front_dist > obstacle_threshold) {
      cmd.linear.x = lin_vel;
      cmd.angular.z = 0.0;
    } else {
      // Front blocked → choose safer side
      cmd.linear.x = 0.0;
      cmd.angular.z = (left_dist > right_dist ? ang_vel : -ang_vel);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Front blocked (%.2f m < %.2f m)! Turning %s (Left=%.2f, Right=%.2f)",
        front_dist, obstacle_threshold,
        (left_dist > right_dist ? "LEFT" : "RIGHT"),
        left_dist, right_dist);
    }

    publisher_->publish(cmd);
  }

  float sanitize(float dist) {
    if (std::isnan(dist) || std::isinf(dist))
      return std::numeric_limits<float>::max();
    return dist;
  }

  int clampIndex(int idx, size_t size) {
    if (idx < 0) return 0;
    if (idx >= (int)size) return size-1;
    return idx;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto patrol_node = std::make_shared<Patrol>();
  rclcpp::spin(patrol_node);
  rclcpp::shutdown();
  return 0;
}
