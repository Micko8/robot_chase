#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class RobotChase : public rclcpp::Node {
public:
  RobotChase() : Node("robot_chase") {
    target_frame_ =
        this->declare_parameter<std::string>("target_frame", "rick_base_link");
    source_frame_ =
        this->declare_parameter<std::string>("source_frame", "morty_base_link");
    rate_hz_ = this->declare_parameter<double>("rate_hz", 30.0);
    kp_distance_ = this->declare_parameter<double>("kp_distance", 2.0);
    kp_yaw_ = this->declare_parameter<double>("kp_yaw", 2.0);
    yaw_tolerance_ = this->declare_parameter<double>("yaw_tolerance", 0.2);
    desired_distance_ =
        this->declare_parameter<double>("desired_distance", 0.6);
    max_angular_speed_ =
        this->declare_parameter<double>("max_angular_speed", 1.5);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/rick/cmd_vel", 10);

    const auto timer_period =
        std::chrono::duration<double>(1.0 / std::max(rate_hz_, 0.1));
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&RobotChase::lookup_transform, this));

    RCLCPP_INFO(this->get_logger(), "Looking up latest transform: %s -> %s",
                source_frame_.c_str(), target_frame_.c_str());
  }

private:
  void lookup_transform() {
    geometry_msgs::msg::TransformStamped transform;

    try {
      transform = tf_buffer_->lookupTransform(target_frame_, source_frame_,
                                              tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Could not transform %s to %s: %s",
                           source_frame_.c_str(), target_frame_.c_str(),
                           ex.what());
      return;
    }

    const auto &translation = transform.transform.translation;
    const double distance = std::hypot(translation.x, translation.y);
    const double error_distance = distance - desired_distance_;
    const double error_yaw = std::atan2(translation.y, translation.x);

    auto cmd = geometry_msgs::msg::Twist();

    if (std::abs(error_yaw) < yaw_tolerance_) {
      cmd.linear.x = kp_distance_ * error_distance;
    } else {
      cmd.linear.x = 0.0;
    }

    if (std::abs(error_yaw) > 0.1) {
      cmd.angular.z = kp_yaw_ * error_yaw;
    } else {
      cmd.angular.z = 0.0;
    }

    publisher_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "distance: %.3f | error_distance: %.3f | error_yaw: %.3f | "
        "cmd.linear.x: %.3f | cmd.angular.z: %.3f",
        distance, error_distance, error_yaw, cmd.linear.x, cmd.angular.z);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
  std::string source_frame_;
  double rate_hz_;
  double kp_distance_;
  double kp_yaw_;
  double desired_distance_;
  double max_angular_speed_;
  double yaw_tolerance_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();
  return 0;
}
