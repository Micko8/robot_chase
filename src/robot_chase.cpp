#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class RobotChase : public rclcpp::Node {
public:
  RobotChase() : Node("robot_chase") {
    target_frame_ =
        this->declare_parameter<std::string>("target_frame", "rick_base_link");
    source_frame_ =
        this->declare_parameter<std::string>("source_frame", "morty_base_link");
    rate_hz_ = this->declare_parameter<double>("rate_hz", 1.0);
    kp_distance_ = this->declare_parameter<double>("kp_distance", 2.0);
    kp_yaw_ = this->declare_parameter<double>("kp_yaw", 2.0);
    offset_xyz_ = this->declare_parameter<std::vector<double>>(
        "offset_xyz", std::vector<double>{1.2, 0.0, 0.0});
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto timer_period =
        std::chrono::duration<double>(1.0 / std::max(rate_hz_, 0.1));
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&RobotChase::lookup_transform, this));

    RCLCPP_INFO(this->get_logger(), "Looking up latest transform: %s -> %s",
                source_frame_.c_str(), target_frame_.c_str());

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/rick/cmd_vel", 10);
  }

private:
  void lookup_transform() {
    geometry_msgs::msg::TransformStamped transform;

    try {
      transform = tf_buffer_->lookupTransform(target_frame_, source_frame_,
                                              tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                  source_frame_.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    const auto &translation = transform.transform.translation;
    const auto &rotation = transform.transform.rotation;

    RCLCPP_INFO(this->get_logger(),
                "%s -> %s | translation [x: %.3f, y: %.3f, z: %.3f] | "
                "rotation [x: %.3f, y: %.3f, z: %.3f, w: %.3f]",
                source_frame_.c_str(), target_frame_.c_str(), translation.x,
                translation.y, translation.z, rotation.x, rotation.y,
                rotation.z, rotation.w);

    const double error_distance =
        std::sqrt(std::pow(translation.x, 2.0) + std::pow(translation.y, 2.0));

    const double error_yaw = std::atan2(translation.y, translation.x);

    RCLCPP_INFO(this->get_logger(),
                "error_distance : %0.3f | error_yaw : %0.3f", error_distance,
                error_yaw);

    auto cmd = geometry_msgs::msg::Twist();

    cmd.linear.x = kp_distance_ * error_distance;
    cmd.angular.z = kp_yaw_ * error_yaw;
    if (cmd.angular.z < 1.5) {
      cmd.angular.z = 0;
    }

    RCLCPP_INFO(this->get_logger(),
                "cmd.linear.x : %0.3f | cmd.angular.z : %0.3f", cmd.linear.x,
                cmd.angular.z);

    publisher_->publish(cmd);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
  std::string source_frame_;
  double rate_hz_;
  double kp_distance_;
  double kp_yaw_;
  std::vector<double> offset_xyz_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();
  return 0;
}
