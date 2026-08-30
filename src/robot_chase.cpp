#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
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
    rate_hz_ = this->declare_parameter<double>("rate_hz", 1.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
  std::string source_frame_;
  double rate_hz_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();
  return 0;
}
