#include "Life_cycle_control/Detection.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace Life_cycle_control
{
using std::placeholders::_1;
using namespace std::chrono_literals;

Detection::Detection()
: Node("yolo_3d_to_tf"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  detection_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "/detections_3d", 10, std::bind(&Detection::generate_tf, this, _1));

  timer_ = create_wall_timer(500ms, std::bind(&Detection::publish_tf, this));

  // Inicializar transformaciones
  transform_.header.frame_id = "base_footprint";
  transform_.child_frame_id = "target";
  reset_transform();
}

void
Detection::generate_tf(const vision_msgs::msg::Detection3DArray &msg)
{
  if (msg.detections.empty()) {
    RCLCPP_WARN(this->get_logger(), "No detections available.");
    reset_transform();
    return;
  }

  if (msg.header.frame_id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Error: header.frame_id is empty!");
    reset_transform();
    return;
  }

  const auto &detection = msg.detections[0];
  const auto &position = detection.bbox.center.position;

  if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
    RCLCPP_WARN(this->get_logger(), "Invalid detection position: NaN or Inf");
    reset_transform();
    return;
  }

  geometry_msgs::msg::TransformStamped opt2target_msg;
  opt2target_msg.header.stamp = now();
  opt2target_msg.header.frame_id = "camera_link";
  opt2target_msg.child_frame_id = "target";
  opt2target_msg.transform.translation.x = position.z;
  opt2target_msg.transform.translation.y = -position.x;
  opt2target_msg.transform.translation.z = -position.y;

  tf2::Transform bf2opt, opt2target;
  std::string error;
  if (tf_buffer_.canTransform("base_footprint", "camera_link", tf2::TimePointZero, &error)) {
    auto bf2opt_msg = tf_buffer_.lookupTransform("base_footprint", "camera_link", tf2::TimePointZero);
    tf2::fromMsg(bf2opt_msg.transform, bf2opt);
    tf2::fromMsg(opt2target_msg.transform, opt2target);
    tf2::Transform bf2target = bf2opt * opt2target;
    transform_.header.stamp = now();
    transform_.header.frame_id = "base_footprint";
    transform_.child_frame_id = "target";
    transform_.transform = tf2::toMsg(bf2target);

    RCLCPP_INFO(this->get_logger(), "Detection transformed to base_footprint [%.2f, %.2f, %.2f]",
                transform_.transform.translation.x,
                transform_.transform.translation.y,
                transform_.transform.translation.z);
  } else {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", error.c_str());
    reset_transform();
  }
}

void
Detection::publish_tf()
{
  if (transform_.header.frame_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "Transform frame_id is empty. Skipping publish.");
    return;
  }

  // Publicar la transformación si es válida
  tf_broadcaster_->sendTransform(transform_);
}

void
Detection::reset_transform()
{
  transform_.transform.translation.x = 0.0;
  transform_.transform.translation.y = 0.0;
  transform_.transform.translation.z = 0.0;
  transform_.header.stamp = now();
}
}  // namespace Life_cycle_control