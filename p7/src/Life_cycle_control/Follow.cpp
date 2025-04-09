#include "Life_cycle_control/Follow.hpp"
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

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

Follow::Follow()
: LifecycleNode("Life_cycle_control"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  vlin_pid_(0.0, 1.0, 0.0, 0.7), // PID para velocidad lineal
  vrot_pid_(0.0, 1.0, 0.3, 1.0)  // PID para velocidad angular
{
  // Publicador para enviar comandos de velocidad
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

CallbackReturn
Follow::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Configuring...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
Follow::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Activating...");

  // Timer que ejecutará el ciclo de control cada 100ms
  timer_ = create_wall_timer(100ms, std::bind(&Follow::control_cycle, this));

  // Activar el publicador de velocidad
  vel_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
Follow::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Deactivating...");

  // Detener el timer y desactivar el publicador de velocidad
  timer_ = nullptr;
  vel_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
Follow::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Cleaning Up...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
Follow::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Shutting Down...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
Follow::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Error State");
  return CallbackReturn::SUCCESS;
}

void
Follow::control_cycle()
{
  tf2::Stamped<tf2::Transform> bf2target;
  std::string error;

  // Intentar obtener la transformación entre "base_footprint" y "target"
  if (tf_buffer_.canTransform("base_footprint", "target", tf2::TimePointZero, &error)) {
    auto bf2target_msg = tf_buffer_.lookupTransform("base_footprint", "target", tf2::TimePointZero);
    tf2::fromMsg(bf2target_msg, bf2target);

    double x = bf2target.getOrigin().x();  // Posición X de "target"
    double y = bf2target.getOrigin().y();  // Posición Y de "target"

    // Calcular ángulo y distancia al objetivo
    double angle = atan2(y, x);
    double dist = sqrt(x * x + y * y);

    double vel_rot = 0.0;
    double vel_lin = 0.0;

    RCLCPP_INFO(get_logger(), "Distancias al objetivo: %f X: %f Y: %f", dist, x, y);

    // Calcular velocidad angular con PID
    vel_rot = std::clamp(vrot_pid_.get_output(angle), -1.5, 1.5);

    // Control de avance o retroceso
    if (dist > 1.0) {
      vel_lin = std::clamp(vlin_pid_.get_output(dist - 1.0), -0.3, 0.3);
    } else if (dist < 0.8) {
      vel_lin = 0.0; // Retrocede si está demasiado cerca
    } else {
      vel_lin = 0.0;  // No moverse si está en el rango ideal
    }

    geometry_msgs::msg::Twist vel;
    vel.linear.x = vel_lin;
    vel.angular.z = vel_rot;

    RCLCPP_INFO(get_logger(), "Vlin: %f Vrot: %f", vel_lin, vel_rot);

    // Si la distancia ha cambiado drásticamente, detener el robot
    if (abs(last_distance_ - dist) > 1.5) {
      vel.linear.x = 0.0;
      vel.angular.z = 0.0;
    }

    vel_pub_->publish(vel);

    // Guardar la última distancia
    last_distance_ = dist;

  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Error en TF odom -> base_footprint [<< " << error << "]");
  }
}

}  // namespace Life_cycle_control
