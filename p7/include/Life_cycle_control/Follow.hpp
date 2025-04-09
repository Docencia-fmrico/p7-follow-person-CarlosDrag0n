#ifndef LIFE_CYCLE_CONTROL__FOLLOW_HPP_
#define LIFE_CYCLE_CONTROL__FOLLOW_HPP_

// Incluir bibliotecas necesarias para el nodo, suscripciones, publicaciones, y transformaciones
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

#include "Life_cycle_control/PIDController.hpp"  // Controlador PID personalizado

namespace Life_cycle_control
{

// Clase que maneja el ciclo de vida y el control para seguir un objetivo
class Follow : public rclcpp_lifecycle::LifecycleNode
{
public:
  // Definir las macros de punteros inteligentes para la clase Follow
  RCLCPP_SMART_PTR_DEFINITIONS(Follow)

  // Alias para la devolución de llamada de los estados del ciclo de vida
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Constructor de la clase Follow
  Follow();

  // Métodos para manejar los estados del ciclo de vida del nodo
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state);

private:
  // Publicador de velocidades (Twist) para controlar el movimiento del robot
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  // Temporizador para ejecutar ciclos de control de forma periódica
  rclcpp::TimerBase::SharedPtr timer_;

  // Buffer y listener para gestionar transformaciones entre marcos de referencia
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Distancia anterior, utilizada para comparar el progreso del seguimiento
  double last_distance_;

  // Ciclo de control para generar velocidades de seguimiento
  void control_cycle();

  // Controladores PID para velocidad lineal (vlin_pid_) y rotacional (vrot_pid_)
  PIDController vlin_pid_, vrot_pid_;

};

}  // namespace Life_cycle_control

#endif  // LIFE_CYCLE_CONTROL__FOLLOW_HPP_
