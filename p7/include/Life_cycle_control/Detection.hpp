#ifndef LIFE_CYCLE_CONTROL__DETECTION_HPP_
#define LIFE_CYCLE_CONTROL__DETECTION_HPP_

// Incluir bibliotecas necesarias para el nodo, suscripciones, publicaciones y transformaciones
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "yolo_msgs/msg/detection_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/transform_broadcaster.h>

namespace Life_cycle_control
{

// Clase principal para gestionar las detecciones y publicar transformaciones
class Detection : public rclcpp::Node
{
public:
  // Constructor de la clase
  Detection();

private:
  // Suscripción a las detecciones 3D
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
  
  // Publicación de velocidades o comandos de movimiento
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  
  // Transmisor de transformaciones para transmitir las transformaciones de coordenadas
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Variables para almacenar transformaciones
  geometry_msgs::msg::TransformStamped transform_;
  geometry_msgs::msg::TransformStamped last_transform_;
  
  // Temporizador para publicar en intervalos regulares
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Buffer de transformaciones para gestionar las transformaciones entre marcos
  tf2::BufferCore tf_buffer_;
  
  // Listener para recibir transformaciones
  tf2_ros::TransformListener tf_listener_;

  // Función para generar la transformación a partir de la detección 3D
  void generate_tf(const vision_msgs::msg::Detection3DArray &msg);

  // Función para publicar la transformación en el sistema de coordenadas
  void publish_tf();
  
  // Función para restablecer la transformación en caso de error
  void reset_transform();
};

}  // namespace Life_cycle_control

#endif  //  LIFE_CYCLE_CONTROL__DETECTION_HPP_

