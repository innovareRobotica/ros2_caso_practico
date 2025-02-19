#include "rclcpp/rclcpp.hpp"
#include "interfaces_personalizadas/srv/estado_semaforo.hpp"

#include <memory>

// Función que maneja la solicitud del servicio
void getstate(const std::shared_ptr<interfaces_personalizadas::srv::EstadoSemaforo::Request> request,
          std::shared_ptr<interfaces_personalizadas::srv::EstadoSemaforo::Response> response)
{ 
  (void)request; // Evita la advertencia de variable no utilizada
  
  // Asigna valores a la respuesta
  response->semaforo_estado = "ROJO";
  response->tiempo_cambio = 50;
  
  // Muestra un mensaje en la consola indicando la respuesta enviada
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enviando respuesta: [%s]", response->semaforo_estado.c_str());
}

int main(int argc, char **argv)
{
  // Inicializa el nodo ROS2
  rclcpp::init(argc, argv);

  // Crea un nodo con el nombre "Manager_Semaforo"
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("Manager_Semaforo");

  // Crea el servicio "Estado_Semaforo" y lo asocia con la función getstate
  rclcpp::Service<interfaces_personalizadas::srv::EstadoSemaforo>::SharedPtr service =
    node->create_service<interfaces_personalizadas::srv::EstadoSemaforo>("Estado_Semaforo", &getstate);

  // Mensaje indicando que el servicio está listo
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Listo para enviar datos:");

  // Mantiene el nodo en ejecución
  rclcpp::spin(node);

  // Cierra el nodo al terminar
  rclcpp::shutdown();
}
