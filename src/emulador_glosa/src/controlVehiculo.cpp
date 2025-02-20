#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interfaces_personalizadas/msg/velocidad_meta.hpp"

// Nodo "Control_Vehiculo", encargado de recibir la velocidad objetivo del vehículo
class ControlVehiculo : public rclcpp::Node
{
public:
  ControlVehiculo()
  : Node("Control_Vehiculo") // Nombre del nodo
  {
    // Callback que se ejecutará cuando llegue un mensaje con la velocidad meta
    auto topic_callback =
      [this](interfaces_personalizadas::msg::VelocidadMeta::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "Velocidad meta recibida: '%f'", msg->velocidad);
      };

    // Suscripción al topic "Velocidad_Meta" con una cola de tamaño 10 , recordar {callback(msg)}
    subscription_ =
      this->create_subscription<interfaces_personalizadas::msg::VelocidadMeta>("Velocidad_Meta", 10, topic_callback);
  }

private:
  // Puntero a la suscripción del topic
  rclcpp::Subscription<interfaces_personalizadas::msg::VelocidadMeta>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // Inicializa ROS2
  rclcpp::init(argc, argv);

  // Crea el nodo "Control_Vehiculo" y lo mantiene en ejecución
  rclcpp::spin(std::make_shared<ControlVehiculo>());

  // Apaga ROS2 al finalizar
  rclcpp::shutdown();
  return 0;
}
