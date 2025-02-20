#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces_personalizadas/msg/evento_vial.hpp"

using namespace std::chrono_literals;

// Nodo "Publicador_simple", encargado de publicar eventos viales, como intersecciones
class PublicadorSimple : public rclcpp::Node
{
public:
  PublicadorSimple()
  : Node("Publicador_simple"), count_(500) // Inicializa el nodo y el contador de distancia
  {
    // Publicador para el topic "Evento_Vial"
    publisher_ = this->create_publisher<interfaces_personalizadas::msg::EventoVial>("Evento_Vial", 10);

    // Callback del temporizador que publica eventos viales periódicamente
    auto timer_callback =
      [this]() -> void {
        auto message = interfaces_personalizadas::msg::EventoVial();
        message.id_evento = "interseccion"; // Definimos el tipo de evento
        message.distancia = this->count_--;  // Decrementa la distancia con cada publicación
        RCLCPP_INFO(this->get_logger(), "Evento publicado: '%s', Distancia: '%f'", 
                    message.id_evento.c_str(), message.distancia);
        this->publisher_->publish(message);  // Publica el mensaje
      };

    // Temporizador que ejecuta el callback cada 500ms
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;  // Temporizador para ejecutar publicaciones periódicas
  rclcpp::Publisher<interfaces_personalizadas::msg::EventoVial>::SharedPtr publisher_;  // Publicador del mensaje de evento vial
  size_t count_;  // Contador de distancia restante para el evento vial
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Crea y mantiene en ejecución el nodo "Publicador_simple"
  rclcpp::spin(std::make_shared<PublicadorSimple>());
  
  rclcpp::shutdown();
  return 0;
}
