#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces_personalizadas/msg/evento_vial.hpp"
#include "interfaces_personalizadas/msg/velocidad_meta.hpp"
#include "interfaces_personalizadas/srv/estado_semaforo.hpp"

// Nodo "Gestor_Velocidad", encargado de ajustar la velocidad meta en función de eventos viales y semáforos
class GestorVelocidad : public rclcpp::Node
{
public:
  GestorVelocidad()
  : Node("Gestor_Velocidad"), velocidadMeta_(50.0) // Inicialización del nodo y velocidad por defecto
  {
    // Callback del temporizador: publica la velocidad meta periódicamente
    auto timer_callback =
      [this]() -> void {
        auto message = interfaces_personalizadas::msg::VelocidadMeta();
        message.velocidad = this->velocidadMeta_;
        RCLCPP_INFO(this->get_logger(), "Velocidad publicada: '%f'", message.velocidad);
        this->publisher_->publish(message);
      };

    // Callback de suscripción: responde a eventos viales
    auto topic_callback =
      [this](interfaces_personalizadas::msg::EventoVial::UniquePtr msg) -> void {
        if (msg->id_evento == "interseccion" && msg->distancia == 400) {
            RCLCPP_WARN(this->get_logger(), "Evento vial detectado: Intersección a 400m");

            auto request = std::make_shared<interfaces_personalizadas::srv::EstadoSemaforo::Request>();
            
            // Llamada asíncrona al servicio de estado del semáforo
            this->client_->async_send_request(
              request,
              [this, distancia = msg->distancia](rclcpp::Client<interfaces_personalizadas::srv::EstadoSemaforo>::SharedFuture future) {
                auto response = future.get();
                this->velocidadMeta_ = calcular_velocidad(distancia, response->tiempo_cambio);
                RCLCPP_INFO(this->get_logger(), "Nueva velocidad calculada: '%f'", this->velocidadMeta_);
              });
        }
      };

    // Suscripción al topic "Evento_Vial"
    subscription_ = this->create_subscription<interfaces_personalizadas::msg::EventoVial>(
        "Evento_Vial", 10, topic_callback);

    // Cliente del servicio "Estado_Semaforo"
    client_ = this->create_client<interfaces_personalizadas::srv::EstadoSemaforo>("Estado_Semaforo");

    // Publicador de la velocidad meta en "Velocidad_Meta"
    publisher_ = this->create_publisher<interfaces_personalizadas::msg::VelocidadMeta>(
        "Velocidad_Meta", 10);

    // Temporizador que ejecuta el callback cada 500ms
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback);
  }

private:
    // Función para calcular la velocidad meta en base a la distancia y el tiempo restante del semáforo
    float calcular_velocidad(float distancia_restante, float tiempo_restante) {
        return (tiempo_restante > 0) ? distancia_restante / tiempo_restante : 0.0;
    }    

    // Variables y punteros a los componentes de ROS2
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces_personalizadas::msg::VelocidadMeta>::SharedPtr publisher_;
    rclcpp::Subscription<interfaces_personalizadas::msg::EventoVial>::SharedPtr subscription_;
    rclcpp::Client<interfaces_personalizadas::srv::EstadoSemaforo>::SharedPtr client_;
    float velocidadMeta_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Crea el nodo "Gestor_Velocidad" y lo mantiene en ejecución
  auto node = std::make_shared<GestorVelocidad>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
