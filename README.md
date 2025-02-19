# Caso Práctico de ROS2: Topics y Servicios con Mensajes y Servicios Customizados

# Caso Práctico de ROS2: Topics y Servicios con Mensajes y Servicios Customizados

## Descripción
Este repositorio presenta un caso práctico de implementación en **ROS2**, donde se exploran **topics y servicios** utilizando **mensajes y servicios customizados**. La idea es emular el comportamiento de un **carro inteligente** comunicándose con un **semáforo inteligente**, inspirado en el sistema **GLOSA (Green Light Optimal Speed Advisory)**.

El sistema está compuesto por **cuatro nodos**:
- 🚗 **Planificador de Ruta**: Determina la trayectoria del vehículo.
- ⚡ **Gestor de Velocidad**: Regula la velocidad en función del semáforo.
- 🚦 **Manager del Semáforo Inteligente**: Controla el estado del semáforo y envía datos al vehículo.
- 🛠 **Control del Vehículo**: Recibe las instrucciones y ejecuta la conducción.

El sistema incluye **dos mensajes personalizados** y **un servicio**. En la imagen de abajo se ilustra el sistema de manera sencilla.

## Características principales
- ✅ Definición y uso de **mensajes customizados** en topics
- ✅ Implementación de **servicios customizados** para solicitudes específicas
- ✅ Ejemplo práctico con nodos en C++
- ✅ Configuración de paquetes y archivos `.srv` y `.msg`

## Requisitos
- Tener instalado **ROS2 Jazzy**
- Dependencias necesarias especificadas en `package.xml`

## Instalación y Ejecución
1. Clonar el repositorio  de ROS2:
   ```bash
   git clone <URL_DEL_REPOSITORIO>
   cd ~/ros2_caso_practico
   colcon build
   source install/setup.bash
   ```

2. Ejecutar los nodos:
   ```bash
   ros2 run <paquete> <nodo>
   ```

3. Verificar la comunicación de topics:
   ```bash
   ros2 topic list
   ros2 topic echo /nombre_del_topic
   ```

4. Llamar al servicio:
   ```bash
   ros2 service call /nombre_del_servicio <tipo_de_servicio>
   ```

## Licencia
Este proyecto está bajo la licencia Apache 2.0 . Consulta el archivo `LICENSE` para más detalles.
