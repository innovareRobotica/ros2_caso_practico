from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo que planifica la ruta del vehículo
        Node(
            package='emulador_glosa',             # Nombre del paquete 
            namespace='sistema1',                 # Namespace para diferenciar el vehículo 1
            executable='planificadorRuta',        # Nombre del ejecutable de tu nodo
            name='Planificador_Ruta'              # Nombre del nodo para el vehículo 1
        ),
        
        # Nodo que controla el movimiento del primer vehículo
        Node(
            package='emulador_glosa',             # Nombre del paquete 
            namespace='sistema1',                 # Namespace para diferenciar el vehículo 1
            executable='controlVehiculo',         # Nombre del ejecutable de tu nodo
            name='Control_Vehiculo'               # Nombre del nodo para el vehículo 1
        ),

        # Nodo que maneja el estado de los semáforos para el primer vehículo
        Node(
            package='emulador_glosa',             # Nombre del paquete 
            namespace='sistema1',                 # Namespace para diferenciar el vehículo 1
            executable='managerSemaforo',         # Nombre del ejecutable de tu nodo
            name='Manager_Semaforo'               # Nombre del nodo para la gestión del semáforo
        ),

        # Nodo que gestiona la velocidad del primer vehículo
        Node(
            package='emulador_glosa',             # Nombre del paquete
            namespace='sistema1',                 # Namespace para diferenciar el vehículo 1
            executable='gestorVelocidad',         # Nombre del ejecutable de tu nodo
            name='Gestor_Velocidad'               # Nombre del nodo para el control de velocidad
        ),
    ])
