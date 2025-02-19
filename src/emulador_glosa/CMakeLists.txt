cmake_minimum_required(VERSION 3.8)
project(emulador_glosa)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Encontrar dependencias del nodo
find_package(rclcpp REQUIRED)
find_package(interfaces_personalizadas REQUIRED)

# Agregar ejecutables planificador ruta 
add_executable(planificadorRuta src/planificadorRuta.cpp)
ament_target_dependencies(planificadorRuta rclcpp interfaces_personalizadas)

# Agregar ejecutables control vehiculo
add_executable(controlVehiculo src/controlVehiculo.cpp)
ament_target_dependencies(controlVehiculo rclcpp interfaces_personalizadas)

# Agregar ejecutables manager semaforo
add_executable(managerSemaforo src/managerSemaforo.cpp)
ament_target_dependencies(managerSemaforo rclcpp interfaces_personalizadas)

# Agregar ejecutables gestor velocidad
add_executable(gestorVelocidad src/gestorVelocidad.cpp)
ament_target_dependencies(gestorVelocidad rclcpp interfaces_personalizadas)

# Agregar ejecutables al proyecto
install(TARGETS
  planificadorRuta
  controlVehiculo
  managerSemaforo
  gestorVelocidad
  DESTINATION lib/${PROJECT_NAME})

# Instalar launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
