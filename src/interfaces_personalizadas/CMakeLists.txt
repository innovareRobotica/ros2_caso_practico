cmake_minimum_required(VERSION 3.8)
project(interfaces_personalizadas)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Encontrar dependencias del nodo
find_package(rclcpp REQUIRED)

# Agregar ejecutables planificador ruta 
add_executable(publicadorSimple src/publicadorSimple.cpp)
ament_target_dependencies(publicadorSimple rclcpp)

# Agregar para poder generar mensajes
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EventoVial.msg"
  "msg/VelocidadMeta.msg"
  "srv/EstadoSemaforo.srv"
)

# Requerido para usar mensajes en el mismo package, antigua version
# rosidl_target_interfaces(publicadorSimple
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Obtener el tipo de soporte para el mensaje
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Vincular el tipo de soporte con el nodo
target_link_libraries(publicadorSimple "${cpp_typesupport_target}")

# Agregar ejecutables al proyecto
install(TARGETS
  publicadorSimple
  DESTINATION lib/${PROJECT_NAME})

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
