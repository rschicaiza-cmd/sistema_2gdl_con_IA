cmake_minimum_required(VERSION 3.8)
project(mensajes_personalizados)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeteccionLider.msg"
  "msg/ErrorVisual.msg"
  "msg/ComandoGimbal.msg"
  "msg/ComandoBase.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
