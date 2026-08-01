cmake_minimum_required(VERSION 3.8)
project(runner_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AdapterState.msg"
  "msg/EncoderState.msg"
  "msg/KeyboardState.msg"
  "msg/SpeedEnvelopeEntry.msg"
  "msg/SpeedEnvelopeStatus.msg"
  "msg/SystemTelemetry.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
