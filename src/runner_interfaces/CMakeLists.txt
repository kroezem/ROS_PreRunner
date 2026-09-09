cmake_minimum_required(VERSION 3.8)
project(runner_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavigationRequest.msg"
  "msg/NavigationState.msg"
  "msg/StopRequest.msg"
  "msg/StopState.msg"
  "msg/AdapterState.msg"
  "msg/EncoderState.msg"
  "msg/KeyboardState.msg"
  "msg/LocalControlState.msg"
  "msg/CommandAuthorityState.msg"
  "msg/ConvertedCommand.msg"
  "msg/MapCatalogEntry.msg"
  "msg/MapRequest.msg"
  "msg/MapState.msg"
  "msg/ModeRequest.msg"
  "msg/ModeState.msg"
  "msg/ManualDemand.msg"
  "msg/PaddockControlEvent.msg"
  "msg/PaddockControlLease.msg"
  "msg/RecordingEntry.msg"
  "msg/RecordingRequest.msg"
  "msg/RecordingState.msg"
  "msg/SpeedEnvelopeEntry.msg"
  "msg/SpeedEnvelopeStatus.msg"
  "msg/SystemTelemetry.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
