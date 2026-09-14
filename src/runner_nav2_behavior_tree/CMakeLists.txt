cmake_minimum_required(VERSION 3.8)
project(runner_nav2_behavior_tree)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(runner_nav2_behavior_tree_nodes SHARED
  src/commitment_nodes.cpp)
target_include_directories(runner_nav2_behavior_tree_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(runner_nav2_behavior_tree_nodes PUBLIC cxx_std_17)
# Required by behaviortree_cpp's BT_REGISTER_NODES macro: without it,
# BT_RegisterNodesFromPlugin compiles with internal linkage and
# BT::BehaviorTreeFactory::registerFromPlugin cannot dlsym it, so none of
# this plugin's custom node types (PathExists, CandidatePathValid, etc.)
# are ever registered.
target_compile_definitions(runner_nav2_behavior_tree_nodes PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(runner_nav2_behavior_tree_nodes
  behaviortree_cpp
  geometry_msgs
  nav2_behavior_tree
  nav2_msgs
  nav2_util
  nav_msgs
  rclcpp
  std_msgs
  tf2_ros)

install(TARGETS runner_nav2_behavior_tree_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  ament_add_gtest(test_blockage_persistence test/test_blockage_persistence.cpp)
  target_include_directories(test_blockage_persistence PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(runner_nav2_behavior_tree_nodes)
ament_package()
