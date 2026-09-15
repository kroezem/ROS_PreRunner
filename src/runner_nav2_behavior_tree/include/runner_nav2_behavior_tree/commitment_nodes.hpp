// Copyright 2026 matti
// Licensed under the Apache License, Version 2.0

#ifndef RUNNER_NAV2_BEHAVIOR_TREE__COMMITMENT_NODES_HPP_
#define RUNNER_NAV2_BEHAVIOR_TREE__COMMITMENT_NODES_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "runner_interfaces/msg/path_speed_profile.hpp"
#include "runner_interfaces/srv/set_path_speed_profile.hpp"
#include "runner_path_speed_profile/path_speed_profile.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

#include "runner_nav2_behavior_tree/blockage_persistence.hpp"

namespace runner_nav2_behavior_tree
{

class PathExistsCondition : public BT::ConditionNode
{
public:
  PathExistsCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CandidatePathValidCondition : public BT::ConditionNode
{
public:
  CandidatePathValidCondition(
    const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client_;
  std::chrono::milliseconds server_timeout_;
};

class PersistentPathValidCondition : public BT::ConditionNode
{
public:
  PersistentPathValidCondition(
    const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool makeForwardCorridor(
    const nav_msgs::msg::Path & path, double corridor_length,
    double max_progress_search_distance, const std::string & robot_frame,
    double transform_tolerance, nav_msgs::msg::Path & corridor);
  void publish(const std::string & event, const std::string & reason);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::chrono::milliseconds server_timeout_;
  BlockagePersistence persistence_;
  nav_msgs::msg::Path committed_path_;
  std::size_t closest_index_{0};
  bool validation_unavailable_reported_{false};
};

class ReportPathCommitment : public BT::SyncActionNode
{
public:
  ReportPathCommitment(
    const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class GeneratePathSpeedProfile : public BT::SyncActionNode
{
public:
  GeneratePathSpeedProfile(
    const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr controller_parameter_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr policy_parameter_client_;
  rclcpp::Client<runner_interfaces::srv::SetPathSpeedProfile>::SharedPtr profile_client_;
  rclcpp::Publisher<runner_interfaces::msg::PathSpeedProfile>::SharedPtr publisher_;
  std::chrono::milliseconds server_timeout_;
};

}  // namespace runner_nav2_behavior_tree

#endif  // RUNNER_NAV2_BEHAVIOR_TREE__COMMITMENT_NODES_HPP_
