// Copyright 2026 matti
// Licensed under the Apache License, Version 2.0

#include "runner_nav2_behavior_tree/commitment_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/json_export.h"
#include "nav2_behavior_tree/json_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace runner_nav2_behavior_tree
{

namespace
{

double distance(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs)
{
  return std::hypot(
    lhs.pose.position.x - rhs.pose.position.x,
    lhs.pose.position.y - rhs.pose.position.y);
}

rclcpp::QoS eventQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
}

runner_path_speed_profile::ProfileConfig readSpeedPolicyParameters(
  const rcl_interfaces::srv::GetParameters::Response & response)
{
  runner_path_speed_profile::ProfileConfig config;
  if (response.values.size() != 9u || std::any_of(
      response.values.begin(), response.values.end(), [](const auto & value) {
        return value.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      }))
  {
    return config;
  }
  config.creep_speed = response.values[0].double_value;
  config.clearance_half_speed = response.values[1].double_value;
  config.curvature_window = response.values[2].double_value;
  config.max_lateral_acceleration = response.values[3].double_value;
  config.footprint_radius = response.values[4].double_value;
  config.braking_linear = response.values[5].double_value;
  config.braking_constant = response.values[6].double_value;
  config.reaction_time_s = response.values[7].double_value;
  config.recovery_acceleration = response.values[8].double_value;
  return config;
}

}  // namespace

BT::PortsList PathExistsCondition::providedPorts()
{
  BT::RegisterJsonDefinition<nav_msgs::msg::Path>();
  return {BT::InputPort<nav_msgs::msg::Path>("path", "Committed path")};
}

BT::NodeStatus PathExistsCondition::tick()
{
  nav_msgs::msg::Path path;
  return getInput("path", path) && !path.poses.empty() ?
         BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

CandidatePathValidCondition::CandidatePathValidCondition(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  client_ = node_->create_client<nav2_msgs::srv::IsPathValid>("is_path_valid");
  server_timeout_ =
    config.blackboard->get<std::chrono::milliseconds>("server_timeout");
}

BT::PortsList CandidatePathValidCondition::providedPorts()
{
  BT::RegisterJsonDefinition<nav_msgs::msg::Path>();
  BT::RegisterJsonDefinition<std::chrono::milliseconds>();
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "Candidate path"),
    BT::InputPort<std::string>("global_frame", "map", "Planning frame"),
    BT::InputPort<std::string>("robot_frame", "base_link", "Robot frame"),
    BT::InputPort<double>("transform_tolerance", 0.3, "TF tolerance in seconds"),
    BT::InputPort<std::chrono::milliseconds>("server_timeout"),
    BT::OutputPort<std::string>("rejection_reason", "Candidate rejection reason")
  };
}

BT::NodeStatus CandidatePathValidCondition::tick()
{
  nav_msgs::msg::Path path;
  std::string global_frame;
  std::string robot_frame;
  double transform_tolerance;
  getInput("path", path);
  getInput("global_frame", global_frame);
  getInput("robot_frame", robot_frame);
  getInput("transform_tolerance", transform_tolerance);
  getInput("server_timeout", server_timeout_);

  if (path.poses.empty()) {
    setOutput("rejection_reason", std::string("candidate_empty"));
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_buffer_, global_frame, robot_frame, transform_tolerance))
  {
    setOutput("rejection_reason", std::string("candidate_current_pose_unavailable"));
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
  request->path = path;
  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    setOutput("rejection_reason", std::string("candidate_validation_unavailable"));
    return BT::NodeStatus::FAILURE;
  }
  if (!future.get()->is_valid) {
    setOutput("rejection_reason", std::string("candidate_invalid"));
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

PersistentPathValidCondition::PersistentPathValidCondition(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  client_ = node_->create_client<nav2_msgs::srv::IsPathValid>("is_path_valid");
  publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "/navigation/path_commitment_state", eventQos());
  server_timeout_ =
    config.blackboard->get<std::chrono::milliseconds>("server_timeout");
}

BT::PortsList PersistentPathValidCondition::providedPorts()
{
  BT::RegisterJsonDefinition<nav_msgs::msg::Path>();
  BT::RegisterJsonDefinition<std::chrono::milliseconds>();
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "Committed path"),
    BT::InputPort<double>("corridor_length", 1.25, "Forward corridor length in metres"),
    BT::InputPort<unsigned int>(
      "required_observations", 3, "Consecutive lethal observations required"),
    BT::InputPort<double>(
      "max_progress_search_distance", 2.0,
      "Maximum forward path distance searched for current progress"),
    BT::InputPort<std::string>("robot_frame", "base_link", "Robot frame"),
    BT::InputPort<double>("transform_tolerance", 0.3, "TF tolerance in seconds"),
    BT::InputPort<std::chrono::milliseconds>("server_timeout"),
    BT::OutputPort<std::string>("replan_reason", "Reason for an authorized replan")
  };
}

BT::NodeStatus PersistentPathValidCondition::tick()
{
  nav_msgs::msg::Path path;
  getInput("path", path);
  if (path.poses.empty()) {
    committed_path_ = path;
    closest_index_ = 0;
    persistence_.reset();
    return BT::NodeStatus::FAILURE;
  }

  if (path != committed_path_) {
    committed_path_ = path;
    closest_index_ = 0;
    persistence_.reset();
    validation_unavailable_reported_ = false;
  }

  double corridor_length;
  double max_progress_search_distance;
  double transform_tolerance;
  std::string robot_frame;
  unsigned int required_observations;
  getInput("corridor_length", corridor_length);
  getInput("max_progress_search_distance", max_progress_search_distance);
  getInput("transform_tolerance", transform_tolerance);
  getInput("robot_frame", robot_frame);
  getInput("required_observations", required_observations);
  getInput("server_timeout", server_timeout_);

  nav_msgs::msg::Path corridor;
  if (!makeForwardCorridor(
      path, corridor_length, max_progress_search_distance, robot_frame,
      transform_tolerance, corridor))
  {
    if (!validation_unavailable_reported_) {
      publish("committed_path_retained", "progress_pose_unavailable");
      validation_unavailable_reported_ = true;
    }
    return BT::NodeStatus::SUCCESS;
  }

  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
  request->path = std::move(corridor);
  auto future = client_->async_send_request(request);
  const auto spin_result = rclcpp::spin_until_future_complete(node_, future, server_timeout_);
  if (spin_result != rclcpp::FutureReturnCode::SUCCESS) {
    if (!validation_unavailable_reported_) {
      publish("committed_path_retained", "global_validation_unavailable");
      validation_unavailable_reported_ = true;
    }
    return BT::NodeStatus::SUCCESS;
  }
  validation_unavailable_reported_ = false;

  const auto response = future.get();
  const auto before = persistence_.blockedObservations();
  const bool persistent = persistence_.observe(
    response->is_valid, required_observations);
  if (response->is_valid) {
    if (before > 0) {
      publish("committed_path_retained", "transient_blockage_cleared");
    }
    return BT::NodeStatus::SUCCESS;
  }

  if (!persistent) {
    std::ostringstream reason;
    reason << "transient_blockage_" << persistence_.blockedObservations() << "_of_" <<
      std::max(1u, required_observations);
    publish("committed_path_retained", reason.str());
    return BT::NodeStatus::SUCCESS;
  }

  setOutput("replan_reason", std::string("persistent_blockage"));
  if (before < std::max(1u, required_observations)) {
    publish("replan_requested", "persistent_blockage");
  }
  return BT::NodeStatus::FAILURE;
}

bool PersistentPathValidCondition::makeForwardCorridor(
  const nav_msgs::msg::Path & path, const double corridor_length,
  const double max_progress_search_distance, const std::string & robot_frame,
  const double transform_tolerance, nav_msgs::msg::Path & corridor)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_buffer_, path.header.frame_id, robot_frame, transform_tolerance))
  {
    return false;
  }

  const auto & poses = path.poses;
  std::size_t search_end = closest_index_ + 1;
  double searched = 0.0;
  while (search_end < poses.size() && searched < max_progress_search_distance) {
    searched += distance(poses[search_end - 1], poses[search_end]);
    ++search_end;
  }

  auto closest = closest_index_;
  auto closest_distance = distance(robot_pose, poses[closest]);
  for (std::size_t index = closest_index_ + 1; index < search_end; ++index) {
    const auto candidate_distance = distance(robot_pose, poses[index]);
    if (candidate_distance < closest_distance) {
      closest = index;
      closest_distance = candidate_distance;
    }
  }
  closest_index_ = closest;

  std::size_t corridor_end = closest + 1;
  double corridor_distance = 0.0;
  while (corridor_end < poses.size() && corridor_distance < corridor_length) {
    corridor_distance += distance(poses[corridor_end - 1], poses[corridor_end]);
    ++corridor_end;
  }

  corridor.header = path.header;
  corridor.poses.assign(poses.begin() + closest, poses.begin() + corridor_end);
  return true;
}

void PersistentPathValidCondition::publish(
  const std::string & event, const std::string & reason)
{
  std_msgs::msg::String message;
  message.data = "event=" + event + " reason=" + reason;
  publisher_->publish(message);
}

ReportPathCommitment::ReportPathCommitment(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  publisher_ = node->create_publisher<std_msgs::msg::String>(
    "/navigation/path_commitment_state", eventQos());
}

BT::PortsList ReportPathCommitment::providedPorts()
{
  return {
    BT::InputPort<std::string>("event", "Commitment event"),
    BT::InputPort<std::string>("reason", "Commitment reason")
  };
}

BT::NodeStatus ReportPathCommitment::tick()
{
  std::string event;
  std::string reason;
  if (!getInput("event", event) || !getInput("reason", reason)) {
    return BT::NodeStatus::FAILURE;
  }
  std_msgs::msg::String message;
  message.data = "event=" + event + " reason=" + reason;
  publisher_->publish(message);
  return BT::NodeStatus::SUCCESS;
}

GeneratePathSpeedProfile::GeneratePathSpeedProfile(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  server_timeout_ = config.blackboard->get<std::chrono::milliseconds>("server_timeout");
  costmap_subscriber_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
    rclcpp::Node::WeakPtr(node_), "/global_costmap/costmap_raw");
  controller_parameter_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>(
    "/controller_server/get_parameters");
  policy_parameter_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>(
    "/bt_navigator/get_parameters");
  profile_client_ = node_->create_client<runner_interfaces::srv::SetPathSpeedProfile>(
    "/controller_server/FollowPath/set_path_speed_profile");
  publisher_ = node_->create_publisher<runner_interfaces::msg::PathSpeedProfile>(
    "/navigation/path_speed_profile", eventQos());
}

BT::PortsList GeneratePathSpeedProfile::providedPorts()
{
  BT::RegisterJsonDefinition<nav_msgs::msg::Path>();
  BT::RegisterJsonDefinition<std::chrono::milliseconds>();
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "Newly committed path"),
    BT::InputPort<std::chrono::milliseconds>("server_timeout")
  };
}

BT::NodeStatus GeneratePathSpeedProfile::tick()
{
  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Path speed profile fallback: committed path unavailable");
    return BT::NodeStatus::SUCCESS;
  }

  getInput("server_timeout", server_timeout_);

  runner_path_speed_profile::ProfileConfig config;
  auto policy_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  policy_request->names = {
    "speed_policy.creep_speed", "speed_policy.clearance_half_speed",
    "speed_policy.curvature_window", "speed_policy.max_lateral_acceleration",
    "speed_policy.footprint_radius",
    "speed_policy.braking_linear", "speed_policy.braking_constant",
    "speed_policy.reaction_time_s",
    "speed_policy.recovery_acceleration"};
  auto policy_future = policy_parameter_client_->async_send_request(policy_request);
  if (rclcpp::spin_until_future_complete(node_, policy_future, server_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    config = readSpeedPolicyParameters(*policy_future.get());
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "Path speed profile policy unavailable from /bt_navigator; using committed defaults");
  }

  double preset_ceiling = config.creep_speed;
  bool preset_available = false;
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names.push_back("FollowPath.desired_linear_vel");
  auto future = controller_parameter_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, server_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    const auto response = future.get();
    if (response->values.size() == 1u &&
      response->values.front().type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
    {
      preset_ceiling = response->values.front().double_value;
      preset_available = std::isfinite(preset_ceiling) && preset_ceiling >= config.creep_speed;
    }
  }

  std::vector<double> clearance;
  bool costmap_available = false;
  try {
    const auto costmap = costmap_subscriber_->getCostmap();
    if (costmap) {
      clearance = runner_path_speed_profile::costmapClearance(
        path, *costmap, config.footprint_radius);
      costmap_available = clearance.size() == path.poses.size();
    }
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      node_->get_logger(), "Path speed profile costmap unavailable: %s", error.what());
  }

  auto profile = runner_path_speed_profile::makeProfile(
    path, clearance, preset_ceiling, config);
  profile.header.stamp = node_->now();

  if (!profile_client_->wait_for_service(server_timeout_)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Path speed profile handoff unavailable; refusing to dispatch committed path");
    return BT::NodeStatus::FAILURE;
  }
  auto profile_request =
    std::make_shared<runner_interfaces::srv::SetPathSpeedProfile::Request>();
  profile_request->path = path;
  profile_request->profile = profile;
  auto profile_future = profile_client_->async_send_request(profile_request);
  if (rclcpp::spin_until_future_complete(node_, profile_future, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Path speed profile handoff timed out; refusing to dispatch committed path");
    return BT::NodeStatus::FAILURE;
  }
  const auto profile_response = profile_future.get();
  if (!profile_response->accepted) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Path speed profile handoff rejected (%s); refusing to dispatch committed path",
      profile_response->reason.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // The service response establishes that the controller cached this exact
  // path/profile pair before FollowPath can send the path action goal. Keep
  // the topic as the observable, transient-local record of that handoff.
  publisher_->publish(profile);
  if (!preset_available || !costmap_available) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Path speed profile published with conservative fallback (%s%s)",
      preset_available ? "" : "preset unavailable",
      costmap_available ? "" :
      (preset_available ? "costmap unavailable" : ", costmap unavailable"));
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace runner_nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<runner_nav2_behavior_tree::PathExistsCondition>(
    "PathExists");
  factory.registerNodeType<runner_nav2_behavior_tree::CandidatePathValidCondition>(
    "CandidatePathValid");
  factory.registerNodeType<runner_nav2_behavior_tree::PersistentPathValidCondition>(
    "PersistentPathValid");
  factory.registerNodeType<runner_nav2_behavior_tree::ReportPathCommitment>(
    "ReportPathCommitment");
  factory.registerNodeType<runner_nav2_behavior_tree::GeneratePathSpeedProfile>(
    "GeneratePathSpeedProfile");
}
