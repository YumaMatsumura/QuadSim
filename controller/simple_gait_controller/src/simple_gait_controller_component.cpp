// Copyright 2023 Yuma Matsumura All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simple_gait_controller/simple_gait_controller_component.hpp"

namespace quad_sim
{
namespace controller
{
SimpleGaitController::SimpleGaitController(const rclcpp::NodeOptions & options)
: Node("simple_gait_controller_node", options)
{
  // Set velocity parameters
  {
    const auto validate_and_fix_velocity_params = [&]() {
      const auto swap_value = [this](double & x, double & y) {
        double tmp = x;
        x = y;
        y = tmp;
      };

      if (min_vel_x_ > max_vel_x_) {
        RCLCPP_WARN(
          this->get_logger(), "min_vel_x has a larger value than max_vel_x, so swap the values.");
        swap_value(min_vel_x_, max_vel_x_);
      }
      if (min_vel_y_ > max_vel_y_) {
        RCLCPP_WARN(
          this->get_logger(), "min_vel_y has a larger value than max_vel_y, so swap the values.");
        swap_value(min_vel_y_, max_vel_y_);
      }
      if (min_vel_x_ > 0.0) {
        RCLCPP_WARN(this->get_logger(), "min_vel_x has a larger value than zero, so set zero.");
        min_vel_x_ = 0.0;
      }
      if (min_vel_y_ > 0.0) {
        RCLCPP_WARN(this->get_logger(), "min_vel_y has a larger value than zero, so set zero.");
        min_vel_y_ = 0.0;
      }
      if (max_vel_x_ < 0.0) {
        RCLCPP_WARN(this->get_logger(), "max_vel_x has a smaller value than zero, so set zero.");
        max_vel_x_ = 0.0;
      }
      if (max_vel_y_ < 0.0) {
        RCLCPP_WARN(this->get_logger(), "max_vel_y has a smaller value than zero, so set zero.");
        max_vel_y_ = 0.0;
      }
      if (max_vel_theta_ < 0.0) {
        RCLCPP_WARN(
          this->get_logger(), "max_vel_theta has a smaller value than zero, so set zero.");
        max_vel_theta_ = 0.0;
      }

      RCLCPP_INFO_STREAM(this->get_logger(), "min_vel_x: " << min_vel_x_);
      RCLCPP_INFO_STREAM(this->get_logger(), "min_vel_y: " << min_vel_y_);
      RCLCPP_INFO_STREAM(this->get_logger(), "max_vel_x: " << max_vel_x_);
      RCLCPP_INFO_STREAM(this->get_logger(), "max_vel_y: " << max_vel_y_);
      RCLCPP_INFO_STREAM(this->get_logger(), "max_vel_theta: " << max_vel_theta_);
    };

    min_vel_x_ = this->declare_parameter<double>("min_vel_x", -0.4);
    min_vel_y_ = this->declare_parameter<double>("min_vel_y", -0.2);
    max_vel_x_ = this->declare_parameter<double>("max_vel_x", 0.4);
    max_vel_y_ = this->declare_parameter<double>("max_vel_y", 0.2);
    max_vel_theta_ = this->declare_parameter<double>("max_vel_theta", 1.0);

    validate_and_fix_velocity_params();
  }

  // Set robot link and joint data
  {
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");
    joint_names_ = this->declare_parameter<std::vector<std::string>>("joint_names", {""});
    if (joint_names_.size() != 12) {
      RCLCPP_ERROR(this->get_logger(), "The number of joint names is invalid.");
      std::exit(EXIT_FAILURE);
    }

    const auto robot_description =
      this->declare_parameter<std::string>("robot_description", std::string(""));
    auto urdf_parser = std::make_shared<quad_sim::common::URDFParser>(robot_description);

    // base_to_hip_originsの初期化
    base_to_hip_origins_["lf"] = urdf_parser->getRelativePosition("base_link", "lf_hip_link");
    base_to_hip_origins_["rf"] = urdf_parser->getRelativePosition("base_link", "rf_hip_link");
    base_to_hip_origins_["lb"] = urdf_parser->getRelativePosition("base_link", "lb_hip_link");
    base_to_hip_origins_["rb"] = urdf_parser->getRelativePosition("base_link", "rb_hip_link");
  }

  // Initialize core controller
  {
    const auto stance_duty = this->declare_parameter<double>("stance_duty", 0.50);
    const auto stance_height = this->declare_parameter<double>("stance_height", 0.01);
    const auto swing_height = this->declare_parameter<double>("swing_height", 0.04);
    const auto walking_height = this->declare_parameter<double>("walking_height", 0.20);
    const auto is_valid_gait_params = [&]() {
      if (stance_duty == 0.0 || stance_duty == 1.0) {
        RCLCPP_ERROR(this->get_logger(), "stance_duty is set to an invalid value.");
        return false;
      }
      if (swing_height == 0.0) {
        RCLCPP_ERROR(this->get_logger(), "swing_height is set to an invalid value.");
        return false;
      }
      if (swing_height > walking_height) {
        RCLCPP_ERROR(
          this->get_logger(), "swing_height is set to a greater value than walking_height.");
        return false;
      }
      return true;
    };

    if (!is_valid_gait_params()) {
      RCLCPP_ERROR(this->get_logger(), "Gait parameters are not configured properly.");
      std::exit(EXIT_FAILURE);
    }

    kinematics_ = std::make_shared<quad_sim::controller::Kinematics>();
    foot_speed_ = std::make_shared<quad_sim::controller::FootSpeedCalculator>();
    phase_generator_ = std::make_shared<quad_sim::controller::PhaseGenerator>(0.5, 0.1);
    trajectory_planner_ = std::make_shared<quad_sim::controller::EllipseTrajectoryPlanner>(
      stance_duty, stance_height, swing_height, walking_height, 0.5);

    joint_states_.reserve(12);
    rclcpp::Time current_time = this->now();
    std::unordered_map<std::string, double> start_phases = {
      {"lf", 0.0}, {"rf", M_PI}, {"lb", M_PI}, {"rb", 0.0}};
    phase_generator_->setStartPhases(start_phases, current_time);
  }

  // Create publisher and subscriber
  {
    const auto joint_control_topic =
      this->declare_parameter<std::string>("joint_control_topic", "position_controller/commands");

    pub_joint_control_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      joint_control_topic, rclcpp::SystemDefaultsQoS());
    pub_joint_state_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10));
    pub_marker_array_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers", rclcpp::QoS(10));
    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      std::bind(&SimpleGaitController::twistCallback, this, std::placeholders::_1));
    sub_dynamic_joint_state_ = this->create_subscription<control_msgs::msg::DynamicJointState>(
      "/dynamic_joint_states", rclcpp::QoS(10),
      std::bind(&SimpleGaitController::jointStatesCallback, this, std::placeholders::_1));
  }

  // Create timer
  {
    const auto controller_hz = this->declare_parameter<double>("controller_hz", 200.0);
    const auto period_ns = rclcpp::Rate(controller_hz).period();
    control_timer_ =
      this->create_wall_timer(period_ns, std::bind(&SimpleGaitController::timerCallback, this));
  }
}

SimpleGaitController::~SimpleGaitController()
{
  if (control_timer_) {
    control_timer_->cancel();
    control_timer_.reset();
  }
}

void SimpleGaitController::timerCallback()
{
  rclcpp::Time current_time = this->now();
  std::unordered_map<std::string, geometry_msgs::msg::Twist> target_foot_twists;
  std::unordered_map<std::string, geometry_msgs::msg::Point> target_foot_points;
  std::unordered_map<std::string, geometry_msgs::msg::Point> last_foot_points;
  std::unordered_map<std::string, double> phases;
  std::vector<double> target_joint_positions(12);

  target_foot_twists.reserve(4);
  target_foot_points.reserve(4);
  last_foot_points.reserve(4);
  phases.reserve(4);
  // target_joint_positions.reserve(12);

  // joint_statesから順運動学により脚先座標を求める
  kinematics_->solveLegsForwardKinematics(last_foot_points, joint_states_);

  // ロボットの速度司令から各脚先の接地する目標位置を求める
  foot_speed_->calculateFootSpeeds(last_foot_points, target_foot_twists, robot_twist_);

  // トロット歩容の位相生成
  phase_generator_->tick(phases, current_time);

  // 各脚先速度から各脚先位置を求める
  trajectory_planner_->calculateTargetPoints(phases, target_foot_twists, target_foot_points);

  // 各脚先位置から各関節の角度を求める（逆運動学）
  kinematics_->solveLegsInverseKinematics(target_foot_points, target_joint_positions);

  // Gazeboモータに司令を送る
  publishJoints(target_joint_positions);
  publishJointStates(target_joint_positions);
  publishMarkers(target_foot_points);
}

void SimpleGaitController::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  robot_twist_.linear.x = std::clamp(msg->linear.x, min_vel_x_, max_vel_x_);
  robot_twist_.linear.y = std::clamp(msg->linear.y, min_vel_y_, max_vel_y_);
  robot_twist_.angular.z = std::clamp(msg->angular.z, -max_vel_theta_, max_vel_theta_);
}

void SimpleGaitController::jointStatesCallback(
  const control_msgs::msg::DynamicJointState::SharedPtr msg)
{
  if (msg->interface_values.size() != 12) {
    return;
  }

  for (size_t i = 0; i < msg->interface_values.size(); i++) {
    joint_states_[i] = msg->interface_values[i].values[0];
  }
}

void SimpleGaitController::publishJoints(const std::vector<double> & target_joints)
{
  size_t total_subscription_count = pub_joint_control_->get_subscription_count() +
    pub_joint_control_->get_intra_process_subscription_count();
  if (total_subscription_count > 0) {
    auto joint_cmd = std::make_unique<std_msgs::msg::Float64MultiArray>();

    joint_cmd->data.reserve(12);
    for (size_t i = 0; i < 12; i++) {
      joint_cmd->data.push_back(target_joints[i]);
    }

    pub_joint_control_->publish(std::move(joint_cmd));
  }
}

void SimpleGaitController::publishJointStates(const std::vector<double> & target_joints)
{
  size_t total_subscription_count = pub_joint_state_->get_subscription_count() +
    pub_joint_state_->get_intra_process_subscription_count();
  if (total_subscription_count > 0) {
    auto joint_states = std::make_unique<sensor_msgs::msg::JointState>();

    joint_states->header.stamp = this->now();
    joint_states->name.reserve(12);
    joint_states->position.reserve(12);
    for (size_t i = 0; i < 12; i++) {
      joint_states->name.push_back(joint_names_[i]);
      joint_states->position.push_back(target_joints[i]);
    }

    pub_joint_state_->publish(std::move(joint_states));
  }
}

void SimpleGaitController::publishMarkers(
  std::unordered_map<std::string, geometry_msgs::msg::Point> & foot_points)
{
  size_t total_subscription_count = pub_marker_array_->get_subscription_count() +
    pub_marker_array_->get_intra_process_subscription_count();
  if (total_subscription_count > 0) {
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    marker_array->markers.reserve(4);

    std::unordered_map<std::string, geometry_msgs::msg::Point> base_to_foot_points;
    base_to_foot_points["lf"] = foot_points["lf"] + base_to_hip_origins_["lf"];
    base_to_foot_points["rf"] = foot_points["rf"] + base_to_hip_origins_["rf"];
    base_to_foot_points["lb"] = foot_points["lb"] + base_to_hip_origins_["lb"];
    base_to_foot_points["rb"] = foot_points["rb"] + base_to_hip_origins_["rb"];

    int id = 0;
    for (auto it = base_to_foot_points.begin(); it != base_to_foot_points.end(); it++) {
      visualization_msgs::msg::Marker foot_target;
      foot_target.header.frame_id = base_frame_id_;
      foot_target.header.stamp = this->now();
      foot_target.id = id;
      foot_target.type = visualization_msgs::msg::Marker::SPHERE;
      foot_target.action = visualization_msgs::msg::Marker::ADD;
      foot_target.pose.position.x = it->second.x;
      foot_target.pose.position.y = it->second.y;
      foot_target.pose.position.z = it->second.z;
      foot_target.scale.x = 0.05;
      foot_target.scale.y = 0.05;
      foot_target.scale.z = 0.05;
      foot_target.color.r = 1.0;
      foot_target.color.g = 0.0;
      foot_target.color.b = 0.0;
      foot_target.color.a = 1.0;
      marker_array->markers.push_back(foot_target);

      id++;
    }

    pub_marker_array_->publish(std::move(marker_array));
  }
}

}  // namespace controller
}  // namespace quad_sim

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(quad_sim::controller::SimpleGaitController)
