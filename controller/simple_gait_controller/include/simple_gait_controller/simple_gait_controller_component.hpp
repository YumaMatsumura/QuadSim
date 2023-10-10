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

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/rclcpp.hpp>

#include "geometry_utils/geometry_utils.hpp"
#include "quad_urdf_parser/urdf_parser.hpp"
#include "simple_gait_controller/controller_core/ellipse_trajectory_planner.hpp"
#include "simple_gait_controller/controller_core/foot_speed_calculator.hpp"
#include "simple_gait_controller/controller_core/kinematics.hpp"
#include "simple_gait_controller/controller_core/phase_generator.hpp"

namespace quad_sim
{
namespace controller
{

class SimpleGaitController : public rclcpp::Node
{
public:
  explicit SimpleGaitController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SimpleGaitController();

private:
  void timerCallback();
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void jointStatesCallback(const control_msgs::msg::DynamicJointState::SharedPtr msg);
  void publishJoints(const std::vector<double> & target_joints);
  void publishJointStates(const std::vector<double> & target_joints);
  void publishMarkers(std::unordered_map<std::string, geometry_msgs::msg::Point> & foot_points);

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_joint_control_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_array_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr sub_dynamic_joint_state_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::shared_ptr<quad_sim::controller::Kinematics> kinematics_;
  std::shared_ptr<quad_sim::controller::FootSpeedCalculator> foot_speed_;
  std::shared_ptr<quad_sim::controller::PhaseGenerator> phase_generator_;
  std::shared_ptr<quad_sim::controller::EllipseTrajectoryPlanner> trajectory_planner_;

  geometry_msgs::msg::Twist robot_twist_;

  std::unordered_map<std::string, geometry_msgs::msg::Point> base_to_hip_origins_;
  std::vector<double> joint_states_;
  std::vector<std::string> joint_names_;
  std::string base_frame_id_;
  double min_vel_x_;
  double min_vel_y_;
  double max_vel_x_;
  double max_vel_y_;
  double max_vel_theta_;
  double stance_duty_;
  double swing_height_;
  double stance_height_;
  double walking_height_;
};

}  // namespace controller
}  // namespace quad_sim
