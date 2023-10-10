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

#include "simple_gait_controller/controller_core/ellipse_trajectory_planner.hpp"

namespace quad_sim
{
namespace controller
{

EllipseTrajectoryPlanner::EllipseTrajectoryPlanner(
  double stance_duty, double stance_height, double swing_height, double walking_height,
  double gait_period)
{
  stance_duty_ = stance_duty;
  stance_height_ = stance_height;
  swing_height_ = swing_height;
  walking_height_ = walking_height;
  gait_period_ = gait_period;
}

void EllipseTrajectoryPlanner::calculateTargetPoints(
  std::unordered_map<std::string, double> & phases,
  std::unordered_map<std::string, geometry_msgs::msg::Twist> & foot_twists,
  std::unordered_map<std::string, geometry_msgs::msg::Point> & foot_points)
{
  calculateTargetPoint(0.1, phases["lf"], foot_twists["lf"], foot_points["lf"]);
  calculateTargetPoint(-0.1, phases["rf"], foot_twists["rf"], foot_points["rf"]);
  calculateTargetPoint(0.1, phases["lb"], foot_twists["lb"], foot_points["lb"]);
  calculateTargetPoint(-0.1, phases["rb"], foot_twists["rb"], foot_points["rb"]);
}

void EllipseTrajectoryPlanner::calculateTargetPoint(
  double shift_z, const double & phase, const geometry_msgs::msg::Twist & foot_twist,
  geometry_msgs::msg::Point & foot_point)
{
  double angle = std::atan2(foot_twist.linear.y, foot_twist.linear.x);
  double x = 0.0;  // 楕円軌道を描く平面の横軸
  double y = 0.0;  // 楕円軌道を描く平面の縦軸

  double step_length =
    gait_period_ * stance_duty_ * std::hypot(foot_twist.linear.x, foot_twist.linear.y);

  if (phase < 2.0 * M_PI * (1.0 - stance_duty_)) {  // 遊脚時
    x = (step_length / 2.0) * std::cos(phase);      // 楕円の媒介変数表示
    y = swing_height_ * std::sin(phase);            // 楕円の媒介変数表示
  } else {                                          // 立脚時
    // x = (step_length / 2.0) * std::cos(phase); // 楕円の媒介変数表示
    // y = stance_height_ * std::sin(phase); // 楕円の媒介変数表示
    x = (step_length / 2.0) * std::cos(phase);
    y = 0.0;
  }

  // 座標変換
  Eigen::Translation<double, 3> translation(0.0, shift_z, -walking_height_);
  Eigen::Quaterniond rotation = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(angle - M_PI, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  Eigen::Affine3d affine = translation * rotation;

  Eigen::Vector3d p0(x, y, 0.0);
  Eigen::Vector3d p1 = affine * p0;

  foot_point.x = p1(0);
  foot_point.y = p1(1);
  foot_point.z = p1(2);
}

void EllipseTrajectoryPlanner::updateParams(
  double stance_duty, double stance_height, double swing_height, double walking_height,
  double gait_period)
{
  stance_duty_ = stance_duty;
  stance_height_ = stance_height;
  swing_height_ = swing_height;
  walking_height_ = walking_height;
  gait_period_ = gait_period;
}

}  // namespace controller
}  // namespace quad_sim
