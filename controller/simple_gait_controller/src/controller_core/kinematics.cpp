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

#include "simple_gait_controller/controller_core/kinematics.hpp"

namespace quad_sim
{
namespace controller
{

Kinematics::Kinematics()
{
}

void Kinematics::solveLegsForwardKinematics(
  std::unordered_map<std::string, geometry_msgs::msg::Point> & foot_points,
  const std::vector<double> & joints)
{
  forwardKinematics(
    foot_points["lf"], joints[0], joints[1], joints[2], quad_sim::controller::hip_y_length,
    quad_sim::controller::upper_leg_z_length, quad_sim::controller::lower_leg_z_length);
  forwardKinematics(
    foot_points["rf"], joints[3], joints[4], joints[5], -quad_sim::controller::hip_y_length,
    quad_sim::controller::upper_leg_z_length, quad_sim::controller::lower_leg_z_length);
  forwardKinematics(
    foot_points["lb"], joints[6], joints[7], joints[8], quad_sim::controller::hip_y_length,
    quad_sim::controller::upper_leg_z_length, quad_sim::controller::lower_leg_z_length);
  forwardKinematics(
    foot_points["rb"], joints[9], joints[10], joints[11], -quad_sim::controller::hip_y_length,
    quad_sim::controller::upper_leg_z_length, quad_sim::controller::lower_leg_z_length);
}

void Kinematics::forwardKinematics(
  geometry_msgs::msg::Point & foot_point, const double & hip_joint, const double & upper_joint,
  const double & lower_joint, const double l1, const double l2, const double l3)
{
  foot_point.x = -l2 * std::sin(upper_joint) - l3 * std::sin(upper_joint + lower_joint);
  foot_point.y = l1 * std::cos(hip_joint) + l2 * std::sin(hip_joint) * std::cos(upper_joint) +
    l3 * std::sin(hip_joint) * std::cos(upper_joint + lower_joint);
  foot_point.z = l1 * std::sin(hip_joint) - l2 * std::cos(hip_joint) * std::cos(upper_joint) -
    l3 * std::cos(hip_joint) * std::cos(upper_joint + lower_joint);
}

void Kinematics::solveLegsInverseKinematics(
  std::unordered_map<std::string, geometry_msgs::msg::Point> & foot_points,
  std::vector<double> & joints)
{
  inverseKinematics(
    foot_points["lf"], joints[0], joints[1], joints[2], quad_sim::controller::hip_y_length,
    quad_sim::controller::upper_leg_z_length, quad_sim::controller::lower_leg_z_length);
  inverseKinematics(
    foot_points["rf"], joints[3], joints[4], joints[5], -quad_sim::controller::hip_y_length,
    quad_sim::controller::upper_leg_z_length, quad_sim::controller::lower_leg_z_length);
  inverseKinematics(
    foot_points["lb"], joints[6], joints[7], joints[8], quad_sim::controller::hip_y_length,
    quad_sim::controller::upper_leg_z_length, quad_sim::controller::lower_leg_z_length);
  inverseKinematics(
    foot_points["rb"], joints[9], joints[10], joints[11], -quad_sim::controller::hip_y_length,
    quad_sim::controller::upper_leg_z_length, quad_sim::controller::lower_leg_z_length);
}

void Kinematics::inverseKinematics(
  const geometry_msgs::msg::Point & foot_point, double & hip_joint, double & upper_joint,
  double & lower_joint, const double l1, const double l2, const double l3)
{
  double x = foot_point.x;
  double y = foot_point.y;
  double z = foot_point.z;

  double origin_to_foot = std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2);
  double hip_to_foot = std::pow(l1, 2) + std::pow(l2, 2) + std::pow(l3, 2);
  double d1 = std::sqrt(std::pow(y, 2) + std::pow(z, 2) - std::pow(l1, 2));
  double d2 = (origin_to_foot - hip_to_foot) / (2 * l2 * l3);
  d2 = std::clamp(d2, -1.0, 1.0);  // 誤差によりacosの定義域を超えないようにクランプ

  hip_joint = std::atan2(z, y) + std::atan2(d1, l1);
  lower_joint = -std::acos(d2);  // [-pi, 0]のためマイナス（実際は[-pi/2, 0]の範囲で使用）
  upper_joint =
    std::atan2(-x, d1) - std::atan2(l3 * std::sin(lower_joint), (l2 + l3 * std::cos(lower_joint)));

  hip_joint = clampAngle(hip_joint);
  upper_joint = clampAngle(upper_joint);
  lower_joint = clampAngle(lower_joint);
}

double Kinematics::clampAngle(double angle)
{
  angle = std::fmod(angle, 2.0 * M_PI);
  if (angle >= M_PI) {
    angle -= 2.0 * M_PI;
  } else if (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

}  // namespace controller
}  // namespace quad_sim
