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

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point.hpp>

#include "simple_gait_controller/link_length.hpp"

namespace quad_sim
{
namespace controller
{

class Kinematics
{
public:
  Kinematics();

  // Forward Kinematics
  void solveLegsForwardKinematics(
    std::unordered_map<std::string, geometry_msgs::msg::Point> & foot_points,
    const std::vector<double> & joints);
  void forwardKinematics(
    geometry_msgs::msg::Point & foot_point, const double & hip_joint, const double & upper_joint,
    const double & lower_joint, const double l1, const double l2, const double l3);

  // Inverse Kinematics
  void solveLegsInverseKinematics(
    std::unordered_map<std::string, geometry_msgs::msg::Point> & foot_points,
    std::vector<double> & joints);
  void inverseKinematics(
    const geometry_msgs::msg::Point & foot_point, double & hip_joint, double & upper_joint,
    double & lower_joint, const double l1, const double l2, const double l3);
  double clampAngle(double angle);
};

}  // namespace controller
}  // namespace quad_sim
