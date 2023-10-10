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

#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <Eigen/Geometry>

namespace quad_sim
{
namespace controller
{

class EllipseTrajectoryPlanner
{
public:
  explicit EllipseTrajectoryPlanner(
    double stance_duty, double stance_height, double swing_height, double walking_height,
    double gait_period);
  void calculateTargetPoints(
    std::unordered_map<std::string, double> & phases,
    std::unordered_map<std::string, geometry_msgs::msg::Twist> & foot_twists,
    std::unordered_map<std::string, geometry_msgs::msg::Point> & foot_points);
  void calculateTargetPoint(
    double shift_z, const double & phase, const geometry_msgs::msg::Twist & foot_twist,
    geometry_msgs::msg::Point & foot_point);
  void updateParams(
    double stance_duty, double swing_height, double stance_height, double walking_height,
    double gait_period);

private:
  double stance_duty_;
  double stance_height_;
  double swing_height_;
  double walking_height_;
  double gait_period_;
};

}  // namespace controller
}  // namespace quad_sim
