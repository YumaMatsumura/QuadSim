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

#include "simple_gait_controller/controller_core/foot_speed_calculator.hpp"

namespace quad_sim
{
namespace controller
{

FootSpeedCalculator::FootSpeedCalculator()
{
  base_to_hip_origins_["lf"].x = quad_sim::controller::body_to_hip_x;
  base_to_hip_origins_["lf"].y = quad_sim::controller::body_to_hip_y;
  base_to_hip_origins_["rf"].x = quad_sim::controller::body_to_hip_x;
  base_to_hip_origins_["rf"].y = -quad_sim::controller::body_to_hip_y;
  base_to_hip_origins_["lb"].x = -quad_sim::controller::body_to_hip_x;
  base_to_hip_origins_["lb"].y = quad_sim::controller::body_to_hip_y;
  base_to_hip_origins_["rb"].x = -quad_sim::controller::body_to_hip_x;
  base_to_hip_origins_["rb"].y = -quad_sim::controller::body_to_hip_y;
}

void FootSpeedCalculator::calculateFootSpeeds(
  std::unordered_map<std::string, geometry_msgs::msg::Point> & last_foot_points,
  std::unordered_map<std::string, geometry_msgs::msg::Twist> & foot_twists,
  const geometry_msgs::msg::Twist & robot_twist)
{
  std::unordered_map<std::string, geometry_msgs::msg::Point> base_to_feet;
  base_to_feet["lf"] = base_to_hip_origins_["lf"] + last_foot_points["lf"];
  base_to_feet["rf"] = base_to_hip_origins_["rf"] + last_foot_points["rf"];
  base_to_feet["lb"] = base_to_hip_origins_["lb"] + last_foot_points["lb"];
  base_to_feet["rb"] = base_to_hip_origins_["rb"] + last_foot_points["rb"];

  calculateFootSpeed(base_to_feet["lf"], foot_twists["lf"], robot_twist);
  calculateFootSpeed(base_to_feet["rf"], foot_twists["rf"], robot_twist);
  calculateFootSpeed(base_to_feet["lb"], foot_twists["lb"], robot_twist);
  calculateFootSpeed(base_to_feet["rb"], foot_twists["rb"], robot_twist);
}

void FootSpeedCalculator::calculateFootSpeed(
  geometry_msgs::msg::Point base_to_foot, geometry_msgs::msg::Twist & foot_twist,
  const geometry_msgs::msg::Twist & robot_twist)
{
  // 脚先の位置を求める
  double distance = std::hypot(base_to_foot.x, base_to_foot.y);
  double angle = std::atan2(base_to_foot.y, base_to_foot.x);

  // 脚先速度を求める
  foot_twist.linear.x = robot_twist.linear.x - distance * robot_twist.angular.z * std::sin(angle);
  foot_twist.linear.y = robot_twist.linear.y - distance * robot_twist.angular.z * std::cos(angle);
}

}  // namespace controller
}  // namespace quad_sim
