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

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "geometry_utils/geometry_utils.hpp"
#include "simple_gait_controller/link_length.hpp"

namespace quad_sim
{
namespace controller
{

class FootSpeedCalculator
{
public:
  explicit FootSpeedCalculator();
  void calculateFootSpeeds(
    std::unordered_map<std::string, geometry_msgs::msg::Point> & last_foot_points,
    std::unordered_map<std::string, geometry_msgs::msg::Twist> & foot_twists,
    const geometry_msgs::msg::Twist & robot_twist);
  void calculateFootSpeed(
    geometry_msgs::msg::Point base_to_foot, geometry_msgs::msg::Twist & foot_twist,
    const geometry_msgs::msg::Twist & robot_twist);

private:
  std::unordered_map<std::string, geometry_msgs::msg::Point> base_to_hip_origins_;
};

}  // namespace controller
}  // namespace quad_sim
