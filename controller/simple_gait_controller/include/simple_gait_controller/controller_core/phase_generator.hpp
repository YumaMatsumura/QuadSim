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

#include <rclcpp/time.hpp>

namespace quad_sim
{
namespace controller
{

class PhaseGenerator
{
public:
  explicit PhaseGenerator(double gait_period, double max_elapsed_time);
  void tick(std::unordered_map<std::string, double> & phases, const rclcpp::Time & current_time);
  void generatePhase(double & phase, double elapsed_time);
  void updateParams(double gait_period, double max_elapsed_time);
  void setStartPhases(
    const std::unordered_map<std::string, double> & start_phase, const rclcpp::Time & start_time);

private:
  rclcpp::Time last_time_;
  std::unordered_map<std::string, double> phases_;
  double gait_period_;
  double max_elapsed_time_;
};

}  // namespace controller
}  // namespace quad_sim
