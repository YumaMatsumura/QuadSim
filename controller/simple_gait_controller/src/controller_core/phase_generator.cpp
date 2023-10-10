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

#include "simple_gait_controller/controller_core/phase_generator.hpp"

namespace quad_sim
{
namespace controller
{

PhaseGenerator::PhaseGenerator(double gait_period, double max_elapsed_time)
{
  gait_period_ = gait_period;
  max_elapsed_time_ = max_elapsed_time;
}

void PhaseGenerator::tick(
  std::unordered_map<std::string, double> & phases, const rclcpp::Time & current_time)
{
  rclcpp::Duration elapsed_duration = current_time - last_time_;
  last_time_ = current_time;
  double elapsed_time = std::clamp(elapsed_duration.seconds(), 0.0, max_elapsed_time_);

  generatePhase(phases_["lf"], elapsed_time);
  generatePhase(phases_["rf"], elapsed_time);
  generatePhase(phases_["lb"], elapsed_time);
  generatePhase(phases_["rb"], elapsed_time);

  phases = phases_;
}

void PhaseGenerator::generatePhase(double & phase, double elapsed_time)
{
  phase += 2.0 * M_PI * (elapsed_time / gait_period_);

  while (phase >= 2.0 * M_PI) {
    phase -= 2.0 * M_PI;
  }
}

void PhaseGenerator::updateParams(double gait_period, double max_elapsed_time)
{
  gait_period_ = gait_period;
  max_elapsed_time_ = max_elapsed_time;
}

void PhaseGenerator::setStartPhases(
  const std::unordered_map<std::string, double> & start_phases, const rclcpp::Time & start_time)
{
  phases_ = start_phases;
  last_time_ = start_time;
}

}  // namespace controller
}  // namespace quad_sim
