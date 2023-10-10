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

#include <urdf/model.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
// #include <kdl/tree.hpp>
// #include <kdl_parser/kdl_parser.hpp>

#include <geometry_msgs/msg/point.hpp>

namespace quad_sim
{
namespace common
{

class URDFParser
{
public:
  explicit URDFParser(const std::string & file_path);
  ~URDFParser();
  geometry_msgs::msg::Point getRelativePosition(std::string start_link, std::string end_link);

private:
  rclcpp::Logger logger_{rclcpp::get_logger("urdf_parser")};
  // KDL::Tree tree_;
  urdf::Model model_;
};

}  // namespace common
}  // namespace quad_sim
