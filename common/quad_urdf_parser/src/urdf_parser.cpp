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

#include "quad_urdf_parser/urdf_parser.hpp"

namespace quad_sim
{
namespace common
{

URDFParser::URDFParser(const std::string & urdf)
{
  if (!model_.initString(urdf) && !model_.initFile(urdf)) {
    RCLCPP_ERROR(logger_, "Failed to initialize urdf model.");
  }
  // if (model_.initString(urdf)) {
  //   if (kdl_parser::treeFromString(urdf, tree_)) {
  //     RCLCPP_ERROR(logger_, "Failed to extract kdl tree.");
  //   }
  // } else if (model_.initFile(urdf)) {
  //   if (kdl_parser::treeFromFile(urdf, tree_)) {
  //     RCLCPP_ERROR(logger_, "Failed to extract kdl tree.");
  //   }
  // } else {
  //   RCLCPP_ERROR(logger_, "Failed to initialize urdf model.");
  // }
}

URDFParser::~URDFParser()
{
}

geometry_msgs::msg::Point URDFParser::getRelativePosition(
  std::string start_link, std::string end_link)
{
  geometry_msgs::msg::Point relative_position;
  relative_position.x = 0.0;
  relative_position.y = 0.0;
  relative_position.z = 0.0;

  std::string target_parent_link = end_link;

  while (start_link != target_parent_link) {
    auto target_link_ptr = model_.getLink(target_parent_link);
    auto target_pose = target_link_ptr->parent_joint->parent_to_joint_origin_transform;

    target_parent_link = target_link_ptr->getParent()->name;
    relative_position.x += target_pose.position.x;
    relative_position.y += target_pose.position.y;
    relative_position.z += target_pose.position.z;
  }

  return relative_position;
}

}  // namespace common
}  // namespace quad_sim
