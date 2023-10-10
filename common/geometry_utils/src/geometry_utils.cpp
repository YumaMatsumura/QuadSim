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

#include "geometry_utils/geometry_utils.hpp"

geometry_msgs::msg::Point operator+(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
{
  geometry_msgs::msg::Point ret;

  ret.x = p1.x + p2.x;
  ret.y = p1.y + p2.y;
  ret.z = p1.z + p2.z;

  return ret;
}
