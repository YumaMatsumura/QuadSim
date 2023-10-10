cmake_minimum_required(VERSION 3.8)
project(simple_gait_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_utils REQUIRED)
find_package(quad_urdf_parser REQUIRED)
find_package(Eigen3 REQUIRED)

set(simple_gait_controller_executable simple_gait_controller)
set(simple_gait_controller_library ${simple_gait_controller_executable}_core)
set(dependencies
  rclcpp
  rclcpp_components
  control_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_utils
  quad_urdf_parser
  Eigen3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${simple_gait_controller_executable}
  src/simple_gait_controller_node.cpp
  src/simple_gait_controller_component.cpp
  src/controller_core/kinematics.cpp
  src/controller_core/foot_speed_calculator.cpp
  src/controller_core/phase_generator.cpp
  src/controller_core/ellipse_trajectory_planner.cpp
)
add_library(${simple_gait_controller_library} SHARED
  src/simple_gait_controller_component.cpp
  src/controller_core/kinematics.cpp
  src/controller_core/foot_speed_calculator.cpp
  src/controller_core/phase_generator.cpp
  src/controller_core/ellipse_trajectory_planner.cpp
)
ament_target_dependencies(${simple_gait_controller_executable}
  ${dependencies}
)
ament_target_dependencies(${simple_gait_controller_library}
  ${dependencies}
)
rclcpp_components_register_nodes(${simple_gait_controller_library}
  "quad_sim::controller::SimpleGaitController"
)

install(TARGETS
  ${simple_gait_controller_executable}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  ${simple_gait_controller_library}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_clang_format)
  ament_lint_auto_find_test_dependencies()
  ament_clang_format(
    CONFIG_FILE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format>
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${simple_gait_controller_library})
ament_export_dependencies(${dependencies})
ament_package()
