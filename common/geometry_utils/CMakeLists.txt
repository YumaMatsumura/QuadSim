cmake_minimum_required(VERSION 3.8)
project(geometry_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

set(geometry_utils_library geometry_utils)
set(dependencies
  geometry_msgs
)

include_directories(include)
add_library(${geometry_utils_library} SHARED
  src/geometry_utils.cpp
)
ament_target_dependencies(${geometry_utils_library}
  ${dependencies}
)

install(TARGETS ${geometry_utils_library}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
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
ament_export_libraries(${geometry_utils_library})
ament_export_dependencies(${dependencies})
ament_package()
