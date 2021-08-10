cmake_minimum_required(VERSION 3.5)
project(mpc_guidance)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_ros_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP yaml-cpp)

set( sources "lib/auv_guidance/src/math_lib.cpp"
    "lib/auv_guidance/src/rot3d.cpp"
    "lib/auv_guidance/src/min_jerk_trajectory.cpp"
    "lib/auv_guidance/src/min_jerk_time_solver.cpp"
    "lib/auv_guidance/src/basic_trajectory.cpp"
    "lib/auv_guidance/src/simultaneous_trajectory.cpp"
    "lib/auv_guidance/src/long_trajectory.cpp"
    "lib/auv_guidance/src/tgen_limits.cpp"
    "lib/auv_guidance/src/waypoint.cpp" )

add_executable(mpc_guidance src/guidance_node.cpp
    src/los_steering.cpp
    lib/auv_guidance/src/math_lib.cpp
    lib/auv_guidance/src/rot3d.cpp
    lib/auv_guidance/src/min_jerk_trajectory.cpp
    lib/auv_guidance/src/min_jerk_time_solver.cpp
    lib/auv_guidance/src/basic_trajectory.cpp
    lib/auv_guidance/src/simultaneous_trajectory.cpp
    lib/auv_guidance/src/long_trajectory.cpp
    lib/auv_guidance/src/tgen_limits.cpp
    lib/auv_guidance/src/waypoint.cpp)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)
target_link_libraries(mpc_guidance
  ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(mpc_guidance rclcpp custom_ros_interfaces geometry_msgs nav_msgs Eigen3 Ceres)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(TARGETS mpc_guidance DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  params
  launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
ament_package()
