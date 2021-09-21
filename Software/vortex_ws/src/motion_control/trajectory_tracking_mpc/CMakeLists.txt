cmake_minimum_required(VERSION 3.5)
project(trajectory_tracking_mpc)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(custom_ros_interfaces)
find_package(Ceres REQUIRED)
find_package(message_filters)
find_package(PkgConfig REQUIRED)

add_executable(guidance_node src/guidance_node.cpp
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
add_executable(maneuver_node src/maneuver_node.cpp  src/maneuver_mpc.cpp)
add_executable(depth_node src/depth_node.cpp  src/depth_mpc.cpp)
add_executable(thrust_merger_node src/thrust_merger_node.cpp)

target_link_libraries(guidance_node
  ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
target_link_libraries(maneuver_node
  ${EIGEN3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ipopt
)
target_link_libraries(depth_node
  ${EIGEN3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ipopt
)
target_include_directories(guidance_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME} ${EIGEN3_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/lib/auv_guidance/include)
target_include_directories(maneuver_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME} ${EIGEN3_INCLUDE_DIRS})
target_include_directories(depth_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME} ${EIGEN3_INCLUDE_DIRS})
target_include_directories(thrust_merger_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

ament_target_dependencies(guidance_node rclcpp custom_ros_interfaces geometry_msgs nav_msgs Eigen3 Ceres)
ament_target_dependencies(maneuver_node rclcpp custom_ros_interfaces geometry_msgs Eigen3)
ament_target_dependencies(depth_node rclcpp custom_ros_interfaces geometry_msgs Eigen3)
ament_target_dependencies(thrust_merger_node rclcpp geometry_msgs message_filters)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS guidance_node maneuver_node depth_node thrust_merger_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  params
  launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
ament_package()
