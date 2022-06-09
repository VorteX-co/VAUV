cmake_minimum_required(VERSION 3.5)
project(motion_planning)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
add_definitions(-std=c++17)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
add_subdirectory(libs/ruckig)

add_executable(local_planning src/local_planning/optimal_trajectory.cpp src/local_planning/steering.cpp src/local_planning/sinusodial_trajectory.cpp src/local_planning/local_planning_node.cpp)

target_link_libraries(local_planning ${EIGEN3_LIBRARIES} ruckig)
target_include_directories(local_planning
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/local_planning
  ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(local_planning rclcpp std_msgs geometry_msgs  nav_msgs Eigen3 tf2_eigen)

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

install(TARGETS local_planning DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  libs
  launch
  params
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
