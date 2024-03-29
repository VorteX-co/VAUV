cmake_minimum_required(VERSION 3.5)
project(trajectory_tracking_lqr)

add_definitions(-std=c++17)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
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
find_package(custom_ros_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(LAPACK REQUIRED)
add_subdirectory(libs/lin_alg_tools)
add_subdirectory(libs/geometry)
add_subdirectory(libs/ruckig)

add_executable(${PROJECT_NAME} src/lqr.cpp src/trajectory.cpp src/controller_node.cpp src/los_steering.cpp)

target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBRARIES} ${LAPACK_LIBRARIES} ruckig)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
  libs/lin_alg_tools/include
  libs/geometry/include)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs geometry_msgs custom_ros_interfaces nav_msgs Eigen3 tf2_eigen)

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
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  libs
  launch
  params
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
ament_package()
