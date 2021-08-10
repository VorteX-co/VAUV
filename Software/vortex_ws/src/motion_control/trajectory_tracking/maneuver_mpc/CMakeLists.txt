cmake_minimum_required(VERSION 3.5)
project(maneuver_mpc)

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
find_package(geometry_msgs REQUIRED)
find_package(custom_ros_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP yaml-cpp)

add_executable(MMPC src/controller_node.cpp  src/maneuver_mpc.cpp)
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
target_include_directories(MMPC PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS})
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
target_link_libraries(MMPC ipopt python2.7 ${YAML_CPP_LIBRARIES})
else()
target_link_libraries(MMPC ipopt python)
endif()
ament_target_dependencies(MMPC rclcpp custom_ros_interfaces geometry_msgs Eigen3)

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
install(TARGETS MMPC DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  params
  launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
