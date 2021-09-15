cmake_minimum_required(VERSION 3.5)
project(sensors)

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

set(CMAKE_CXX_FLAGS "-pthread")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread chrono system)
find_package(custom_ros_interfaces REQUIRED)

add_executable(pub src/A50-DVL/dvl_serial_pub.cpp src/A50-DVL/VDVL/executor.cpp src/A50-DVL/VDVL/serial_node.cpp src/A50-DVL/VDVL/serial_port.cpp)
target_link_libraries(pub ${Boost_LIBRARIES})
target_include_directories(pub PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/VDVL ${Boost_INCLUDE_DIRS})
ament_target_dependencies(pub rclcpp custom_ros_interfaces Boost)

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

install(TARGETS pub DESTINATION lib/${PROJECT_NAME})


ament_package()

