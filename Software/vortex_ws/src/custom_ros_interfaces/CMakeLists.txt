cmake_minimum_required(VERSION 3.5)
project(custom_ros_interfaces)

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
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
"msg/SensorStatus.msg"
"msg/NavController.msg"
"msg/ServoMsg.msg"
"msg/RcMsg.msg"
"msg/Attitude.msg"
"msg/Depth.msg"
"msg/SetThrustChannels.msg"
"srv/Arm.srv"
"srv/SetMode.srv"
"srv/Heartbeat.srv"
"srv/PublishData.srv"
)

ament_package()
