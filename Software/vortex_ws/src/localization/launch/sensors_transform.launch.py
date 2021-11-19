# Copyright 2021 VorteX-co.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='localization', node_executable='sensors_transform', output='screen'),
        launch_ros.actions.Node(
            # Publish a static coordinate transform to tf2
            # using an x/y/z offset in meters and roll/pitch/yaw in radians
            # frame_id = swift/base_link
            # child_frame_id = swift/dvl_link
            package='tf2_ros', node_executable='static_transform_publisher',
            arguments=["0.06593", "-0.006", "-0.13673", "0", "1.57", "0",
                       "swift/base_link", "swift/dvl_link"]),
        launch_ros.actions.Node(
            # IMU
            package='tf2_ros', node_executable='static_transform_publisher',
            arguments=["0.02407", "-0.16119", "0.05839", "0", "0.0", "0",
                       "swift/base_link", "swift/imu_link"]),
        launch_ros.actions.Node(
            # Pressure sensor
            package='tf2_ros', node_executable='static_transform_publisher',
            arguments=["-0.08048", "-0.01294", "0.06524	", "0", "0.0", "0",
                       "swift/base_link", "swift/pressure_link"])
    ])
