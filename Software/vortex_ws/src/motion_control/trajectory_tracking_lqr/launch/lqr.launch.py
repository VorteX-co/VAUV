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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectory_tracking_lqr',
            node_executable='trajectory_tracking_lqr',
            node_name='controller_node',
            parameters=[
                get_package_share_directory(
                    'trajectory_tracking_lqr') + '/params/config.yaml'
            ],
            output='screen'
        )
    ])
