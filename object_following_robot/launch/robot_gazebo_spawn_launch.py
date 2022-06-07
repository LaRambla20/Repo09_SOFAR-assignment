# Copyright (c) 2018 Intel Corporation
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
from ament_index_python.packages import get_package_share_directory #

import launch.actions
import launch_ros.actions
from launch_ros.actions import Node

import os #


def generate_launch_description():

    # Sdf model path from package share directory
    sdf_model_path = 'models/two_wheeled_robot_description/model.sdf' #

    # Get the launch directory
    bringup_dir = get_package_share_directory('object_following_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    sdf_model_path = os.path.join(bringup_dir, sdf_model_path)

    return LaunchDescription([
        # TODO(orduno) might not be necessary to have it's own package
        launch_ros.actions.Node(
            package='nav2_gazebo_spawner',
            executable='nav2_gazebo_spawner',
            output='screen',
            arguments=[
                '--robot_name', launch.substitutions.LaunchConfiguration('robot_name'),
                '--robot_namespace', launch.substitutions.LaunchConfiguration('robot_name'),
                '--sdf', sdf_model_path, #
                #'--turtlebot_type', launch.substitutions.LaunchConfiguration('turtlebot_type'),
                '-x', launch.substitutions.LaunchConfiguration('x_pose'),
                '-y', launch.substitutions.LaunchConfiguration('y_pose'),
                '-z', launch.substitutions.LaunchConfiguration('z_pose')]),
    ])
