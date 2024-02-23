# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

gazebo_ros2_control_demos_path = os.path.join(
    get_package_share_directory("xarm_shuidi_description"),
)
xacro_file = os.path.join(
    gazebo_ros2_control_demos_path, "urdf", "xarm_shuidi.urdf.xacro"
)
urdf_path = os.path.join(gazebo_ros2_control_demos_path, "urdf", "xarm_shuidi.urdf")


def generate_launch_description():
    start_rviz = LaunchConfiguration("start_rviz", default=True)

    rviz_config_path = os.path.join(
        gazebo_ros2_control_demos_path, "rviz", "show_urdf.rviz"
    )
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent="  ")
    f = open(urdf_path, "w")
    f.write(robot_desc)
    f.close()
    params = {"robot_description": robot_desc}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        # parameters=[
        #     {
        #         "source_list": [
        #             "{}{}/joint_states".format(
        #                 prefix.perform(context), hw_ns.perform(context)
        #             )
        #         ]
        #     }
        # ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="both",
        arguments=["-d", [rviz_config_path]],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz,
        ]
    )
