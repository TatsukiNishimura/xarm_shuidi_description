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

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import xacro
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

gazebo_ros2_control_demos_path = os.path.join(
    get_package_share_directory("xarm_shuidi_description"),
)
xacro_file = os.path.join(
    gazebo_ros2_control_demos_path, "urdf", "xarm_shuidi.urdf.xacro"
)
urdf_path = os.path.join(gazebo_ros2_control_demos_path, "urdf", "xarm_shuidi.urdf")


def generate_launch_description():
    show_rviz = LaunchConfiguration("show_rviz", default=True)
    world_file = os.path.join(gazebo_ros2_control_demos_path, "world", "empty.world")

    rviz_config_path = os.path.join(
        gazebo_ros2_control_demos_path, "rviz", "show_urdf.rviz"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
        launch_arguments={"world": world_file}.items(),
    )

    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent="  ")
    f = open(urdf_path, "w")
    f.write(robot_desc)
    f.close()
    params = {"robot_description": robot_desc}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "diff_drive"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "diff_drive_base_controller",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="both",
        arguments=["-d", [rviz_config_path]],
        condition=IfCondition(show_rviz),
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[
                        load_joint_state_controller,
                    ],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_joint_trajectory_controller],
                )
            ),
            gazebo,
            node_robot_state_publisher,
            spawn_entity,
            rviz,
        ]
    )
