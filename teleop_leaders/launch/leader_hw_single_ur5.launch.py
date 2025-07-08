#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
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
#
# Author: Wonho Yoon, Sungho Woo

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. "
                        "If changed, then also joint names in the controllers' configuration "
                        "must be updated.",
        ),
        DeclareLaunchArgument(
            "usb_port",
            default_value="/dev/ttyUSB0",
            description="USB port for the robot.",
        ),
        DeclareLaunchArgument(
            "feedback_param_yaml",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("teleop_leaders"),
                    "config",
                    "feedback_param_ur5.yaml",
                ]
            ),
            description="Feedback parameter file.",
        ),
        # DeclareLaunchArgument(
        #     "description_file",
        #     default_value="leaders_1_arm.urdf.xacro",
        #     description="URDF/XACRO description file with the robot.",
        # ),
    ]
    
    usb_port = LaunchConfiguration("usb_port")
    feedback_param_yaml = LaunchConfiguration("feedback_param_yaml")

    # Launch configurations
    description_file = PathJoinSubstitution(
        [
            FindPackageShare("teleop_leaders"),
            "description",
            "urdf",
            "leader_ur5.urdf.xacro",
        ]
    )

    #description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")

    # Robot controllers config file path
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("teleop_leaders"),
            "config",
            "ros2_controllers_leader.yaml",
        ]
    )

    # Robot description from Xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "prefix:=",
            prefix,
            " ",
            "usb_port:=",
            usb_port,
            # maybe add yaml_file here
        ]
    )
    robot_description = {"robot_description": robot_description_content, 
                         "feedback_param_yaml": feedback_param_yaml}


    # ros2_control Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            robot_controllers],
        output="both",
        #arguments=['--ros-args', '--log-level', 'debug'],
    )


    # Controller spawner node
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "feedback_controller",
            "joint_state_broadcaster",
            '--ros-args', '--log-level', 'debug'
        ],
        parameters=[robot_description],
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Wrap everything in a namespace 'leader'
    leader_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('leaders'),
            control_node,
            robot_controller_spawner,
            robot_state_publisher_node,
        ]
    )

    # Return combined LaunchDescription
    return LaunchDescription(declared_arguments + [leader_with_namespace])
