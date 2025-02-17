# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

namespace_value = 'camera1' #Should work in theory. If not, just edit them all manually lol

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'camera_width',
            default_value='1280',
            description='The USB camera input width.'),
        DeclareLaunchArgument(
            'camera_height',
            default_value='720',
            description='The USB camera input height.'),
    ]

    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')

    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='rectify',
        namespace=namespace_value, #CHANGE HERE
        parameters=[{
            'output_width': camera_width,
            'output_height': camera_height,
        }]
    )

    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace=namespace_value, #CHANGE HERE
        remappings=[
            ('image', 'image_rect'),
            ('camera_info', 'camera_info_rect')
        ]
    )

    usb_cam_params_path = os.path.join(
        get_package_share_directory('isaac_ros_apriltag'),
        'config',
        'usb_cam_params.yaml' #CHANGE HERE TO PARAMS FILE NAME (stored at /opt/ros/humble/share/issac_ros_apriltag/config)
    )
    usb_cam_node = ComposableNode(
        package='usb_cam',
        plugin='usb_cam::UsbCamNode',
        name='usb_cam',
        namespace=namespace_value, #ADD THIS LINE HERE, IT IS NOT HERE ORIGINALLY SO YOU MUST TYPE OUT THE WHOLE THING
        parameters=[usb_cam_params_path]
    )

    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace=namespace_value, #CHANGE HERE
        executable='component_container_mt',
        composable_node_descriptions=[
            rectify_node,
            apriltag_node,
            usb_cam_node
        ],
        output='screen'
    )

    return launch.LaunchDescription(launch_args + [apriltag_container])