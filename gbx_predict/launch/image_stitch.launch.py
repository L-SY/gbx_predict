#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the path to the package
    pkg_share = FindPackageShare('gbx_predict')
    
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/hk_camera/cs050/image_raw',
        description='Input image topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/stitched_image',
        description='Output stitched image topic'
    )
    
    debug_topic_arg = DeclareLaunchArgument(
        'debug_topic',
        default_value='/debug_image',
        description='Debug image topic'
    )
    
    use_python_arg = DeclareLaunchArgument(
        'use_python',
        default_value='true',
        description='Use Python node instead of C++ node'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'image_stitch_params.yaml']),
        description='Path to the configuration file'
    )
    
    # Create the node
    image_stitch_node = Node(
        package='gbx_predict',
        executable='image_stitch_node.py',
        name='image_stitch_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            # Launch arguments will override YAML values if provided
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'debug_topic': LaunchConfiguration('debug_topic'),
            }
        ],
        remappings=[
            ('~/input', LaunchConfiguration('input_topic')),
            ('~/output', LaunchConfiguration('output_topic')),
            ('~/debug', LaunchConfiguration('debug_topic')),
        ]
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        debug_topic_arg,
        use_python_arg,
        config_file_arg,
        image_stitch_node
    ]) 