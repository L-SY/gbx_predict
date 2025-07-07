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
    
    # Declare launch arguments - these will override YAML config if provided
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='',  # Empty means use YAML config
        description='Input image topic (overrides YAML config if provided)'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='',  # Empty means use YAML config
        description='Output stitched image topic (overrides YAML config if provided)'
    )
    
    debug_topic_arg = DeclareLaunchArgument(
        'debug_topic',
        default_value='',  # Empty means use YAML config
        description='Debug image topic (overrides YAML config if provided)'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'image_stitch_params.yaml']),
        description='Path to the configuration file'
    )
    
    # Build parameters list
    parameters = [LaunchConfiguration('config_file')]
    
    # Only override YAML values if launch arguments are provided (non-empty)
    override_params = {}
    
    # Note: In practice, LaunchConfiguration can't be checked for emptiness at launch time
    # So we'll use a different approach - provide the parameters and let ROS2 handle precedence
    input_topic_val = LaunchConfiguration('input_topic')
    output_topic_val = LaunchConfiguration('output_topic')
    debug_topic_val = LaunchConfiguration('debug_topic')
    
    # Add override parameters (these will take precedence over YAML)
    override_params = {
        'input_topic': input_topic_val,
        'output_topic': output_topic_val,
        'debug_topic': debug_topic_val,
    }
    
    parameters.append(override_params)
    
    # Create the node
    image_stitch_node = Node(
        package='gbx_predict',
        executable='image_stitch_node.py',
        name='image_stitch_node',
        output='screen',
        parameters=parameters,
        remappings=[
            # Topic remappings (alternative way to change topics)
            # Uncomment and modify as needed:
            # ('/camera/image_raw', '/your_camera_topic'),
        ]
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        debug_topic_arg,
        config_file_arg,
        image_stitch_node
    ]) 