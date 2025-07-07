#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the path to the package
    pkg_share = FindPackageShare('gbx_predict')
    
    # Configuration file path
    config_file = PathJoinSubstitution([pkg_share, 'config', 'image_stitch_params.yaml'])
    
    # Create the image stitching node
    image_stitch_node = Node(
        package='gbx_predict',
        executable='image_stitch_node.py',
        name='image_stitch_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            # Remap topics if needed
            # ('/camera/image_raw', '/your_camera_topic'),
        ]
    )
    
    return LaunchDescription([
        image_stitch_node
    ]) 