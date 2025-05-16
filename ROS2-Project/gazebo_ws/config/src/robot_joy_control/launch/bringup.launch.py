from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy_node'),
        Node(package='robot_joy_control', executable='joy_control', name='joy_control'),
        Node(package='robot_joy_control', executable='main_control', name='main_control'),
        #Node(package)
    ])
