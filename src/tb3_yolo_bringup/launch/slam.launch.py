import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    # Paths ==========================================================
    pkg_tb3_yolo_bringup = FindPackageShare("tb3_yolo_bringup")
    pkg_turtlebot3_cartographer = FindPackageShare("turtlebot3_cartographer")

    # Launch Arguments / Configs ==========================================================
    # Sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    ld.add_action(arg_use_sim_time)
    
    # Cartographer config dir
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir',
        default=PathJoinSubstitution([pkg_tb3_yolo_bringup, 'params']))
    arg_cartographer_config_dir = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=cartographer_config_dir,
        description='Full path to config file to load')
    ld.add_action(arg_cartographer_config_dir)

    # Cartographer basename
    configuration_basename = LaunchConfiguration('configuration_basename', default='slam.lua')
    arg_configuration_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value=configuration_basename,
        description='Name of lua file for cartographer')
    ld.add_action(arg_configuration_basename)

    # Resolution
    resolution = LaunchConfiguration('resolution', default='0.02')
    arg_resolution = DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid')
    ld.add_action(arg_resolution)

    # Period sec for occupancy grid
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    arg_period_sec = DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period')
    ld.add_action(arg_period_sec)

    # Launch Files ==========================================================
    launch_cartographer = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_turtlebot3_cartographer,
                    'launch',
                    'cartographer.launch.py'
                ])
            ),
            launch_arguments={
                'cartographer_config_dir': cartographer_config_dir,
                'configuration_basename': configuration_basename,
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec,
            }.items(),
        )
    ld.add_action(launch_cartographer)

    return ld