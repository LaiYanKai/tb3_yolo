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

    # Launch Arguments / Configs ==========================================================
    map = LaunchConfiguration('map',
        default=PathJoinSubstitution([
            pkg_tb3_yolo_bringup,
            'maps',
            'map.yaml'
        ])
    )
    arg_map = DeclareLaunchArgument(
            'map',
            default_value=map,
            description='Full path to map file to load')
    ld.add_action(arg_map)

    params_file = LaunchConfiguration('params_file',
        default=PathJoinSubstitution([
            pkg_tb3_yolo_bringup,
            'params',
            'run.yaml'
        ])
    )
    arg_params_file = DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to map file to load')
    ld.add_action(arg_params_file)

    # Other Launch Files ==========================================================
    launch_localize = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_tb3_yolo_bringup,
                    'launch',
                    'nav2_localize.launch.py'
                ])
            ),
            launch_arguments={
                'map': map,
                'params_file': params_file}.items(),
        )
    ld.add_action(launch_localize)

    launch_view_yolo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_tb3_yolo_bringup,
                    'launch',
                    'view_yolo.launch.py'
                ])
            )
        )
    ld.add_action(launch_view_yolo)

    # Nodes ==========================================================
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', 
            PathJoinSubstitution([
                pkg_tb3_yolo_bringup,
                'rviz',
                'run.rviz'
            ])
        ],
        parameters=[{'use_sim_time': False}],
        output='screen')
    ld.add_action(node_rviz)


    node_mapper = Node(
        package="tb3_yolo_mapper",
        executable="tb3_yolo_mapper",
        output="screen",
        parameters=[params_file,
            {
                "filepath": map
            },  # override the filepath param (i.e. path to map)
        ],
    )
    ld.add_action(node_mapper)
    

    node_planner = Node(
        package="tb3_yolo_planner",
        executable="tb3_yolo_planner",
        output="screen",
        parameters=[params_file],
    )
    ld.add_action(node_planner)

    node_behavior = Node(
        package="tb3_yolo_tasks",
        executable="tb3_yolo_behavior",
        output="screen",
        parameters=[params_file],
    )
    ld.add_action(node_behavior)


    node_controller = Node(
        package="tb3_yolo_tasks",
        executable="tb3_yolo_controller",
        output="screen",
        parameters=[params_file],
    )
    ld.add_action(node_controller)

    return ld