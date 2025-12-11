import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    # Nodes ==========================================================
    node_viewer = Node(
        package='tb3_yolo_camera',
        executable='tb3_yolo_viewer',
        name='tb3_yolo_viewer',
        parameters=[{'topic': '/yolo/image/compressed', 'window': 'view_yolo'}],
        output='screen')
    ld.add_action(node_viewer)

    node_yolo = Node(
        package="tb3_yolo_tasks",
        executable="tb3_yolo_yolo",
        name = 'tb3_yolo_yolo',
        output="screen",
    )
    ld.add_action(node_yolo)
    
    return ld