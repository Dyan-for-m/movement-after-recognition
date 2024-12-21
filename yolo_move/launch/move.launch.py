from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取yolo_bringup包的路径
    yolo_bringup_path = os.path.join(
        get_package_share_directory('yolo_bringup'),
        'launch',
        'yolo.launch.py'
    )

    return LaunchDescription([
        # 启动yolo.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yolo_bringup_path)
        ),

        # 启动move节点
        ExecuteProcess(
            cmd=['ros2', 'run', 'yolo_move', 'move'],
            output='screen'
        )
    ])