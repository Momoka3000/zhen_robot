from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动仿真底层 (amr)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('amr'), 'launch', 'robot_launch.py'))
        ),
        # 2. 启动视觉 (vision)
        Node(package='vision', executable='color_detector'),
        # 3. 启动手臂 (arm_control)
        Node(package='arm_control', executable='arm_controller'),
        # 4. 启动逻辑 (main_logic)
        Node(package='main_logic', executable='task_manager'),
    ])