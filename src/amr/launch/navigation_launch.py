import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 定义文件路径
    amr_pkg = get_package_share_directory('amr')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    
    map_yaml_file = os.path.join(amr_pkg, 'map', 'warehouse_map.yaml')
    params_file = os.path.join(amr_pkg, 'config', 'nav2_params.yaml')

    # 2. 声明 Launch 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_dir = LaunchConfiguration('map', default=map_yaml_file)

    # 3. 包含官方的 Nav2 启动逻辑（最稳定的方式）
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        nav2_bringup_launch
    ])