import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的路径
    pkg_amr = get_package_share_directory('amr')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 定义文件路径
    urdf_file = os.path.join(pkg_amr, 'urdf', 'amr_robot.urdf')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. 启动 Gazebo 服务
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        )
    )

    # 2. 启动 Gazebo 界面
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 3. 启动 robot_state_publisher (发布 TF 坐标树)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file).read()
        }]
    )

    # 4. 运行 spawn_entity 脚本将机器人放入 Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'amr_robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher,
        spawn_entity
    ])