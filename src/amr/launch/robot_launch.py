import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 设置路径
    package_name = 'amr'
    pkg_share = get_package_share_directory(package_name)
    world_path = os.path.join(pkg_share, 'worlds', 'simple_warehouse.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'amr_robot.urdf')

    # 2. 启动 Gazebo Server (后端)
    # 注意：这里加入了环境变量，防止虚拟机闪退
    start_gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen',
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1', 'SVGA_VGPU10': '0'}
    )

    # 3. 启动 Gazebo Client (界面)
    start_gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1', 'SVGA_VGPU10': '0'}
    )

    # 4. 启动 Robot State Publisher (解决你 RViz 没模型的问题)
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_path],
        output='screen'
    )

    # 5. 自动生成机器人到 Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'amr_robot', '-file', urdf_path, '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen'
    )

    return LaunchDescription([
        start_gzserver,
        start_gzclient,
        start_robot_state_publisher,
        spawn_entity
    ])