import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("amr")
    world_path = os.path.join(pkg_share, "worlds", "warehouse1.world")
    urdf_path = os.path.join(pkg_share, "urdf", "amr_robot.urdf")

    with open(urdf_path, "r", encoding="utf-8") as file:
        robot_description = file.read()

    gazebo_env = {
        "LIBGL_ALWAYS_SOFTWARE": "1",
        "SVGA_VGPU10": "0",
    }

    start_gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world_path,
        ],
        output="screen",
        additional_env=gazebo_env,
    )

    start_gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        additional_env=gazebo_env,
    )

    start_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "amr_robot",
            "-topic",
            "robot_description",
            "-x",
            "3.9",
            "-y",
            "-4.35",
            "-z",
            "0.5",
            "-Y",
            "1.5708",
        ],
        output="screen",
    )

    delayed_spawn_entity = TimerAction(
        period=8.0,
        actions=[spawn_entity],
    )

    return LaunchDescription([
        start_gzserver,
        start_gzclient,
        start_robot_state_publisher,
        delayed_spawn_entity,
    ])
