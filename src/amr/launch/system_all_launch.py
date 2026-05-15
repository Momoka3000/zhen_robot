import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    amr_share = get_package_share_directory("amr")
    rviz_config = os.path.join(amr_share, "rviz", "warehouse.rviz")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(amr_share, "launch", "robot_launch.py")
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(amr_share, "launch", "navigation_launch.py")
            )
        ),
        TimerAction(
            period=25.0,
            actions=[
                Node(package="vision", executable="color_detector"),
                Node(package="arm_control", executable="arm_controller"),
                Node(
                    package="main_logic",
                    executable="task_manager",
                    parameters=[
                        {
                            "use_sim_time": True,
                            "auto_charge_when_idle": False,
                        }
                    ],
                ),
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
