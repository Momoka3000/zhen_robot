import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    amr_share = get_package_share_directory("amr")
    rviz_config = os.path.join(amr_share, "rviz", "warehouse.rviz")
    initial_battery_percent = LaunchConfiguration("initial_battery_percent")
    min_battery_to_start = LaunchConfiguration("min_battery_to_start")
    battery_resume_level = LaunchConfiguration("battery_resume_level")
    simulated_charge_rate_percent_per_sec = LaunchConfiguration("simulated_charge_rate_percent_per_sec")
    task_battery_cost_percent = LaunchConfiguration("task_battery_cost_percent")
    use_battery_simulator = LaunchConfiguration("use_battery_simulator")
    charger_yaw_tolerance = LaunchConfiguration("charger_yaw_tolerance")
    charger_align_angular_speed = LaunchConfiguration("charger_align_angular_speed")

    return LaunchDescription([
        DeclareLaunchArgument(
            "initial_battery_percent",
            default_value="100.0",
            description="Initial simulated battery percentage.",
        ),
        DeclareLaunchArgument(
            "min_battery_to_start",
            default_value="30.0",
            description="Minimum battery percentage required before starting a task.",
        ),
        DeclareLaunchArgument(
            "battery_resume_level",
            default_value="80.0",
            description="Battery percentage required to leave charger and resume tasks.",
        ),
        DeclareLaunchArgument(
            "simulated_charge_rate_percent_per_sec",
            default_value="8.0",
            description="Simulated charging rate in percent per second.",
        ),
        DeclareLaunchArgument(
            "task_battery_cost_percent",
            default_value="36.0",
            description="Simulated battery percentage consumed after each completed task.",
        ),
        DeclareLaunchArgument(
            "use_battery_simulator",
            default_value="true",
            description="Start the simulated battery publisher. Set false when using a real battery topic.",
        ),
        DeclareLaunchArgument(
            "charger_yaw_tolerance",
            default_value="0.02",
            description="Yaw tolerance in radians for charger tail alignment.",
        ),
        DeclareLaunchArgument(
            "charger_align_angular_speed",
            default_value="0.10",
            description="Maximum angular speed used for charger tail alignment.",
        ),
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
                    executable="battery_simulator",
                    condition=IfCondition(use_battery_simulator),
                    parameters=[
                        {
                            "use_sim_time": True,
                            "initial_battery_percent": initial_battery_percent,
                            "simulated_charge_rate_percent_per_sec": simulated_charge_rate_percent_per_sec,
                            "task_battery_cost_percent": task_battery_cost_percent,
                        }
                    ],
                ),
                Node(
                    package="main_logic",
                    executable="task_manager",
                    parameters=[
                        {
                            "use_sim_time": True,
                            "auto_charge_when_idle": False,
                            "initial_battery_percent": initial_battery_percent,
                            "min_battery_to_start": min_battery_to_start,
                            "battery_resume_level": battery_resume_level,
                            "charger_yaw_tolerance": charger_yaw_tolerance,
                            "charger_align_angular_speed": charger_align_angular_speed,
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
