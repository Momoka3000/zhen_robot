from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,  # 仿真环境必须设为 True
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'minimum_laser_range': 0.2,
                'maximum_laser_range': 12.0,
                'transform_timeout': 0.5,
                'tf_buffer_duration': 30.0,
                'mode': 'mapping'
            }]
        )
    ])
