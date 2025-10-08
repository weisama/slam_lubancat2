import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

            Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom_rf2o',    # 发布的坐标话题
                    'publish_tf' : False,           # 是否发布 odom -> base_link
                    'base_frame_id' : 'base_link',  # 机器人坐标系
                    'odom_frame_id' : 'odom',       # 坐标话题的坐标系
                    'init_pose_from_topic' : '',    # 初始位姿
                    'freq' : 10.0                   # 运行频率保持和雷达一致
                    }],
            ),
    ])
