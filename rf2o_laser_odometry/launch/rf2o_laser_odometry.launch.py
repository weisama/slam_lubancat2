import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 声明一个 launch 参数：是否发布 tf
    declare_publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',  # 默认发布 tf
        description='Whether to publish odom -> base_link transform'
    )

    # 创建 launch description
    return LaunchDescription([
        declare_publish_tf_arg,

        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'publish_tf': LaunchConfiguration('publish_tf'),  # 使用 launch 参数
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 10.0
            }],
        ),
    ])
