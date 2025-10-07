from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取功能包路径
    pkg_share = FindPackageShare('karto_slam')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'karto_slam_params.yaml']),
            description='Full path to the parameter file'
        ),
        
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('scan', '/scan'),  # 确保订阅正确的激光雷达话题
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        )
    ])
