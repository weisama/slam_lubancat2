# launch/gmapping_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 参数文件路径 - 固定路径
    params_file = PathJoinSubstitution([
        FindPackageShare('gmapping_launch'),
        'config',
        'my_parameter.yaml'
    ])
    
    return LaunchDescription([
        # 启动slam_gmapping节点
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[params_file],
            # 固定重映射到指定的里程计话题
            remappings=[
                ('scan', 'scan'),  # 激光雷达话题保持默认
                ('odom', '/odom_rf2o'),    # 重映射odom话题
            ]
        )
    ])