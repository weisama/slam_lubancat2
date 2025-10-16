import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('karto_slam')
    
    # 设置参数文件路径
    params_file = LaunchConfiguration('params_file')
    
    # 声明启动参数
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'my_parameter.yaml']),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )
    
    # 设置ROS域名（可选）
    set_ros_domain_cmd = SetEnvironmentVariable(
        'ROS_DOMAIN_ID',
        '0'
    )
    
    # SLAM Toolbox节点 - 在线同步建图模式
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/odom', '/odom_rf2o'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加动作
    ld.add_action(set_ros_domain_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(slam_toolbox_node)
    
    return ld