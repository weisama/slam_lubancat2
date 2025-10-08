from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    ## ***** Launch arguments *****
    # 定义是否使用仿真时间参数 (默认为False，实际机器人部署时通常设为False)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')

    ## ***** File paths ******
    # 获取cartographer_ros包的共享路径
    pkg_share = FindPackageShare('cartographer_navigation').find('cartographer_navigation')

    # 配置文件的完整路径
    config_dir = os.path.join(pkg_share, 'config')
    config_file = os.path.join(config_dir, 'mylaser.lua')

    # Cartographer SLAM主节点
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        arguments = [
            '-configuration_directory', config_dir,  # 使用自定义配置目录
            '-configuration_basename', 'mylaser.lua'],  # 使用自定义配置文件
        remappings = [
            ('scan', 'scan'),
            ('odom', '/odom_rf2o')],  # 添加话题重映射确保订阅正确的话题
        output = 'screen'
        )

    # Cartographer占用网格节点
    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},  # 硬编码为False (注意：应使用参数更灵活)
            {'resolution': 0.05}],    # 设置地图分辨率为5cm
        )
    
    # 构建启动描述
    return LaunchDescription([
        use_sim_time_arg,  # 时间参数
        cartographer_node,           # SLAM核心节点
        cartographer_occupancy_grid_node,  # 占用网格生成节点
    ])

