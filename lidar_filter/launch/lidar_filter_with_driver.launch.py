from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 lslidar_driver 包的路径
    lslidar_driver_dir = get_package_share_directory('lslidar_driver')
    lslidar_launch = os.path.join(lslidar_driver_dir, 'launch', 'lslidar_launch.py')

    # 启动 lslidar_driver
    lslidar_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lslidar_launch)
    )

    # 启动 lidar_filter 节点
    lidar_filter_node = Node(
        package='lidar_filter',
        executable='lidar_filter_node',
        name='lidar_filter',
        output='screen',
        remappings=[
            ('/scan_raw', '/scan_raw'),  # 输入
            ('/scan', '/scan')           # 输出
        ]
    )

    return LaunchDescription([
        lslidar_driver_node,
        lidar_filter_node
    ])

