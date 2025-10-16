from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # === 1. 获取各包路径 ===
    lslidar_driver_dir = get_package_share_directory('lslidar_driver')
    tf_broadcaster_dir = get_package_share_directory('tf')

    # === 2. 各 launch 文件路径 ===
    lslidar_launch = os.path.join(lslidar_driver_dir, 'launch', 'lslidar_launch.py')
    tf_broadcaster_launch = os.path.join(tf_broadcaster_dir, 'launch', 'tf_broadcaster.launch.py')

    # === 3. 启动 lslidar_driver ===
    lslidar_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lslidar_launch)
    )

    # === 4. 启动 lidar_filter 节点 ===
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

    # === 5. 启动 tf_broadcaster ===
    tf_broadcaster_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf_broadcaster_launch)
    )

    # === 6. 返回 LaunchDescription ===
    return LaunchDescription([
        lslidar_driver_node,
        lidar_filter_node,
        tf_broadcaster_node
    ])