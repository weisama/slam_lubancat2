#!/usr/bin/python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # === 1. 获取用户参数文件路径 ===
    user_ws = os.path.expanduser('~/slam_ws/src/upper/config')
    param_file = os.path.join(user_ws, 'params.yaml')
    
    if os.path.exists(param_file):
        with open(param_file, 'r') as f:
            user_params = yaml.safe_load(f)
        lidar_name = user_params.get('lidar_name', 'N10')
        scan_topic = user_params.get('scan_topic', '/scan_raw')
        serial_port_ = user_params.get('serial_port_', '/dev/usb_robot')
    else:
        lidar_name = 'N10'
        scan_topic = '/scan_raw'
        serial_port_ = '/dev/usb_robot'
        print(f"[WARN] {param_file} 不存在，使用默认参数")

    # === 2. 根据雷达型号设置 target_size ===
    if lidar_name == "N10":
        target_size = 450
    elif lidar_name == "N10_P":
        target_size = 530
    elif lidar_name == "M10":
        target_size = 1000
    elif lidar_name == "M10_P":
        target_size = 1667
    else:
        target_size = 450
        print(f"[WARN] Unknown lidar_name={lidar_name}, default target_size=450")

    # === 3. driver YAML 文件路径 ===
    driver_yaml = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lsx10.yaml')

    # === 4. 启动 lslidar_driver LifecycleNode ===
    lslidar_driver_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[
            driver_yaml,
            {
                'lidar_name': lidar_name,
                'scan_topic': scan_topic,
                'serial_port_': serial_port_
            }
        ],
    )

    # === 5. 启动 LidarFilter 节点 ===
    lidar_filter_node = Node(
        package='lidar_filter',
        executable='lidar_filter_node',
        name='lidar_filter',
        output='screen',
        parameters=[{
            'target_size': target_size,
            'scan_topic': scan_topic
        }]
    )

    # === 6. 启动 TF broadcaster ===
    tf_launch_file = os.path.join(get_package_share_directory('tf'), 'launch', 'tf_broadcaster.launch.py')
    tf_broadcaster_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf_launch_file)
    )

    # === 7. 返回 LaunchDescription ===
    return LaunchDescription([
        lslidar_driver_node,
        lidar_filter_node,
        tf_broadcaster_node
    ])
