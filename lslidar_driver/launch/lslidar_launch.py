#!/usr/bin/python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    # === 1. 获取参数文件路径 ===
    user_ws = os.path.expanduser('~/slam_ws/src/upper/config')
    param_file = os.path.join(user_ws, 'params.yaml')
    
    # 读取用户参数
    if os.path.exists(param_file):
        with open(param_file, 'r') as f:
            user_params = yaml.safe_load(f)
        lidar_name = user_params.get('lidar_name', 'N10')  # 默认 N10
        scan_topic = user_params.get('scan_topic', '/scan_raw')  # 默认 /scan_raw
        serial_port_ = user_params.get('serial_port_', '/dev/usb_robot')  # 默认 /dev/usb_robot
    else:
        lidar_name = 'N10'
        scan_topic = '/scan_raw'
        serial_port_ = '/dev/usb_robot'
        print(f"[WARN] {param_file} 不存在，使用默认参数")

    # === 2. 驱动 YAML 文件路径 ===
    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lsx10.yaml')

    # === 3. 启动 LifecycleNode ===
    driver_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[
            driver_dir,
            {
                'lidar_name': lidar_name,
                'scan_topic': scan_topic,
                'serial_port_': serial_port_
            }
        ],
    )

    return LaunchDescription([driver_node])
