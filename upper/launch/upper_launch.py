import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 默认串口，可通过 launch 参数覆盖
    serial_port = "/dev/ttyAMA0"
    baud_rate = 921600

    bag_dir = os.path.expanduser('~/slam_ws/src/upper/bag')
    rviz_file = os.path.expanduser('~/slam_ws/src/lslidar_driver/rviz/lslidar.rviz')

    bag_files = [os.path.join(bag_dir, f) for f in os.listdir(bag_dir) if f.endswith('.db3')]

    return LaunchDescription([
        Node(
            package='upper',
            executable='upper',
            name='upper',
            parameters=[{'serial_port': serial_port, 'baud_rate': baud_rate}],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play'] + bag_files if bag_files else [],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_file],
            output='screen'
        )
    ])
