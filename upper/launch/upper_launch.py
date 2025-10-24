import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ==== 参数声明 ====
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyS3',
        description='Serial port path (e.g. /dev/ttyUSB0)'
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='921600',
        description='Serial baud rate'
    )
    tcp_ip_arg = DeclareLaunchArgument(
        'tcp_ip',
        default_value='192.168.0.1',
        description='TCP listen IP (use 0.0.0.0 to listen on all interfaces)'
    )
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='9999',
        description='TCP listen port'
    )

    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    tcp_ip = LaunchConfiguration('tcp_ip')
    tcp_port = LaunchConfiguration('tcp_port')

    # ==== ROS2 节点 ====
    upper_node = Node(
        package='upper',
        executable='upper',
        name='upper',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'tcp_ip': tcp_ip,
            'tcp_port': tcp_port
        }],
        output='screen'
    )

    # ==== 播放 bag 文件 ====
    bag_dir = os.path.expanduser('~/slam_ws/src/upper/bag')
    bag_files = [os.path.join(bag_dir, f) for f in os.listdir(bag_dir) if f.endswith('.db3')]
    play_bag = None
    if bag_files:
        play_bag = ExecuteProcess(
            cmd=['ros2', 'bag', 'play'] + bag_files,
            output='screen'
        )

    # ==== 启动 RViz ====
    rviz_file = os.path.expanduser('~/slam_ws/src/lslidar_driver/rviz/lslidar.rviz')
    rviz_proc = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_file],
        output='screen'
    )

    # ==== LaunchDescription ====
    actions = [
        serial_port_arg, baud_rate_arg,
        tcp_ip_arg, tcp_port_arg,
        upper_node, rviz_proc
    ]
    if play_bag:
        actions.append(play_bag)

    return LaunchDescription(actions)

