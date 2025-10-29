import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    # ==== 获取参数 ====
    tcp_ip = LaunchConfiguration('tcp_ip').perform(context)
    tcp_port = int(LaunchConfiguration('tcp_port').perform(context))     # ✅ 转为整数
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    play_bag_flag = LaunchConfiguration('play_bag').perform(context).lower() == 'true'

    # ==== ROS2 节点 ====
    upper_node = Node(
        package='upper',
        executable='upper_node',   # ✅ 可执行文件名称
        name='upper',
        parameters=[{
            'tcp_ip': tcp_ip,
            'tcp_port': tcp_port
        }],
        output='screen'
    )

    actions = [upper_node]

    # ==== 选择是否播放 bag 文件 ====
    if play_bag_flag:
        bag_dir = os.path.expanduser('~/slam_ws/src/upper/bag')
        bag_files = [os.path.join(bag_dir, f) for f in os.listdir(bag_dir) if f.endswith('.db3')]
        if bag_files:
            play_bag = ExecuteProcess(
                cmd=['ros2', 'bag', 'play', '--loop'] + bag_files,
                output='screen'
            )
            actions.append(play_bag)
        else:
            print(f"[WARN] No bag files found in {bag_dir}")

    # ==== 选择是否启动 RViz ====
    if use_rviz:
        rviz_file = os.path.expanduser('~/slam_ws/src/lslidar_driver/rviz/lslidar.rviz')
        if os.path.exists(rviz_file):
            rviz_proc = ExecuteProcess(
                cmd=['rviz2', '-d', rviz_file],
                output='screen'
            )
            actions.append(rviz_proc)
        else:
            print(f"[WARN] RViz file not found: {rviz_file}")

    return actions


def generate_launch_description():
    # ==== 参数声明 ====
    tcp_ip_arg = DeclareLaunchArgument(
        'tcp_ip',
        default_value='0.0.0.0',
        description='TCP listen IP (use 0.0.0.0 to listen on all interfaces)'
    )
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='6666',
        description='TCP listen port (integer)'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz2 (true/false)'
    )
    play_bag_arg = DeclareLaunchArgument(
        'play_bag',
        default_value='false',
        description='Whether to play bag file in loop (true/false)'
    )

    return LaunchDescription([
        tcp_ip_arg,
        tcp_port_arg,
        use_rviz_arg,
        play_bag_arg,
        OpaqueFunction(function=launch_setup)
    ])

