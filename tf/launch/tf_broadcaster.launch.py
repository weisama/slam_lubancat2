from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 发布 map -> odom 的变换
        # 在实际应用中，此变换通常由里程计或 SLAM 节点（如你的 Cartographer）动态发布。
        # 此处发布一个静态初始变换，作为SLAM建图的起点。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        # 发布 odom -> base_link 的变换
        # 同样，此变换通常由里程计节点提供。此处发布静态变换意味着你暂时假设机器人底盘没有移动。
        # 这对于调试传感器关系是可行的，但长期运行需替换为真实的里程计源。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),
        # 发布 base_link -> laser_link 的变换
        # 这是你激光雷达在机器人上的**固定安装位置**，是使用静态变换发布器的典型场景。
        # 参数含义：x=0米, y=0米, z=0米, 绕roll轴旋转0度, 绕pitch轴旋转0度, 绕yaw轴旋转0度。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'],
            output='screen'
        ),
    ])
