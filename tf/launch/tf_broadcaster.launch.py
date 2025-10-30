from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from math import radians
import yaml

def generate_launch_description():

    pkg_share = FindPackageShare('upper')
    param_file = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])

    def launch_nodes(context, *args, **kwargs):
        # 读取 YAML 文件
        param_path = param_file.perform(context)
        try:
            with open(param_path, 'r') as f:
                params = yaml.safe_load(f)
        except Exception:
            params = {}

        # 获取 base_link -> laser_link 参数，如果不存在就用 0
        x = float(params.get('x', 0)) / 100.0       # cm -> m
        y = float(params.get('y', 0)) / 100.0
        z = float(params.get('z', 0)) / 100.0
        roll = radians(float(params.get('roll', 0)))   # deg -> rad
        pitch = radians(float(params.get('pitch', 0)))
        yaw = radians(float(params.get('yaw', 0)))

        # 仿真时间参数
        use_sim_time = params.get('use_sim_time', False)

        return [
            # map -> odom
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_broadcaster',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['--x', '0', '--y', '0', '--z', '0',
                           '--roll', '0', '--pitch', '0', '--yaw', '0',
                           '--frame-id', 'map', '--child-frame-id', 'odom'],
                output='screen'
            ),
            # odom -> base_link
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='odom_to_base_broadcaster',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['--x', '0', '--y', '0', '--z', '0',
                           '--roll', '0', '--pitch', '0', '--yaw', '0',
                           '--frame-id', 'odom', '--child-frame-id', 'base_link'],
                output='screen'
            ),
            # base_link -> laser_link
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_laser_broadcaster',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '--x', str(x),
                    '--y', str(y),
                    '--z', str(z),
                    '--roll', str(roll),
                    '--pitch', str(pitch),
                    '--yaw', str(yaw),
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'laser_link'
                ],
                output='screen'
            )
        ]

    return LaunchDescription([
        OpaqueFunction(function=launch_nodes)
    ])

