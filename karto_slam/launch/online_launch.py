import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml

def launch_setup(context, *args, **kwargs):
    # 获取包路径
    pkg_dir = get_package_share_directory('karto_slam')
    
    # 参数优先级：launch参数 > params.yaml > my_parameter.yaml
    mode_launch_arg = LaunchConfiguration('mode').perform(context)
    sync_async_launch_arg = LaunchConfiguration('sync_async').perform(context)
    
    # 读取params.yaml文件
    params_yaml_path = os.path.expanduser('~/slam_ws/src/upper/config/params.yaml')
    mode_from_params = None
    sync_async_from_params = None
    
    if os.path.exists(params_yaml_path):
        with open(params_yaml_path, 'r') as f:
            params_data = yaml.safe_load(f)
            mode_from_params = params_data.get('mode')
            sync_async_from_params = params_data.get('sync_async')
        print(f"[INFO] 从 {params_yaml_path} 读取: mode={mode_from_params}, sync_async={sync_async_from_params}")
    else:
        print(f"[WARN] 参数文件不存在: {params_yaml_path}")
    
    # 确定最终使用的mode和sync_async
    final_mode = mode_launch_arg if mode_launch_arg else mode_from_params
    final_sync_async = sync_async_launch_arg if sync_async_launch_arg else sync_async_from_params
    
    # 读取默认参数文件以获取雷达话题和其他参数
    default_params_file = LaunchConfiguration('params_file')
    default_params_path = default_params_file.perform(context)
    
    scan_topic = "scan"  # 默认值
    other_params = {}
    
    if os.path.exists(default_params_path):
        with open(default_params_path, 'r') as f:
            default_params_data = yaml.safe_load(f)
            # 如果launch参数和params.yaml都没有设置，使用my_parameter.yaml中的默认值
            if final_mode is None:
                final_mode = default_params_data.get('mode', 'mapping')
            if final_sync_async is None:
                final_sync_async = default_params_data.get('sync_async', 'async')
            
            # 读取雷达话题
            scan_topic = default_params_data.get('scan_topic', 'scan')
            
            # 保存其他参数用于调试
            other_params = default_params_data
            
        print(f"[INFO] 从 {default_params_path} 读取参数文件")
    else:
        print(f"[WARN] 默认参数文件不存在: {default_params_path}")
    
    # 设置默认值
    if final_mode is None:
        final_mode = 'mapping'
    if final_sync_async is None:
        final_sync_async = 'async'
    
    # 打印关键参数信息
    print("\n" + "="*50)
    print("SLAM 配置参数汇总:")
    print("="*50)
    print(f"运行模式 (mode): {final_mode}")
    print(f"同步模式 (sync_async): {final_sync_async}")
    print(f"雷达话题 (scan_topic): {scan_topic}")
    print(f"里程计话题 (odom_topic): /odom_rf2o")
    print(f"参数来源:")
    if mode_launch_arg:
        print(f"  - mode: 来自launch参数")
    elif mode_from_params is not None:
        print(f"  - mode: 来自params.yaml")
    else:
        print(f"  - mode: 来自my_parameter.yaml或默认值")
    
    if sync_async_launch_arg:
        print(f"  - sync_async: 来自launch参数")
    elif sync_async_from_params is not None:
        print(f"  - sync_async: 来自params.yaml")
    else:
        print(f"  - sync_async: 来自my_parameter.yaml或默认值")
    print("="*50 + "\n")
    
    # 根据sync_async选择可执行文件
    if final_sync_async == 'async':
        executable_name = 'async_slam_toolbox_node'
        print(f"[INFO] 使用异步模式: {executable_name}")
    else:
        executable_name = 'sync_slam_toolbox_node'
        print(f"[INFO] 使用同步模式: {executable_name}")
    
    # 创建参数字典
    from launch_ros.parameter_descriptions import ParameterFile
    param_file = ParameterFile(default_params_file, allow_substs=True)
    
    # SLAM Toolbox节点
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable=executable_name,
        name='slam_toolbox',
        output='screen',
        parameters=[param_file, {'mode': final_mode}],
        remappings=[
            ('/odom', '/odom_rf2o'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    return [slam_toolbox_node]

def generate_launch_description():
    # 声明启动参数
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([get_package_share_directory('karto_slam'), 'config', 'my_parameter.yaml']),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )
    
    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='',  # 空字符串表示未设置
        description='SLAM mode: mapping or localization (overrides params.yaml)'
    )
    
    declare_sync_async_cmd = DeclareLaunchArgument(
        'sync_async',
        default_value='',  # 空字符串表示未设置
        description='Sync/Async mode: sync or async (overrides params.yaml)'
    )
    
    # 设置ROS域名（可选）
    set_ros_domain_cmd = SetEnvironmentVariable(
        'ROS_DOMAIN_ID',
        '0'
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加动作
    ld.add_action(set_ros_domain_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_sync_async_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld