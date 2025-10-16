include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- 坐标系设置
  map_frame = "map",  -- 全局地图坐标系，发布tf变换的起始点，坐标系A
  tracking_frame = "base_link",  -- SLAM算法跟踪的机器人基座坐标系
  published_frame = "base_link",  -- 通常是base_link，发布tf变换的结束点，坐标系C
  odom_frame = "odom",  -- 发布tf变换的中点，坐标系C
  
  -- 功能开关
  provide_odom_frame = true,  -- 是否添加中间点B，A->C 或 A->B->C
  publish_frame_projected_to_2d = true,  -- 将3D位姿投影到2D，忽略俯仰角带来的地图偏差
  use_odometry = true,  -- 是否使用外部里程计数据
  use_nav_sat = false,  -- 是否使用GPS数据
  use_landmarks = false,  -- 是否使用路标点
  
  -- 传感器数据设置
  num_laser_scans = 1,  -- 订阅的单线激光雷达话题数量
  num_multi_echo_laser_scans = 0,  -- 订阅的多回波激光雷达话题数量
  num_subdivisions_per_laser_scan = 1,  -- 每帧激光数据分割的子帧数
  num_point_clouds = 0,  -- 订阅的点云话题数量
  
  -- 超时和发布频率设置
  lookup_transform_timeout_sec = 0.2,  -- TF变换查找超时时间（秒）
  submap_publish_period_sec = 0.3,  -- 子地图发布周期
  pose_publish_period_sec = 5e-3,  -- 位姿发布周期（高频）
  trajectory_publish_period_sec = 30e-3,  -- 轨迹发布周期
  
  -- 数据采样率设置（1.0表示使用所有数据）
  rangefinder_sampling_ratio = 1.,  -- 测距仪数据采样率
  odometry_sampling_ratio = 1.,  -- 里程计数据采样率
  fixed_frame_pose_sampling_ratio = 1.,  -- 固定坐标系位姿采样率
  imu_sampling_ratio = 1.,  -- IMU数据采样率
  landmarks_sampling_ratio = 1.,  -- 路标数据采样率
}

MAP_BUILDER.use_trajectory_builder_2d = true  -- 启用2D轨迹构建器，适用于平面移动机器人

-- 2D轨迹构建器参数
TRAJECTORY_BUILDER_2D.min_range = 0.15  -- 激光雷达最小有效测量距离
TRAJECTORY_BUILDER_2D.max_range = 5.0  -- 激光雷达最大有效测量距离
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.  -- 缺失数据的射线长度
TRAJECTORY_BUILDER_2D.use_imu_data = true  -- 是否使用IMU数据#############
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 是否使用在线相关扫描匹配

-- IMU相关参数（新增）
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.  -- IMU重力时间常数#############
TRAJECTORY_BUILDER_2D.pose_extrapolator.use_imu_based = false  -- 对于2D SLAM，通常设置为false#############

-- 运动滤波参数
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.03)  -- 位姿更新最小角度阈值

-- Ceres扫描匹配器参数
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight =10.	--扫描匹配点云和地图匹配程度，值越大，点云和地图匹配置信度越高
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20  -- 最大迭代次数，影响优化精度和耗时
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 200  -- 平移项的优化权重，值越大对平移误差越敏感
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 400  -- 显著提高，加强对先验旋转的约束########

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 累积多帧数据，改善高速移动时的数据质量
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05  -- 较小的体素提供更精细的地图但计算量更大
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45  -- 每个子地图包含的雷达数据帧数，值小更新快，值大更稳定
MAP_BUILDER.num_background_threads = 2  -- 后台处理线程数，CPU核心数

-- 位姿图优化参数
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 闭环检测的最小匹配分数，值高减少错误闭环，值低更容易检测到闭环
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- 全局定位的最小匹配分数

-- 采样率参数
POSE_GRAPH.global_sampling_ratio = 0001  -- 全局约束采样率
POSE_GRAPH.constraint_builder.sampling_ratio = 0.001  -- 约束构建采样率

-- 优化频率设置
POSE_GRAPH.optimize_every_n_nodes = 90  -- 每N个节点执行一次优化,越小地图更新越频繁,num_range_data*2

return options
