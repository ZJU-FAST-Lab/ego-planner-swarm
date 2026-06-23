import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id', default=0)
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')

    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id)
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic)

    # ==========================================
    # 🌟 核心規劃大腦 (Advanced Param) 🌟
    # Jetson Orin Nano 8G 客製版
    # ==========================================
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': '20.0',
            'map_size_y_': '5.0',
            'map_size_z_': '3.0',
            'odometry_topic': '/odometry/filtered',  # 用 EKF 融合後的 odom(座標一致性)
            'obj_num_set': '0',  
            
            'camera_pose_topic': 'camera_pose',
            'depth_topic': 'camera/depth/image_raw',
            
            # --- 1. 相機內參(對應 drone_manager.py 的 camera_info)---
            'cx': '240.0',
            'cy': '180.0',
            'fx': '460.0',
            'fy': '460.0',
            
            # --- 2. 動力學限制(放開規劃,實際限速由 drone_manager 壓制)---
            'max_vel': '0.6',
            'max_acc': '1.0',           # 從 0.5 提升,refine 才有調整空間
            'planning_horizon': '3.0',  # 從 4.0 縮短,降低目標掉進障礙物機率
            'use_distinctive_trajs': 'True',
            'flight_type': '1', 

            # --- 3. 🌟 地圖與避障(Orin Nano 8G 平衡值)---
            'grid_map/resolution': '0.18',          # 從 0.10/0.15 → 0.18:8GB RAM 友善
            'grid_map/obstacles_inflation': '0.15', # Tello 半徑 + VO 抖動容錯
            
            # --- 4. 射線有效範圍(對齊 drone_manager 深度截斷)---
            'grid_map/min_ray_length': '0.4',      
            'grid_map/max_ray_length': '4.5',      
            
            # --- 5. 軌跡優化性格 ---
            'optimization/lambda_collision': '2.0', # 撞牆代價拉高
            'optimization/lambda_smooth': '0.5',    # 平滑代價降低,給 refine 空間
            'optimization/dist0': '0.25',           # 安全距離

            # --- 6. 深度過濾 ---
            'grid_map/depth_filter_tolerance': '0.5', 
            'grid_map/depth_filter_maxdist': '4.5'
            
            # ⚠️ 已移除:'cloud_topic': '/virtual_wall'
            #    原因:remap 拼接後變成 'drone_0_/virtual_wall' 是無效 topic
        }.items()
    )
    
    # ==========================================
    # 🌟 軌跡伺服器 (將曲線轉換為實時速度指令) 🌟
    # ==========================================
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        remappings=[
            ('position_cmd', '/position_cmd'),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 0.5}  # 室內低速追蹤,前瞻時間短
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(drone_id_cmd)
    ld.add_action(odom_topic_cmd)
    
    ld.add_action(advanced_param_include)

    odom_relay_node = Node(
    package='topic_tools',
    executable='relay',
    name='ego_planner_odom_relay',
    arguments=['/odometry/filtered', '/drone_0_/odometry/filtered'],
    output='screen',
    )
    ld.add_action(odom_relay_node)
    ld.add_action(traj_server_node)

    return ld