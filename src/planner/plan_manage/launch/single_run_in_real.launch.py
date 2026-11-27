import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('ego_planner'), 'launch')
    defaults = {
        'drone_id': '0',
        # 沿用机载全局里程计，仅点云来自 Super-LIO；话题可在启动时覆盖。
        'odom_topic': '/odom_global_001',
        'cloud_topic': '/lio/cloud_world',
        'pos_cmd_topic': '/planning/pos_cmd',
        'map_size_x': '50.0',
        'map_size_y': '25.0',
        'map_size_z': '2.0',
        'obj_num': '10',
        'use_rviz': 'false',
    }
    arguments = [DeclareLaunchArgument(name, default_value=value)
                 for name, value in defaults.items()]
    drone_id = LaunchConfiguration('drone_id')
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'advanced_param_real.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'odometry_topic': LaunchConfiguration('odom_topic'),
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'map_size_x_': LaunchConfiguration('map_size_x'),
            'map_size_y_': LaunchConfiguration('map_size_y'),
            'map_size_z_': LaunchConfiguration('map_size_z'),
            'obj_num_set': LaunchConfiguration('obj_num'),
            # 手动目标模式：等待 /move_base_simple/goal，不执行预设航点。
            'flight_type': '1',
            # 速度、加速度和规划视距沿用旧机配置，单位依次为 m/s、m/s²、m。
            'max_vel': '0.3',
            'max_acc': '1.0',
            'planning_horizon': '6.0',
        }.items(),
    )
    trajectory_server = Node(
        package='ego_planner', executable='traj_server',
        name=['drone_', drone_id, '_traj_server'], output='screen',
        remappings=[
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('position_cmd', LaunchConfiguration('pos_cmd_topic')),
        ],
        parameters=[{'traj_server/time_forward': 1.0}],
    )
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(launch_dir, 'super_lio.rviz')],
        remappings=[
            ('/odom_global_001', LaunchConfiguration('odom_topic')),
            ('/lio/cloud_world', LaunchConfiguration('cloud_topic')),
            ('/local_obstacles', ['drone_', drone_id, '_grid/grid_map/occupancy_inflate']),
            ('/planned_path', ['drone_', drone_id, '_plan_vis/optimal_list']),
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )
    # 定位与点云由机载节点提供；这里不启动模拟器或地图生成器。
    return LaunchDescription([*arguments, planner, trajectory_server, rviz])
