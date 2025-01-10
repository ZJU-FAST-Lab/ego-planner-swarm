import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # LaunchConfiguration definitions
    map_size_x = LaunchConfiguration('map_size_x', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z', default=5.0)
    init_x = LaunchConfiguration('init_x', default=0.0)
    init_y = LaunchConfiguration('init_y', default=0.0)
    init_z = LaunchConfiguration('init_z', default=0.0)
    target_x = LaunchConfiguration('target_x', default=20.0)
    target_y = LaunchConfiguration('target_y', default=20.0)
    target_z = LaunchConfiguration('target_z', default=1.0)
    drone_id = LaunchConfiguration('drone_id', default=0)
    odom_topic = LaunchConfiguration('odom_topic', default='visual_slam/odom')
    obj_num = LaunchConfiguration('obj_num', default=10)

    # DeclareLaunchArgument definitions
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    init_x_cmd = DeclareLaunchArgument('init_x', default_value=init_x, description='Initial x position of the drone')
    init_y_cmd = DeclareLaunchArgument('init_y', default_value=init_y, description='Initial y position of the drone')
    init_z_cmd = DeclareLaunchArgument('init_z', default_value=init_z, description='Initial z position of the drone')
    target_x_cmd = DeclareLaunchArgument('target_x', default_value=target_x, description='Target x position')
    target_y_cmd = DeclareLaunchArgument('target_y', default_value=target_y, description='Target y position')
    target_z_cmd = DeclareLaunchArgument('target_z', default_value=target_z, description='Target z position')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='ID of the drone')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')
    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of moving objects')

    use_dynamic = LaunchConfiguration('use_dynamic', default=True)  
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')

    # Include advanced parameters
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'obj_num_set': obj_num,
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            'cloud_topic': 'pcl_render_node/cloud',
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            'max_vel': str(2.0),
            'max_acc': str(3.0),
            'planning_horizon': str(7.5),
            'use_distinctive_trajs': 'True',
            'flight_type': str(2),
            'point_num': str(1),
            'point0_x': target_x,
            'point0_y': target_y,
            'point0_z': target_z,
            
            'point1_x': str(0.0),
            'point1_y': str(15.0),
            'point1_z': str(1.0),
            
            'point2_x': str(15.0),
            'point2_y': str(0.0),
            'point2_z': str(1.0),
            
            'point3_x': str(0.0),
            'point3_y': str(-15.0),
            'point3_z': str(1.0),
            
            'point4_x': str(-15.0),
            'point4_y': str(0.0),
            'point4_z': str(1.0),
        }.items()
    )

    # Trajectory server node
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )

    # Include simulator 
    simulator_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_dynamic_cmd': use_dynamic,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': init_x,
            'init_y_': init_y,
            'init_z_': init_z,
            'odometry_topic': odom_topic
        }.items()
    )

    # Object generator node
    obj_generator_node = Node(
        package='plan_env',
        executable='obj_generator',
        name='obj_generator',
        output='screen',
        parameters=[
            {'obj_generator/obj_num': obj_num},
            {'obj_generator/x_size': 12.0},
            {'obj_generator/y_size': 12.0},
            {'obj_generator/h_size': 1.0},
            {'obj_generator/vel': 1.5},
            {'obj_generator/yaw_dot': 2.0},
            {'obj_generator/acc_r1': 1.0},
            {'obj_generator/acc_r2': 1.0},
            {'obj_generator/acc_z': 0.0},
            {'obj_generator/scale1': 0.5},
            {'obj_generator/scale2': 1.0},
            {'obj_generator/interval': 100.0},
            {'obj_generator/input_type': 1}
        ]
    )

    # Create LaunchDescription and add actions
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(init_x_cmd)
    ld.add_action(init_y_cmd)
    ld.add_action(init_z_cmd)
    ld.add_action(target_x_cmd)
    ld.add_action(target_y_cmd)
    ld.add_action(target_z_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(use_dynamic_cmd)

    # Add nodes and includes
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(simulator_include)
    # ld.add_action(obj_generator_node)    

    return ld
