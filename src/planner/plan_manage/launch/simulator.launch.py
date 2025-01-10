import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
import launch_ros.actions
import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
# from launch.substitutions import EqualsSubstitution, NotEqualsSubstitution
from launch.substitutions import PythonExpression

def generate_launch_description():
    # LaunchConfigurations
    init_x = LaunchConfiguration('init_x_', default=0.0)
    init_y = LaunchConfiguration('init_y_', default=0.0)
    init_z = LaunchConfiguration('init_z_', default=0.0)
    obj_num = LaunchConfiguration('obj_num', default=1)
    map_size_x_ = LaunchConfiguration('map_size_x_', default=10.0)
    map_size_y_ = LaunchConfiguration('map_size_y_', default=10.0)
    map_size_z_ = LaunchConfiguration('map_size_z_', default=5.0)
    c_num = LaunchConfiguration('c_num', default=5)
    p_num = LaunchConfiguration('p_num', default=20)
    min_dist = LaunchConfiguration('min_dist', default=1.0)
    odometry_topic = LaunchConfiguration('odometry_topic', default='visual_slam/odom')
    drone_id = LaunchConfiguration('drone_id', default=0)

    # DeclareLaunchArguments
    init_x_arg = DeclareLaunchArgument('init_x_', default_value=init_x, description='Initial X position')
    init_y_arg = DeclareLaunchArgument('init_y_', default_value=init_y, description='Initial Y position')
    init_z_arg = DeclareLaunchArgument('init_z_', default_value=init_z, description='Initial Z position')
    obj_num_arg = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects')
    map_size_x_arg = DeclareLaunchArgument('map_size_x_', default_value=map_size_x_, description='Map size X')
    map_size_y_arg = DeclareLaunchArgument('map_size_y_', default_value=map_size_y_, description='Map size Y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z_', default_value=map_size_z_, description='Map size Z')
    c_num_arg = DeclareLaunchArgument('c_num', default_value=c_num, description='Circle number')
    p_num_arg = DeclareLaunchArgument('p_num', default_value=p_num, description='Polygon number')
    min_dist_arg = DeclareLaunchArgument('min_dist', default_value=min_dist, description='Minimum distance')
    odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value=odometry_topic, description='Odometry topic')
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    
    # 地图属性以及是否使用动力学仿真
    use_mockamap = LaunchConfiguration('use_mockamap', default=False) # map_generator or mockamap 
    
    use_mockamap_arg = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Choose map type, map_generator or mockamap')
    
    use_dynamic = LaunchConfiguration('use_dynamic', default=True)  
    use_dynamic_arg = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')

    # Node Definitions
    random_forest_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        remappings=[
            ('odometry', odometry_topic)
        ],
        parameters=[
            {'map/x_size': map_size_x_},
            {'map/y_size': map_size_y_},
            {'map/z_size': map_size_z_},
            {'map/resolution': 0.1},
            {'ObstacleShape/seed': 1},
            {'map/obs_num': p_num},
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 0.7},
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': c_num},
            {'ObstacleShape/radius_l': 0.7},
            {'ObstacleShape/radius_h': 0.5},
            {'ObstacleShape/z_l': 0.7},
            {'ObstacleShape/z_h': 0.8},
            {'ObstacleShape/theta': 0.5},
            {'sensing/radius': 5.0},
            {'sensing/rate': 10.0},
            {'min_distance': min_dist}
        ],
        condition = UnlessCondition(use_mockamap)
    )

    mockamap_node = Node(
        package='mockamap',
        executable='mockamap_node',
        name='mockamap_node',
        output='screen',
        remappings=[
            ('/mock_map', '/map_generator/global_cloud')
        ],
        parameters=[
            {'seed': 127},
            {'update_freq': 0.5},
            {'resolution': 0.1},
            {'x_length': PythonExpression(['int(', map_size_x_, ')'])},
            {'y_length': PythonExpression(['int(', map_size_y_, ')'])},
            {'z_length': PythonExpression(['int(', map_size_z_, ')'])},
            {'type': 1},
            {'complexity': 0.05},
            {'fill': 0.12},
            {'fractal': 1},
            {'attenuation': 0.1}
        ],
        condition = IfCondition(use_mockamap)
    )
    
    # dynamic
    so3_quadrotor_simulator = launch_ros.actions.Node(
        package='so3_quadrotor_simulator', executable='so3_quadrotor_simulator',
        output='screen', name=['drone_', drone_id, '_quadrotor_simulator_so3'], 
        parameters=[{'rate/odom': 100.0},
                    {'simulator/init_state_x': init_x},
                    {'simulator/init_state_y': init_y},
                    {'simulator/init_state_z': init_z}],
        
        remappings=[('odom', ['drone_', drone_id, '_visual_slam/odom']),
                    ('cmd', ['drone_', drone_id, '_so3_cmd']),
                    ('force_disturbance', ['drone_', drone_id,'_force_disturbance']),
                    ('moment_disturbance', ['drone_', drone_id,'_moment_disturbance'])],
        condition = IfCondition(use_dynamic)
        ) 
    
    gains_file = os.path.join(
        get_package_share_directory('so3_control'),
        'config',
        'gains_hummingbird.yaml'
    )
    corrections_file = os.path.join(
        get_package_share_directory('so3_control'),
        'config',
        'corrections_hummingbird.yaml'
    )
    
    so3_control_component = launch_ros.descriptions.ComposableNode(
            package='so3_control', 
            plugin='SO3ControlComponent',
            name=['drone_', drone_id, '_so3_control_component'],
            parameters=[
                {'so3_control/init_state_x': init_x},
                {'so3_control/init_state_y': init_y},
                {'so3_control/init_state_z': init_z},
                {'mass': 0.98},
                {'use_angle_corrections': False},
                {'use_external_yaw': False},
                {'gains/rot/z': 1.0},
                {'gains/ang/z': 0.1},
                gains_file,
                corrections_file
            ],
            remappings=[('odom', ['drone_', drone_id, '_visual_slam/odom']),
                        ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
                        ('motors', ['drone_', drone_id, '_motors']),
                        ('corrections', ['drone_', drone_id, '_corrections']),
                        ('so3_cmd', ['drone_', drone_id, '_so3_cmd'])],
            condition = IfCondition(use_dynamic)
            )
    
    so3_control_container = ComposableNodeContainer(
            name=['drone_', drone_id, '_so3_control_container'],
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                so3_control_component
            ],
            output='screen',
            condition = IfCondition(use_dynamic)
        )

    poscmd_2_odom_node = Node(
        package='poscmd_2_odom',
        executable='poscmd_2_odom',
        name=['drone_', drone_id, '_poscmd_2_odom'],
        output='screen',
        parameters=[
            {'init_x': init_x},
            {'init_y': init_y},
            {'init_z': init_z}
        ],
        remappings=[
            ('command', ['drone_', drone_id, '_planning/pos_cmd']),
            ('odometry', ['drone_', drone_id, '_', odometry_topic])
        ],
        condition = UnlessCondition(use_dynamic)
    )

    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name=['drone_', drone_id, '_odom_visualization'],
        output='screen',
        remappings=[
            ('odom', ['drone_', drone_id, '_visual_slam/odom']),
            ('robot', ['drone_', drone_id, '_vis/robot']),
            ('path', ['drone_', drone_id, '_vis/path']),
            ('time_gap', ['drone_', drone_id, '_vis/time_gap']),
            # ('pose', ['drone_', drone_id, '_vis/pose']),
            # ('velocity', ['drone_', drone_id, '_vis/velocity']),
            # ('covariance', ['drone_', drone_id, '_vis/covariance']),
            # ('covariance_velocity', ['drone_', drone_id, '_vis/covariance_velocity']),
            # ('trajectory', ['drone_', drone_id, '_vis/trajectory']),
            # ('sensor', ['drone_', drone_id, '_vis/sensor']),
            # ('height', ['drone_', drone_id, '_vis/height']),
        ],
        parameters=[
            {'color/a': 1.0},
            {'color/r': 0.0},
            {'color/g': 0.0},
            {'color/b': 0.0},
            {'covariance_scale': 100.0},
            {'robot_scale': 1.0},
            {'tf45': False},
            {'drone_id': drone_id}
        ]
    )
    
    camera_file = os.path.join( 
        get_package_share_directory('local_sensing'), 
        'config', 
        'camera.yaml' 
    )

    pcl_render_node = Node(
        package='local_sensing',
        executable='pcl_render_node',
        name=['drone_', drone_id, '_pcl_render_node'],
        output='screen',
        parameters=[
            {'sensing_horizon': 5.0},
            {'sensing_rate': 30.0},
            {'estimation_rate': 30.0},
            {'map/x_size': map_size_x_},
            {'map/y_size': map_size_y_},
            {'map/z_size': map_size_z_},
            camera_file
        ],
        remappings=[
            ('global_map', '/map_generator/global_cloud'),
            ('odometry', ['drone_', drone_id, '_', odometry_topic]),
            ('pcl_render_node/cloud', ['drone_', drone_id, '_pcl_render_node/cloud']),
            ('depth', ['drone_', drone_id, '_pcl_render_node/depth'])
        ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()

    # Add LaunchArguments
    ld.add_action(init_x_arg)
    ld.add_action(init_y_arg)
    ld.add_action(init_z_arg)
    ld.add_action(obj_num_arg)
    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)
    ld.add_action(c_num_arg)
    ld.add_action(p_num_arg)
    ld.add_action(min_dist_arg)
    ld.add_action(odometry_topic_arg)
    ld.add_action(drone_id_arg)
    
    ld.add_action(use_mockamap_arg)
    ld.add_action(use_dynamic_arg)

    # Add Nodes
    # ld.add_action(random_forest_node)
    # ld.add_action(mockamap_node)
    ld.add_action(so3_quadrotor_simulator)
    ld.add_action(so3_control_container)
    ld.add_action(poscmd_2_odom_node)
    
    ld.add_action(odom_visualization_node)
    ld.add_action(pcl_render_node)

    return ld