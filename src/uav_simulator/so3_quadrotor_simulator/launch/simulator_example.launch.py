import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
import launch_ros.actions
import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    init_x = LaunchConfiguration('init_x', default = -4.0)
    init_y = LaunchConfiguration('init_y', default = 0.0)
    init_z = LaunchConfiguration('init_z', default = 4.0)
    
    init_x_cmd = DeclareLaunchArgument('init_x', default_value=init_x)
    init_y_cmd = DeclareLaunchArgument('init_y', default_value=init_y)
    init_z_cmd = DeclareLaunchArgument('init_z', default_value=init_z)
    
    so3_quadrotor_simulator = launch_ros.actions.Node(
            package='so3_quadrotor_simulator', executable='so3_quadrotor_simulator',
            output='screen', name="quadrotor_simulator_so3", 
            parameters=[{'rate/odom': 100.0},
                        {'simulator/init_state_x': init_x},
                        {'simulator/init_state_y': init_y},
                        {'simulator/init_state_z': init_z}],
            
            remappings=[('odom', '/sim/odom'),
                        ('cmd', '/so3_cmd'),
                        ('imu', '/sim/imu')]
            )
    
    # 读取配置文件路径，通过yaml文件初始化参数
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
            name='so3_control_component',
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
            remappings=[('odom', '/sim/odom'),
                        ('position_cmd', '/position_cmd'),
                        ('motors', 'motors'),
                        ('corrections', 'corrections'),
                        ('so3_cmd', '/so3_cmd'),
                        ('imu', '/sim/imu')]
            )
    
    so3_control_container = ComposableNodeContainer(
            name='so3_control_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                so3_control_component
            ],
            output='screen'
        )
    
    visualization_node = launch_ros.actions.Node(
            package='odom_visualization', executable='odom_visualization',
            name='odom_visualization', output='screen',
            parameters=[{'color/a': 0.5},
                        {'color/r': 1.0},
                        {'color/g': 0.0},
                        {'color/b': 0.0},
                        {'covariance_scale': 100.0},
                        {'robot_scale': 1.0},
                        {'tf45': True}],
            
            remappings=[('odom', '/sim/odom')])


    rviz_config_path = os.path.join(get_package_share_directory('so3_quadrotor_simulator'), 'config', 'rviz.rviz')
    rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', rviz_config_path])

    ld = LaunchDescription()
    ld.add_action(init_x_cmd)
    ld.add_action(init_y_cmd)
    ld.add_action(init_z_cmd)
    ld.add_action(so3_quadrotor_simulator)
    ld.add_action(so3_control_container)
    ld.add_action(visualization_node)
    ld.add_action(rviz_node)

    return ld