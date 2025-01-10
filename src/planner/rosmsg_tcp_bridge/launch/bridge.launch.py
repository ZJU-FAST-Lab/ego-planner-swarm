from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 定义 LaunchConfiguration
    drone_id = LaunchConfiguration('drone_id', default = 1)
    
    # 声明参数
    drone_id_cmd = DeclareLaunchArgument(
        'drone_id',
        default_value= drone_id,
        description='ID of the drone'
    )

    # 节点定义
    rosmsg_tcp_bridge_node = Node(
        package='rosmsg_tcp_bridge',
        executable='bridge_node',
        name=['bridge_node_', drone_id],  # 动态生成节点名称
        output='screen',
        parameters=[
            {'next_drone_ip': '127.0.0.1'},
            {'broadcast_ip': '127.0.0.255'},
            {'drone_id': drone_id},
            {'odom_max_freq': 70.0}
        ],
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('my_odom', '/vins_estimator/imu_propagate')
        ]
    )

    # 定义 LaunchDescription
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(drone_id_cmd)

    # 添加节点
    ld.add_action(rosmsg_tcp_bridge_node)

    return ld
