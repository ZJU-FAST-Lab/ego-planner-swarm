import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    my_id = LaunchConfiguration('my_id', default = 1)
    odom_topic = LaunchConfiguration('odom_topic', default = "/vins_estimator/imu_propagate")
    
    # 声明参数
    my_id_cmd = DeclareLaunchArgument(
        'my_id',
        default_value = my_id,
        description = 'ID of the drone'
    )

    odom_topic_cmd = DeclareLaunchArgument(
        'odom_topic',
        default_value = odom_topic,
        description = 'Topic for odometry data'
    )
    
    # 读取config文件
    camera_file = os.path.join(
        get_package_share_directory('drone_detect'),
        'config',
        'camera.yaml'
    )
    default_file = os.path.join(
        get_package_share_directory('drone_detect'),
        'config',
        'default.yaml'
    )

    # 定义节点
    drone_detect_node = Node(
        package='drone_detect',
        executable='drone_detect',
        name='test_drone_detect',
        output='screen',
        parameters=[
            {'my_id': my_id},
            {'debug_flag': False},
            camera_file,
            default_file
        ],
        remappings=[
            ('odometry', odom_topic),
            ('depth', '/camera/depth/image_rect_raw')
            # ('camera_pose', f'/drone_{LaunchConfiguration("my_id")}_pcl_render_node/camera_pose'),
            # ('drone0', f'/drone_0_{LaunchConfiguration("odom_topic")}'),
            # ('drone1', f'/drone_1_{LaunchConfiguration("odom_topic")}'),
            # ('drone2', f'/drone_2_{LaunchConfiguration("odom_topic")}')
        ]
    )

    # 定义 LaunchDescription
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(my_id_cmd)
    ld.add_action(odom_topic_cmd)

    # 添加节点
    ld.add_action(drone_detect_node)

    return ld