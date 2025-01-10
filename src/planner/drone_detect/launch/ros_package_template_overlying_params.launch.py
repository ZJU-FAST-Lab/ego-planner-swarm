import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明覆盖参数文件路径
    default_param_file = LaunchConfiguration('default_param_file', 
                                            default=os.path.join(os.path.dirname(__file__), '..', 'config', 'default.yaml'))
    overlying_param_file = LaunchConfiguration('overlying_param_file')
    
    # 声明默认参数文件路径
    default_param_file_cmd = DeclareLaunchArgument(
        'default_param_file',
        default_value=default_param_file,
        description = 'Path to the default parameter file.'
    )
    
    overlying_param_file_cmd = DeclareLaunchArgument(
        'overlying_param_file',
        default_value = overlying_param_file,
        description = 'Path to the parameter file that can override the defaults.'
    )

    # 节点定义
    ros_package_template_node = Node(
        package='ros_package_template',
        executable='ros_package_template',
        name='ros_package_template',
        output='screen',
        parameters=[
            default_param_file,
            overlying_param_file_cmd
        ]
    )

    # 启动描述
    ld = LaunchDescription()

    # 添加声明参数
    ld.add_action(default_param_file_cmd)
    ld.add_action(overlying_param_file_cmd)

    # 添加节点
    ld.add_action(ros_package_template_node)

    return ld
