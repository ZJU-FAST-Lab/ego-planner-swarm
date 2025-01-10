import os

from launch import LaunchDescription
import launch_ros.actions
import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_path = os.path.join(get_package_share_directory('ego_planner'), 'launch', 'default.rviz')
    rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', rviz_config_path])

    # 定义 LaunchDescription
    ld = LaunchDescription()

    # 添加节点
    ld.add_action(rviz_node)

    return ld