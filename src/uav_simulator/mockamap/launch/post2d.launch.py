import os

from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mockamap_node = launch_ros.actions.Node(
            package='mockamap', executable='mockamap_node',
            output='screen',
            parameters=[{'seed': 511},
                        {'update_freq': 0.1},
                        # box edge length, unit meter
                        {'resolution': 0.1},
                        # map size unit meter
                        {'x_length': 10},
                        {'y_length': 10},
                        {'z_length': 4},
                        # 1 perlin noise 3D, 2 perlin box random map, 3 2d maze still developing
                        {'type': 2},
                        {'width_min': 0.6},
                        {'width_max': 1.5},
                        {'obstacle_number': 50}])
    
    rviz_config_path = os.path.join(get_package_share_directory('mockamap'), 'config', 'rviz.rviz')
    rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', rviz_config_path])

    ld = LaunchDescription()
    ld.add_action(mockamap_node)
    ld.add_action(rviz_node)

    return ld