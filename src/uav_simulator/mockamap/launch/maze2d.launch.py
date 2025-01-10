import os
from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mockamap_node = launch_ros.actions.Node(
        package='mockamap', executable='mockamap_node', 
        name='mockamap_node', output='screen',
        parameters=[{'seed': 510},
                    {'update_freq': 1.0},
                    # box edge length, unit meter
                    {'resolution': 0.1},
                    # map size unit meter
                    {'x_length': 20},
                    {'y_length': 20},
                    {'z_length': 2},
                    {'type': 3},
                    {'road_width': 0.5},
                    {'add_wall_x': 0},
                    {'add_wall_y': 0},
                    # maze type: 1 recursive division maze
                    {'maze_type': 1}]
    )

    rviz_config_path = os.path.join(get_package_share_directory('mockamap'), 'config', 'rviz.rviz')

    rviz_node = launch_ros.actions.Node(
        package='rviz2', executable='rviz2', name='rviz', output='screen',
        arguments=['--display-config', rviz_config_path]
    )

    ld = LaunchDescription()
    ld.add_action(mockamap_node)
    ld.add_action(rviz_node)

    return ld
