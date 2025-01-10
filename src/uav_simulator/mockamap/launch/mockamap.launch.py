import os

from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mockamap_node = launch_ros.actions.Node(
            package='mockamap', executable='mockamap_node',
            output='screen',
            parameters=[{'seed': 511},
                        {'update_freq': 1.0},
                        # box edge length, unit meter
                        {'resolution': 0.1},
                        # map size unit meter
                        {'x_length': 10},
                        {'y_length': 10},
                        {'z_length': 3},
                        # 1 perlin noise 3D, 2 perlin box random map, 3 2d maze still developing
                        {'type': 1},
                        # 1 perlin noise parameters
                        # complexity:  base noise frequency,large value will be complex. typical 0.0 ~ 0.5
                        # fill:        infill persentage. typical: 0.4 ~ 0.0
                        # fractal:     large value will have more detail
                        # attenuation: for fractal attenuation. typical: 0.0 ~ 0.5
                        {'complexity': 0.03},
                        {'fill': 0.3},
                        {'fractal': 1},
                        {'attenuation': 0.1},
                        {'width_min': 0.6},
                        {'width_max': 1.5},
                        {'obstacle_number': 50},
                        {'road_width': 0.5},
                        {'add_wall_x': 0},
                        {'add_wall_y': 1},
                        # maze type: 1 recursive division maze
                        {'maze_type': 1},
                        {'numNodes': 40},
                        {'connectivity': 0.8},
                        {'nodeRad': 1},
                        {'roadRad': 10}])
    
    rviz_config_path = os.path.join(get_package_share_directory('mockamap'), 'config', 'rviz.rviz')
    rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', rviz_config_path])

    ld = LaunchDescription()
    ld.add_action(mockamap_node)
    ld.add_action(rviz_node)

    return ld