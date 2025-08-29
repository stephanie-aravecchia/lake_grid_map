import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
            get_package_share_directory('lake_grid_map'),
            'config',
            'lake_side_metadata.yaml'
            )

    dem_grid_map = Node(
            package='lake_grid_map',
            namespace='hus',
            executable = 'dem2gridmap',
            output = 'screen',
            parameters = [config],
            remappings=
            [
                ('/tf','tf'),
                ('/tf_static', 'tf_static'),
                ]
            )

    return LaunchDescription([dem_grid_map])
