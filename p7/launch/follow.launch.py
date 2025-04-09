import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('camera')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    detector_cmd = Node(package='p7',
                        executable='Detecion',
                        output='screen',
                        remappings=[
                          ('/input_detections', '/detections_3d')
                        ])

    follow_person_cmd = Node(package='p7',
                        executable='Busca',
                        output='screen')

    ld = LaunchDescription()
    ld.add_action(detector_cmd)
    ld.add_action(follow_person_cmd)

    return ld