import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    package_share_directory = get_package_share_directory('wall_following_control')
    parameters_file_path = os.path.join(package_share_directory, 'config', 'pid_config.yaml')

    wall_following = Node(
        package='wall_following_control',
        executable='steering_exe',
        name='steering_node',
        parameters=[parameters_file_path]
        # parameters=['/my_f1_ws/src/wall_following_control/config/pid_config.yaml']
    )

    ld.add_action(wall_following)
    
    return ld