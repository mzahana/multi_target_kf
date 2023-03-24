import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   
    # Add argument for the path of the yaml file

    config = os.path.join(
        get_package_share_directory('multi_target_kf'),
        'config',
        'kf_param.yaml'
    )

    # KF node
    kf_node = Node(
        package='multi_target_kf',
        executable='tracker_node',
        name='kf_tracker_node',
        output='screen',
        parameters=[config]
    )

    ld = LaunchDescription()

    ld.add_action(kf_node)

    return ld