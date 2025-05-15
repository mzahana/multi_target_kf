# launch/modular_kf.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Add arguments for the launch file
    kf_yaml = LaunchConfiguration('kf_yaml')
    detections_topic = LaunchConfiguration('detections_topic')
    namespace = LaunchConfiguration('kf_ns')
    model_type = LaunchConfiguration('model_type')

    # Default config path
    config = os.path.join(
        get_package_share_directory('multi_target_kf'),
        'config',
        'kf_param.yaml'
    )
    
    # Launch arguments
    kf_yaml_launch_arg = DeclareLaunchArgument(
        'kf_yaml',
        default_value=config,
        description='Path to the YAML file with KF parameters'
    )

    detections_topic_launch_arg = DeclareLaunchArgument(
        'detections_topic',
        default_value='measurement/pose_array',
        description='Topic for incoming measurements'
    )

    namespace_launch_arg = DeclareLaunchArgument(
        'kf_ns',
        default_value='',
        description='Namespace for the KF node'
    )
    
    model_type_launch_arg = DeclareLaunchArgument(
        'model_type',
        default_value='0',
        description='Motion model type: 0=CONSTANT_VELOCITY, 1=CONSTANT_ACCEL, etc.'
    )

    # KF node
    kf_node = Node(
        package='multi_target_kf',
        executable='tracker_node',
        name='kf_tracker_node',
        namespace=namespace,
        output='screen',
        parameters=[
            kf_yaml,
            {'model_type': model_type}  # Allow overriding the model type from the command line
        ],
        remappings=[('measurement/pose_array', detections_topic)]
    )

    ld = LaunchDescription()

    ld.add_action(kf_yaml_launch_arg)
    ld.add_action(detections_topic_launch_arg)
    ld.add_action(namespace_launch_arg)
    ld.add_action(model_type_launch_arg)
    ld.add_action(kf_node)

    return ld