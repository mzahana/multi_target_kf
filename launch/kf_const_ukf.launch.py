# launch/kf_const_ukf.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Add argument for the path of the yaml file
    kf_yaml = LaunchConfiguration('kf_yaml')
    detections_topic = LaunchConfiguration('detections_topic')
    measurement_topic = LaunchConfiguration('measurement_topic')
    namespace = LaunchConfiguration('kf_ns')
    model_type = LaunchConfiguration('model_type')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Default config for UKF model
    config = os.path.join(
        get_package_share_directory('multi_target_kf'),
        'config',
        'kf_param_ukf.yaml'
    )
    
    kf_yaml_launch_arg = DeclareLaunchArgument(
        'kf_yaml',
        default_value=config,
        description='Path to the YAML file with KF parameters'
    )

    detections_topic_launch_arg = DeclareLaunchArgument(
        'detections_topic',
        default_value='detections',
        description='Topic for incoming measurements as custom detections message'
    )
    
    measurement_topic_launch_arg = DeclareLaunchArgument(
        'measurement_topic',
        default_value='measurement/pose_array',
        description='Topic for incoming measurements as pose array'
    )

    namespace_launch_arg = DeclareLaunchArgument(
        'kf_ns',
        default_value='',
        description='Namespace for the KF node'
    )
    
    model_type_launch_arg = DeclareLaunchArgument(
        'model_type',
        default_value='2',  # 2 = ADAPTIVE_ACCEL_UKF
        description='Motion model type: 0=CONSTANT_VELOCITY, 1=CONSTANT_ACCELERATION, 2=ADAPTIVE_ACCEL_UKF'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false', 
        description='simulation time vs. real time'
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
            {'model_type': model_type,
            'use_sim_time': use_sim_time}
        ],
        remappings=[('detections', detections_topic),
                    ('measurement/pose_array', measurement_topic)
                    ]
    )

    ld = LaunchDescription()

    ld.add_action(kf_yaml_launch_arg)
    ld.add_action(detections_topic_launch_arg)
    ld.add_action(measurement_topic_launch_arg)
    ld.add_action(namespace_launch_arg)
    ld.add_action(model_type_launch_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(kf_node)

    return ld