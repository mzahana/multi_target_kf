# launch/kf_rviz.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Arguments for the KF node
    model_type = LaunchConfiguration('model_type')
    
    model_type_launch_arg = DeclareLaunchArgument(
        'model_type',
        default_value='1',  # 0 = CONSTANT_VELOCITY
        description='Motion model type: 0=CONSTANT_VELOCITY, 1=CONSTANT_ACCELERATION'
    )
    
    # Include the appropriate tracker launch file
    tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('multi_target_kf'), 'launch'),
            '/kf_const_vel.launch.py'
        ]),
        launch_arguments={
            'model_type': model_type
        }.items()
    )
    
    # Configuration for RViz
    rviz_config = os.path.join(
        get_package_share_directory('multi_target_kf'),
        'config',
        'tracker_visualization.rviz'
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Simulation node to generate sample data
    simulation_node = Node(
        package='multi_target_kf',
        executable='simulate_circle.py',
        name='simulation_node',
        output='screen'
    )
    
    ld = LaunchDescription()
    
    ld.add_action(model_type_launch_arg)
    ld.add_action(tracker_launch)
    ld.add_action(rviz_node)
    ld.add_action(simulation_node)
    
    return ld