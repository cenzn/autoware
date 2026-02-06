# This is the launch file that starts up the QCar2 nodes for 2D lidar scan matching

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments with default values
    declare_args = [
        DeclareLaunchArgument('init_x', default_value='0.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_th', default_value='0.0'),
        DeclareLaunchArgument('steer_gain', default_value='1.0'),
        
    ]

    vehicle = Node(
        package='qcar2_nodes',
        executable='vehicle_interface',
        name='vehicle_interface',
        parameters=[{
        'steer_gain':LaunchConfiguration('steer_gain'),
    }]
    )

    perception = Node(
            package='qcar2_nodes',
            executable='fake_perception_interface',
            name='fake_perception_interface'
        )
    
    localization = Node(
            package='qcar2_nodes',
            executable='scan_matcher_interface',
            name='scan_matcher_interface',
            parameters=[{
                    'init_x':LaunchConfiguration('init_x'),
                    'init_y':LaunchConfiguration('init_y'),
                    'init_th':LaunchConfiguration('init_th'),
                }]
        )
     
    return LaunchDescription(
        declare_args+[
        vehicle,
        perception,
        localization,
    ])
