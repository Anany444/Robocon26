import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    team_arg = DeclareLaunchArgument(
        'team',
        default_value='red',
        description='Team color for initial and second navigation goals: red or blue'
    )
    team_config = LaunchConfiguration('team')

    return LaunchDescription([
        team_arg,

        # 1. Unitree LiDAR Driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('unitree_lidar_ros2'), 'launch.py')
            )
        ),

        # 2. Joy Serial Node (Bringup)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('r2_joy_serial'), 'launch', 'joy_serial_bringup.launch.py')
            )
        ),

        # 3. Point LIO Mapping L2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('point_lio'), 'launch', 'mapping_unilidar_l2.launch.py')
            )
        ),

        
        # 4. Start Foxglove Bridge with suppressed INFO logs
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
            ),
            # This passes the log-level argument straight to the underlying node
            launch_arguments={'args': '--ros-args --log-level WARN'}.items()
        ),
         
        # 5. Delayed Launch (5 seconds after initial bringup) for Controller & Spearhead Vision
        TimerAction(
            period=5.0,
            actions=[

                Node(
                    package='robocon_controller',
                    executable='test_drive_controller',
                    name='test_drive_controller',
                    parameters=[{'team': team_config}],
                    output='screen'
                ),
                Node(
                    package='robocon_vision',
                    executable='spear_detection',
                    name='spear_detection',
                    output='screen'
                )
            ]
        )
    ])


