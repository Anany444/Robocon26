from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', robocon_description='Use simulation (Gazebo) clock if true'),
        
        # ICP Odometry Node (3D LiDAR Scan Matching)
        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[{
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'wait_for_transform': 0.2,
                'use_sim_time': use_sim_time,
                # ICP tuning parameters for better performance
                'Icp/PointToPlane': 'true',
                'Icp/VoxelSize': '0.1',
                'Odom/ScanKeyFrameThr': '0.6',
                'Odom/ResetCountdown': '1',
                'publish_tf': True,
                'publish_null_when_lost': False,
            }],
            remappings=[
                ('scan_cloud', '/scan/points'),
                ('odom', '/icp_odom')
            ]
        )
    ])
