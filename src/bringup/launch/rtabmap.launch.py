from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    parameters = [{
        'frame_id': 'base_footprint',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'use_sim_time': use_sim_time,
        'approx_sync': True,
        'wait_for_transform': 0.2,
        
        # Frame Cropping (Trim the robot's arms out of the camera view)
        'RGBD/RoiRatios': '0.0 0.0 0.0 0.3',   # Crop the bottom 30% of the image
        
        # Settings for Occupancy Grid projection
        'Grid/RangeMin': '0.5',        # Ignore anything closer than 50cm (like the chassis/arms)
        'Grid/RangeMax': '10.0',       # RealSense max range
        'Grid/RayTracing': 'true',
        'Grid/3D': 'true',            # Output a 2D map for Nav2
        'Reg/Force3DoF': 'false',      # Allow Z, Roll, and Pitch to change for ramps and stairs
    }]

    remappings = [
        ('rgb/image', '/camera/rgbd/image'),
        ('rgb/camera_info', '/camera/rgbd/camera_info'),
        ('depth/image', '/camera/rgbd/depth_image'),
        ('odom', '/odometry/filtered')
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        
        # Main RTAB-Map SLAM node
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'] # The '-d' argument deletes the previous mapping database
        ),
        
        # RTAB-Map Visualizer (optional, great for tuning)
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings
        ),
    ])
