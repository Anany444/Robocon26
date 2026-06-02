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
        # Format: Left Right Top Bottom
        'Grid/DepthRoiRatios': '0.20 0.20 0.0 0.0',   # Crop 20% from left/right, 0% from bottom
        
        # Settings for Perfect 3D Mapping (No 2D Projections)
        'Grid/RangeMin': '0.5',        # Ignore anything closer than 50cm (like the chassis/arms)
        'Grid/RangeMax': '10.0',       # RealSense max range
        'Grid/RayTracing': 'true',     # Clear dynamic obstacles / empty space
        'Grid/3D': 'true',             # Output a full 3D OctoMap (Not a flat 2D map)
        'Grid/MaxObstacleHeight': '10.0', # Map high structures, don't cap at robot height
        'Reg/Force3DoF': 'false',      # Allow Z, Roll, and Pitch to change for ramps and stairs
        
        # Quality enhancements
        'Grid/NoiseFilteringRadius': '0.05',
        'Grid/NoiseFilteringMinNeighbors': '2',
        
        # Stop False Loop Closures from Destroying Perfect Odometry
        'Kp/MaxFeatures': '-1',          # Disable visual loop closure detection entirely
        'RGBD/ProximityBySpace': 'false', # Disable proximity detection
        'RGBD/LinearUpdate': '0.0',      # Always update map (ignore distance threshold)
        'RGBD/AngularUpdate': '0.0',     # Always update map (ignore angle threshold)
    }]

    remappings = [
        ('rgb/image', '/camera/rgbd/image'),
        ('rgb/camera_info', '/camera/rgbd/camera_info'),
        ('depth/image', '/camera/rgbd/depth_image'),
        ('odom', '/ground_truth_odom')
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', robocon_description='Use simulation (Gazebo) clock if true'),
        
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
