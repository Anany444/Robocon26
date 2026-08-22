import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    pkg_description_name = 'robocon_description'
    pkg_description_share = get_package_share_directory(pkg_description_name)

    pkg_bringup_name = 'robocon_bringup'
    pkg_bringup_share = get_package_share_directory(pkg_bringup_name)

    # 1. Path to URDF, World, and RViz Config
    urdf_file = os.path.join(pkg_description_share, 'urdf', 'r2.urdf')
    world_file = os.path.join(pkg_description_share, 'worlds', 'robocon.sdf')
    rviz_file = os.path.join(pkg_bringup_share, 'config', 'rviz_config.rviz')

    # 2. Set Gazebo Resource Path (Finds meshes)
    install_dir = os.path.join(pkg_description_share, '..')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_dir
    )

    # 4. Launch Gazebo 
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 5. Spawn the Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'r2',
            '-topic', 'robot_description',
            '-x', '1.4',
            '-y', '-5.6',
            '-z', '0.02',
            '-Y', '1.5708' 
        ],
        output='screen',
    )

    # 6. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}, {'use_sim_time': True}],
    )

    # 7. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            #'/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/ground_truth_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )

    # 8. RViz2 (NEW!)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file] # Load our config file
    )

    # 9. Controller Spawners
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    load_mecanum_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller"],
    )
    load_extrusion_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["extrusion_controller"],
    )
    
    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    relay_cmd_vel = Node(
        package="topic_tools",
        executable="relay",
        name="relay_cmd_vel_to_mecanum_drive",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/mecanum_drive_controller/reference_unstamped",
            }
        ],
        output="screen",
    )

    gui_node = Node(
        package='robocon_controller',
        executable='gui_node',
        name='gui_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 10. EKF Node
    ekf_config = os.path.join(pkg_bringup_share, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}]
    )

    # 11. ICP Odometry
    icp_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup_share, 'launch', 'icp_odom.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 12. Zone Publisher
    zone_publisher_node = Node(
        package='robocon_state',
        executable='zone_publisher',
        name='zone_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 13. Mission Executor (Behavior Tree)
    mission_executor_node = Node(
        package='robocon_behaviour',
        executable='mission_executor',
        name='mission_executor',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 14. Goal Pose Controller
    goal_pose_controller_node = Node(
        package='robocon_controller',
        executable='goal_pose_controller',
        name='goal_pose_controller',
        output='screen'
    )

    # 15. Planner Node (Replaced by 3-tier terminals)
    planner_node = Node(
        package='robocon_planner',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    # 16. Vision Node
    vision_node = Node(
        package='robocon_vision',
        executable='kfs_detection',
        name='kfs_detection',
        output='screen'
    )

    controller_node = Node(
        package='robocon_controller',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        spawn_entity,
        robot_state_publisher,
        bridge,
        rviz_node,
        load_joint_state_broadcaster,
        load_mecanum_drive_controller,
        load_extrusion_controller,
        load_gripper_controller,
        load_arm_controller,
        relay_cmd_vel,
        gui_node,
        # icp_odom_launch,
        # ekf_node,
        zone_publisher_node,
        mission_executor_node, 
        goal_pose_controller_node,
        planner_node, 
        vision_node,
        controller_node
    ])
