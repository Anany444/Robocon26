from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    lf_pwm_factor = LaunchConfiguration('lf_pwm_factor')
    rf_pwm_factor = LaunchConfiguration('rf_pwm_factor')
    rr_pwm_factor = LaunchConfiguration('rr_pwm_factor')
    lr_pwm_factor = LaunchConfiguration('lr_pwm_factor')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    joy_run_node = Node(
        package='joy_serial_esp',
        executable='joy_runner',
        name='joy_runner',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    serial_print_node = Node(
        package='joy_serial_esp',
        executable='serial_printer',
        name='serial_printer',
        parameters=[{
            'use_sim_time': use_sim_time,
            'lf_pwm_factor': lf_pwm_factor,
            'rf_pwm_factor': rf_pwm_factor,
            'rr_pwm_factor': rr_pwm_factor,
            'lr_pwm_factor': lr_pwm_factor
        }]
    )  

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),
        DeclareLaunchArgument(
            'lf_pwm_factor',
            default_value='1.0',
            description='PWM factor for left front motor (0.0 to 1.0)'),
        DeclareLaunchArgument(
            'rf_pwm_factor',
            default_value='1.0',
            description='PWM factor for right front motor (0.0 to 1.0)'),
        DeclareLaunchArgument(
            'rr_pwm_factor',
            default_value='1.0',
            description='PWM factor for right rear motor (0.0 to 1.0)'),
        DeclareLaunchArgument(
            'lr_pwm_factor',
            default_value='1.0',
            description='PWM factor for left rear motor (0.0 to 1.0)'),
        joy_node,
        joy_run_node,
        serial_print_node
    ])
