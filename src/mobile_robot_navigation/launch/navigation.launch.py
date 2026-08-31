import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_navigation = get_package_share_directory(
        'mobile_robot_navigation'
    )

    pkg_nav2_bringup = get_package_share_directory(
        'nav2_bringup'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    default_params_file = os.path.join(
        pkg_navigation,
        'config',
        'nav2_params.yaml'
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_nav2_bringup,
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items()
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start Nav2 lifecycle nodes'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to Nav2 parameter file'
        ),

        nav2_navigation,
    ])
