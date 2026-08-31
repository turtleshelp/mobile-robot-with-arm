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
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    default_map = os.path.join(
        pkg_navigation,
        'maps',
        'mobile_robot_map.yaml'
    )

    default_params = os.path.join(
        pkg_navigation,
        'config',
        'nav2_params.yaml'
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_nav2_bringup,
                'launch',
                'localization_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items()
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map YAML file'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params
        ),

        localization,
    ])
