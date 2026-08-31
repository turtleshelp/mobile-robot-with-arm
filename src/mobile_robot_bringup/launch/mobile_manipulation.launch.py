import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # =====================================
    # Package directories
    # =====================================

    pkg_description = get_package_share_directory(
        'mobile_robot_description'
    )

    pkg_navigation = get_package_share_directory(
        'mobile_robot_navigation'
    )

    pkg_nav2_bringup = get_package_share_directory(
        'nav2_bringup'
    )

    pkg_moveit = get_package_share_directory(
        'mobile_robot_moveit_config'
    )

    # =====================================
    # Explicit navigation files
    # =====================================

    map_file = os.path.join(
        pkg_navigation,
        'maps',
        'mobile_robot_map.yaml'
    )

    nav2_params_file = os.path.join(
        pkg_navigation,
        'config',
        'nav2_params.yaml'
    )

    # =====================================
    # Gazebo + robot
    # =====================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_description,
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # =====================================
    # Map server + AMCL
    # =====================================

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
            'params_file': nav2_params_file,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items()
    )

    # =====================================
    # Nav2
    # =====================================

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_nav2_bringup,
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items()
    )

    # =====================================
    # MoveIt 2
    # =====================================

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_moveit,
                'launch',
                'move_group.launch.py'
            )
        ),
        launch_arguments={
            'use_sim': 'true',
        }.items()
    )

    # =====================================
    # Startup sequence
    # =====================================

    initial_pose = Node(
        package='mobile_robot_bringup',
        executable='initial_pose_publisher.py',
        name='initial_pose_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([

        LogInfo(
            msg='[Bringup] Starting Gazebo and robot controllers...'
        ),

        gazebo,

        TimerAction(
            period=9.0,
            actions=[
                LogInfo(
                    msg='[Bringup] Starting map server and AMCL...'
                ),
                localization,
            ]
        ),

        TimerAction(
            period=10.0,
            actions=[
                LogInfo(
                    msg='[Bringup] Initializing AMCL pose...'
                ),
                initial_pose,
            ]
        ),

        TimerAction(
            period=16.0,
            actions=[
                LogInfo(
                    msg='[Bringup] Starting Nav2...'
                ),
                navigation,
            ]
        ),

        TimerAction(
            period=18.0,
            actions=[
                LogInfo(
                    msg='[Bringup] Starting MoveIt 2...'
                ),
                move_group,
            ]
        ),
    ])
