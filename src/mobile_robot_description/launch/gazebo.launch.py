from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_mobile_robot = get_package_share_directory('mobile_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(
        pkg_mobile_robot,
        'urdf',
        'mobile_base.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Start Gazebo PAUSED so the arm cannot collapse
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gazebo_ros,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'extra_gazebo_args': '-u'
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mobile_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )

    # Advance Gazebo only a few iterations.
    # This allows ros2_control to finish controller activation
    # without giving the arm enough time to collapse.
    step_simulation = ExecuteProcess(
        cmd=['gz', 'world', '-m', '10'],
        output='screen'
    )

    # Once controllers are active, start normal physics.
    unpause_simulation = ExecuteProcess(
        cmd=['gz', 'world', '-p', '0'],
        output='screen'
    )

    return LaunchDescription([

        gazebo,
        robot_state_publisher,
        spawn_robot,

        # Give Gazebo and gazebo_ros2_control time to start.
        TimerAction(
            period=3.0,
            actions=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                gripper_controller_spawner
            ]
        ),

        # Equivalent to clicking Gazebo "single step"
        # several times manually.
        TimerAction(
            period=6.0,
            actions=[step_simulation]
        ),

        # Controllers should now be active, so physics can run.
        TimerAction(
            period=7.0,
            actions=[unpause_simulation]
        ),
    ])
