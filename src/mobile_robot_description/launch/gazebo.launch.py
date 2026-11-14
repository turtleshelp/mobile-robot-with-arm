from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, DeclareLaunchArgument, AppendEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import math
import os
import tempfile

import yaml


def demo_cameras_sdf(config, record_dir):
    """Static model with one camera per entry in demo_cameras.yaml.
    Gazebo saves every frame (in simulation time) to record_dir/<name>."""

    links = []
    for name, cam in config['cameras'].items():
        px, py, pz = cam['position']
        tx, ty, tz = cam['look_at']
        dx, dy, dz = tx - px, ty - py, tz - pz
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(-dz, math.hypot(dx, dy))  # + looks down
        out = os.path.join(record_dir, name)
        os.makedirs(out, exist_ok=True)
        links.append(f'''
    <link name="{name}">
      <pose>{px} {py} {pz} 0 {pitch:.4f} {yaw:.4f}</pose>
      <sensor name="{name}_cam" type="camera">
        <always_on>true</always_on>
        <update_rate>{config['update_rate']}</update_rate>
        <camera>
          <horizontal_fov>{cam['fov']}</horizontal_fov>
          <image>
            <width>{config['width']}</width>
            <height>{config['height']}</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.05</near><far>50</far></clip>
          <save enabled="true"><path>{out}</path></save>
        </camera>
      </sensor>
    </link>''')

    return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="demo_cameras">
    <static>true</static>{''.join(links)}
  </model>
</sdf>
'''


def spawn_demo_cameras(context, pkg_mobile_robot):

    if context.perform_substitution(LaunchConfiguration('record')).lower() != 'true':
        return []

    record_dir = os.path.expanduser(
        context.perform_substitution(LaunchConfiguration('record_dir'))
    )

    with open(os.path.join(pkg_mobile_robot, 'config', 'demo_cameras.yaml')) as f:
        config = yaml.safe_load(f)

    sdf = tempfile.NamedTemporaryFile(
        'w', suffix='_demo_cameras.sdf', delete=False
    )
    sdf.write(demo_cameras_sdf(config, record_dir))
    sdf.close()

    return [
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', sdf.name, '-entity', 'demo_cameras'],
            output='screen'
        )
    ]


def generate_launch_description():

    pkg_mobile_robot = get_package_share_directory('mobile_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(
        pkg_mobile_robot,
        'urdf',
        'mobile_base.urdf.xacro'
    )

    world_file = os.path.join(
        pkg_mobile_robot,
        'worlds',
        'mobile_robot.world'
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
            'world': world_file,
            'extra_gazebo_args': '-u',
            # gui:=false runs without the Gazebo window (much less CPU)
            'gui': LaunchConfiguration('gui'),
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

    # Let Gazebo resolve package:// meshes of the
    # OpenMANIPULATOR-X (otherwise the arm is
    # invisible unless ~/.bashrc sets this).
    model_paths = [
        os.path.dirname(get_package_share_directory(pkg))
        for pkg in (
            'mobile_robot_description',
            'turtlebot3_manipulation_description',
        )
    ]

    gazebo_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.pathsep.join(model_paths)
    )

    return LaunchDescription([

        gazebo_model_path,

        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Show the Gazebo window (false = headless)'
        ),

        DeclareLaunchArgument(
            'record',
            default_value='false',
            description='Spawn the demo cameras (config/demo_cameras.yaml) '
                        'and save their frames for a video'
        ),

        DeclareLaunchArgument(
            'record_dir',
            default_value='/tmp/mobile_robot_recording',
            description='Where the demo cameras save their frames'
        ),

        gazebo,
        robot_state_publisher,
        spawn_robot,

        OpaqueFunction(
            function=spawn_demo_cameras,
            args=[pkg_mobile_robot]
        ),

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
