import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('linebot_description')
    bringup = get_package_share_directory('linebot_bringup')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # NEW: lets Gazebo resolve package://linebot_description/meshes/...
    set_res = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(desc))
    set_res2 = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', desc)

    world_arg = DeclareLaunchArgument(
        'world', default_value=os.path.join(desc, 'worlds', 'line_track.sdf'))

    xacro_file = os.path.join(desc, 'urdf', 'robot.urdf')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items())

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    # CHANGED: -z 0.03 -> 0.06 so the wheels aren't buried in the ground
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'linebot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.06', '-Y', '0.0'],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(bringup, 'config', 'bridge.yaml'),
                     'use_sim_time': True}],
        output='screen')

    follower = Node(
        package='linebot_follower', executable='line_follower',
        parameters=[os.path.join(bringup, 'config', 'follower_params.yaml'),
                    {'use_sim_time': True}],
        output='screen')

    return LaunchDescription([
        set_res,                       # NEW: must come first
        set_res2,
        world_arg,
        gazebo, rsp, bridge,
        TimerAction(period=3.0, actions=[spawn]),
        TimerAction(period=6.0, actions=[follower]),
    ])