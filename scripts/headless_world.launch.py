#!/usr/bin/env python3
"""TurtleBot3 world, server-only — CellOps (WorkCell Part E).

The stock ``turtlebot3_world.launch.py`` unconditionally starts the Gazebo GUI
client (``gz sim -g``) with ``on_exit_shutdown: true``. Under WSLg the GUI
intermittently segfaults inside the NVIDIA D3D12 driver (libnvwgf2umx.so — the
known turtlebot3_simulations#247 failure mode), and because it is a *required*
process the crash kills the physics server, the bridge, and everything else
with it.

Orchestrated SLAM/Nav2 runs need no GUI, so this launch replicates the stock
file minus the client: server + robot spawn + ros_gz bridge (via the TB3 spawn
launch) + robot_state_publisher. A GUI can still be attached later to the
running server with ``gz sim -g`` (ideally LIBGL_ALWAYS_SOFTWARE=1 for demo
recording — llvmpipe is slower but does not crash).

Usage:  ros2 launch /path/to/headless_world.launch.py [x_pose:=..] [y_pose:=..]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    launch_dir = os.path.join(tb3_gazebo, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    world = os.path.join(tb3_gazebo, 'worlds', 'turtlebot3_world.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    set_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(tb3_gazebo, 'models'))

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)          # NOTE: no gzclient — see module docstring
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(set_resources)
    return ld
