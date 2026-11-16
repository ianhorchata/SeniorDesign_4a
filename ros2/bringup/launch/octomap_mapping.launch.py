"""Webots break room + ME480Robot + OctoMap + path planner + RViz.

    ros2 launch bringup octomap_mapping.launch.py [map:=saved.bt] [world:=other.wbt] [rviz:=false]

Save the map of a running session, to start from it the next time with map:=...
    ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=$PWD/saved.bt
"""

import os

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from bringup.ros_world import KEYBOARD_ROBOT_NAME, make_ros_world
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.utils import controller_url_prefix
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher

SIM_TIME = {'use_sim_time': True}


def launch_webots(context):
    """Start Webots on a copy of the world whose robot is driven from ROS."""
    bringup_dir = get_package_share_directory('bringup')
    world = LaunchConfiguration('world').perform(context)
    if not os.path.isabs(world):
        world = os.path.join(bringup_dir, 'worlds', world)
    ros_world, robot_name = make_ros_world(world)

    webots = WebotsLauncher(world=ros_world, ros2_supervisor=True)

    robot_driver = WebotsController(
        robot_name=robot_name,
        parameters=[
            {'robot_description': os.path.join(bringup_dir, 'resource', 'me480_webots.urdf')},
            SIM_TIME,
        ],
        respawn=True,
    )

    # Keys pressed in the Webots window -> /cmd_vel. Attaches to the body-less
    # robot that make_ros_world() added, the same way the Ros2Supervisor does.
    keyboard_control = ExecuteProcess(
        cmd=['python3', '-u', os.path.join(
            bringup_dir, 'controllers', KEYBOARD_ROBOT_NAME, KEYBOARD_ROBOT_NAME + '.py')],
        additional_env={
            'WEBOTS_CONTROLLER_URL': controller_url_prefix() + KEYBOARD_ROBOT_NAME,
            'WEBOTS_HOME': get_package_prefix('webots_ros2_driver'),
        },
        output='screen',
        respawn=True,
    )

    return [
        webots,
        webots._supervisor,
        robot_driver,
        keyboard_control,
        # Closing Webots shuts everything else down.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ]


def saved_map_resolution(map_file):
    """Voxel size of a map saved by octomap_saver_node, from the header of the file."""
    with open(map_file, 'rb') as file:
        for line in file:
            words = line.split()
            if words[:1] == [b'res'] and len(words) == 2:
                return float(words[1])
            if words[:1] == [b'data']:
                break
    raise RuntimeError(f'"{map_file}" is not an OctoMap file: there is no "res" in its header.')


def launch_mapping(context):
    """Start the OctoMap server and the path planner, on a saved map if one is given."""
    bringup_dir = get_package_share_directory('bringup')
    octomap_config = os.path.join(bringup_dir, 'config', 'octomap_server.yaml')
    with open(octomap_config) as config_file:
        resolution = yaml.safe_load(config_file)['octomap_server']['ros__parameters']['resolution']
    octomap_parameters = [octomap_config, SIM_TIME]

    map_file = LaunchConfiguration('map').perform(context)
    if map_file:
        map_file = os.path.abspath(os.path.expanduser(map_file))
        if os.path.splitext(map_file)[1] not in ('.bt', '.ot'):
            raise RuntimeError(f'map:="{map_file}" must be a .bt or .ot file.')
        if not os.path.isfile(map_file):
            raise RuntimeError(f'map:="{map_file}" does not exist.')
        # octomap_server then uses the voxel size of the file, not the one in its config.
        resolution = saved_map_resolution(map_file)
        octomap_parameters.append({'octomap_path': map_file})

    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=octomap_parameters,
        remappings=[('cloud_in', '/PointCloud2')],
    )

    # Plans on the map's voxels, so it has to use the same voxel size.
    path_planner = Node(
        package='bringup',
        executable='path_planner',
        output='screen',
        parameters=[
            os.path.join(bringup_dir, 'config', 'path_planner.yaml'),
            {'resolution': float(resolution)},
            SIM_TIME,
        ],
    )

    return [octomap_server, path_planner]


def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')

    with open(os.path.join(bringup_dir, 'urdf', 'me480_robot.urdf')) as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}, SIM_TIME],
    )

    localization = Node(
        package='bringup',
        executable='localization',
        output='screen',
        parameters=[SIM_TIME],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'octomap.rviz')],
        parameters=[SIM_TIME],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='break_room.wbt',
            description='World in bringup/worlds, or an absolute path. Must contain one ME480Robot.'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Saved OctoMap (.bt or .ot) to start from instead of an empty map. '
                        'The lidar keeps updating it.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        # First, so that a wrong map:=... stops the launch before Webots is started.
        OpaqueFunction(function=launch_mapping),
        OpaqueFunction(function=launch_webots),
        robot_state_publisher,
        localization,
        rviz,
    ])
