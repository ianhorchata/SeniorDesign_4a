"""Webots break room + ME480Robot + PointCloud->PointCloud2 + OctoMap + RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from bringup.ros_world import make_ros_world
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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

    return [
        webots,
        webots._supervisor,
        robot_driver,
        # Closing Webots shuts everything else down.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    converter_dir = get_package_share_directory('point_cloud_converter')

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

    point_cloud_converter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(converter_dir, 'launch', 'point_cloud_converter.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[os.path.join(bringup_dir, 'config', 'octomap_server.yaml'), SIM_TIME],
        remappings=[('cloud_in', '/PointCloud2')],
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
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        OpaqueFunction(function=launch_webots),
        robot_state_publisher,
        localization,
        point_cloud_converter,
        octomap_server,
        rviz,
    ])
