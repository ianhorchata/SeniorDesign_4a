from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='point_cloud_converter',
            executable='point_cloud_converter_node',
            name='point_cloud_converter',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('points_in', '/PointCloud'),
                ('points2_out', '/PointCloud2'),
            ],
        ),
    ])
