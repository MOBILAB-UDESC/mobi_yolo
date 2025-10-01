from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_yolo_detection = get_package_share_directory('yolo_detection')

    detection_node = Node(
        package='yolo_detection',
        executable='detection_node',
        name='detection_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'visualize_rgb_image': LaunchConfiguration('rgb'),
             'visualize_depth_image': LaunchConfiguration('depth')},
            LaunchConfiguration('detection_config'),
        ],
    )

    rviz2_path = PathJoinSubstitution([pkg_yolo_detection, 'rviz', 'camera.rviz'])

    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package="rviz2",
        executable="rviz2",
        name="arm_rviz2",
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', rviz2_path],
    )

    args = [
        DeclareLaunchArgument(
            'depth',
            default_value='false',
            choices=['true', 'false'],
            description='Enable opencv visualization for depth image'
        ),
        DeclareLaunchArgument(
            'detection_config',
            default_value=PathJoinSubstitution([pkg_yolo_detection, 'config', 'detection.yaml']),
            description='Full path to the detection parameters file to use for the node'
        ),
        DeclareLaunchArgument(
            'rgb',
            default_value='true',
            choices=['true', 'false'],
            description='Enable opencv visualization for rgb image'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use simulation time'
        ),
    ]

    return LaunchDescription([
        *args,
        detection_node,
        rviz2_node
    ])
