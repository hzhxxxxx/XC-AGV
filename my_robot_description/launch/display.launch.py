from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_file = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'urdf',
        'xiaoche_hardware.urdf.xacro',
    ])

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'),
            ' ',
            LaunchConfiguration('model'),
        ])
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=model_file,
            description='Absolute path to robot URDF/Xacro file.',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Start joint_state_publisher_gui when available.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_gui')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
