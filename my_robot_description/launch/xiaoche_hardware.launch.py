from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_file = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'urdf',
        'xiaoche_hardware.urdf.xacro',
    ])
    controllers_file = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'config',
        'xiaoche_controllers.yaml',
    ])

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'),
            ' ',
            robot_description_file,
        ])
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'controllers_file',
            default_value=controllers_file,
            description='Path to ros2_control controller yaml.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[robot_description, LaunchConfiguration('controllers_file')],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
    ])
