from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
        )
    )

    prefix = LaunchConfiguration('prefix')

    # Get URDF via xacro

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ros2_iiwa_description'), 'urdf', 'iiwa.urdf.xacro']
            ),
            ' ',
            'prefix:=',prefix,
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('ros2_iiwa_description'), 'rviz', 'iiwa.rviz']
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=['-d', rviz_config_file],
    )

    nodes = [
        joint_state_publisher_node,
        robot_state_pub,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)