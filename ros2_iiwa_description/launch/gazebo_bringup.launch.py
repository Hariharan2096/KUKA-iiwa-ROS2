from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os, xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    description_package = "ros2_iiwa_description"
    description_file = "iiwa.urdf.xacro"
    controllers_file = "iiwa_controllers.yaml"

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
        )
    )
    prefix = LaunchConfiguration('prefix')
    
    
    simulation_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", controllers_file]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            ' ',
            'prefix:=',prefix,
        ]
    )

    # xacro_file = os.path.join(
    #     get_package_share_directory('ros2_iiwa_description'), 'urdf', 'iiwa.urdf.xacro'
    # )

    # robot_description_content = xacro.parse(open(xacro_file))
    # xacro.process_doc(robot_description_content)

    # robot_description = {"robot_description": robot_description_content.toxml()}

    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher Node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"]
    )

    controller_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[simulation_controllers],
        output='both',
    )

    # Rviz

    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare('ros2_iiwa_description'), 'rviz', 'iiwa.rviz']
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=['-d', rviz_config_file],
    # )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # Gazebo Simulation

    sim_world = PathJoinSubstitution(
        [FindPackageShare(description_package), 'worlds', 'empty.world']
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution([FindPackageShare("gazebo_ros"), 'launch', 'gazebo.launch.py'])
            ]
        ),
        launch_arguments={'verbose': 'false', 'world': sim_world}.items()
    )

    robot_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', robot_description, '-entity', 'my_robot'],
        output='screen',
    )

    nodes_to_start = [gazebo,
                      robot_spawner_node,
                      robot_state_pub_node,
                      joint_state_publisher_node,
                    #   controller_node,
                    #   joint_state_broadcaster_spawner,
                    #   joint_controller_spawner_node,
                      ]

    return LaunchDescription(declared_arguments + nodes_to_start)