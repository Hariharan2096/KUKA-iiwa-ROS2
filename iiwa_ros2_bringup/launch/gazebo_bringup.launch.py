from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    description_package = "ros2_iiwa_description"
    description_file = "iiwa.urdf.xacro"

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
        )
    )
    prefix = LaunchConfiguration('prefix')
    

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

    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher Node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
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
        arguments=['-topic', robot_description, '-entity', 'iiwa_gazebo'],
        output='screen',
    )

    # ROS2 Control Nodes

    controllers_file = "iiwa_controllers.yaml"

        
    simulation_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", controllers_file]
    )


    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[simulation_controllers, "--controller-manager", "/controller_manager"]
    # )

    # controller_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[simulation_controllers],
    #     output='both',
    # )
    # delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     ),
    #     condition=IfCondition(use_sim),
    # )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    nodes_to_start = [gazebo,
                      robot_state_pub_node,
                      robot_spawner_node,
                      ]

    return LaunchDescription(declared_arguments + nodes_to_start)