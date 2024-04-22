# from launch import LaunchDescription
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

# def generate_launch_description():
            
#     controllers_file = "iiwa_controllers.yaml"

        
#     simulation_controllers = PathJoinSubstitution(
#         [FindPackageShare(description_package), "config", controllers_file]
#     )


#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
#     )

#     joint_controller_spawner_node = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_trajectory_controller", "-c", "/controller_manager"]
#     )

#     controller_node = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         parameters=[robot_description, simulation_controllers],
#         output='both',
#     )


#     return LaunchDescription()