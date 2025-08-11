import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- CHANGE: Set the package name to your robotic arm's package ---
    pkg_name = 'ros2srrc_robots'
    # pkg_name = 'abb_gripper'
    pkg_path = FindPackageShare(pkg_name)

    # --- FIX: Reference the correct .xacro file for the robotic arm using PathJoinSubstitution ---
    # This ensures the launch system handles substitutions correctly before passing the path to xacro.
    # xacro_file = PathJoinSubstitution([
    #     pkg_path,
    #     'urdf',
    #     'irb120_macro.urdf.xacro'
    # ])

    # This ensures the launch system handles substitutions correctly before passing the path to xacro.
    xacro_file = PathJoinSubstitution([
        pkg_path,
        'irb120',
        'urdf',
        'irb120_macro.urdf.xacro'
    ])


    # xacro_file = PathJoinSubstitution([
    #     pkg_path,
    #     'urdf',
    #     'abb_gripper.urdf.xacro'
    # ])
    # --- FIX: Process the xacro file into a URDF string ---
    # We pass the PathJoinSubstitution object directly to the Command.
    # robot_description_command = Command(['xacro', ' ', xacro_file])
    robot_description_command = Command(['xacro ', ' ', xacro_file])

    # Find the ros_gz_sim package share directory
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Get the path to the gz_sim.launch.py file
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    # --- FIX: Set the GAZEBO resource path to find the meshes ---
    # This is crucial for Gazebo to locate the STL files referenced in your URDF.
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([pkg_path, '']),
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'worlds'])
        ]
    )

    # Start the Gazebo simulation with the empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': ['empty.sdf']}.items(),
    )

    # Publish the robot description from the URDF file
    # This node now takes the processed xacro content as a parameter
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # FIX: Wrap the Command substitution in ParameterValue to ensure it is treated as a string.
        parameters=[{'robot_description': ParameterValue(robot_description_command, value_type=str)}],
    )

    # Spawn the entity from the robot_description topic
    # We update the entity name to match the robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'Test',
            '-x', '1.0', '-y', '1.0', '-z', '10.0'
        ],
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
