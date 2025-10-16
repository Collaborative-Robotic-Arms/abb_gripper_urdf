import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit, OnProcessStart 

def generate_launch_description():
    # --- Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # --- Package Paths & URDF Processing ---
    pkg_name = 'abb_gripper_urdf'
    pkg_path = FindPackageShare(pkg_name)

    xacro_file = PathJoinSubstitution([
        pkg_path,
        'urdf',
        'abb_gripper.urdf.xacro' 
    ])

    robot_description_command = Command(['xacro ', ' ', xacro_file])
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    # Set the GAZEBO resource path
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([pkg_path, '']), 
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'worlds'])
        ]
    )

    # --- Simulation Setup ---
    
    # 1. Gazebo Launch (Starts the simulation and world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(), 
    )
    
    # 2. Gazebo Clock Bridge (Crucial for time synchronization)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            # Bridge the Gazebo clock to the ROS /clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 3. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(robot_description_command, value_type=str)},
            {'use_sim_time': use_sim_time} 
        ],
    )

    # 4. Spawn Entity (Launched immediately)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'abb_gripper_urdf', 
            '-x', '1.0', '-y', '1.0', '-z', '1.0'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- CONTROL SETUP (NO EXTERNAL controller_manager_node) ---
    
    # The Controller Manager is now run internally by the Gazebo plugin.
    # We only need the spawner nodes.

    def make_spawner_node(name, controller_manager):
        # Spawners target the Controller Manager instance that runs inside Gazebo
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', controller_manager],
            parameters=[
                {'use_sim_time': use_sim_time} 
            ],
            output='screen',
        )

    # 1. Spawner node for Joint State Broadcaster
    joint_state_broadcaster_spawner = make_spawner_node(
        'joint_state_broadcaster', '/controller_manager'
    )
    
    # 2. Spawner node for Finger 1 Position Controller
    gripper_1_controller_spawner = make_spawner_node(
        'gripper_finger_1_controller', '/controller_manager'
    )
    
    # 3. Spawner node for Finger 2 Position Controller
    gripper_2_controller_spawner = make_spawner_node(
        'gripper_finger_2_controller', '/controller_manager'
    )

    # --- Event Handlers to enforce reliable startup order (Time delay removed) ---
    
    # The robot model is now spawned right away (not delayed by TimerAction).

    # 1. After the model is spawned, start the Joint State Broadcaster
    delay_broadcaster_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity, 
            on_exit=[
                joint_state_broadcaster_spawner
            ]
        )
    )

    # 2. After the Broadcaster is ready, start the first gripper controller
    delay_gripper_1_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                gripper_1_controller_spawner
            ]
        )
    )

    # 3. After the first gripper controller is ready, start the second gripper controller
    delay_gripper_2_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_1_controller_spawner,
            on_exit=[
                gripper_2_controller_spawner
            ]
        )
    )

    return LaunchDescription([
        # Arguments
        declare_use_sim_time_cmd,
        
        # Environment and Simulation
        set_gz_resource_path,
        gazebo,
        clock_bridge, 
        
        # Robot State
        robot_state_publisher,
        
        # Immediate Spawning
        spawn_entity,
        
        # Control nodes, waiting on process exit
        delay_broadcaster_spawner,
        delay_gripper_1_spawner,
        delay_gripper_2_spawner,
    ])
