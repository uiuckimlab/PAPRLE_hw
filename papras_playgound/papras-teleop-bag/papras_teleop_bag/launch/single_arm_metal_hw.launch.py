
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():
    # Arguments
    declared_arguments = [
        DeclareLaunchArgument('log_level', default_value='debug'),
        DeclareLaunchArgument('pipeline', default_value='ompl'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # Gazebo args
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyUSB0'),

        # Model spawn locations
        DeclareLaunchArgument('robot_pos_x', default_value='0'),
        DeclareLaunchArgument('robot_pos_y', default_value='0'),
        DeclareLaunchArgument('robot_pos_z', default_value='0.2'),
        DeclareLaunchArgument('robot_pos_yaw', default_value=str(3.14159)),
    ]
    baud_rate = LaunchConfiguration('baud_rate')
    usb_port = LaunchConfiguration('usb_port')

    # Load Robot Description
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('papras_teleop_bag'),
            'description/urdf',
            'single_7dof_metal_arm_hw.xacro',
        ]),
        ' ',
        'usb_port:=', usb_port,
        ' ',
        'baud_rate:=', baud_rate,
    ])
    robot_description = ParameterValue(robot_description, value_type=str)

    start_rviz = LaunchConfiguration('rviz')
    use_sim = LaunchConfiguration('use_sim_time')
    logger = LaunchConfiguration('log_level')


    # Control-related 
    controller_manager_config = PathJoinSubstitution(
        [
            FindPackageShare('papras_controls'),
            'config',
            'joint_trajectory_controller_7dof.yaml',
        ]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_manager_config
        ],
        output="both",
        condition=UnlessCondition(use_sim),
        #arguments=['--ros-args', '--log-level', logger],
        )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim}],
        output='screen',
        #arguments=['--ros-args', '--log-level', logger],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )


    included_launches = []
    # # RVIZ
    launch_dir = os.path.join(get_package_share_directory('teleop_bag_single_arm_moveit_configs'), 'launch')
    if start_rviz:
        rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/moveit_rviz.launch.py'])
        )
        delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rviz_launch],
            )
        )
        included_launches.append(delay_rviz_after_joint_state_broadcaster_spawner)

    # MoveIt
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/move_group.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim,
        }.items(),
    )
    included_launches.append(move_group_launch)
    


    # Load controllers
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm1_controller'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper1_controller'],
        output='screen',
    )


    delay_arm_controller_spawner_after_joint_state_broadcaster_spawner = \
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        )

    delay_gripper_controller_spawner_after_joint_state_broadcaster_spawner = \
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[gripper_controller_spawner],
            )
        )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_gripper_controller_spawner_after_joint_state_broadcaster_spawner,
    ]


    return LaunchDescription(declared_arguments + included_launches + nodes)
