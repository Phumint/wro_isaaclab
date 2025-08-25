import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Get paths to required files
    pkg_share_path = get_package_share_directory('my_robot_gazebo')
    urdf_path = os.path.join(pkg_share_path, 'urdf', 'steer_bot.urdf.xacro')
    world_path = os.path.join(pkg_share_path, 'worlds', 'empty.world')

    # Process the URDF (Xacro) file
    robot_description_content = xacro.process_file(urdf_path).toprettyxml(indent='  ')
    
    # Declare a launch argument for use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Launch Gazebo server and client
    gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}]
    )

    # Spawn the robot entity in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # ros2_control_node (Controller Manager) - this is what the spawners talk to
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Joint State Broadcaster Spawner (CORRECTED)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager', '--controller-manager-timeout', '1000'],
        output='screen'
    )

    # Ackermann Steering Controller Spawner (CORRECTED)
    ackermann_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller', '-c', 'controller_manager', '--controller-manager-timeout', '1000'],
        output='screen'
    )

    # Ros2 Policy Controller Node
    ros2_policy_controller = Node(
        package='my_robot_controller',
        executable='ros2_policy_controller',
        name='policy_controller_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        gzserver,
        gzclient,
        robot_state_publisher_node,
        spawn_entity_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        ackermann_steering_controller_spawner,
        ros2_policy_controller
    ])
