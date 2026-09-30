import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_share = get_package_share_directory('model_description')
    gazebo_share = get_package_share_directory('gazebo_ros')
    default_world = os.path.join(package_share, 'worlds', 'asphalt.world')
    gazebo_model_path = os.path.dirname(package_share)
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing_model_path:
        gazebo_model_path += os.pathsep + existing_model_path
    urdf = os.path.join(package_share, 'urdf', 'robot.urdf')
    # gazebo_ros2_control passes robot_description through ROS argument parsing;
    # XML comments containing punctuation can make Humble reject that argument.
    robot_description = re.sub(
        r'<!--.*?-->', '', xacro.process_file(urdf).toxml(), flags=re.DOTALL
    )

    return LaunchDescription([
        # ROS Humble's Python nodes need the system Python even when the shell
        # has a pyenv interpreter earlier in PATH.
        SetEnvironmentVariable('PATH', '/usr/bin:/opt/ros/humble/bin:' + os.environ.get('PATH', '')),
        # Gazebo only needs loopback networking for this single-machine sim.
        SetEnvironmentVariable('GAZEBO_IP', '127.0.0.1'),
        # package://model_description meshes are resolved by Gazebo Classic as
        # model://model_description/... from the package share parent.
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        # Use a separate master port in case another Gazebo Classic instance
        # is already listening on the default port 11345.
        SetEnvironmentVariable('GAZEBO_MASTER_URI', 'http://127.0.0.1:11346'),
        # This world uses only local assets. Avoid blocking the GUI while it
        # tries to refresh Gazebo Classic's remote model catalog.
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', 'http://127.0.0.1:1'),
        DeclareLaunchArgument('gui', default_value='true', description='Iniciar Gazebo com interface gráfica'),
        DeclareLaunchArgument('plot', default_value='false', description='Iniciar PlotJuggler para observar /odom'),
        DeclareLaunchArgument('manual', default_value='true', description='Iniciar teleoperacao pelo teclado'),
        DeclareLaunchArgument('world', default_value=default_world, description='Arquivo de mundo SDF'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'world': LaunchConfiguration('world'),
            }.items(),
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=['-entity', 'tri_x', '-topic', 'robot_description', '-z', '0.10'],
        ),
        TimerAction(period=5.0, actions=[
            Node(package='controller_manager', executable='spawner', output='screen',
                 arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']),
        ]),
        TimerAction(period=7.0, actions=[
            Node(package='controller_manager', executable='spawner', output='screen',
                 arguments=['diff_drive_controller', '--controller-manager', '/controller_manager']),
        ]),
        TimerAction(period=9.0, actions=[
            Node(package='topic_tools', executable='relay', name='odom_relay', output='screen',
                 arguments=['/diff_drive_controller/odom', '/odom']),
        ]),
        Node(
            package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard', output='screen', emulate_tty=True,
            prefix='gnome-terminal --',
            condition=IfCondition(LaunchConfiguration('manual')),
            remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
        ),
        Node(
            package='plotjuggler', executable='plotjuggler', name='plotjuggler',
            condition=IfCondition(LaunchConfiguration('plot')),
            arguments=['--ros-args', '--remap', '/tf:=/tf'], output='screen',
        ),
    ])
