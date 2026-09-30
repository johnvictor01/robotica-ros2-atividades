import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_share = get_package_share_directory('trajectory_control')
    model_share = get_package_share_directory('model_description')
    model_launch = os.path.join(model_share, 'launch', 'robot.launch.py')
    config = os.path.join(control_share, 'config', 'control.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mission',
                              description='pose, mission ou figure8'),
        DeclareLaunchArgument('pose_method', default_value='continuous',
                              description='continuous ou three_maneuvers'),
        DeclareLaunchArgument('loop_mission', default_value='true',
                              description='Repetir a missao continuamente'),
        DeclareLaunchArgument('waypoint_radius', default_value='0.10',
                              description='Raio de chegada ao waypoint, em metros'),
        DeclareLaunchArgument('tracking_mode', default_value='feedback',
                              description='open_loop ou feedback'),
        DeclareLaunchArgument('kff', default_value='1.0',
                              description='Ganho proporcional do termo feedforward'),
        DeclareLaunchArgument('omega', default_value='0.09',
                              description='Frequencia angular da trajetoria em oito'),
        DeclareLaunchArgument('A', default_value='2.0',
                              description='Amplitude longitudinal da lemniscata, em metros'),
        DeclareLaunchArgument('B', default_value='4.0',
                              description='Parametro transversal da lemniscata, em metros'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.75',
                              description='Limite linear do controlador, em m/s'),
        DeclareLaunchArgument('max_angular_speed', default_value='1.5',
                              description='Limite angular do controlador, em rad/s'),
        DeclareLaunchArgument('plot', default_value='true',
                              description='Abrir PlotJuggler com a simulacao'),
        DeclareLaunchArgument('record', default_value='false',
                              description='Gravar poses e odometria em rosbag durante a simulacao'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(model_launch),
            launch_arguments={'manual': 'false', 'plot': 'false'}.items(),
        ),
        Node(
            package='trajectory_control', executable='trajectory_controller',
            name='trajectory_controller', output='screen',
            parameters=[config, {
                'mode': LaunchConfiguration('mode'),
                'pose_method': LaunchConfiguration('pose_method'),
                'loop_mission': LaunchConfiguration('loop_mission'),
                'waypoint_radius': LaunchConfiguration('waypoint_radius'),
                'tracking_mode': LaunchConfiguration('tracking_mode'),
                'kff': LaunchConfiguration('kff'),
                'omega': LaunchConfiguration('omega'),
                'A': LaunchConfiguration('A'),
                'B': LaunchConfiguration('B'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'use_sim_time': True,
            }],
            remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
        ),
        Node(
            package='plotjuggler', executable='plotjuggler', name='plotjuggler',
            condition=IfCondition(LaunchConfiguration('plot')),
            output='screen',
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '/trajectory/desired_pose', '/trajectory/actual_pose', '/odom',
            ],
            condition=IfCondition(LaunchConfiguration('record')),
            output='screen',
        ),
    ])
