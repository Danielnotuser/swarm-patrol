import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction

def generate_launch_description():

    package_name = 'diff_drive_robot'

    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')

    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'obstacles.world')

    declare_world = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')

    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Opens rviz if set to True')

    # Robot State Publisher
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'robot.urdf')
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'urdf': urdf_path}.items()
    )

    # Gazebo Sim (server + client) with world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), launch_arguments={'gz_args': ['-r -v1 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn robot
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'diff_bot', '-z', '0.2'],
        output='screen'
    )

    # ROS-GZ Bridge
    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    # Rviz
    rviz_config = os.path.join(get_package_share_directory(package_name), 'rviz', 'bot.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config], output='screen')]
    )

    return LaunchDescription([
        declare_rviz,
        declare_world,
        rviz2,
        rsp,
        gazebo,
        bridge,
        spawn
    ])