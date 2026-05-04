import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share = get_package_share_directory('nav_darp')
    #nav2_bringup_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    namespace = PythonExpression(["'r' + str(", LaunchConfiguration('robot_id'), ")"])

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'velocity_smoother',
    ]

    param_substitutions = {'autostart': autostart}
    remappings = []

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    declare_robot_id_cmd = DeclareLaunchArgument('robot_id', default_value='0')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    load_nodes = GroupAction([
        SetParameter('use_sim_time', use_sim_time),
        PushRosNamespace(namespace=namespace),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings
                       + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
        ),
        Node(
            package='nav_darp',
            executable='darp_path_follower.py',
            name='darp_path_follower',
            output='screen',
            parameters=[{
                'ROBOT_ID': LaunchConfiguration('robot_id'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])

    return LaunchDescription([
        declare_robot_id_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        load_nodes,
    ])
