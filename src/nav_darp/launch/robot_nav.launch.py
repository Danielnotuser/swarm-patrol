import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('nav_darp')
    
    # Resolve LaunchConfigurations to concrete values
    robot_id_str = context.perform_substitution(LaunchConfiguration('robot_id'))
    robot_id_int = int(robot_id_str)
    
    params_file_name = context.perform_substitution(LaunchConfiguration('params_file'))
    params_file_path = os.path.join(pkg_share, 'config', params_file_name)
    
    autostart_str = context.perform_substitution(LaunchConfiguration('autostart'))
    autostart_bool = autostart_str.lower() == 'true'
    
    use_respawn = context.perform_substitution(LaunchConfiguration('use_respawn')).lower() == 'true'
    log_level = context.perform_substitution(LaunchConfiguration('log_level'))
    
    namespace = f'r{robot_id_str}'
    
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'smoother_server',
        'velocity_smoother',
        'bt_navigator',
    ]
    
    param_substitutions = {'autostart': autostart_str}
    
    remappings = [
        ('/tf', '/tf'),
        ('/tf_static', f'/{namespace}/tf_static'),
        ('/clock', f'/{namespace}/clock'),
    ]
    
    # We use RewrittenYaml to ensure the parameters are under the correct root key for the namespace
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file_path,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )
    
    nodes = [
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
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            parameters=[{'autostart': autostart_bool}, {'node_names': lifecycle_nodes}],
        ),
        Node(
            package='nav_darp',
            executable='odom_to_map.py',
            name='odom_to_map',
            output='screen',
            parameters=[{
                'robot_id': robot_id_int,
            }],
        ),
        Node(
            package='nav_darp',
            executable='darp_path_follower.py',
            name='darp_path_follower',
            output='screen',
            parameters=[{
                'robot_id': robot_id_int,
                'global_frame': f'robot{robot_id_str}_map',
            }],
        ),
        Node(
            package='nav_darp',
            executable='realtime_pointcloud.py',
            name='realtime_pointcloud',
            output='screen',
            parameters=[{
                'robot_id': robot_id_int,
                'target_frame': f'{namespace}/laser_frame',
                'global_frame': f'robot{robot_id_str}_map',
                'tf_topic': '/tf',
                'tf_static_topic': f'/{namespace}/tf_static',
            }],
            remappings=[],
        ),
    ]
    
    return [
        GroupAction([
            PushRosNamespace(namespace=namespace),
            *nodes
        ])
    ]

def generate_launch_description():
    declare_robot_id_cmd = DeclareLaunchArgument('robot_id', default_value='0')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value="nav2_params_r0.yaml",
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )
    
    return LaunchDescription([
        declare_robot_id_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        OpaqueFunction(function=launch_setup),
    ])