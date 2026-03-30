import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def spawn_robots(context, *args, **kwargs):
    max_nb_robots = int(LaunchConfiguration('max_nb_robots').perform(context))
    world = LaunchConfiguration('world').perform(context)
    package_dir = get_package_share_directory('diff_drive_robot')
    urdf_xacro_path = os.path.join(package_dir, 'urdf', 'robot.xacro')
    template_yaml_path = os.path.join(package_dir, 'config', 'gz_bridge.yaml.in')

    # Читаем шаблон один раз
    with open(template_yaml_path, 'r') as f:
        template_content = f.read()

    actions = []

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': ['-r -v1 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    actions.append(gazebo)

    # Для каждого робота создаём свой мост
    for i in range(max_nb_robots):
        ns = f'r{i}'

        # Генерируем URDF через xacro
        urdf_str = subprocess.check_output(['xacro', urdf_xacro_path, f'ns:={ns}'], text=True)

        # Robot State Publisher
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': urdf_str,
            }],
            remappings=[('/tf', f'/{ns}/tf'), ('/tf_static', f'/{ns}/tf_static')]
        )
        actions.append(rsp_node)

        # Создаём временный YAML-файл для этого робота
        yaml_content = template_content.replace('{{ namespace }}', ns)
        # Используем уникальное имя в /tmp
        yaml_file = f'/tmp/gz_bridge_{ns}.yaml'
        with open(yaml_file, 'w') as f:
            f.write(yaml_content)

        # Узел моста с этим файлом конфигурации
        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', f'config_file:={yaml_file}'],
            output='screen'
        )
        actions.append(bridge_node)

        # Спавн робота через spawn (если работает)
        spawn_cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-topic', f'/{ns}/robot_description',
            '-name', f'diff_bot_{i}',
            '-x', '0.0',
            '-y', str(i * 1.0),
            '-z', '0.2'
        ]
        actions.append(ExecuteProcess(cmd=spawn_cmd, output='screen'))

    return actions

def generate_launch_description():
    package_name = 'diff_drive_robot'
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'obstacles.world')

    declare_world = DeclareLaunchArgument('world', default_value=world_path)
    declare_rviz = DeclareLaunchArgument('rviz', default_value='False')
    declare_max_robots = DeclareLaunchArgument('max_nb_robots', default_value='2')

    rviz_config = os.path.join(get_package_share_directory(package_name), 'rviz', 'bot.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config], output='screen')]
    )

    return LaunchDescription([
        declare_world,
        declare_rviz,
        declare_max_robots,
        rviz2,
        OpaqueFunction(function=spawn_robots)
    ])