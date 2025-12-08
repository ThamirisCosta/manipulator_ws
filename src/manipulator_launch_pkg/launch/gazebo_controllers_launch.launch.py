from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import re
from yaml import safe_load

def resolve_package_uri(urdf_content: str) -> str:
    """Resolve package:// URIs to absolute paths for Gazebo Sim compatibility."""
    def replace_package_uri(match):
        package_name = match.group(1)
        relative_path = match.group(2)
        try:
            package_path = get_package_share_directory(package_name)
            return os.path.join(package_path, relative_path)
        except Exception:
            return match.group(0)  # Return original if package not found

    pattern = r'package://([^/]+)/(.+?\.stl)'
    return re.sub(pattern, replace_package_uri, urdf_content)

def generate_launch_description():
    # Configurações
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    description_pkg = get_package_share_directory('manipulator_description_pkg')
    controller_pkg = get_package_share_directory('manipulator_controllers_pkg')

    # Caminhos dos arquivos
    gazebo_urdf_path = os.path.join(description_pkg, 'urdf', 'manipulator_description_gazebo.urdf')
    controller_config_path = os.path.join(controller_pkg, 'controllers', 'controllers.yaml')

    # Carregar conteúdo dos arquivos
    with open(gazebo_urdf_path, 'r') as f:
        robot_description_raw = f.read()

    # Resolver package:// URIs para paths absolutos (necessário para Gazebo Sim)
    robot_description = resolve_package_uri(robot_description_raw)

    with open(controller_config_path, 'r') as f:
        controller_config = safe_load(f)['controller_manager']['ros__parameters']

    # Nós
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time},
            controller_config
        ],
        output='screen'
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='gz_spawn_entity',
        arguments=['-string', robot_description, '-name', 'manipulator'],
        output='screen'
    )

    # Controladores
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller'],
        output='screen'
    )

    # Delay para garantir que o Gazebo carregou o plugin e o controller_manager está pronto
    # (mesmo quando gerenciado pelo Gazebo, leva alguns segundos)
    delayed_load_joint_state = TimerAction(
        period=5.0,
        actions=[load_joint_state_controller],
    )

    delayed_load_velocity = TimerAction(
        period=7.0,
        actions=[load_velocity_controller],
    )

    # 1) quando a entidade for spawnada no Gazebo, iniciaremos o controller_manager (se desejado)
    spawn_entity_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[controller_manager],
        ),
        condition=IfCondition(LaunchConfiguration('start_controller_manager'))
    )

    # 2) quando o controller_manager iniciar, carregamos o joint_state_broadcaster
    controller_manager_start_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[load_joint_state_controller],
        ),
        condition=IfCondition(LaunchConfiguration('start_controller_manager'))
    )

    # 3) após o carregamento do joint_state_broadcaster, ativamos o velocity_controller
    joint_state_controller_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_velocity_controller]
        ),
        condition=IfCondition(LaunchConfiguration('start_controller_manager'))
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_controller_manager', default_value='true'),
        
        # Simulação Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('ros_gz_sim'), '/launch/gz_sim.launch.py']
            ),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])],
        ),
        
        # Spawn da entidade no Gazebo (será inicializado primeiro)
        gz_spawn_entity,
        
        # Handlers para sequência de inicialização com controller_manager local
        # (controller_manager será iniciado APÓS o spawn terminar)
        spawn_entity_handler,
        controller_manager_start_handler,
        joint_state_controller_handler,
        
        # Carregamento automático de controladores quando usando Gazebo controller_manager
        # (start_controller_manager:=false)
        delayed_load_joint_state,
        delayed_load_velocity,
    ])
