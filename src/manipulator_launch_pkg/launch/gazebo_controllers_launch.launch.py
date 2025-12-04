from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'position_controller'],
        output='screen'
    )

    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller'],
        output='screen'
    )

    # Event handlers para sequência de inicialização
    spawn_entity_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )
    
    joint_state_controller_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_position_controller, load_velocity_controller]
        )
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Simulação Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('ros_gz_sim'), '/launch/gz_sim.launch.py']
            ),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])],
        ),
        
        # Gerenciador de controladores
        controller_manager,
        
        # Spawn da entidade no Gazebo
        gz_spawn_entity,
        
        # Handlers para sequência de inicialização
        spawn_entity_handler,
        joint_state_controller_handler,
    ])
