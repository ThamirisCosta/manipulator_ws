from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from yaml import safe_load

def generate_launch_description():
    # Configurações
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    description_pkg = get_package_share_directory('manipulator_description_pkg')
    controller_pkg = get_package_share_directory('manipulator_controllers_pkg')
    
    # Caminhos dos arquivos
    urdf_path = os.path.join(description_pkg, 'urdf', 'manipulator_description_real.urdf')
    controller_config_path = os.path.join(controller_pkg, 'controllers', 'controllers.yaml')
    
    # Carregar conteúdo dos arquivos
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    # Carregar apenas a configuração do controller_manager
    with open(controller_config_path, 'r') as f:
        controller_config = safe_load(f)['controller_manager']['ros__parameters']

    # Nós
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'source_list': ['joint_states'],
            'use_sim_time': use_sim_time
        }]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time},
            controller_config,
            controller_config_path  # Adiciona o arquivo completo para os controladores específicos
        ]
    )

    # Controladores
    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        robot_state_publisher,
        joint_state_publisher,
        controller_manager,
        joint_state_broadcaster,
                
        # Sequência de inicialização
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[velocity_controller]
            )
        ),
    ])


