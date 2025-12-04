from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from yaml import safe_load

def generate_launch_description():
    # Configurações
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    description_pkg = get_package_share_directory('manipulator_description_pkg')
    controller_pkg = get_package_share_directory('manipulator_controllers_pkg')

    # Caminhos dos arquivos
    urdf_file = os.path.join(description_pkg, 'urdf', 'manipulator_description.urdf')
    gazebo_urdf_file = os.path.join(description_pkg, 'urdf', 'manipulator_description_gazebo.urdf')
    controller_config_file = os.path.join(controller_pkg, 'controllers', 'controllers.yaml')

    # Carregar conteúdo dos arquivos
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    with open(controller_config_file, 'r') as f:
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
        arguments=['-file', gazebo_urdf_file, '-name', 'manipulator'],
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
        robot_state_publisher,
        joint_state_publisher,

        # Simulação Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('ros_gz_sim'), '/launch/gz_sim.launch.py']
            ),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])],
        ),

        controller_manager,
        gz_spawn_entity,
        spawn_entity_handler,
        joint_state_controller_handler
    ])