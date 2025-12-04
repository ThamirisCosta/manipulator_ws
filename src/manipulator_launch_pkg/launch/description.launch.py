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
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    description_pkg = get_package_share_directory('manipulator_description_pkg')
        
    # Caminhos dos arquivos
    urdf_path = os.path.join(description_pkg, 'urdf', 'manipulator_description.urdf')
    
    # Carregar conteúdo dos arquivos
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
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

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher
    ])
