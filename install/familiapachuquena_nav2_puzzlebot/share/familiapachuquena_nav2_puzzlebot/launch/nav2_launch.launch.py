from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('familiapachuquena_nav2_puzzlebot')
    world_file = os.path.join(pkg_path, 'worlds', 'world_prueba.world')
    rviz_mode = LaunchConfiguration('mode')

    rviz_config_map = os.path.join(pkg_path, 'rviz', 'mapping.rviz')
    rviz_config_nav = os.path.join(pkg_path, 'rviz', 'navigation.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='map', description='Modo: map o nav'),

        # Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        # robot_state_publisher (si usas URDF)
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', os.path.join(pkg_path, 'urdf', 'puzzlebot.urdf')],
            output='screen'
        ),

        # RVIZ con configuración según modo
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d',
                 rviz_config_map if rviz_mode == 'map' else rviz_config_nav],
            output='screen'
        ),
    ])
