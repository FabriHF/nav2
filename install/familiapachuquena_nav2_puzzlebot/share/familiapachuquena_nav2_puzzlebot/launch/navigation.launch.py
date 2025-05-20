from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    base_path = get_package_share_directory('familiapachuquena_nav2_puzzlebot')
    rviz_file = os.path.join(base_path, 'rviz', "nav2_navigating.rviz")
    map_value = LaunchConfiguration('map_name').perform(context)

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            base_path,
            'map',
            f"{map_value}_map.yaml"
        )
    )

    param_file_name = 'puzzlebot.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            base_path,
            'param',
            param_file_name
        )
    )

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return [
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])