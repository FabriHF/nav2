from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

pkg_share = get_package_share_directory('familiapachuquena_nav2_puzzlebot')
controllers_yaml = os.path.join(pkg_share, 'config', 'diff_controllers.yaml')


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    map_name = LaunchConfiguration('map_name').perform(context)

    world_file = f"{map_name}.world"
    robot = 'puzzlebot_jetson_lidar_ed'

    if map_name == 'hexagonal':
        pos_x, pos_y, pos_yaw = '-2.0', '-0.5', '0.0'
    elif map_name == 'puzzlebot':
        pos_x, pos_y, pos_yaw = '1.15', '1.20', '3.1416'

    sim_time = 'true'
    pause_gazebo = 'false'
    gazebo_verbosity = 4

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_resources = get_package_share_directory('familiapachuquena_nav2_puzzlebot')

    robot_path = os.path.join(gazebo_resources, 'urdf', 'mcr2_robots', f"{robot}.xacro")
    world_path = os.path.join(gazebo_resources, 'worlds', world_file)
    gazebo_models_path = os.path.join(gazebo_resources, 'models')
    gazebo_plugins_path = os.path.join(gazebo_resources, 'plugins')
    gazebo_media_path = os.path.join(gazebo_models_path, 'models', 'media', 'materials')
    bridge_config = os.path.join(gazebo_resources, 'config', f"{robot}.yaml")

    # Argumentos de lanzamiento
    declare_x = DeclareLaunchArgument('x', default_value=pos_x, description='Posición x del robot')
    declare_y = DeclareLaunchArgument('y', default_value=pos_y, description='Posición y del robot')
    declare_yaw = DeclareLaunchArgument('yaw', default_value=pos_yaw, description='Posición yaw del robot')
    declare_sim = DeclareLaunchArgument('use_sim_time', default_value=sim_time, description='Usar tiempo simulado')
    declare_pause = DeclareLaunchArgument('pause', default_value=pause_gazebo, description='Iniciar Gazebo pausado')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pause = LaunchConfiguration('pause')

    # Variables de entorno para Gazebo
    set_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{gazebo_models_path}:{gazebo_media_path}"
    )
    set_plugins = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=gazebo_plugins_path
    )

    # Descripción del robot
    robot_description = Command(['xacro ', robot_path])

    # Lanzar Gazebo
    gz_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    start_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': ['-r', f'-v {gazebo_verbosity} ', world_path],
            'on_exit_shutdown': 'true',
            'pause': pause
        }.items()
    )

    # Publicar estado del robot
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(robot_description, value_type=str),
            "use_sim_time": use_sim_time,
        }],
    )

    # Spawnear robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "puzzlebot", "-topic", "robot_description", "-x", x, "-y", y, "-Y", yaw],
        output="screen",
    )

    # Bridge ROS–Gazebo
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # Bridge de imagen (si aplica)
    image_bridge = None
    if robot != "puzzlebot_hacker_ed":
        image_bridge = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['camera']
        )

    actions = [
        declare_x, declare_y, declare_yaw, declare_sim, declare_pause,
        set_resources, set_plugins,
        robot_state_pub, start_gz, spawn_robot, bridge_node
    ]
    if image_bridge:
        actions.append(image_bridge)

    launch_dir = os.path.join(gazebo_resources, 'launch')
    if mode == 'nav':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
                launch_arguments={'map_name': LaunchConfiguration('map_name')}.items()
            )
        )
    elif mode == 'map':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mapping.launch.py')),
                launch_arguments={'map_name': LaunchConfiguration('map_name')}.items()
            )
        )
        )
    else:
        raise RuntimeError(f"[gazebo_world.launch.py] Modo desconocido '{mode}', se esperaba 'nav' o 'map'.")

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='nav', description='nav o map'),
        DeclareLaunchArgument('map_name', default_value='hexagonal', description='hexagonal o puzzlebot'),
        OpaqueFunction(function=launch_setup)
    ])
