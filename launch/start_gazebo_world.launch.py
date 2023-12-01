import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    world_file_name = 'basic.world'
    package_name = "sim_cf2"
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    install_dir = get_package_prefix(package_name)
    world = os.path.join(get_package_share_directory(package_name), 'worlds', world_file_name)
    model_path = os.path.join(get_package_share_directory(package_name), 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + '/share' + ':' + model_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    # Launch configuration variables specific to simulation
    headless = LaunchConfiguration('headless')
    use_simulator = LaunchConfiguration('use_simulator')

    # Declare launch arguments for Gazebo
    declare_simulator_cmd = DeclareLaunchArgument(
      name='headless',
      default_value='False',
      description='Whether to execute gzclient')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
      name='use_sim_time',
      default_value='true',
      description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
      name='use_simulator',
      default_value='True',
      description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
      name='world_name',
      default_value=world,
      description='Full path to the world model file to load')

    declare_pause_cmd = DeclareLaunchArgument(
      name='pause',
      default_value="false",
      description='Start Gazebo paused')

    declare_pause_cmd = DeclareLaunchArgument(
      name='pause',
      default_value="true",
      description='Start Gazebo paused')

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
      condition=IfCondition(use_simulator),
      launch_arguments={'world': world}.items()
      )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
      condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
      )

    ld = LaunchDescription()
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_pause_cmd)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    return ld
