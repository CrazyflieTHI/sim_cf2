import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    package_description = "sim_cf2"

    # Get the arguments from the main.launch.xml file
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_file = LaunchConfiguration('robot_file').perform(context)
    enable_ground_truth = LaunchConfiguration('enable_ground_truth').perform(context)
    enable_wind = LaunchConfiguration('enable_wind').perform(context)
    color_prop_front = LaunchConfiguration('color_prop_front').perform(context)
    color_prop_back = LaunchConfiguration('color_prop_back').perform(context)
    rotors_description_dir = LaunchConfiguration('rotors_description_dir').perform(context)
    enable_mr_deck = LaunchConfiguration('enable_mr_deck').perform(context)
    mr_deck_visualize = LaunchConfiguration('mr_deck_visualize').perform(context)
    robot_description_topic_name = "/" + namespace + "_robot_description"
    robot_state_publisher_name = namespace +  "_robot_state_publisher"
    joint_state_topic_name = "/" + namespace + "/joint_states"

    # Process the robot description xacro file
    robot_desc_path = os.path.join(get_package_share_directory(
    package_description), "models", "rotors_description", "urdf", robot_file)
    robot_desc = xacro.process_file(robot_desc_path, mappings={'namespace' : namespace,
                                                               'enable_ground_truth' : enable_ground_truth,
                                                               'enable_wind' : enable_wind,
                                                               'color_prop_front' : color_prop_front,
                                                               'color_prop_back' : color_prop_back,
                                                               'enable_mr_deck' : enable_mr_deck,
                                                               'mr_deck_visualize' : mr_deck_visualize,
                                                               'rotors_description_dir' : rotors_description_dir})
    xml = robot_desc.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=robot_state_publisher_name,
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': xml}],
        remappings=[("/robot_description", robot_description_topic_name),
                    ("/joint_states", joint_state_topic_name)
                    ],
        output="screen"
    )

    return [robot_state_publisher_node]

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument('namespace', default_value='cf1')
    robot_file_arg = DeclareLaunchArgument('robot_file', default_value='crazyflie.urdf')
    enable_ground_truth_arg = DeclareLaunchArgument('enable_ground_truth', default_value='false')
    enable_wind_arg = DeclareLaunchArgument('enable_wind', default_value='false')
    color_prop_front_arg = DeclareLaunchArgument('color_prop_front', default_value='Black')
    color_prop_back_arg = DeclareLaunchArgument('color_prop_back', default_value='Black')
    enable_mr_deck_arg = DeclareLaunchArgument('enable_mr_deck', default_value='false')
    mr_deck_visualize_arg = DeclareLaunchArgument('mr_deck_visualize', default_value='false')
    rotors_description_dir_arg = DeclareLaunchArgument('rotors_description_dir', default_value='')
    return LaunchDescription([
        namespace_arg,
        robot_file_arg,
        enable_ground_truth_arg,
        enable_wind_arg,
        color_prop_front_arg,
        color_prop_back_arg,
        enable_mr_deck_arg,
        mr_deck_visualize_arg,
        rotors_description_dir_arg,
        OpaqueFunction(function = launch_setup)
        ])
