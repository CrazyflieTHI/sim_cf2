import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    entity_name = LaunchConfiguration('namespace').perform(context)
    x_initial = LaunchConfiguration('x_initial').perform(context)
    y_initial = LaunchConfiguration('y_initial').perform(context)
    z_initial = LaunchConfiguration('z_initial').perform(context)
    roll_initial = LaunchConfiguration('roll_initial').perform(context)
    pitch_initial = LaunchConfiguration('pitch_initial').perform(context)
    yaw_initial = LaunchConfiguration('yaw_initial').perform(context)

    robot_description_topic_name = "/" + entity_name + "_robot_description"
    robot_state_publisher_name= entity_name + "_robot_state_publisher"

    # Spawn ROBOT in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='crazyflie_spawn_entity',
        output='screen',
        emulate_tty=True,
        arguments=['-entity',
                   entity_name,
                   '-x', str(x_initial),
                   '-y', str(y_initial),
                   '-z', str(z_initial),
                   '-R', str(roll_initial),
                   '-P', str(pitch_initial),
                   '-Y', str(yaw_initial),
                   '-topic', robot_description_topic_name
                   ],
        remappings=[("/robot_state_publisher", robot_state_publisher_name)
                    ]
    )

    return [spawn_robot]

def generate_launch_description():

    entity_name_arg = DeclareLaunchArgument('namespace', default_value='cf1')
    x_initial_arg = DeclareLaunchArgument('x_initial', default_value='0.0')
    y_initial_arg = DeclareLaunchArgument('y_initial', default_value='0.0')
    z_initial_arg = DeclareLaunchArgument('z_initial', default_value='0.0')
    roll_initial_arg = DeclareLaunchArgument('roll_initial', default_value='0.0')
    pitch_initial_arg = DeclareLaunchArgument('pitch_initial', default_value='0.0')
    yaw_initial_arg = DeclareLaunchArgument('yaw_initial', default_value='0.0')

    return LaunchDescription([
        entity_name_arg,
        x_initial_arg,
        y_initial_arg,
        z_initial_arg,
        roll_initial_arg,
        pitch_initial_arg,
        yaw_initial_arg,
        OpaqueFunction(function = launch_setup)
        ])
