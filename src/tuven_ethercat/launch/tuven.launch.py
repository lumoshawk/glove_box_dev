from launch import LaunchDescription
from launch_ros.actions import Node

from tuven_ethercat import system


def generate_launch_description():
    params = system.load_params()
    robot_description, controllers = system.generate(params)

    # This ros2_control version reads /robot_description from a topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers],
        output='screen',
    )
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[system.CONTROLLER],
    )
    return LaunchDescription([robot_state_publisher, control_node, spawner])
