from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='emdb_simulator',
            executable='test_scene_loader',
            name='robocasa_rollout_node',
            output='screen',
            parameters=[{
                'task': 'KitchenLift',
                'control_mode': 'teleop',
            }],
        ),
        Node(
            package='emdb_simulator',
            executable='test_keyboard_client',
            name='keyboard_client_node',
            output='screen',
        ),
    ])