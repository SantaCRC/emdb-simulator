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
                'env_name': 'robocasa/PickPlaceCounterToCabinet',
                'split': 'pretrain',
                'seed': 0,
                'num_rollouts': 3,
                'num_steps': 100,
                'video_path': '/tmp/test.mp4',
                'run_once': True,
                'render': True,
            }],
        ),
        Node(
            package='emdb_simulator',
            executable='test_keyboard_client',
            name='keyboard_client_node',
            output='screen',
        ),
        Node(
            package='emdb_simulator',
            executable='test_position_server',
            name='position_server_node',
            output='screen',
        ),
    ])