from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='emdb_simulator',
            executable='scene_loader',
            name='robocasa_rollout_node',
            output='screen',
            parameters=[{
                'control_mode': 'rl',
            }],
        ),
        Node(
            package='emdb_policy',
            executable='policy_node',
            name='emdb_agent_bridge',
            output='screen',
        ),
    ])
