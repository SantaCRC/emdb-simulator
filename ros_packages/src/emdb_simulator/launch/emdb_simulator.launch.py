from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    teleop = LaunchConfiguration('teleop')
    control_mode = PythonExpression(["'teleop' if '", teleop, "' == 'true' else 'rl'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='Launch the keyboard client and drive scene_loader in teleop mode instead of rl.',
        ),
        Node(
            package='emdb_simulator',
            executable='scene_loader',
            name='robocasa_rollout_node',
            output='screen',
            parameters=[{
                'task': 'KitchenLift',
                'control_mode': control_mode,
            }],
        ),
        Node(
            package='emdb_simulator',
            executable='keyboard_client',
            name='keyboard_client_node',
            output='screen',
            condition=IfCondition(teleop),
        ),
    ])
