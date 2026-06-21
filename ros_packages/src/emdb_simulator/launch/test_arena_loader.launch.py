from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arena_file = LaunchConfiguration("arena_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "arena_file",
            default_value="config/arenas/base_arena.yaml",
            description="Relative path to the base arena YAML inside the package share directory",
        ),
        Node(
            package="emdb_simulator",
            executable="test_arena_loader",
            name="test_arena_loader",
            output="screen",
            parameters=[{
                "arena_file": arena_file,
            }],
        ),
    ])