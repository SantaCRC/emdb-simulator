from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "base_arena_file",
            default_value="config/arenas/base_arena.yaml",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value="worlds/kitchen_test.yaml",
        ),
        DeclareLaunchArgument(
            "objects_file",
            default_value="objects/objects_test.yaml",
        ),
        DeclareLaunchArgument(
            "rate_hz",
            default_value="60.0",
        ),
        DeclareLaunchArgument(
            "seed",
            default_value="42",
        ),
        Node(
            package="emdb_simulator",
            executable="test_object_loader",
            name="test_object_loader",
            output="screen",
            parameters=[{
                "base_arena_file": LaunchConfiguration("base_arena_file"),
                "world_file": LaunchConfiguration("world_file"),
                "objects_file": LaunchConfiguration("objects_file"),
                "rate_hz": LaunchConfiguration("rate_hz"),
                "seed": LaunchConfiguration("seed"),
            }],
        ),
    ])