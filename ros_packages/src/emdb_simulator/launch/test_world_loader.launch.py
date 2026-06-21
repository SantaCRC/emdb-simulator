from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_file = LaunchConfiguration("world_file")
    rate_hz = LaunchConfiguration("rate_hz")
    render_mode = LaunchConfiguration("render_mode")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world_file",
            default_value="worlds/kitchen_test.yaml",
            description="Relative path inside config/ to the world YAML file",
        ),
        DeclareLaunchArgument(
            "rate_hz",
            default_value="60.0",
            description="Simulation step rate in Hz",
        ),
        DeclareLaunchArgument(
            "render_mode",
            default_value="human",
            description="MuJoCo render mode",
        ),
        Node(
            package="emdb_simulator",
            executable="test_world_loader",
            name="test_world_loader",
            output="screen",
            parameters=[{
                "world_file": world_file,
                "rate_hz": rate_hz,
                "render_mode": render_mode,
            }],
        ),
    ])