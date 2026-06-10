from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    render_mode = LaunchConfiguration('render_mode')
    rate_hz = LaunchConfiguration('rate_hz')
    joint_state_topic = LaunchConfiguration('joint_state_topic')
    cmd_topic = LaunchConfiguration('cmd_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'render_mode',
            default_value='human',
            description='human, rgb_array, depth_array, rgbd_tuple o none'
        ),
        DeclareLaunchArgument(
            'rate_hz',
            default_value='50.0',
            description='Frecuencia de simulacion'
        ),
        DeclareLaunchArgument(
            'joint_state_topic',
            default_value='/joint_states',
            description='Topic para publicar los estados de las articulaciones'
        ),
        DeclareLaunchArgument(
            'cmd_topic',
            default_value='/action',
            description='Topic para suscribirse a los comandos de accion'
        ),
        Node(
            package='emdb_simulator',
            executable='test_simulator',
            name='gym_mujoco_bridge',
            output='screen',
            parameters=[{
                'render_mode': render_mode,
                'rate_hz': rate_hz,
                'joint_state_topic': joint_state_topic,
                'cmd_topic': cmd_topic,
                
            }],
        )
    ])