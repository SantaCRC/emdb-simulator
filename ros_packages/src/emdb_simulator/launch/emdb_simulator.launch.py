from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    teleop = LaunchConfiguration('teleop')
    control_mode = PythonExpression(["'teleop' if '", teleop, "' == 'true' else 'rl'"])
    headless = LaunchConfiguration('headless')
    perception_mode = LaunchConfiguration('perception_mode')
    record_video = LaunchConfiguration('record_video')
    record_video_dir = LaunchConfiguration('record_video_dir')
    record_video_episodes = LaunchConfiguration('record_video_episodes')
    record_video_camera = LaunchConfiguration('record_video_camera')
    record_video_fps = LaunchConfiguration('record_video_fps')
    record_video_width = LaunchConfiguration('record_video_width')
    record_video_height = LaunchConfiguration('record_video_height')
    record_video_stride = LaunchConfiguration('record_video_stride')
    record_video_crf = LaunchConfiguration('record_video_crf')
    record_video_keep_successes = LaunchConfiguration('record_video_keep_successes')
    preview_camera = LaunchConfiguration('preview_camera')
    preview_camera_names = LaunchConfiguration('preview_camera_names')
    custom_cameras_file = LaunchConfiguration('custom_cameras_file')
    env_seed = LaunchConfiguration('env_seed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='Launch the keyboard client and drive scene_loader in teleop mode instead of rl.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description="Don't create the on-screen mjviewer window or render to it "
                        '(has_renderer=False), regardless of control_mode. Unrelated to '
                        'record_video/offscreen rendering. Turn on for an rl-mode training '
                        "run where nobody's watching the window.",
        ),
        DeclareLaunchArgument(
            'perception_mode',
            default_value='unified',
            description='How object perceptions are published: unified, grouped, split, or mdb.',
        ),
        DeclareLaunchArgument(
            'record_video',
            default_value='false',
            description='Enable per-episode offscreen mp4 recording of the simulation.',
        ),
        DeclareLaunchArgument(
            'record_video_dir',
            default_value='/tmp/emdb_videos',
            description='Parent directory for recorded episode videos '
                        '(one timestamped run subdir per launch).',
        ),
        DeclareLaunchArgument(
            'record_video_episodes',
            default_value='all',
            description=(
                "Which episodes to record: 'all', a single episode id (e.g. '5'), "
                "or comma-separated ids/ranges (e.g. '0-2,10-12')."
            ),
        ),
        DeclareLaunchArgument(
            'record_video_camera',
            default_value='robot0_agentview_center',
            description='Fixed MuJoCo camera name used for offscreen video recording.',
        ),
        DeclareLaunchArgument(
            'record_video_fps',
            default_value='-1.0',
            description='Output video fps; -1 = auto (publish_rate / record_video_stride).',
        ),
        DeclareLaunchArgument(
            'record_video_width',
            default_value='1280',
            description='Recording frame width, in pixels.',
        ),
        DeclareLaunchArgument(
            'record_video_height',
            default_value='720',
            description='Recording frame height, in pixels.',
        ),
        DeclareLaunchArgument(
            'record_video_stride',
            default_value='1',
            description='Capture every Nth simulation step (1 = every step).',
        ),
        DeclareLaunchArgument(
            'record_video_crf',
            default_value='18',
            description='libx264 CRF quality '
                        '(0=lossless, 18=near-lossless, 23=default, 51=worst).',
        ),
        DeclareLaunchArgument(
            'record_video_keep_successes',
            default_value='false',
            description='Also keep any episode where the task succeeds, even if its index '
                        'falls outside record_video_episodes (in-range episodes are always '
                        'kept regardless of success). Costs more: every episode is recorded '
                        'while this is on, and non-in-range/non-successful ones are deleted '
                        'right after.',
        ),
        DeclareLaunchArgument(
            'preview_camera',
            default_value='false',
            description='Open the interactive viewer fixed on the camera named in '
                        'preview_camera_names (first name wins, no split-view for '
                        "multiple names; empty/'all' uses the default free camera), "
                        'replacing the normal teleop/rl render loop -- no episodes are '
                        'recorded or stepped via /step_action while this is on.',
        ),
        DeclareLaunchArgument(
            'preview_camera_names',
            default_value='all',
            description="Camera to preview with preview_camera:=true: 'all'/empty for "
                        'the default free camera, or a comma-separated list of camera '
                        'names (only the first is used).',
        ),
        DeclareLaunchArgument(
            'custom_cameras_file',
            default_value='',
            description='Path to a YAML file defining extra cameras (see '
                        'config/cameras/example_custom_cameras.yaml). Empty = none. '
                        'Only supported for task=KitchenLift.',
        ),
        DeclareLaunchArgument(
            'env_seed',
            default_value='-1',
            description='RoboCasa Kitchen(seed=...) for reproducible object placement and '
                        'robot start pose/facing. -1 (default) = unseeded (system entropy, '
                        'varies every reset). A fixed seed reproduces the same starting '
                        'configuration on every fresh launch of this node, but env.rng still '
                        'advances across resets within one running session, so only the '
                        "first episode after each launch is guaranteed to match -- restart "
                        'with the same seed for an identical repeat.',
        ),
        Node(
            package='emdb_simulator',
            executable='scene_loader',
            name='robocasa_rollout_node',
            output='screen',
            parameters=[{
                'task': 'KitchenLift',
                'control_mode': control_mode,
                'headless': ParameterValue(headless, value_type=bool),
                'perception_mode': perception_mode,
                'record_video': ParameterValue(record_video, value_type=bool),
                'record_video_dir': record_video_dir,
                'record_video_episodes': record_video_episodes,
                'record_video_camera': record_video_camera,
                'record_video_fps': ParameterValue(record_video_fps, value_type=float),
                'record_video_width': ParameterValue(record_video_width, value_type=int),
                'record_video_height': ParameterValue(record_video_height, value_type=int),
                'record_video_stride': ParameterValue(record_video_stride, value_type=int),
                'record_video_crf': ParameterValue(record_video_crf, value_type=int),
                'record_video_keep_successes': ParameterValue(
                    record_video_keep_successes, value_type=bool
                ),
                'preview_camera': ParameterValue(preview_camera, value_type=bool),
                'preview_camera_names': preview_camera_names,
                'custom_cameras_file': custom_cameras_file,
                'env_seed': ParameterValue(env_seed, value_type=int),
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
