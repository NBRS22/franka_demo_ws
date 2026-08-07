from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CONDA_BASE = '/home/ngr/miniconda3'
SAM3_ENV = 'SAM3'
SAM3_DIR = '/home/ngr/Documents/FP3/SAM3'
GRASPGEN_ENV = 'GraspGen'
GRASPGEN_DIR = '/home/ngr/Documents/FP3/GraspGen'
MAX_SAM3_ATTEMPTS = 3
MAX_GRASPGEN_ATTEMPTS = 3


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description=(
            'true: source caméra = Isaac Sim (pas de driver Realsense, depth '
            '32FC1 en mètres). false (défaut): vraie D455 (driver Realsense '
            'lancé, depth 16UC1 en millimètres).'
        )
    )
    use_sim = LaunchConfiguration('use_sim')

    pipeline_actions = [

        # Realsense D455 — uniquement en mode réel. En mode sim, Isaac Sim
        # publie déjà sur les mêmes topics (/camera/camera/...).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ]),
            launch_arguments={
                'align_depth.enable': 'true'
            }.items(),
            condition=UnlessCondition(use_sim),
        ),

        Node(
            package='gemini_er_bridge',
            executable='camera_bridge_node',
            name='camera_bridge',
            output='screen'
        ),

        Node(
            package='flow_manager',
            executable='camera_buffer_node',
            name='camera_buffer',
            output='screen'
        ),

        Node(
            package='flow_manager',
            executable='task_validator_node',
            name='task_validator',
            output='screen'
        ),

        Node(
            package='flow_manager',
            executable='pointcloud_node',
            name='pointcloud_node',
            output='screen',
            parameters=[{'use_sim': use_sim}],
        ),

        Node(
            package='flow_manager',
            executable='grasp_selector_node',
            name='grasp_selector',
            output='screen'
        ),

        Node(
            package='flow_manager',
            executable='flow_manager_node',
            name='flow_manager',
            output='screen'
        ),

        Node(
            package='sam3_bridge',
            executable='sam3_bridge_node',
            name='sam3_bridge',
            output='screen'
        ),

        Node(
            package='graspgen_bridge',
            executable='graspgen_bridge_node',
            name='graspgen_bridge',
            output='screen'
        ),

        Node(
            package='gemini_er_bridge',
            executable='command_bridge_node',
            name='command_bridge',
            output='screen'
        ),

    ]

    # Both SAM3 and GraspGen must be ready before the pipeline starts; each
    # check finishes independently and in any order, so track readiness here
    # instead of unconditionally returning pipeline_actions from either one
    # (which would launch it twice if both succeed).
    ready = {'sam3': False, 'graspgen': False, 'launched': False}

    def launch_pipeline_once_ready():
        if ready['launched'] or not (ready['sam3'] and ready['graspgen']):
            return []
        ready['launched'] = True
        return pipeline_actions

    # SAM3 management lives entirely here, not in sam3_bridge_node: launch the
    # server, wait for it (health-check retries handle the GPU model load
    # time), and if it's still unreachable after MAX_SAM3_ATTEMPTS full
    # launch+wait cycles, give up and take the whole launcher down. A crash
    # after the pipeline is already running re-enters this same cycle instead
    # of shutting down immediately.
    def launch_sam3(attempt):
        server = ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'source {CONDA_BASE}/etc/profile.d/conda.sh && '
                f'conda activate {SAM3_ENV} && '
                f'cd {SAM3_DIR} && '
                'python -m sam3_server'
            ],
            name='sam3_server',
            output='screen'
        )
        waiter = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'flow_manager', 'check_prerequisites',
                '--only', 'sam3', '--max-retries', '60', '--retry-delay', '5'
            ],
            name='wait_for_sam3',
            output='screen'
        )

        def on_wait_exit(event, context):
            if event.returncode == 0:
                ready['sam3'] = True
                return launch_pipeline_once_ready()
            if attempt >= MAX_SAM3_ATTEMPTS:
                return [
                    LogInfo(msg=(
                        f'SAM3 server still unreachable after {attempt} attempt(s) '
                        '— aborting launch.'
                    )),
                    Shutdown(reason='SAM3 server unreachable'),
                ]
            return [
                LogInfo(msg=(
                    f'SAM3 server unreachable (attempt {attempt}/{MAX_SAM3_ATTEMPTS}), '
                    'restarting it...'
                )),
                *launch_sam3(attempt + 1),
            ]

        def on_server_exit(event, context):
            if event.returncode < 0:
                return  # killed by a signal (e.g. Ctrl+C) — normal shutdown
            if not ready['launched']:
                return  # startup race, on_wait_exit above is the source of truth
            return [
                LogInfo(msg=f'sam3_server crashed (code={event.returncode}) — restarting it...'),
                *launch_sam3(1),
            ]

        return [
            server,
            waiter,
            RegisterEventHandler(OnProcessExit(target_action=server, on_exit=on_server_exit)),
            RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=on_wait_exit)),
        ]

    # Same pattern as launch_sam3 above.
    def launch_graspgen(attempt):
        server = ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'source {CONDA_BASE}/etc/profile.d/conda.sh && '
                f'conda activate {GRASPGEN_ENV} && '
                f'cd {GRASPGEN_DIR} && '
                'python client-server/graspgen_server.py'
            ],
            name='graspgen_server',
            output='screen'
        )
        waiter = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'flow_manager', 'check_prerequisites',
                '--only', 'graspgen', '--max-retries', '60', '--retry-delay', '5'
            ],
            name='wait_for_graspgen',
            output='screen'
        )

        def on_wait_exit(event, context):
            if event.returncode == 0:
                ready['graspgen'] = True
                return launch_pipeline_once_ready()
            if attempt >= MAX_GRASPGEN_ATTEMPTS:
                return [
                    LogInfo(msg=(
                        f'GraspGen server still unreachable after {attempt} attempt(s) '
                        '— aborting launch.'
                    )),
                    Shutdown(reason='GraspGen server unreachable'),
                ]
            return [
                LogInfo(msg=(
                    f'GraspGen server unreachable (attempt {attempt}/{MAX_GRASPGEN_ATTEMPTS}), '
                    'restarting it...'
                )),
                *launch_graspgen(attempt + 1),
            ]

        def on_server_exit(event, context):
            if event.returncode < 0:
                return  # killed by a signal (e.g. Ctrl+C) — normal shutdown
            if not ready['launched']:
                return  # startup race, on_wait_exit above is the source of truth
            return [
                LogInfo(msg=f'graspgen_server crashed (code={event.returncode}) — restarting it...'),
                *launch_graspgen(1),
            ]

        return [
            server,
            waiter,
            RegisterEventHandler(OnProcessExit(target_action=server, on_exit=on_server_exit)),
            RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=on_wait_exit)),
        ]

    return LaunchDescription([
        use_sim_arg,
        *launch_sam3(1),
        *launch_graspgen(1),
    ])
