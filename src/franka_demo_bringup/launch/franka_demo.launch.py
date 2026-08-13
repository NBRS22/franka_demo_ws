import os

from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# External perception servers (separate conda envs, not part of this colcon workspace).
# cf. CLAUDE.md racine — "Serveurs externes" et "Ports ZMQ".
SAM3_DIR = '/home/ngr/Documents/FP3/SAM3'
SAM3_CONDA_ENV = 'SAM3'
SAM3_HOST = '127.0.0.1'
SAM3_PORT = 5557

GRASPGEN_DIR = '/home/ngr/Documents/FP3/GraspGen'
GRASPGEN_CONDA_ENV = 'GraspGen'
GRASPGEN_HOST = '127.0.0.1'
GRASPGEN_PORT = 5558

HEALTH_CHECK_TIMEOUT_S = '180'

# RealSense D455 stream profile ('WIDTHxHEIGHTxFPS') — color and depth kept identical
# so aligned_depth_to_color stays pixel-indexed the same as the color mask (cf.
# CLAUDE.md racine, "Convention de coordonnées" / create_pointcloud_node). Nothing
# downstream (create_pointcloud_node, sam3_bridge_node, ...) hardcodes a resolution —
# they all read height/width from the message/response itself — so changing this is
# the only place that needs touching to lower the resolution.
REALSENSE_COLOR_PROFILE = '1280x720x30'
REALSENSE_DEPTH_PROFILE = '1280x720x30'


def _conda_run_cmd(env_name, workdir, *command):
    inner = ' '.join(['conda', 'run', '-n', env_name, '--no-capture-output', *command])
    return ['bash', '-c', f'cd {workdir} && exec {inner}']


def generate_launch_description():
    bringup_share = get_package_share_directory('franka_demo_bringup')
    wait_script = os.path.join(bringup_share, 'scripts', 'wait_for_zmq_health.py')

    sam3_server = ExecuteProcess(
        cmd=_conda_run_cmd(SAM3_CONDA_ENV, SAM3_DIR, 'python', '-m', 'sam3_server'),
        name='sam3_server',
        output='screen',
    )

    graspgen_server = ExecuteProcess(
        cmd=_conda_run_cmd(GRASPGEN_CONDA_ENV, GRASPGEN_DIR, 'python', 'client-server/graspgen_server.py'),
        name='graspgen_server',
        output='screen',
    )

    def _wait_for(name, host, port):
        return ExecuteProcess(
            cmd=[
                'python3', wait_script,
                '--name', name,
                '--host', host,
                '--port', str(port),
                '--timeout', HEALTH_CHECK_TIMEOUT_S,
            ],
            name=f'wait_for_{name.lower()}',
            output='screen',
        )

    wait_sam3 = _wait_for('SAM3', SAM3_HOST, SAM3_PORT)
    wait_graspgen = _wait_for('GraspGen', GRASPGEN_HOST, GRASPGEN_PORT)

    realsense = GroupAction(
        scoped=True,
        forwarding=False,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('realsense2_camera'),
                        'launch',
                        'rs_launch.py',
                    )
                ),
                launch_arguments={
                    'align_depth.enable': 'true',
                    'initial_reset': 'true',
                    'log_level': 'warn',
                    'rgb_camera.color_profile': REALSENSE_COLOR_PROFILE,
                    'depth_module.depth_profile': REALSENSE_DEPTH_PROFILE,
                }.items(),
            ),
        ],
    )

    robot_task_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_task_manager'),
                'launch',
                'robot_task_manager.launch.py',
            )
        ),
    )

    def _on_wait_exit(next_actions, failure_reason):
        def _handler(event, context):
            if event.returncode == 0:
                return next_actions
            return [EmitEvent(event=Shutdown(reason=failure_reason))]
        return _handler

    return LaunchDescription([

        sam3_server,
        graspgen_server,
        wait_sam3,

        # If either external server dies at any point (startup crash or later), tear
        # down the whole launch — RealSense, bridges, robot_task_manager included.
        RegisterEventHandler(OnProcessExit(
            target_action=sam3_server,
            on_exit=[EmitEvent(event=Shutdown(reason='SAM3 server exited'))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=graspgen_server,
            on_exit=[EmitEvent(event=Shutdown(reason='GraspGen server exited'))],
        )),

        # Sequential health-check gate: only start RealSense + robot_task_manager
        # once both external servers have answered {'status': 'ok'} on their ZMQ
        # health port. wait_graspgen is only started after wait_sam3 succeeds — the
        # two servers still boot in parallel (started above), so this just serializes
        # the *checks*, not the actual server startup.
        RegisterEventHandler(OnProcessExit(
            target_action=wait_sam3,
            on_exit=_on_wait_exit([wait_graspgen], 'SAM3 server failed health check'),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=wait_graspgen,
            on_exit=_on_wait_exit([realsense, robot_task_manager], 'GraspGen server failed health check'),
        )),

    ])
