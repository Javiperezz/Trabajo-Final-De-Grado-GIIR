
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription , ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ORANGE_USER = "jpertru"
ORANGE_HOST = "192.168.0.110"     
ORANGE_DIR  = "/home/jpertru/chappie-voice"
ORANGE_VENV = f"{ORANGE_DIR}/venv/bin/activate"
ORANGE_CMD  = f"cd {ORANGE_DIR} && source {ORANGE_VENV} && python3 -u chappie.py2>&1 | tee /tmp/chappie_voice.log"

def _include(pkg: str, launch_file: str) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(pkg),
                'launch',
                launch_file,
            ])
        )
    )

def _remote_ssh(user: str, host: str, cmd: str) -> ExecuteProcess:

    remote_script = (
        "source /opt/ros/humble/setup.bash 2>/dev/null || true; "
        "source ~/chappie_ws/install/setup.bash 2>/dev/null || true; "
        f"{cmd}"
    )
    return ExecuteProcess(
        cmd=[
            "ssh", "-t",
            "-o", "StrictHostKeyChecking=accept-new",
            "-o", "ServerAliveInterval=15",
            f"{user}@{host}",
            f"bash -lc '{remote_script}'",
        ],
        name="orange_pi_voice_remote",
        output="screen",
    )




def generate_launch_description():
    return LaunchDescription([
        _include('view_robot_pkg',  'chappie_perception.launch.py'),
        _include('chappie_control', 'cascade_balance.launch.py'),
        _include('chappie_control', 'obstacle_guard.launch.py'),
        _include('chappie_control', 'joy_cmd_vel.launch.py'),
        _remote_ssh (ORANGE_USER, ORANGE_HOST , ORANGE_CMD),
    ])
