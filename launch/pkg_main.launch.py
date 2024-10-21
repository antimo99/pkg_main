from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    traj_action_server = Node(
        package="pkg_main",
        executable="traj_action_server",
        output="screen",
    )

    task = Node(
        package="pkg_main",
        executable="task",
        output="screen"
    )

    return LaunchDescription([
        traj_action_server,
        task
        ])
