import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory(
            "anobot_manipulation"
        ),
        "config",
        "mock_pick_cycle.yaml",
    )

    cycle_server = Node(
        package="anobot_manipulation",
        executable="mock_pick_cycle_server",
        name="mock_pick_cycle_server",
        output="screen",
        parameters=[
            config_file,
        ],
    )

    return LaunchDescription([
        cycle_server,
    ])