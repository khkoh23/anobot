from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory(
            "anobot_perception"
        ),
        "config",
        "workstation_acquisition.yaml",
    )

    acquisition_server = Node(
        package="anobot_perception",
        executable=("workstation_acquisition_server"),
        name=("workstation_acquisition_server"),
        output="screen",
        parameters=[config_file,],
    )

    return LaunchDescription([
        acquisition_server,
    ])