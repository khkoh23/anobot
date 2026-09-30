from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

import os


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "anobot",
            package_name="anobot_moveit_config",
        )
        .to_moveit_configs()
    )

    moveit_py_config = os.path.join(
        get_package_share_directory(
            "anobot_manipulation"
        ),
        "config",
        "moveit_py.yaml",
    )

    plan_node = Node(
        package="anobot_manipulation",
        executable="plan_to_pregrasp",
        name="plan_to_pregrasp",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_py_config,
        ],
    )

    return LaunchDescription([
        plan_node,
    ])