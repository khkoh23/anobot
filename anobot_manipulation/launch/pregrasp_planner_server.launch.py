import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "anobot",
            package_name="anobot_moveit_config",
        )
        .planning_pipelines(
            pipelines=[
                "ompl",
                "pilz_industrial_motion_planner",
            ]
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

    planner = Node(
        package="anobot_manipulation",
        executable="pregrasp_planner_server",
        name="plan_to_pregrasp",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_py_config,
            {
                "planning_frame": "ur_base",
                "target_frame": "workstation_pregrasp",
                "grasp_target_frame": "workstation_grasp",
                "end_effector_link": "anobot_grasp_frame",
                "tf_timeout_s": 5.0,
                "scene_sync_delay_s": 2.0,
                "allow_execution": False,
                "mock_hardware": True,
                "maximum_plan_age_s": 30.0,
                "endpoint_verification_timeout_s": 5.0,
                "maximum_grasp_position_error_m": 0.003,
                "maximum_grasp_orientation_error_deg": 1.0,
            },
        ],
    )

    return LaunchDescription([
        planner,
    ])