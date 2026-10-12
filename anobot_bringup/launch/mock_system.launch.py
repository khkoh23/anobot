import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def package_launch(package_name, launch_file):
    return PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory(package_name),
            "launch",
            launch_file,
        )
    )


def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder(
        "anobot",
        package_name="anobot_moveit_config",)
        .to_moveit_configs()
    )
    launch_moveit_rviz = LaunchConfiguration("launch_moveit_rviz")
    launch_mock_cycle_server = LaunchConfiguration("launch_mock_cycle_server")
    launch_ui = LaunchConfiguration("launch_ui")
    use_fake_workstation_marker = LaunchConfiguration("use_fake_workstation_marker")
    launch_real_perception = LaunchConfiguration("launch_real_perception")
    realsense_serial_no = LaunchConfiguration("realsense_serial_no")

    declared_arguments = [
        DeclareLaunchArgument(
            "launch_moveit_rviz",
            default_value="true",
            description=("Launch standalone MoveIt RViz."),
        ),
        DeclareLaunchArgument(
            "launch_mock_cycle_server",
            default_value="false",
            description=("Launch the legacy mock pickup-cycle orchestration service."),
        ),
        DeclareLaunchArgument(
            "launch_ui",
            default_value="false",
            description=("Launch the Anobot Qt operator interface."),
        ),
        DeclareLaunchArgument(
            "use_fake_workstation_marker",
            default_value="true",
            description=("Publish the fixed mock calibrated workstation-marker transform."),
        ),
        DeclareLaunchArgument(
            "launch_real_perception",
            default_value="false",
            description=("Launch RealSense, ArUco and calibrated workstation-marker publishing."),
        ),
        DeclareLaunchArgument(
            "realsense_serial_no",
            default_value="",
            description=("RealSense serial number. Leave empty to use the first available camera."),
        ),
    ]

    ur_bridge = IncludeLaunchDescription(
        package_launch(
            "anobot_ur_bridge",
            "ur_bringup.launch.py",
        ),
        launch_arguments={
            "use_mock_hardware": "true",
            "launch_rviz": "false",
            "activate_joint_controller": "true",
            "initial_joint_controller": (
                "scaled_joint_trajectory_controller"
            ),
        }.items(),
    )

    # The AGV is fixed during manipulation development.
    # This replaces the temporary dummy_odom.py process.
    world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mock_world_to_base_footprint",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--qx", "0.0",
            "--qy", "0.0",
            "--qz", "0.0",
            "--qw", "1.0",
            "--frame-id", "world",
            "--child-frame-id", "base_footprint",
        ],
        output="screen",
    )

    move_group = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                package_launch(
                    "anobot_moveit_config",
                    "move_group.launch.py",
                )
            )
        ],
    )

    rviz = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                package_launch(
                    "anobot_moveit_config",
                    "moveit_rviz.launch.py",
                ),
                condition=IfCondition(launch_moveit_rviz),
            )
        ],
    )

    fake_workstation_marker = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="mock_workstation_marker",
                arguments=[
                    "--x", "-0.233",
                    "--y", "-0.909",
                    "--z", "-0.027",
                    "--qx", "0.503",
                    "--qy", "0.501",
                    "--qz", "0.502",
                    "--qw", "-0.494",
                    "--frame-id", "ur_base",
                    "--child-frame-id",
                    "workstation_marker_26_calibrated",
                ],
                output="screen",
                condition=IfCondition(use_fake_workstation_marker),
            )
        ],
    )

    real_perception = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                package_launch(
                    "anobot_bringup",
                    "real_perception.launch.py",
                ),
                launch_arguments={
                    "serial_no": realsense_serial_no,
                }.items(),
                condition=IfCondition(
                    launch_real_perception
                ),
            )
        ],
    )

    grasp_pose = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="anobot_manipulation",
                executable="workstation_grasp_pose",
                name="workstation_grasp_pose",
                output="screen",
                parameters=[
                    PathJoinSubstitution([
                        FindPackageShare(
                            "anobot_manipulation"
                        ),
                        "config",
                        "workstation_grasp_pose.yaml",
                    ])
                ],
            )
        ],
    )

    # RViz starts before this scene node to avoid the
    # late-discovery problem currently observed.
    workstation_scene = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="anobot_scene",
                executable="workstation_scene",
                name="workstation_scene",
                output="screen",
                parameters=[
                    PathJoinSubstitution([
                        FindPackageShare(
                            "anobot_scene"
                        ),
                        "config",
                        "workstation_scene.yaml",
                    ])
                ],
            )
        ],
    )

    rod_lifecycle = TimerAction(
        period=10.5,
        actions=[
            Node(
                package="anobot_scene",
                executable="rod_lifecycle_server",
                name="rod_lifecycle_server",
                output="screen",
                parameters=[
                    PathJoinSubstitution([
                        FindPackageShare(
                            "anobot_scene"
                        ),
                        "config",
                        "rod_lifecycle.yaml",
                    ])
                ],
            )
        ],
    )

    manipulation = TimerAction(
        period=11.0,
        actions=[
            IncludeLaunchDescription(
                package_launch(
                    "anobot_manipulation",
                    "pregrasp_planner_server.launch.py",
                )
            )
        ],
    )

    task_executor = TimerAction(
        period=14.0,
        actions=[
            IncludeLaunchDescription(
                package_launch(
                    "anobot_bt",
                    "task_executor.launch.py",
                )
            )
        ],
    )

    user_interface = TimerAction(
        period=16.0,
        actions=[
            Node(
                package="anobot_ui",
                executable="anobot_ui",
                name="anobot_ui",
                output="screen",
                condition=IfCondition(launch_ui),
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    {
                        "real_perception_enabled": (
                            ParameterValue(
                                launch_real_perception,
                                value_type=bool,
                            )
                        ),
                        "camera_topic": (
                            # "/camera/camera/color/image_raw"
                            "/aruco_single/result"
                        ),
                    },
                ],
            )
        ],
    )

    mock_pick_cycle = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                package_launch(
                    "anobot_manipulation",
                    "mock_pick_cycle_server.launch.py",
                ),
                condition=IfCondition(launch_mock_cycle_server),
            )
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            ur_bridge,
            world_to_base,
            move_group,
            rviz,
            real_perception,
            fake_workstation_marker,
            grasp_pose,
            workstation_scene,
            rod_lifecycle,
            manipulation,
            task_executor,
            user_interface,
            mock_pick_cycle,
        ]
    )