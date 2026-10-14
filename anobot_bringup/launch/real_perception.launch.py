import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    serial_no = LaunchConfiguration("serial_no")
    calibration_path = LaunchConfiguration("calibration_path")
    marker_id = LaunchConfiguration("marker_id")
    marker_size = LaunchConfiguration("marker_size")
    color_profile = LaunchConfiguration("color_profile")
    maximum_time_difference_ms = LaunchConfiguration("maximum_time_difference_ms")
    launch_tcp_pose_publisher = LaunchConfiguration("launch_tcp_pose_publisher")
    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"),
        "launch",
        "rs_launch.py",
    )
    
    declared_arguments = [
        DeclareLaunchArgument(
            "serial_no",
            default_value="",
            description=(
                "RealSense serial number. Leave empty "
                "to use the first available device."
            ),
        ),
        DeclareLaunchArgument(
            "calibration_path",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("HOME"),
                "anobot_ws",
                "handeye_data",
                "ur10e_d435i_run01",
                "handeye_factory_run01.yaml",
            ]),
            description=(
                "Eye-in-hand calibration YAML."
            ),
        ),
        DeclareLaunchArgument(
            "marker_id",
            default_value="26",
            description="Workstation ArUco marker ID.",
        ),
        DeclareLaunchArgument(
            "marker_size",
            default_value="0.0500",
            description=(
                "Physical ArUco marker size in metres."
            ),
        ),
        DeclareLaunchArgument(
            "color_profile",
            default_value="640x480x15",
            description=(
                "RealSense color stream profile."
            ),
        ),
        DeclareLaunchArgument(
            "maximum_time_difference_ms",
            default_value="30.0",
            description=(
                "Maximum marker/joint-state time "
                "difference accepted by the calibrated "
                "marker publisher."
            ),
        ),
        DeclareLaunchArgument(
            "launch_tcp_pose_publisher",
            default_value="true",
            description=(
                "Launch the custom TF-based TCP pose publisher. "
                "Disable when the physical UR driver's "
                "tcp_pose_broadcaster is active."
            ),
        ),
    ]

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            realsense_launch
        ),
        launch_arguments={
            "camera_name": "camera",
            "camera_namespace": "camera",
            "serial_no": serial_no,
            "base_frame_id": "link",
            "tf_prefix": "",
            "initial_reset": "false",
            "enable_color": "true",
            "enable_depth": "false",
            "enable_infra": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
            "enable_motion": "false",
            "publish_tf": "true",
            "rgb_camera.color_profile": (
                color_profile
            ),
            "output": "screen",
        }.items(),
    )
    aruco = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="aruco_ros",
                executable="single",
                name="aruco_single",
                output="screen",
                parameters=[
                    {
                        "image_is_rectified": True,
                        "marker_id": marker_id,
                        "marker_size": marker_size,
                        "reference_frame": ("camera_color_optical_frame"),
                        "camera_frame": ("camera_color_optical_frame"),
                        "marker_frame": ("workstation_marker"),
                        "corner_refinement": "SUBPIX",
                    }
                ],
                remappings=[
                    ("/image", "/camera/camera/color/image_raw",),
                    ("/camera_info", "/camera/camera/color/camera_info",),
                ],
            )
        ],
    )

    tcp_pose_publisher = Node(
        package="anobot_bringup",
        executable="tcp_pose_publisher",
        name="tcp_pose_broadcaster",
        output="screen",
        condition=IfCondition(launch_tcp_pose_publisher),
        parameters=[
            {
                "base_frame": "ur_base",
                "tcp_frame": "ur_tool0",
                "output_topic": (
                    "/tcp_pose_broadcaster/pose"
                ),
                "publish_rate_hz": 50.0,
                "tf_timeout_s": 0.05,
            }
        ],
    )

    calibrated_marker = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="handeye_utility",
                executable="publish_calibrated_marker_pose",
                name="publish_calibrated_marker_pose",
                output="screen",
                parameters=[
                    {
                        "calibration_path": (calibration_path),
                        "maximum_time_difference_ms": (maximum_time_difference_ms),
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            realsense,
            aruco,
            tcp_pose_publisher,
            calibrated_marker,
        ]
    )