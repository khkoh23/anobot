from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # UR-specific arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")

    # Description / hardware arguments
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    headless_mode = LaunchConfiguration("headless_mode")
    kinematics_parameters_file = LaunchConfiguration("kinematics_parameters_file")

    # Controller arguments
    controllers_file = LaunchConfiguration("controllers_file")
    update_rate_config_file = LaunchConfiguration("update_rate_config_file")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")

    # Visualization
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # UR driver helper arguments
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")

    # -------------------------------------------------------------------------
    # Robot description
    # This replaces rsp.launch.py.
    # -------------------------------------------------------------------------
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("anobot_ur_bridge"),
                    "urdf",
                    "anobot_ur_bridge.urdf.xacro",
                ]
            ),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "kinematics_parameters_file:=",
            kinematics_parameters_file,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, 
            value_type=str,
        ),
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # -------------------------------------------------------------------------
    # ros2_control node
    # -------------------------------------------------------------------------
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(controllers_file, allow_substs=True),
        ],
        output="screen",
    )

    # -------------------------------------------------------------------------
    # UR driver helper nodes
    # -------------------------------------------------------------------------
    dashboard_client_node = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(
            AndSubstitution(
                launch_dashboard_client,
                NotSubstitution(use_mock_hardware),
            )
        ),
        parameters=[
            {
                "robot_ip": robot_ip,
            }
        ],
    )

    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        parameters=[
            {
                "headless_mode": headless_mode,
            },
            {
                "robot_ip": robot_ip,
            },
        ],
    )

    tool_communication_node = Node(
        package="ur_robot_driver",
        executable="tool_communication.py",
        name="ur_tool_comm",
        output="screen",
        condition=IfCondition(use_tool_communication),
        parameters=[
            {
                "robot_ip": robot_ip,
                "tcp_port": tool_tcp_port,
                "device_name": tool_device_name,
            }
        ],
    )

    urscript_interface_node = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        parameters=[
            {
                "robot_ip": robot_ip,
            }
        ],
    )

    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_mock_hardware),
        parameters=[
            {
                "headless_mode": headless_mode,
            },
            {
                "joint_controller_active": activate_joint_controller,
            },
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                    "tcp_pose_broadcaster",
                    "ur_configuration_controller",
                ]
            },
        ],
    )

    # -------------------------------------------------------------------------
    # RViz
    # -------------------------------------------------------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=[
            "-d",
            rviz_config_file,
        ],
    )

    # -------------------------------------------------------------------------
    # Custom anobot_ur_bridge node
    # -------------------------------------------------------------------------
    trajectory_until_node = Node(
        package="anobot_ur_bridge",
        executable="trajectory_until_node",
        name="trajectory_until_node",
        output="screen",
        parameters=[
            {
                "motion_controller": initial_joint_controller,
            }
        ],
    )

    # -------------------------------------------------------------------------
    # Controller spawners
    # -------------------------------------------------------------------------
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []

        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
            output="screen",
        )

    controllers_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "tcp_pose_broadcaster",
        "ur_configuration_controller",
    ]

    controllers_inactive = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
        "tool_contact_controller",
    ]

    initial_joint_controller_value = initial_joint_controller.perform(context)

    if activate_joint_controller.perform(context).lower() == "true":
        controllers_active.append(initial_joint_controller_value)

        if initial_joint_controller_value in controllers_inactive:
            controllers_inactive.remove(initial_joint_controller_value)

    if use_mock_hardware.perform(context).lower() == "true":
        mock_unsupported_controllers = [
            "io_and_status_controller",
            "speed_scaling_state_broadcaster",
            "force_torque_sensor_broadcaster",
            "tcp_pose_broadcaster",
            "ur_configuration_controller",
            "force_mode_controller",
            "passthrough_trajectory_controller",
            "freedrive_mode_controller",
            "tool_contact_controller",
        ]
        for controller in mock_unsupported_controllers:
            if controller in controllers_active:
                controllers_active.remove(controller)
            if controller in controllers_inactive:
                controllers_inactive.remove(controller)

    controller_spawners = [
        controller_spawner(controllers_active, active=True),
        controller_spawner(controllers_inactive, active=False),
    ]

    nodes_to_start = [
        control_node,
        dashboard_client_node,
        robot_state_helper_node,
        tool_communication_node,
        controller_stopper_node,
        urscript_interface_node,
        robot_state_publisher_node,
        rviz_node,
        trajectory_until_node,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # -------------------------------------------------------------------------
    # UR-specific arguments
    # -------------------------------------------------------------------------
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur7e",
                "ur10",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",
            description="IP address by which the robot can be reached.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )

    # -------------------------------------------------------------------------
    # Controller/configuration arguments
    # -------------------------------------------------------------------------
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_robot_driver"),
                    "config",
                    "ur_controllers.yaml",
                ]
            ),
            description="YAML file with the controllers configuration.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value=[
                LaunchConfiguration("ur_type"),
                "_",
            ],
            description=(
                "tf_prefix of the joint names, useful for multi-robot setup. "
                "If changed, joint names in the controllers configuration must also be updated. "
                "This launch argument is also available to controller YAML substitutions."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description=(
                "Enable mock command interfaces for sensors used for simple simulations. "
                "Used only if use_mock_hardware is true."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
            ],
            description="Initially loaded robot controller.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("anobot_ur_bridge"),
                    "rviz",
                    "ur_bridge.rviz",
                ]
            ),
            description="RViz config file to use when launching RViz.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client",
            default_value="true",
            description="Launch Dashboard Client.",
        )
    )

    # -------------------------------------------------------------------------
    # Tool communication arguments
    # These are retained from ur_control.launch.py.
    # -------------------------------------------------------------------------
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Only available for e-series robots.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_parity",
            default_value="0",
            description=(
                "Parity configuration for serial communication. "
                "Only effective if use_tool_communication is true."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description=(
                "Baud rate configuration for serial communication. "
                "Only effective if use_tool_communication is true."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description=(
                "Stop bits configuration for serial communication. "
                "Only effective if use_tool_communication is true."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_rx_idle_chars",
            default_value="1.5",
            description=(
                "RX idle chars configuration for serial communication. "
                "Only effective if use_tool_communication is true."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tx_idle_chars",
            default_value="3.5",
            description=(
                "TX idle chars configuration for serial communication. "
                "Only effective if use_tool_communication is true."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description=(
                "File descriptor that will be generated for the tool communication device. "
                "The user must be allowed to write to this location. "
                "Only effective if use_tool_communication is true."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description=(
                "Remote port used for bridging the tool's serial device. "
                "Only effective if use_tool_communication is true."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="0",
            description="Tool voltage that will be set up.",
        )
    )

    # -------------------------------------------------------------------------
    # Network/port arguments retained from ur_control.launch.py
    # -------------------------------------------------------------------------
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="0.0.0.0",
            description=(
                "IP that will be used for the robot controller to communicate back to the driver."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port opened to forward URScript commands to the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description=(
                "Port opened to send cyclic instructions from the driver to the robot controller."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description=(
                "The driver offers an interface to query the external_control URScript on this port."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50003",
            description="Port opened for trajectory control.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "update_rate_config_file",
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur_robot_driver"),
                        "config",
                    ]
                ),
                "/",
                LaunchConfiguration("ur_type"),
                "_update_rate.yaml",
            ],
            description="YAML file containing the controller update rate.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("anobot_ur_bridge"),
                    "config",
                    "default_ur10e_calibration.yaml",
                ]
            ),
            description="UR calibration YAML passed to the robot description xacro.",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )
