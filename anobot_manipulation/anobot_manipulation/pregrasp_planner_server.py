#!/usr/bin/env python3

import threading
import time
import math
import rclpy

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import DisplayTrajectory
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


PLANNING_GROUP = "ur_arm"
PLANNING_FRAME = "ur_base"
TARGET_FRAME = "workstation_pregrasp"
END_EFFECTOR_LINK = "anobot_grasp_frame"


class PregraspPlannerServer(Node):

    def __init__(self, robot, arm, pilz_lin_parameters):
        # Ignore the launch-level __node remapping for this API node.
        # The remapping remains available to the internal MoveItPy node.
        super().__init__(
            "pregrasp_planner_api",
            use_global_arguments=False,
        )

        self.robot = robot
        self.arm = arm
        self.pilz_lin_parameters = pilz_lin_parameters
        self.planning_lock = threading.Lock()
        self.latest_trajectory = None
        self.latest_plan_time = None
        self.latest_grasp_trajectory = None
        self.latest_grasp_plan_time = None
        self.latest_retreat_trajectory = None
        self.latest_retreat_plan_time = None

        self.declare_parameter(
            "planning_frame",
            PLANNING_FRAME,
        )
        self.declare_parameter(
            "target_frame",
            TARGET_FRAME,
        )
        self.declare_parameter(
            "grasp_target_frame",
            "workstation_grasp",
        )
        self.declare_parameter(
            "end_effector_link",
            END_EFFECTOR_LINK,
        )
        self.declare_parameter(
            "tf_timeout_s",
            5.0,
        )
        self.declare_parameter(
            "scene_sync_delay_s",
            2.0,
        )
        self.declare_parameter(
             "allow_execution",
            False,
        )
        self.declare_parameter(
            "mock_hardware",
            True,
        )
        self.declare_parameter(
            "maximum_plan_age_s",
            30.0,
        )
        self.declare_parameter(
            "maximum_grasp_position_error_m",
            0.003,
        )
        self.declare_parameter(
            "maximum_grasp_orientation_error_deg",
            1.0,
        )
        self.declare_parameter(
            "endpoint_verification_timeout_s",
            5.0,
        )

        self.tf_node = Node(
            "pregrasp_planner_tf_listener",
            use_global_arguments=False,
        )

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.tf_node,
            spin_thread=True,
        )

        display_qos = QoSProfile(depth=1)
        display_qos.reliability = ReliabilityPolicy.RELIABLE
        display_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.display_publisher = self.create_publisher(
            DisplayTrajectory,
            "/display_planned_path",
            display_qos,
        )

        self.plan_service = self.create_service(
            Trigger,
            "/manipulation/plan_pregrasp",
            self.plan_pregrasp_callback,
        )
        self.execute_service = self.create_service(
            Trigger,
            "/manipulation/execute_pregrasp",
            self.execute_pregrasp_callback,
        )
        self.clear_service = self.create_service(
            Trigger,
            "/manipulation/clear_pregrasp_plan",
            self.clear_plan_callback,
        )
        self.plan_grasp_service = self.create_service(
            Trigger,
            "/manipulation/plan_grasp",
            self.plan_grasp_callback,
        )
        self.plan_linear_grasp_service = self.create_service(
            Trigger,
            "/manipulation/plan_grasp_linear",
            self.plan_linear_grasp_callback,
        )
        self.execute_grasp_service = self.create_service(
            Trigger,
            "/manipulation/execute_grasp_linear",
            self.execute_grasp_linear_callback,
        )
        self.clear_grasp_service = self.create_service(
            Trigger,
            "/manipulation/clear_grasp_plan",
            self.clear_grasp_plan_callback,
        )

        self.plan_retreat_service = self.create_service(
            Trigger,
            "/manipulation/plan_retreat_linear",
            self.plan_retreat_linear_callback,
        )

        self.execute_retreat_service = self.create_service(
            Trigger,
            "/manipulation/execute_retreat_linear",
            self.execute_retreat_linear_callback,
        )

        self.clear_retreat_service = self.create_service(
            Trigger,
            "/manipulation/clear_retreat_plan",
            self.clear_retreat_plan_callback,
        )

        scene_delay = float(
            self.get_parameter("scene_sync_delay_s").value
        )

        self.get_logger().info(
            f"Waiting {scene_delay:.1f} seconds for "
            "planning-scene synchronization"
        )
        time.sleep(scene_delay)

        self.get_logger().info(
            "Persistent pre-grasp planner is ready"
        )
        self.get_logger().info(
            "Services:\n"
            "  /manipulation/plan_pregrasp\n"
            "  /manipulation/execute_pregrasp\n"
            "  /manipulation/clear_pregrasp_plan\n"
            "  /manipulation/plan_grasp\n"
            "  /manipulation/plan_grasp_linear\n"
            "  /manipulation/execute_grasp_linear\n"
            "  /manipulation/clear_grasp_plan\n"
            "  /manipulation/plan_retreat_linear\n"
            "  /manipulation/execute_retreat_linear\n"
            "  /manipulation/clear_retreat_plan"
        )
        self.get_logger().warning(
            "PLAN-ONLY MODE: trajectory execution is disabled"
        )

    def lookup_target_pose(self, target_frame):
        planning_frame = self.get_parameter(
            "planning_frame"
        ).value

        timeout_s = float(
            self.get_parameter("tf_timeout_s").value
        )

        try:
            transform = self.tf_buffer.lookup_transform(
                planning_frame,
                target_frame,
                Time(),
                timeout=Duration(seconds=timeout_s),
            )

        except TransformException as error:
            raise RuntimeError(
                f"Cannot transform {planning_frame} -> "
                f"{target_frame}: {error}"
            ) from error

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = planning_frame

        pose.pose.position.x = (
            transform.transform.translation.x
        )
        pose.pose.position.y = (
            transform.transform.translation.y
        )
        pose.pose.position.z = (
            transform.transform.translation.z
        )

        pose.pose.orientation = (
            transform.transform.rotation
        )

        return pose

    def publish_display_trajectory(self, trajectory):
        message = DisplayTrajectory()
        message.model_id = "anobot"
        message.trajectory.append(
            trajectory.get_robot_trajectory_msg()
        )

        self.display_publisher.publish(message)

    def plan_pregrasp_callback(self, request, response):
        del request

        if not self.planning_lock.acquire(blocking=False):
            response.success = False
            response.message = (
                "Planner is already processing another request"
            )
            return response

        try:
            target_frame = self.get_parameter(
                "target_frame"
            ).value

            target_pose = self.lookup_target_pose(
                target_frame
            )

            end_effector_link = self.get_parameter(
                "end_effector_link"
            ).value

            target_frame = self.get_parameter(
                "target_frame"
            ).value

            self.get_logger().info(
                f"Planning {end_effector_link} to "
                f"{target_frame}: "
                f"xyz=({target_pose.pose.position.x:.4f}, "
                f"{target_pose.pose.position.y:.4f}, "
                f"{target_pose.pose.position.z:.4f})"
            )

            self.arm.set_start_state_to_current_state()

            self.arm.set_goal_state(
                pose_stamped_msg=target_pose,
                pose_link=end_effector_link,
            )

            plan_result = self.arm.plan()

            if not plan_result:
                response.success = False
                response.message = (
                    "MoveIt failed to find a pre-grasp plan"
                )

                self.get_logger().error(
                    response.message
                )
                return response

            trajectory_message = (
                plan_result.trajectory
                .get_robot_trajectory_msg()
            )

            point_count = len(
                trajectory_message
                .joint_trajectory
                .points
            )

            self.latest_retreat_trajectory = None
            self.latest_retreat_plan_time = None

            self.latest_trajectory = plan_result.trajectory
            self.latest_plan_time = time.monotonic()
            self.publish_display_trajectory(
                plan_result.trajectory
            )

            response.success = True
            response.message = (
                f"Plan successful: {point_count} "
                "trajectory points. "
                "Trajectory displayed but not executed."
            )

            self.get_logger().info(
                response.message
            )

            return response

        except Exception as error:
            response.success = False
            response.message = str(error)

            self.get_logger().error(
                f"Pre-grasp planning failed: {error}"
            )

            return response

        finally:
            self.planning_lock.release()

    def close(self):
        listener = getattr(
            self,
            "tf_listener",
            None,
        )

        if listener is not None:
            try:
                listener.unregister()
            except Exception as error:
                self.get_logger().debug(
                    f"TF listener cleanup: {error}"
                )

        tf_node = getattr(
            self,
            "tf_node",
            None,
        )

        if tf_node is not None:
            try:
                tf_node.destroy_node()
            except Exception:
                pass

        self.tf_listener = None
        self.tf_node = None

    def verify_frame_alignment(
        self,
        target_frame,
        actual_frame,
    ):
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            actual_frame,
            Time(),
            timeout=Duration(seconds=2.0),
        )

        translation = transform.transform.translation

        position_error = math.sqrt(
            translation.x ** 2
            + translation.y ** 2
            + translation.z ** 2
        )

        rotation = transform.transform.rotation

        quaternion_norm = math.sqrt(
            rotation.x ** 2
            + rotation.y ** 2
            + rotation.z ** 2
            + rotation.w ** 2
        )

        if quaternion_norm < 1.0e-12:
            raise RuntimeError(
                "Invalid quaternion during grasp verification"
            )

        normalized_w = abs(
            rotation.w / quaternion_norm
        )

        normalized_w = min(
            1.0,
            max(-1.0, normalized_w),
        )

        orientation_error_deg = math.degrees(
            2.0 * math.acos(normalized_w)
        )

        return position_error, orientation_error_deg

    def wait_for_frame_alignment(
        self,
        target_frame,
        actual_frame,
        timeout_s,
        maximum_position_error,
        maximum_orientation_error,
    ):
        deadline = time.monotonic() + timeout_s

        last_position_error = float("inf")
        last_orientation_error = float("inf")

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            try:
                (
                    last_position_error,
                    last_orientation_error,
                ) = self.verify_frame_alignment(
                    target_frame,
                    actual_frame,
                )

                self.get_logger().debug(
                    "Endpoint verification sample: "
                    f"{last_position_error * 1000.0:.2f} mm, "
                    f"{last_orientation_error:.3f} deg"
                )

                if (
                    last_position_error
                    <= maximum_position_error
                    and last_orientation_error
                    <= maximum_orientation_error
                ):
                    return (
                        True,
                        last_position_error,
                        last_orientation_error,
                    )

            except Exception as error:
                self.get_logger().debug(
                    f"Endpoint verification waiting for TF: "
                    f"{error}"
                )

            time.sleep(0.1)

        return (
            False,
            last_position_error,
            last_orientation_error,
        )

    def execute_pregrasp_callback(self, request, response):
        del request

        allow_execution = bool(
            self.get_parameter("allow_execution").value
        )

        mock_hardware = bool(
            self.get_parameter("mock_hardware").value
        )

        if not allow_execution:
            response.success = False
            response.message = (
                "Execution is disabled. Set allow_execution:=true "
                "only for an intentional mock-hardware test."
            )
            return response

        if not mock_hardware:
            response.success = False
            response.message = (
                "Execution rejected: this service is currently "
                "restricted to mock hardware."
            )
            return response

        if self.latest_trajectory is None:
            response.success = False
            response.message = (
                "No stored pre-grasp plan. Call "
                "/manipulation/plan_pregrasp first."
            )
            return response

        maximum_age = float(
            self.get_parameter("maximum_plan_age_s").value
        )

        plan_age = (
            time.monotonic() - self.latest_plan_time
        )

        if plan_age > maximum_age:
            self.latest_trajectory = None
            self.latest_plan_time = None

            response.success = False
            response.message = (
                f"Stored plan is stale ({plan_age:.1f} seconds). "
                "Generate a new plan."
            )
            return response

        if not self.planning_lock.acquire(blocking=False):
            response.success = False
            response.message = (
                "Planner is busy with another request."
            )
            return response

        try:
            self.get_logger().warning(
                "Executing stored pre-grasp trajectory on "
                "MOCK HARDWARE"
            )

            result = self.robot.execute(
                self.latest_trajectory,
                controllers=[],
            )

            if result:
                # The pre-grasp plan has been consumed.
                self.latest_trajectory = None
                self.latest_plan_time = None

                # Any previously planned final approach is now invalid,
                # because its start state may no longer match the robot.
                self.latest_grasp_trajectory = None
                self.latest_grasp_plan_time = None

                response.success = True
                response.message = (
                    "Mock pre-grasp trajectory executed successfully. "
                    "Any stored linear grasp plan was cleared."
                )

                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = (
                    "Mock trajectory execution failed."
                )

                self.get_logger().error(response.message)

            return response

        except Exception as error:
            response.success = False
            response.message = (
                f"Mock execution failed: {error}"
            )

            self.get_logger().error(response.message)
            return response

        finally:
            self.planning_lock.release()

    def clear_plan_callback(self, request, response):
        del request

        self.latest_trajectory = None
        self.latest_plan_time = None

        response.success = True
        response.message = "Stored pre-grasp plan cleared."

        self.get_logger().info(response.message)

        return response

    def plan_grasp_callback(self, request, response):
        del request

        if not self.planning_lock.acquire(blocking=False):
            response.success = False
            response.message = (
                "Planner is already processing another request."
            )
            return response

        try:
            target_frame = self.get_parameter(
                "grasp_target_frame"
            ).value

            target_pose = self.lookup_target_pose(
                target_frame
            )

            end_effector_link = self.get_parameter(
                "end_effector_link"
            ).value

            self.get_logger().warning(
                "Planning final grasp for visualization only. "
                "This trajectory cannot be executed."
            )

            self.get_logger().info(
                f"Planning {end_effector_link} to "
                f"{target_frame}: "
                f"xyz=({target_pose.pose.position.x:.4f}, "
                f"{target_pose.pose.position.y:.4f}, "
                f"{target_pose.pose.position.z:.4f})"
            )

            self.arm.set_start_state_to_current_state()

            self.arm.set_goal_state(
                pose_stamped_msg=target_pose,
                pose_link=end_effector_link,
            )

            plan_result = self.arm.plan()

            if not plan_result:
                response.success = False
                response.message = (
                    "MoveIt failed to find a plan to the "
                    "final grasp pose."
                )
                self.get_logger().error(response.message)
                return response

            trajectory_message = (
                plan_result.trajectory
                .get_robot_trajectory_msg()
            )

            point_count = len(
                trajectory_message
                .joint_trajectory
                .points
            )

            self.publish_display_trajectory(
                plan_result.trajectory
            )

            # Intentionally do not save this trajectory.
            response.success = True
            response.message = (
                f"Grasp plan successful: {point_count} "
                "trajectory points. Displayed only; "
                "execution is disabled."
            )

            self.get_logger().info(response.message)
            return response

        except Exception as error:
            response.success = False
            response.message = str(error)

            self.get_logger().error(
                f"Grasp planning failed: {error}"
            )
            return response

        finally:
            self.planning_lock.release()

    def plan_linear_grasp_callback(self, request, response):
        del request

        if not self.planning_lock.acquire(blocking=False):
            response.success = False
            response.message = "Planner is busy."
            return response

        try:
            grasp_frame = self.get_parameter(
                "grasp_target_frame"
            ).value

            target_pose = self.lookup_target_pose(
                grasp_frame
            )

            end_effector_link = self.get_parameter(
                "end_effector_link"
            ).value

            self.get_logger().warning(
                "Planning Pilz LIN final approach for "
                "visualization only. Execution is disabled."
            )

            self.get_logger().info(
                "Waiting for current state synchronization before LIN planning"
            )
            time.sleep(1.0)

            self.arm.set_start_state_to_current_state()

            self.arm.set_goal_state(
                pose_stamped_msg=target_pose,
                pose_link=end_effector_link,
            )

            plan_result = self.arm.plan(
                single_plan_parameters=(
                    self.pilz_lin_parameters
                )
            )

            if not plan_result:
                response.success = False
                response.message = (
                    "Pilz LIN failed to plan the final approach."
                )
                return response

            trajectory_message = (
                plan_result.trajectory
                .get_robot_trajectory_msg()
            )

            last_point = trajectory_message.joint_trajectory.points[-1]

            self.get_logger().info(
                "Stored LIN endpoint joints: "
                + ", ".join(
                    f"{name}={position:.6f}"
                    for name, position in zip(
                        trajectory_message.joint_trajectory.joint_names,
                        last_point.positions,
                    )
                )
            )

            point_count = len(
                trajectory_message.joint_trajectory.points
            )

            self.latest_grasp_trajectory = (
                plan_result.trajectory
            )
            self.latest_grasp_plan_time = time.monotonic()

            self.publish_display_trajectory(
                plan_result.trajectory
            )

            response.success = True
            response.message = (
                f"Pilz LIN grasp plan successful: "
                f"{point_count} trajectory points. "
                "Stored and displayed; not executed."
            )

            self.get_logger().info(response.message)
            return response

        except Exception as error:
            response.success = False
            response.message = str(error)

            self.get_logger().error(
                f"Linear grasp planning failed: {error}"
            )
            return response

        finally:
            self.planning_lock.release()

    def execute_grasp_linear_callback(
        self,
        request,
        response,
    ):
        del request

        if not bool(
            self.get_parameter("allow_execution").value
        ):
            response.success = False
            response.message = "Execution is disabled."
            return response

        if not bool(
            self.get_parameter("mock_hardware").value
        ):
            response.success = False
            response.message = (
                "Linear grasp execution is restricted "
                "to mock hardware."
            )
            return response

        if self.latest_grasp_trajectory is None:
            response.success = False
            response.message = (
                "No stored linear grasp plan. Call "
                "/manipulation/plan_grasp_linear first."
            )
            return response

        maximum_age = float(
            self.get_parameter("maximum_plan_age_s").value
        )

        plan_age = (
            time.monotonic()
            - self.latest_grasp_plan_time
        )

        if plan_age > maximum_age:
            self.latest_grasp_trajectory = None
            self.latest_grasp_plan_time = None

            response.success = False
            response.message = (
                f"Stored grasp plan is stale "
                f"({plan_age:.1f} seconds)."
            )
            return response

        if not self.planning_lock.acquire(
            blocking=False
        ):
            response.success = False
            response.message = "Planner is busy."
            return response

        try:
            self.get_logger().warning(
                "Executing Pilz LIN grasp trajectory "
                "on MOCK HARDWARE"
            )

            result = self.robot.execute(
                self.latest_grasp_trajectory,
                controllers=[],
            )

            # The stored trajectory must not be reused,
            # regardless of the execution result.
            self.latest_grasp_trajectory = None
            self.latest_grasp_plan_time = None

            if not result:
                self.latest_retreat_trajectory = None
                self.latest_retreat_plan_time = None

                response.success = False
                response.message = (
                    "Mock linear grasp execution failed."
                )
                self.get_logger().error(response.message)
                return response

            maximum_position_error = float(
                self.get_parameter(
                    "maximum_grasp_position_error_m"
                ).value
            )

            maximum_orientation_error = float(
                self.get_parameter(
                    "maximum_grasp_orientation_error_deg"
                ).value
            )

            verification_timeout = float(
                self.get_parameter(
                    "endpoint_verification_timeout_s"
                ).value
            )

            self.get_logger().info(
                "Waiting for executed trajectory state "
                "to propagate through joint states and TF"
            )

            (
                alignment_ok,
                position_error,
                orientation_error,
            ) = self.wait_for_frame_alignment(
                target_frame=self.get_parameter(
                    "grasp_target_frame"
                ).value,
                actual_frame=self.get_parameter(
                    "end_effector_link"
                ).value,
                timeout_s=verification_timeout,
                maximum_position_error=maximum_position_error,
                maximum_orientation_error=maximum_orientation_error,
            )

            self.get_logger().info(
                "Grasp endpoint error: "
                f"translation={position_error * 1000.0:.2f} mm, "
                f"rotation={orientation_error:.3f} deg"
            )

            if not alignment_ok:
                response.success = False
                response.message = (
                    "Trajectory executed, but endpoint "
                    "verification failed: "
                    f"{position_error * 1000.0:.2f} mm, "
                    f"{orientation_error:.3f} deg."
                )

                self.get_logger().error(response.message)
                return response

            self.latest_retreat_trajectory = None
            self.latest_retreat_plan_time = None
            response.success = True
            response.message = (
                "Mock linear grasp trajectory executed "
                "successfully. "
                f"Endpoint error: "
                f"{position_error * 1000.0:.2f} mm, "
                f"{orientation_error:.3f} deg."
            )

            self.get_logger().info(response.message)
            return response

        except Exception as error:
            # Prevent reuse if an exception happened during or
            # after execution.
            self.latest_grasp_trajectory = None
            self.latest_grasp_plan_time = None

            self.latest_retreat_trajectory = None
            self.latest_retreat_plan_time = None

            response.success = False
            response.message = (
                f"Mock linear execution failed: {error}"
            )

            self.get_logger().error(response.message)
            return response

        finally:
            self.planning_lock.release()

    def clear_grasp_plan_callback(
        self,
        request,
        response,
    ):
        del request

        self.latest_grasp_trajectory = None
        self.latest_grasp_plan_time = None

        response.success = True
        response.message = (
            "Stored linear grasp plan cleared."
        )

        self.get_logger().info(response.message)
        return response

    def plan_retreat_linear_callback(
        self,
        request,
        response,
    ):
        del request

        if not self.planning_lock.acquire(
            blocking=False
        ):
            response.success = False
            response.message = "Planner is busy."
            return response

        try:
            # Retreat destination is the original pre-grasp frame.
            retreat_frame = self.get_parameter(
                "target_frame"
            ).value

            target_pose = self.lookup_target_pose(
                retreat_frame
            )

            end_effector_link = self.get_parameter(
                "end_effector_link"
            ).value

            self.get_logger().warning(
                "Planning Pilz LIN retreat for "
                "visualization only."
            )

            self.get_logger().info(
                f"Planning linear retreat of "
                f"{end_effector_link} to {retreat_frame}: "
                f"xyz=({target_pose.pose.position.x:.4f}, "
                f"{target_pose.pose.position.y:.4f}, "
                f"{target_pose.pose.position.z:.4f})"
            )

            self.get_logger().info(
                "Waiting for current state synchronization "
                "before LIN retreat planning"
            )

            time.sleep(1.0)

            self.arm.set_start_state_to_current_state()

            self.arm.set_goal_state(
                pose_stamped_msg=target_pose,
                pose_link=end_effector_link,
            )

            plan_result = self.arm.plan(
                single_plan_parameters=(
                    self.pilz_lin_parameters
                )
            )

            if not plan_result:
                self.latest_retreat_trajectory = None
                self.latest_retreat_plan_time = None

                response.success = False
                response.message = (
                    "Pilz LIN failed to plan the retreat."
                )

                self.get_logger().error(
                    response.message
                )
                return response

            trajectory_message = (
                plan_result.trajectory
                .get_robot_trajectory_msg()
            )

            point_count = len(
                trajectory_message
                .joint_trajectory
                .points
            )

            last_point = (
                trajectory_message
                .joint_trajectory
                .points[-1]
            )

            self.get_logger().info(
                "Stored retreat endpoint joints: "
                + ", ".join(
                    f"{name}={position:.6f}"
                    for name, position in zip(
                        trajectory_message
                        .joint_trajectory
                        .joint_names,
                        last_point.positions,
                    )
                )
            )

            self.latest_retreat_trajectory = (
                plan_result.trajectory
            )
            self.latest_retreat_plan_time = (
                time.monotonic()
            )

            self.publish_display_trajectory(
                plan_result.trajectory
            )

            response.success = True
            response.message = (
                f"Pilz LIN retreat plan successful: "
                f"{point_count} trajectory points. "
                "Stored and displayed; not executed."
            )

            self.get_logger().info(
                response.message
            )

            return response

        except Exception as error:
            self.latest_retreat_trajectory = None
            self.latest_retreat_plan_time = None

            response.success = False
            response.message = (
                f"Linear retreat planning failed: {error}"
            )

            self.get_logger().error(
                response.message
            )

            return response

        finally:
            self.planning_lock.release()

    def execute_retreat_linear_callback(
        self,
        request,
        response,
    ):
        del request

        if not bool(
            self.get_parameter("allow_execution").value
        ):
            response.success = False
            response.message = "Execution is disabled."
            return response

        if not bool(
            self.get_parameter("mock_hardware").value
        ):
            response.success = False
            response.message = (
                "Linear retreat execution is restricted "
                "to mock hardware."
            )
            return response

        if self.latest_retreat_trajectory is None:
            response.success = False
            response.message = (
                "No stored linear retreat plan. Call "
                "/manipulation/plan_retreat_linear first."
            )
            return response

        maximum_age = float(
            self.get_parameter(
                "maximum_plan_age_s"
            ).value
        )

        plan_age = (
            time.monotonic()
            - self.latest_retreat_plan_time
        )

        if plan_age > maximum_age:
            self.latest_retreat_trajectory = None
            self.latest_retreat_plan_time = None

            response.success = False
            response.message = (
                f"Stored retreat plan is stale "
                f"({plan_age:.1f} seconds)."
            )

            return response

        if not self.planning_lock.acquire(
            blocking=False
        ):
            response.success = False
            response.message = "Planner is busy."
            return response

        try:
            self.get_logger().warning(
                "Executing Pilz LIN retreat trajectory "
                "on MOCK HARDWARE"
            )

            result = self.robot.execute(
                self.latest_retreat_trajectory,
                controllers=[],
            )

            # Never reuse an executed trajectory.
            self.latest_retreat_trajectory = None
            self.latest_retreat_plan_time = None

            if not result:
                response.success = False
                response.message = (
                    "Mock linear retreat execution failed."
                )

                self.get_logger().error(
                    response.message
                )

                return response

            maximum_position_error = float(
                self.get_parameter(
                    "maximum_grasp_position_error_m"
                ).value
            )

            maximum_orientation_error = float(
                self.get_parameter(
                    "maximum_grasp_orientation_error_deg"
                ).value
            )

            verification_timeout = float(
                self.get_parameter(
                    "endpoint_verification_timeout_s"
                ).value
            )

            retreat_frame = self.get_parameter(
                "target_frame"
            ).value

            end_effector_link = self.get_parameter(
                "end_effector_link"
            ).value

            self.get_logger().info(
                "Waiting for retreat state to propagate "
                "through joint states and TF"
            )

            (
                alignment_ok,
                position_error,
                orientation_error,
            ) = self.wait_for_frame_alignment(
                target_frame=retreat_frame,
                actual_frame=end_effector_link,
                timeout_s=verification_timeout,
                maximum_position_error=(
                    maximum_position_error
                ),
                maximum_orientation_error=(
                    maximum_orientation_error
                ),
            )

            self.get_logger().info(
                "Retreat endpoint error: "
                f"translation="
                f"{position_error * 1000.0:.2f} mm, "
                f"rotation="
                f"{orientation_error:.3f} deg"
            )

            if not alignment_ok:
                response.success = False
                response.message = (
                    "Retreat executed, but endpoint "
                    "verification failed after "
                    f"{verification_timeout:.1f} seconds: "
                    f"{position_error * 1000.0:.2f} mm, "
                    f"{orientation_error:.3f} deg."
                )

                self.get_logger().error(
                    response.message
                )

                return response

            response.success = True
            response.message = (
                "Mock linear retreat trajectory executed "
                "successfully. "
                f"Endpoint error: "
                f"{position_error * 1000.0:.2f} mm, "
                f"{orientation_error:.3f} deg."
            )

            self.get_logger().info(
                response.message
            )

            return response

        except Exception as error:
            self.latest_retreat_trajectory = None
            self.latest_retreat_plan_time = None

            response.success = False
            response.message = (
                f"Mock linear retreat failed: {error}"
            )

            self.get_logger().error(
                response.message
            )

            return response

        finally:
            self.planning_lock.release()

    def clear_retreat_plan_callback(
        self,
        request,
        response,
    ):
        del request

        self.latest_retreat_trajectory = None
        self.latest_retreat_plan_time = None

        response.success = True
        response.message = (
            "Stored linear retreat plan cleared."
        )

        self.get_logger().info(
            response.message
        )

        return response


def main(args=None):
    rclpy.init(args=args)

    # This name must match the launch node name so MoveIt parameters
    # generated by MoveItConfigsBuilder are available.
    robot = MoveItPy(
        node_name="plan_to_pregrasp",
    )

    arm = robot.get_planning_component(
        PLANNING_GROUP
    )

    pilz_lin_parameters = PlanRequestParameters(
        robot,
        "pilz_lin",
    )

    node = PregraspPlannerServer(
        robot,
        arm,
        pilz_lin_parameters,
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.close()
        node.destroy_node()

        # MoveItPy remains alive for the whole service lifetime.
        arm = None
        robot = None

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()