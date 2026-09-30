#!/usr/bin/env python3

import threading
import time

import rclpy

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
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

    def __init__(self, robot, arm):
        # Ignore the launch-level __node remapping for this API node.
        # The remapping remains available to the internal MoveItPy node.
        super().__init__(
            "pregrasp_planner_api",
            use_global_arguments=False,
        )

        self.robot = robot
        self.arm = arm
        self.planning_lock = threading.Lock()

        self.declare_parameter(
            "planning_frame",
            PLANNING_FRAME,
        )
        self.declare_parameter(
            "target_frame",
            TARGET_FRAME,
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
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
            "Service: /manipulation/plan_pregrasp"
        )
        self.get_logger().warning(
            "PLAN-ONLY MODE: trajectory execution is disabled"
        )

    def lookup_target_pose(self):
        planning_frame = self.get_parameter(
            "planning_frame"
        ).value

        target_frame = self.get_parameter(
            "target_frame"
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
            target_pose = self.lookup_target_pose()

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
        try:
            self.tf_listener.unregister()
        except Exception:
            pass


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

    node = PregraspPlannerServer(
        robot,
        arm,
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