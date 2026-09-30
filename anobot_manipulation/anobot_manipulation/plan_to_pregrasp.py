#!/usr/bin/env python3

import gc
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
from tf2_ros import Buffer, TransformException, TransformListener

PLANNING_GROUP = "ur_arm"
PLANNING_FRAME = "ur_base"
TARGET_FRAME = "workstation_pregrasp"
END_EFFECTOR_LINK = "anobot_grasp_frame"


def lookup_target_pose():
    node = Node("anobot_pregrasp_tf_lookup")
    tf_buffer = Buffer()
    tf_listener = TransformListener(
        tf_buffer,
        node,
        spin_thread=True,
    )
    node.get_logger().info(
        f"Waiting for {PLANNING_FRAME} -> {TARGET_FRAME}"
    )
    deadline = time.monotonic() + 20.0
    transform = None
    last_error = None
    while rclpy.ok() and time.monotonic() < deadline:
        try:
            transform = tf_buffer.lookup_transform(
                PLANNING_FRAME,
                TARGET_FRAME,
                Time(),
                timeout=Duration(seconds=0.5),
            )
            break

        except TransformException as error:
            last_error = error
            time.sleep(0.1)

    if transform is None:
        node.destroy_node()
        raise RuntimeError(
            f"Could not obtain transform "
            f"{PLANNING_FRAME} -> {TARGET_FRAME}"
            f"Last TF error: {last_error}"
        )
    target = PoseStamped()
    target.header.stamp = transform.header.stamp
    target.header.frame_id = PLANNING_FRAME
    target.pose.position.x = (transform.transform.translation.x)
    target.pose.position.y = (transform.transform.translation.y)
    target.pose.position.z = (transform.transform.translation.z)
    target.pose.orientation = (transform.transform.rotation)
    node.get_logger().info(
        "Target pose: "
        f"xyz=({target.pose.position.x:.4f}, "
        f"{target.pose.position.y:.4f}, "
        f"{target.pose.position.z:.4f})"
    )
    tf_listener.unregister()
    node.destroy_node()
    return target


def publish_trajectory(trajectory_message):
    node = Node("anobot_pregrasp_trajectory_display")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    publisher = node.create_publisher(
        DisplayTrajectory,
        "/display_planned_path",
        qos,
    )
    message = DisplayTrajectory()
    message.model_id = "anobot"
    message.trajectory.append(trajectory_message)
    node.get_logger().info(
        "Publishing planned trajectory to RViz. "
        "The trajectory will not be executed."
    )
    # Publish repeatedly so RViz has enough time to discover the topic.
    for _ in range(20):
        message.trajectory_start.joint_state.header.stamp = (
            node.get_clock().now().to_msg()
        )
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    robot = None
    arm = None
    try:
        target_pose = lookup_target_pose()
        robot = MoveItPy(node_name="plan_to_pregrasp",)
        arm = robot.get_planning_component(PLANNING_GROUP)
        print("Waiting 2 seconds for planning-scene synchronization...")
        time.sleep(2.0)
        arm.set_start_state_to_current_state()
        arm.set_goal_state(
            pose_stamped_msg=target_pose,
            pose_link=END_EFFECTOR_LINK,
        )
        print(
            f"Planning {END_EFFECTOR_LINK} "
            f"to {TARGET_FRAME}..."
        )
        result = arm.plan()
        if not result:
            raise RuntimeError(
                "MoveIt failed to find a plan to pre-grasp"
            )
        trajectory = (
            result.trajectory.get_robot_trajectory_msg()
        )
        print(
            "Plan successful: "
            f"{len(trajectory.joint_trajectory.points)} "
            "trajectory points"
        )
        print("PLAN ONLY: no trajectory was executed")
        publish_trajectory(trajectory)
        print(
            "Inspect the planned trajectory in RViz. "
            "Execution remains disabled."
        )
    except Exception as error:
        print(f"Plan-only request failed: {error}")
        raise
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()