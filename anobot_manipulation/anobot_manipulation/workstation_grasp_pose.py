#!/usr/bin/env python3

import math
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

class WorkstationGraspPose(Node):

  def __init__(self):
    super().__init__("workstation_grasp_pose")
    self.declare_parameter(
      "parent_frame",
      "workstation_marker_26_calibrated",
    )
    self.declare_parameter(
      "grasp_frame",
      "workstation_grasp",
    )
    self.declare_parameter(
      "pregrasp_frame",
      "workstation_pregrasp",
    )
    self.declare_parameter(
      "pregrasp_distance",
      0.10,
    )
    self.declare_parameter("x", 0.0)
    self.declare_parameter("y", 0.0)
    self.declare_parameter("z", 0.0)
    self.declare_parameter("roll_deg", 0.0)
    self.declare_parameter("pitch_deg", 0.0)
    self.declare_parameter("yaw_deg", 0.0)
    self.declare_parameter("arrow_length", 0.15)
    self.declare_parameter("arrow_width", 0.02)
    self.declare_parameter("publish_rate_hz", 10.0)
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    self.pose_publisher = self.create_publisher(
      PoseStamped,
      "/workstation_grasp/pose",
      qos,
    )
    self.marker_publisher = self.create_publisher(
      Marker,
      "/workstation_grasp/marker",
      qos,
    )
    self.tf_broadcaster = TransformBroadcaster(self)
    self.add_on_set_parameters_callback(
      self.parameter_callback
    )
    publish_rate = float(
      self.get_parameter("publish_rate_hz").value
    )
    self.timer = self.create_timer(
      1.0 / publish_rate,
      self.publish_grasp_pose,
    )
    self.get_logger().info(
      "Publishing adjustable workstation grasp pose"
    )

  @staticmethod
  def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw

  def parameter_callback(self, parameters):
    for parameter in parameters:
      if parameter.name in (
        "arrow_length",
        "arrow_width",
        "publish_rate_hz",
      ):
        if float(parameter.value) <= 0.0:
          return SetParametersResult(
            successful=False,
            reason="Marker dimensions and rate must be positive",
          )
    return SetParametersResult(successful=True)

  def make_pose(self):
    pose = PoseStamped()
    pose.header.stamp = self.get_clock().now().to_msg()
    pose.header.frame_id = self.get_parameter(
      "parent_frame"
    ).value
    pose.pose.position.x = float(self.get_parameter("x").value)
    pose.pose.position.y = float(self.get_parameter("y").value)
    pose.pose.position.z = float(self.get_parameter("z").value)
    roll = math.radians(float(self.get_parameter("roll_deg").value))
    pitch = math.radians(float(self.get_parameter("pitch_deg").value))
    yaw = math.radians(float(self.get_parameter("yaw_deg").value))
    qx, qy, qz, qw = self.quaternion_from_rpy(roll, pitch, yaw,)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose

  def publish_grasp_pose(self):
    pose = self.make_pose()
    self.pose_publisher.publish(pose)
    grasp_frame = self.get_parameter(
      "grasp_frame"
    ).value
    transform = TransformStamped()
    transform.header = pose.header
    transform.child_frame_id = grasp_frame
    transform.transform.translation.x = (pose.pose.position.x)
    transform.transform.translation.y = (pose.pose.position.y)
    transform.transform.translation.z = (pose.pose.position.z)
    transform.transform.rotation = (pose.pose.orientation)
    self.tf_broadcaster.sendTransform(transform)
    pregrasp = TransformStamped()
    pregrasp.header.stamp = pose.header.stamp
    pregrasp.header.frame_id = grasp_frame
    pregrasp.child_frame_id = self.get_parameter(
      "pregrasp_frame"
    ).value
    # Final approach is along grasp-frame +Z.
    # Therefore, pre-grasp is 0.10 m along local -Z.
    pregrasp.transform.translation.x = 0.0
    pregrasp.transform.translation.y = 0.0
    pregrasp.transform.translation.z = -float(
      self.get_parameter("pregrasp_distance").value
    )
    pregrasp.transform.rotation.x = 0.0
    pregrasp.transform.rotation.y = 0.0
    pregrasp.transform.rotation.z = 0.0
    pregrasp.transform.rotation.w = 1.0
    self.tf_broadcaster.sendTransform(pregrasp)
    marker = Marker()
    marker.header = pose.header
    marker.ns = "workstation_grasp"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose = pose.pose
    # Marker arrow points along the local +X axis.
    marker.scale.x = float(self.get_parameter("arrow_length").value)
    marker.scale.y = float(self.get_parameter("arrow_width").value)
    marker.scale.z = float(self.get_parameter("arrow_width").value)
    marker.color.r = 1.0
    marker.color.g = 0.1
    marker.color.b = 0.1
    marker.color.a = 0.9
    self.marker_publisher.publish(marker)


def main(args=None):
  rclpy.init(args=args)
  node = WorkstationGraspPose()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()