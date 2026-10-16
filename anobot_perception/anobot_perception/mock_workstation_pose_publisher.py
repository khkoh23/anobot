#!/usr/bin/env python3

import rclpy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


class MockWorkstationPosePublisher(Node):

    def __init__(self):
        super().__init__("mock_workstation_pose_publisher")
        self.declare_parameter("base_frame", "ur_base",)
        self.declare_parameter("marker_frame", "workstation_marker_26_calibrated",)
        self.declare_parameter("output_topic", "/workstation_marker/pose_base",)
        self.declare_parameter("publish_rate_hz", 15.0,)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.marker_frame = str(self.get_parameter("marker_frame").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self.publisher = self.create_publisher(PoseStamped, self.output_topic, qos_profile_sensor_data,)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False,)
        self.transform_ready_logged = False
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_pose,)
        self.get_logger().info("Mock workstation pose publisher ready")
        self.get_logger().info(f"  TF input: {self.base_frame} -> " f"{self.marker_frame}")
        self.get_logger().info(f"  Pose output: {self.output_topic}")

    def publish_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.marker_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            # Expected during startup before the static
            # workstation transform has been received.
            return
        message = PoseStamped()
        message.header.stamp = (self.get_clock().now().to_msg())
        message.header.frame_id = self.base_frame
        message.pose.position.x = (transform.transform.translation.x)
        message.pose.position.y = (transform.transform.translation.y)
        message.pose.position.z = (transform.transform.translation.z)
        message.pose.orientation = (transform.transform.rotation)
        self.publisher.publish(message)
        if not self.transform_ready_logged:
            self.transform_ready_logged = True
            self.get_logger().info("Publishing fresh mock calibrated " "workstation poses")


def main(args=None):
    rclpy.init(args=args)
    node = MockWorkstationPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()