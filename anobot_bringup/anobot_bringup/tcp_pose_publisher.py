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


class TcpPosePublisher(Node):

    def __init__(self):
        super().__init__("tcp_pose_broadcaster")
        self.declare_parameter("base_frame", "ur_base",)
        self.declare_parameter("tcp_frame", "ur_tool0",)
        self.declare_parameter("output_topic", "/tcp_pose_broadcaster/pose",)
        self.declare_parameter("publish_rate_hz", 50.0, )
        self.declare_parameter("tf_timeout_s", 0.05,)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tcp_frame = str(self.get_parameter("tcp_frame").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self.tf_timeout_s = float(self.get_parameter("tf_timeout_s").value)
        self.publisher = self.create_publisher(PoseStamped, self.output_topic, qos_profile_sensor_data,)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0), )
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False,)
        self.last_warning_time_ns = 0
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_callback,)
        self.get_logger().info("TCP pose publisher is ready")
        self.get_logger().info(f"  TF: {self.base_frame} " f"-> {self.tcp_frame}")
        self.get_logger().info(f"  Topic: {self.output_topic}")
        self.get_logger().info(f"  Rate: {publish_rate_hz:.1f} Hz")

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, 
                self.tcp_frame, 
                Time(), 
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except TransformException as error:
            self.throttled_warning("Waiting for TCP transform " f"{self.base_frame} -> " f"{self.tcp_frame}: {error}")
            return
        message = PoseStamped()
        # Use the TF sample timestamp instead of creating a
        # new timestamp. The hand-eye publisher synchronizes
        # this message with the ArUco PoseStamped message.
        message.header.stamp = (transform.header.stamp)
        message.header.frame_id = (self.base_frame)
        message.pose.position.x = (transform.transform.translation.x)
        message.pose.position.y = (transform.transform.translation.y)
        message.pose.position.z = (transform.transform.translation.z)
        message.pose.orientation = (transform.transform.rotation)
        self.publisher.publish(message)

    def throttled_warning(self, message):
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self.last_warning_time_ns < 5_000_000_000):
            return
        self.last_warning_time_ns = now_ns
        self.get_logger().warning(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TcpPosePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()