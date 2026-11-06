#!/usr/bin/env python3

import cv2
import math
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from pathlib import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class FixedCameraMarkerPose(Node):

    def __init__(self):
        super().__init__("fixed_camera_marker_pose")
        self.declare_parameter("calibration_path", "",)
        self.declare_parameter("base_frame", "ur_base",)
        self.declare_parameter("camera_frame", "body_cam_color_optical_frame",)
        self.declare_parameter("raw_marker_frame", "workstation_marker",)
        self.declare_parameter("calibrated_marker_frame", "workstation_marker_26_calibrated",)
        self.declare_parameter("output_topic", "/workstation_marker/pose_base",)
        self.declare_parameter("lookup_rate_hz", 30.0,)
        self.declare_parameter("tf_timeout_s", 0.05,)
        self.calibration_path = Path(str(self.get_parameter("calibration_path").value)).expanduser()
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.raw_marker_frame = str(self.get_parameter("raw_marker_frame").value)
        self.calibrated_marker_frame = str(self.get_parameter("calibrated_marker_frame").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.lookup_rate_hz = float(self.get_parameter("lookup_rate_hz").value)
        self.tf_timeout_s = float(self.get_parameter("tf_timeout_s").value)
        if not self.calibration_path.is_file():
            raise FileNotFoundError("Eye-to-hand calibration not found: " f"{self.calibration_path}")
        if self.lookup_rate_hz <= 0.0:
            raise ValueError("lookup_rate_hz must be positive")
        self.T_base_camera = (self.load_calibration(self.calibration_path))
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False,)
        self.tf_broadcaster = (TransformBroadcaster(self))
        self.pose_publisher = (self.create_publisher(PoseStamped, self.output_topic, qos_profile_sensor_data,))
        self.last_marker_stamp = None
        self.first_pose_logged = False
        self.timer = self.create_timer(1.0 / self.lookup_rate_hz, self.publish_calibrated_pose,)
        translation = (self.T_base_camera[:3, 3])
        self.get_logger().info("Fixed-camera eye-to-hand marker " "publisher ready")
        self.get_logger().info(f"  Calibration: " f"{self.calibration_path}")
        self.get_logger().info(f"  Fixed transform: "
            f"{self.base_frame} -> "
            f"{self.camera_frame}"
        )
        self.get_logger().info("  Camera translation: "
            f"x={translation[0]:.6f}, "
            f"y={translation[1]:.6f}, "
            f"z={translation[2]:.6f} m"
        )
        self.get_logger().info(f"  Raw marker: "
            f"{self.camera_frame} -> "
            f"{self.raw_marker_frame}"
        )
        self.get_logger().info(f"  Output TF: "
            f"{self.base_frame} -> "
            f"{self.calibrated_marker_frame}"
        )
        self.get_logger().info(f"  Output pose: " f"{self.output_topic}")

    @staticmethod
    def load_calibration(path: Path):
        storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ,)
        if not storage.isOpened():
            raise RuntimeError(f"Cannot open calibration: {path}")
        matrix = storage.getNode("T").mat()
        storage.release()
        if (matrix is None or matrix.shape != (4, 4)):
            raise RuntimeError("Calibration YAML must contain " "a 4x4 matrix named T")
        matrix = np.asarray(matrix, dtype=float,)
        if not np.all(np.isfinite(matrix)):
            raise RuntimeError("Calibration matrix contains " "non-finite values")
        rotation = matrix[:3, :3]
        orthogonality_error = np.linalg.norm(rotation.T @ rotation - np.eye(3))
        determinant = np.linalg.det(rotation)
        if orthogonality_error > 1.0e-3:
            raise RuntimeError("Calibration rotation is not " "orthonormal")
        if abs(determinant - 1.0) > 1.0e-3:
            raise RuntimeError("Calibration rotation determinant " f"is invalid: {determinant}")
        return matrix

    @staticmethod
    def quaternion_to_rotation(x, y, z, w,):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1.0e-12:
            raise ValueError("Invalid zero quaternion")
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        return np.array([
            [
                1.0 - 2.0 * (y*y + z*z),
                2.0 * (x*y - z*w),
                2.0 * (x*z + y*w),
            ],
            [
                2.0 * (x*y + z*w),
                1.0 - 2.0 * (x*x + z*z),
                2.0 * (y*z - x*w),
            ],
            [
                2.0 * (x*z - y*w),
                2.0 * (y*z + x*w),
                1.0 - 2.0 * (x*x + y*y),
            ],
        ])

    @staticmethod
    def rotation_to_quaternion(rotation):
        trace = float(np.trace(rotation))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * scale
            qx = (rotation[2, 1] - rotation[1, 2]) / scale
            qy = (rotation[0, 2] - rotation[2, 0]) / scale
            qz = (rotation[1, 0] - rotation[0, 1]) / scale
        elif (rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]):
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            qw = (rotation[2, 1] - rotation[1, 2]) / scale
            qx = 0.25 * scale
            qy = (rotation[0, 1] + rotation[1, 0]) / scale
            qz = (rotation[0, 2] + rotation[2, 0]) / scale
        elif (rotation[1, 1] > rotation[2, 2]):
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            qw = (rotation[0, 2] - rotation[2, 0]) / scale
            qx = (rotation[0, 1] + rotation[1, 0]) / scale
            qy = 0.25 * scale
            qz = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            qw = (rotation[1, 0] - rotation[0, 1]) / scale
            qx = (rotation[0, 2] + rotation[2, 0]) / scale
            qy = (rotation[1, 2] + rotation[2, 1]) / scale
            qz = 0.25 * scale
        quaternion = np.array([qx, qy, qz, qw,])
        quaternion /= np.linalg.norm(quaternion)
        return quaternion

    @classmethod
    def transform_to_matrix(cls, transform,):
        translation = (transform.transform.translation)
        rotation = (transform.transform.rotation)
        matrix = np.eye(4)
        matrix[:3, :3] = (cls.quaternion_to_rotation(rotation.x, rotation.y, rotation.z, rotation.w,))
        matrix[:3, 3] = [translation.x, translation.y, translation.z,]
        return matrix

    def publish_calibrated_pose(self):
        try:
            raw_transform = (
                self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.raw_marker_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout_s),
                )
            )
        except TransformException:
            return
        marker_stamp = (raw_transform.header.stamp.sec, raw_transform.header.stamp.nanosec,)
        # Do not continuously republish a stale marker.
        # The acquisition server should see new messages
        # only when the detector produced a new pose.
        if marker_stamp == self.last_marker_stamp:
            return
        self.last_marker_stamp = marker_stamp
        T_camera_marker = (self.transform_to_matrix(raw_transform))
        T_base_marker = (self.T_base_camera @ T_camera_marker)
        quaternion = (self.rotation_to_quaternion(T_base_marker[:3, :3]))
        pose = PoseStamped()
        pose.header.stamp = (raw_transform.header.stamp)
        pose.header.frame_id = (self.base_frame)
        pose.pose.position.x = float(T_base_marker[0, 3])
        pose.pose.position.y = float(T_base_marker[1, 3])
        pose.pose.position.z = float(T_base_marker[2, 3])
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        self.pose_publisher.publish(pose)
        calibrated_transform = (TransformStamped())
        calibrated_transform.header = (pose.header)
        calibrated_transform.child_frame_id = (self.calibrated_marker_frame)
        calibrated_transform.transform.translation.x = (pose.pose.position.x)
        calibrated_transform.transform.translation.y = (pose.pose.position.y)
        calibrated_transform.transform.translation.z = (pose.pose.position.z)
        calibrated_transform.transform.rotation = (pose.pose.orientation)
        self.tf_broadcaster.sendTransform(calibrated_transform)
        if not self.first_pose_logged:
            self.first_pose_logged = True
            self.get_logger().info("Publishing fixed-camera calibrated " "workstation marker poses")


def main(args=None):
    rclpy.init(args=args)
    node = FixedCameraMarkerPose()
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