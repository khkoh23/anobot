#!/usr/bin/env python3

import math
import rclpy
import statistics
import threading
import time
from dataclasses import dataclass
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from typing import List, Optional, Tuple

@dataclass
class AcquisitionQuality:
    sample_count: int
    translation_spread_m: float
    rotation_spread_deg: float


class WorkstationAcquisitionServer(Node):

    def __init__(self):
        super().__init__("workstation_acquisition_server")
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameter("input_pose_topic", "/workstation_marker/pose_base",)
        self.declare_parameter("base_frame", "ur_base",)
        self.declare_parameter("acquired_marker_frame", "workstation_marker_acquired",)
        self.declare_parameter("sample_count", 15, )
        self.declare_parameter("acquisition_timeout_s", 5.0,)
        self.declare_parameter("maximum_input_age_s", 0.25,)
        self.declare_parameter("maximum_translation_spread_m", 0.005,)
        self.declare_parameter("maximum_rotation_spread_deg", 1.5,)
        self.declare_parameter("publish_rate_hz", 10.0,)
        self.input_pose_topic = str(self.get_parameter("input_pose_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.acquired_marker_frame = str(self.get_parameter("acquired_marker_frame").value)
        self.sample_count = int(self.get_parameter("sample_count").value)
        self.acquisition_timeout_s = float(self.get_parameter("acquisition_timeout_s").value)
        self.maximum_input_age_s = float(self.get_parameter("maximum_input_age_s").value)
        self.maximum_translation_spread_m = float(self.get_parameter("maximum_translation_spread_m").value)
        self.maximum_rotation_spread_deg = float(self.get_parameter("maximum_rotation_spread_deg").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._validate_parameters()
        self._condition = threading.Condition()
        self._acquisition_active = False
        self._samples: List[PoseStamped] = []
        self._latest_pose_arrival_time: Optional[float] = None
        self._acquired_pose: Optional[Pose] = None
        self._acquired_quality: Optional[AcquisitionQuality] = None
        self._transform_broadcaster = (TransformBroadcaster(self))
        self._pose_subscription = (self.create_subscription(
            PoseStamped,
            self.input_pose_topic,
            self._pose_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        ))
        self._acquire_service = self.create_service(
            Trigger,
            "/perception/acquire_workstation",
            self._acquire_callback,
            callback_group=self.callback_group,
        )
        self._clear_service = self.create_service(
            Trigger,
            "/perception/clear_workstation",
            self._clear_callback,
            callback_group=self.callback_group,
        )
        self._status_service = self.create_service(
            Trigger,
            "/perception/workstation_status",
            self._status_callback,
            callback_group=self.callback_group,
        )
        self._publish_timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._publish_acquired_transform,
            callback_group=self.callback_group,
        )
        self.get_logger().info("Workstation acquisition server ready")
        self.get_logger().info(f"  Input pose: {self.input_pose_topic}")
        self.get_logger().info(f"  Acquired TF: {self.base_frame} -> " f"{self.acquired_marker_frame}")
        self.get_logger().info(f"  Samples required: {self.sample_count}")
        self.get_logger().info("  Translation spread limit: " f"{self.maximum_translation_spread_m * 1000.0:.2f} mm")
        self.get_logger().info("  Rotation spread limit: " f"{self.maximum_rotation_spread_deg:.2f} deg")

    def _validate_parameters(self):
        if self.sample_count < 3:
            raise ValueError("sample_count must be at least 3")
        if self.acquisition_timeout_s <= 0.0:
            raise ValueError("acquisition_timeout_s must be positive")
        if self.maximum_input_age_s <= 0.0:
            raise ValueError("maximum_input_age_s must be positive")
        if (self.maximum_translation_spread_m <= 0.0):
            raise ValueError("maximum_translation_spread_m " "must be positive")
        if self.maximum_rotation_spread_deg <= 0.0:
            raise ValueError("maximum_rotation_spread_deg " "must be positive")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

    def _pose_callback(self, message: PoseStamped,):
        arrival_time = time.monotonic()
        with self._condition:
            self._latest_pose_arrival_time = (arrival_time)
            if not self._acquisition_active:
                return
            frame_id = message.header.frame_id.lstrip("/")
            expected_frame = self.base_frame.lstrip("/")
            if frame_id != expected_frame:
                self.get_logger().warning(
                    "Ignoring calibrated pose in "
                    f"frame '{message.header.frame_id}'. "
                    f"Expected '{self.base_frame}'."
                )
                return
            if not self._pose_is_finite(message.pose):
                self.get_logger().warning("Ignoring calibrated pose with " "non-finite values")
                return
            quaternion = (
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            )
            if self._quaternion_norm(quaternion) < 1.0e-9:
                self.get_logger().warning("Ignoring calibrated pose with " "invalid zero quaternion")
                return
            self._samples.append(message)
            if (len(self._samples) >= self.sample_count):
                self._condition.notify_all()

    def _acquire_callback(self, request, response,):
        del request
        with self._condition:
            if self._acquisition_active:
                response.success = False
                response.message = ("Workstation acquisition is " "already in progress")
                return response
            if not self._input_pose_is_fresh_locked():
                response.success = False
                response.message = ("No fresh calibrated workstation " "pose is available")
                return response
            self._acquisition_active = True
            self._samples = []
        self.get_logger().info("Starting workstation acquisition")
        deadline = (time.monotonic() + self.acquisition_timeout_s)
        with self._condition:
            while (len(self._samples) < self.sample_count):
                remaining = (deadline - time.monotonic())
                if remaining <= 0.0:
                    break
                self._condition.wait(timeout=remaining)
            samples = list(self._samples)
            self._acquisition_active = False
            self._samples = []
        if len(samples) < self.sample_count:
            response.success = False
            response.message = (
                "Acquisition timed out: received "
                f"{len(samples)}/{self.sample_count} "
                "valid samples"
            )
            self.get_logger().warning(response.message)
            return response
        try:
            representative_pose, quality = (self._evaluate_samples(samples))
        except Exception as error:
            response.success = False
            response.message = (f"Acquisition evaluation failed: " f"{error}")
            self.get_logger().error(response.message)
            return response
        if (quality.translation_spread_m > self.maximum_translation_spread_m):
            response.success = False
            response.message = (
                "Acquisition rejected: translation "
                f"spread "
                f"{quality.translation_spread_m * 1000.0:.2f} mm "
                f"exceeds "
                f"{self.maximum_translation_spread_m * 1000.0:.2f} mm"
            )
            self.get_logger().warning(response.message)
            return response
        if (quality.rotation_spread_deg > self.maximum_rotation_spread_deg):
            response.success = False
            response.message = (
                "Acquisition rejected: rotation "
                f"spread "
                f"{quality.rotation_spread_deg:.2f} deg "
                f"exceeds "
                f"{self.maximum_rotation_spread_deg:.2f} deg"
            )
            self.get_logger().warning(response.message)
            return response
        # Update the acquired pose only after every
        # validation step succeeds. A failed reacquisition
        # therefore preserves the previous valid pose.
        with self._condition:
            self._acquired_pose = (representative_pose)
            self._acquired_quality = quality
        response.success = True
        response.message = (
            "Workstation acquired: "
            f"samples={quality.sample_count}, "
            "translation_spread="
            f"{quality.translation_spread_m * 1000.0:.2f} mm, "
            "rotation_spread="
            f"{quality.rotation_spread_deg:.2f} deg"
        )
        self.get_logger().info(response.message)
        return response

    def _clear_callback(self, request, response,):
        del request
        with self._condition:
            if self._acquisition_active:
                response.success = False
                response.message = ("Cannot clear while acquisition " "is in progress")
                return response
            self._acquired_pose = None
            self._acquired_quality = None
        response.success = True
        response.message = ("Acquired workstation pose cleared")
        self.get_logger().info(response.message)
        return response

    def _status_callback(self, request, response,):
        del request
        with self._condition:
            acquired_pose = self._acquired_pose
            quality = self._acquired_quality
            acquiring = self._acquisition_active
        if acquiring:
            response.success = False
            response.message = ("Workstation state: ACQUIRING")
            return response
        if acquired_pose is None or quality is None:
            response.success = False
            response.message = ("Workstation state: NOT_ACQUIRED")
            return response
        response.success = True
        response.message = (
            "Workstation state: ACQUIRED; "
            f"frame={self.acquired_marker_frame}; "
            f"samples={quality.sample_count}; "
            "translation_spread="
            f"{quality.translation_spread_m * 1000.0:.2f} mm; "
            "rotation_spread="
            f"{quality.rotation_spread_deg:.2f} deg"
        )
        return response

    def _publish_acquired_transform(self):
        with self._condition:
            acquired_pose = self._acquired_pose
        if acquired_pose is None:
            return
        transform = TransformStamped()
        transform.header.stamp = (self.get_clock().now().to_msg())
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = (self.acquired_marker_frame)
        transform.transform.translation.x = (acquired_pose.position.x)
        transform.transform.translation.y = (acquired_pose.position.y)
        transform.transform.translation.z = (acquired_pose.position.z)
        transform.transform.rotation = (acquired_pose.orientation)
        self._transform_broadcaster.sendTransform(transform)

    def _input_pose_is_fresh_locked(self):
        if self._latest_pose_arrival_time is None:
            return False
        age_s = (time.monotonic() - self._latest_pose_arrival_time)
        return age_s <= self.maximum_input_age_s

    def _evaluate_samples(self, samples: List[PoseStamped],) -> Tuple[Pose, AcquisitionQuality]:
        positions = [
            (
                sample.pose.position.x,
                sample.pose.position.y,
                sample.pose.position.z,
            )
            for sample in samples
        ]
        quaternions = [
            self._normalize_quaternion(
                (
                    sample.pose.orientation.x,
                    sample.pose.orientation.y,
                    sample.pose.orientation.z,
                    sample.pose.orientation.w,
                )
            )
            for sample in samples
        ]
        representative_position = (
            statistics.median(position[0] for position in positions),
            statistics.median(position[1] for position in positions),
            statistics.median(position[2] for position in positions),
        )
        representative_quaternion = (
            self._average_quaternions(quaternions)
        )
        translation_spread_m = max(
            self._position_distance(
                position,
                representative_position,
            )
            for position in positions
        )
        rotation_spread_deg = max(
            self._quaternion_angle_deg(
                quaternion,
                representative_quaternion,
            )
            for quaternion in quaternions
        )
        pose = Pose()
        pose.position.x = (representative_position[0])
        pose.position.y = (representative_position[1])
        pose.position.z = (representative_position[2])
        pose.orientation.x = (representative_quaternion[0])
        pose.orientation.y = (representative_quaternion[1])
        pose.orientation.z = (representative_quaternion[2])
        pose.orientation.w = (representative_quaternion[3])
        quality = AcquisitionQuality(
            sample_count=len(samples),
            translation_spread_m=(translation_spread_m),
            rotation_spread_deg=(rotation_spread_deg),
        )
        return pose, quality

    @staticmethod
    def _pose_is_finite(pose: Pose):
        values = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        return all(math.isfinite(value) for value in values)

    @staticmethod
    def _position_distance(first, second,):
        return math.sqrt(
            (first[0] - second[0]) ** 2
            + (first[1] - second[1]) ** 2
            + (first[2] - second[2]) ** 2
        )

    @staticmethod
    def _quaternion_norm(quaternion):
        return math.sqrt(
            sum(
                component * component
                for component in quaternion
            )
        )

    @classmethod
    def _normalize_quaternion(cls, quaternion,):
        norm = cls._quaternion_norm(quaternion)
        if norm < 1.0e-12:
            raise ValueError("Cannot normalize zero quaternion")
        return tuple(component / norm for component in quaternion)

    @classmethod
    def _average_quaternions(cls, quaternions,):
        reference = quaternions[0]
        aligned = []
        for quaternion in quaternions:
            dot = sum(
                first * second
                for first, second
                in zip(
                    reference,
                    quaternion,
                )
            )
            if dot < 0.0:
                quaternion = tuple(
                    -component
                    for component in quaternion
                )
            aligned.append(quaternion)
        summed = tuple(
            sum(
                quaternion[index]
                for quaternion in aligned
            )
            for index in range(4)
        )
        return cls._normalize_quaternion(
            summed
        )

    @staticmethod
    def _quaternion_angle_deg(first, second,):
        dot = abs(
            sum(
                first_component
                * second_component
                for (
                    first_component,
                    second_component,
                ) in zip(first, second)
            )
        )
        dot = max(
            -1.0,
            min(1.0, dot),
        )
        angle_rad = 2.0 * math.acos(dot)
        return math.degrees(angle_rad)


def main(args=None):
    rclpy.init(args=args)
    node = WorkstationAcquisitionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()