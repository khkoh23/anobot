#!/usr/bin/env python3

import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker


class DummyTankMarker(Node):

    def __init__(self):
        super().__init__("dummy_tank_marker")

        self.declare_parameter(
            "frame_id",
            "workstation_marker_26_calibrated",
        )
        self.declare_parameter(
            "mesh_resource",
            "package://anobot_scene/meshes/dummy_tank.STL",
        )
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("roll_deg", 0.0)
        self.declare_parameter("pitch_deg", 0.0)
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("scale_x", 1.0)
        self.declare_parameter("scale_y", 1.0)
        self.declare_parameter("scale_z", 1.0)
        self.declare_parameter("color_r", 0.55)
        self.declare_parameter("color_g", 0.65)
        self.declare_parameter("color_b", 0.75)
        self.declare_parameter("color_a", 0.80)

        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(
            Marker,
            "/dummy_tank/marker",
            marker_qos,
        )

        self.add_on_set_parameters_callback(
            self.parameter_callback
        )

        # Republish periodically because the parent marker frame is dynamic.
        self.timer = self.create_timer(
            0.2,
            self.publish_marker,
        )

        self.get_logger().info(
            "Publishing dummy tank relative to "
            "workstation_marker_26_calibrated"
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
                "scale_x",
                "scale_y",
                "scale_z",
            ):
                if float(parameter.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="Mesh scales must be positive",
                    )

        # Timer will publish the updated marker shortly.
        return SetParametersResult(successful=True)

    def publish_marker(self):
        marker = Marker()

        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.get_parameter(
            "frame_id"
        ).value

        marker.ns = "workstation"
        marker.id = 1
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        marker.mesh_resource = self.get_parameter(
            "mesh_resource"
        ).value

        marker.mesh_use_embedded_materials = False

        marker.pose.position.x = float(
            self.get_parameter("x").value
        )
        marker.pose.position.y = float(
            self.get_parameter("y").value
        )
        marker.pose.position.z = float(
            self.get_parameter("z").value
        )

        roll = math.radians(
            float(self.get_parameter("roll_deg").value)
        )
        pitch = math.radians(
            float(self.get_parameter("pitch_deg").value)
        )
        yaw = math.radians(
            float(self.get_parameter("yaw_deg").value)
        )

        qx, qy, qz, qw = self.quaternion_from_rpy(
            roll,
            pitch,
            yaw,
        )

        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        marker.scale.x = float(
            self.get_parameter("scale_x").value
        )
        marker.scale.y = float(
            self.get_parameter("scale_y").value
        )
        marker.scale.z = float(
            self.get_parameter("scale_z").value
        )

        marker.color.r = float(
            self.get_parameter("color_r").value
        )
        marker.color.g = float(
            self.get_parameter("color_g").value
        )
        marker.color.b = float(
            self.get_parameter("color_b").value
        )
        marker.color.a = float(
            self.get_parameter("color_a").value
        )

        # Zero means that the marker remains until replaced or deleted.
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    node = DummyTankMarker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()