#!/usr/bin/env python3

import math
from pathlib import Path

import numpy as np
import rclpy
import trimesh

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from shape_msgs.msg import Mesh, MeshTriangle
from visualization_msgs.msg import Marker, MarkerArray


class WorkstationScene(Node):

    def __init__(self):
        super().__init__("workstation_scene")

        self.declare_parameter(
            "frame_id",
            "workstation_marker_26_calibrated",
        )
        self.declare_parameter(
            "collision_publish_delay_s",
            3.0,
        )

        self.declare_parameter(
          "publish_visual_markers",
          False,
        )

        self.declare_object_parameters(
            prefix="tank",
            object_id="dummy_tank",
            mesh_file="meshes/dummy_tank.STL",
            mesh_resource=(
                "package://anobot_scene/"
                "meshes/dummy_tank.STL"
            ),
            x=0.060,
            y=0.125,
            z=-0.005,
            roll_deg=-90.0,
            pitch_deg=-90.0,
            yaw_deg=0.0,
            color=(0.55, 0.65, 0.75, 0.80),
        )

        self.declare_object_parameters(
            prefix="rod",
            object_id="anodizing_rod",
            mesh_file="meshes/anodizing_rod.STL",
            mesh_resource=(
                "package://anobot_scene/"
                "meshes/anodizing_rod.STL"
            ),
            x=0.060,
            y=0.125,
            z=-0.005,
            roll_deg=-90.0,
            pitch_deg=-90.0,
            yaw_deg=0.0,
            color=(0.80, 0.80, 0.85, 1.00),
        )

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = (
            ReliabilityPolicy.RELIABLE
        )
        marker_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        collision_qos = QoSProfile(depth=10)
        collision_qos.reliability = (
            ReliabilityPolicy.RELIABLE
        )
        collision_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            "/workstation/markers",
            marker_qos,
        )

        self.collision_publisher = self.create_publisher(
            CollisionObject,
            "/collision_object",
            collision_qos,
        )

        self.meshes = {
            "tank": self.load_collision_mesh("tank"),
            "rod": self.load_collision_mesh("rod"),
        }

        self.marker_timer = None

        if bool(
            self.get_parameter(
                "publish_visual_markers"
            ).value
        ):
            self.marker_timer = self.create_timer(
                0.2,
                self.publish_markers,
            )

        delay = float(
            self.get_parameter(
                "collision_publish_delay_s"
            ).value
        )

        self.registration_timer = self.create_timer(
            delay,
            self.register_collision_scene,
        )

        self.registered = False

        self.get_logger().info(
            "Workstation scene started"
        )
        self.get_logger().info(
            "Objects: dummy_tank, anodizing_rod"
        )

    def declare_object_parameters(
        self,
        prefix,
        object_id,
        mesh_file,
        mesh_resource,
        x,
        y,
        z,
        roll_deg,
        pitch_deg,
        yaw_deg,
        color,
    ):
        self.declare_parameter(
            f"{prefix}.object_id",
            object_id,
        )
        self.declare_parameter(
            f"{prefix}.mesh_file",
            mesh_file,
        )
        self.declare_parameter(
            f"{prefix}.mesh_resource",
            mesh_resource,
        )

        self.declare_parameter(f"{prefix}.x", x)
        self.declare_parameter(f"{prefix}.y", y)
        self.declare_parameter(f"{prefix}.z", z)

        self.declare_parameter(
            f"{prefix}.roll_deg",
            roll_deg,
        )
        self.declare_parameter(
            f"{prefix}.pitch_deg",
            pitch_deg,
        )
        self.declare_parameter(
            f"{prefix}.yaw_deg",
            yaw_deg,
        )

        self.declare_parameter(
            f"{prefix}.scale_x",
            1.0,
        )
        self.declare_parameter(
            f"{prefix}.scale_y",
            1.0,
        )
        self.declare_parameter(
            f"{prefix}.scale_z",
            1.0,
        )

        self.declare_parameter(
            f"{prefix}.color_r",
            color[0],
        )
        self.declare_parameter(
            f"{prefix}.color_g",
            color[1],
        )
        self.declare_parameter(
            f"{prefix}.color_b",
            color[2],
        )
        self.declare_parameter(
            f"{prefix}.color_a",
            color[3],
        )

    @staticmethod
    def quaternion_from_rpy(
        roll,
        pitch,
        yaw,
    ):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def object_pose(self, prefix):
        pose = Pose()

        pose.position.x = float(
            self.get_parameter(
                f"{prefix}.x"
            ).value
        )
        pose.position.y = float(
            self.get_parameter(
                f"{prefix}.y"
            ).value
        )
        pose.position.z = float(
            self.get_parameter(
                f"{prefix}.z"
            ).value
        )

        roll = math.radians(
            float(
                self.get_parameter(
                    f"{prefix}.roll_deg"
                ).value
            )
        )
        pitch = math.radians(
            float(
                self.get_parameter(
                    f"{prefix}.pitch_deg"
                ).value
            )
        )
        yaw = math.radians(
            float(
                self.get_parameter(
                    f"{prefix}.yaw_deg"
                ).value
            )
        )

        qx, qy, qz, qw = (
            self.quaternion_from_rpy(
                roll,
                pitch,
                yaw,
            )
        )

        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        return pose

    def object_scale(self, prefix):
        return np.array([
            float(
                self.get_parameter(
                    f"{prefix}.scale_x"
                ).value
            ),
            float(
                self.get_parameter(
                    f"{prefix}.scale_y"
                ).value
            ),
            float(
                self.get_parameter(
                    f"{prefix}.scale_z"
                ).value
            ),
        ])

    def mesh_path(self, prefix):
        relative_path = self.get_parameter(
            f"{prefix}.mesh_file"
        ).value

        return (
            Path(
                get_package_share_directory(
                    "anobot_scene"
                )
            )
            / relative_path
        )

    def load_collision_mesh(self, prefix):
        path = self.mesh_path(prefix)

        if not path.is_file():
            raise FileNotFoundError(
                f"{prefix} mesh not found: {path}"
            )

        loaded = trimesh.load(
            str(path),
            force="mesh",
            process=False,
        )

        if isinstance(loaded, trimesh.Scene):
            geometries = tuple(
                loaded.geometry.values()
            )

            if not geometries:
                raise RuntimeError(
                    f"{prefix} mesh has no geometry"
                )

            loaded = trimesh.util.concatenate(
                geometries
            )

        if not isinstance(
            loaded,
            trimesh.Trimesh,
        ):
            raise RuntimeError(
                f"Unsupported {prefix} mesh type: "
                f"{type(loaded).__name__}"
            )

        vertices = (
            np.asarray(
                loaded.vertices,
                dtype=float,
            )
            * self.object_scale(prefix)
        )

        faces = np.asarray(
            loaded.faces,
            dtype=int,
        )

        if (
            len(vertices) == 0
            or len(faces) == 0
        ):
            raise RuntimeError(
                f"{prefix} mesh is empty"
            )

        mesh = Mesh()

        for vertex in vertices:
            point = Point()
            point.x = float(vertex[0])
            point.y = float(vertex[1])
            point.z = float(vertex[2])
            mesh.vertices.append(point)

        for face in faces:
            triangle = MeshTriangle()
            triangle.vertex_indices = [
                int(face[0]),
                int(face[1]),
                int(face[2]),
            ]
            mesh.triangles.append(triangle)

        bounds = np.array([
            vertices.min(axis=0),
            vertices.max(axis=0),
        ])

        dimensions = bounds[1] - bounds[0]

        self.get_logger().info(
            f"Loaded {prefix} mesh: "
            f"{len(vertices)} vertices, "
            f"{len(faces)} triangles, "
            f"dimensions="
            f"({dimensions[0]:.4f}, "
            f"{dimensions[1]:.4f}, "
            f"{dimensions[2]:.4f}) m"
        )

        return mesh

    def make_visual_marker(
        self,
        prefix,
        marker_id,
    ):
        marker = Marker()

        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )
        marker.header.frame_id = (
            self.get_parameter(
                "frame_id"
            ).value
        )

        marker.ns = "workstation"
        marker.id = marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        marker.mesh_resource = (
            self.get_parameter(
                f"{prefix}.mesh_resource"
            ).value
        )

        marker.mesh_use_embedded_materials = False
        marker.pose = self.object_pose(prefix)

        scale = self.object_scale(prefix)

        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])

        marker.color.r = float(
            self.get_parameter(
                f"{prefix}.color_r"
            ).value
        )
        marker.color.g = float(
            self.get_parameter(
                f"{prefix}.color_g"
            ).value
        )
        marker.color.b = float(
            self.get_parameter(
                f"{prefix}.color_b"
            ).value
        )
        marker.color.a = float(
            self.get_parameter(
                f"{prefix}.color_a"
            ).value
        )

        return marker

    def publish_markers(self):
        markers = MarkerArray()

        markers.markers.append(
            self.make_visual_marker(
                "tank",
                1,
            )
        )
        markers.markers.append(
            self.make_visual_marker(
                "rod",
                2,
            )
        )

        self.marker_publisher.publish(markers)

    def make_collision_object(self, prefix):
        collision = CollisionObject()

        collision.header.stamp = (
            self.get_clock().now().to_msg()
        )
        collision.header.frame_id = (
            self.get_parameter(
                "frame_id"
            ).value
        )

        collision.id = self.get_parameter(
            f"{prefix}.object_id"
        ).value

        collision.meshes = [
            self.meshes[prefix]
        ]
        collision.mesh_poses = [
            self.object_pose(prefix)
        ]

        collision.operation = (
            CollisionObject.ADD
        )

        return collision

    def remove_collision_object(
        self,
        object_id,
    ):
        remove = CollisionObject()

        remove.header.stamp = (
            self.get_clock().now().to_msg()
        )
        remove.header.frame_id = (
            self.get_parameter(
                "frame_id"
            ).value
        )

        remove.id = object_id
        remove.operation = CollisionObject.REMOVE

        self.collision_publisher.publish(remove)

    def register_collision_scene(self):
        if self.registered:
            return

        tank_id = self.get_parameter(
            "tank.object_id"
        ).value
        rod_id = self.get_parameter(
            "rod.object_id"
        ).value

        self.remove_collision_object(tank_id)
        self.remove_collision_object(rod_id)

        self.collision_publisher.publish(
            self.make_collision_object("tank")
        )
        self.collision_publisher.publish(
            self.make_collision_object("rod")
        )

        self.registered = True
        self.registration_timer.cancel()

        self.get_logger().info(
            "Registered workstation collision objects: "
            f"{tank_id}, {rod_id}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = WorkstationScene()

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