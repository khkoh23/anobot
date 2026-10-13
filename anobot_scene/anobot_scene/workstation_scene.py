#!/usr/bin/env python3

import math
import numpy as np
import rclpy
import trimesh
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from pathlib import Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import Mesh, MeshTriangle
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

class WorkstationScene(Node):

    def __init__(self):
        super().__init__("workstation_scene")
        self.declare_parameter("frame_id", "workstation_marker_26_calibrated",)
        self.declare_parameter("object_id", "dummy_tank",)
        self.declare_parameter("mesh_file", "meshes/dummy_tank.STL",)
        self.declare_parameter("mesh_resource", "package://anobot_scene/meshes/dummy_tank.STL",)
        self.declare_parameter("x", 0.060)
        self.declare_parameter("y", 0.125)
        self.declare_parameter("z", -0.005)
        self.declare_parameter("roll_deg", -90.0)
        self.declare_parameter("pitch_deg", -90.0)
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("scale_x", 1.0)
        self.declare_parameter("scale_y", 1.0)
        self.declare_parameter("scale_z", 1.0)
        self.declare_parameter("color_r", 0.55)
        self.declare_parameter("color_g", 0.65)
        self.declare_parameter("color_b", 0.75)
        self.declare_parameter("color_a", 0.80)
        self.declare_parameter("publish_visual_marker", False,)
        self.declare_parameter("planning_frame", "world",)
        self.declare_parameter("auto_register", False,)
        self.declare_parameter("transform_timeout_s", 1.0,)
        self.callback_group = ReentrantCallbackGroup()
        self.apply_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
            callback_group=self.callback_group,
        )
        self.apply_service = self.create_service(
            Trigger,
            "/scene/apply_workstation_scene",
            self.apply_scene_callback,
            callback_group=self.callback_group,
        )
        self.remove_service = self.create_service(
            Trigger,
            "/scene/remove_workstation_scene",
            self.remove_scene_callback,
            callback_group=self.callback_group,
        )
        self.tf_buffer = Buffer(
            cache_time=Duration(seconds=10.0)
        )
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = (ReliabilityPolicy.RELIABLE)
        marker_qos.durability = (DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_publisher = self.create_publisher(
            Marker,
            "/workstation/tank_marker",
            marker_qos,
        )
        self.tank_mesh = self.load_tank_mesh()
        self.registered = False
        if not self.apply_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("/apply_planning_scene is unavailable")
        self.registration_timer = None
        if bool(self.get_parameter("auto_register").value):
            self.registration_timer = self.create_timer(
                1.0,
                self.register_scene,
                callback_group=self.callback_group,
            )
        if bool(self.get_parameter("publish_visual_marker").value):
            self.marker_timer = self.create_timer(
                0.2,
                self.publish_marker,
                callback_group=self.callback_group,
            )
        else:
            self.marker_timer = None
        self.get_logger().info("Tank-only workstation scene started")

    @staticmethod
    def quaternion_from_rpy(roll, pitch, yaw):
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

    def tank_pose(self):
        pose = Pose()
        pose.position.x = float(self.get_parameter("x").value)
        pose.position.y = float(self.get_parameter("y").value)
        pose.position.z = float(self.get_parameter("z").value)
        roll = math.radians(float(self.get_parameter("roll_deg").value))
        pitch = math.radians(float(self.get_parameter("pitch_deg").value))
        yaw = math.radians(float(self.get_parameter("yaw_deg").value))
        qx, qy, qz, qw = (self.quaternion_from_rpy(roll, pitch, yaw,))
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def scale_vector(self):
        return np.array([
            float(self.get_parameter("scale_x").value),
            float(self.get_parameter("scale_y").value),
            float(self.get_parameter("scale_z").value),
        ])

    def load_tank_mesh(self):
        mesh_path = (Path(get_package_share_directory("anobot_scene")) / self.get_parameter("mesh_file").value)
        if not mesh_path.is_file():
            raise FileNotFoundError(f"Tank mesh not found: {mesh_path}")
        loaded = trimesh.load(
            str(mesh_path),
            force="mesh",
            process=False,
        )
        if isinstance(loaded, trimesh.Scene):
            geometries = tuple(loaded.geometry.values())
            if not geometries:
                raise RuntimeError("Tank mesh contains no geometry")
            loaded = trimesh.util.concatenate(geometries)
        if not isinstance(loaded,trimesh.Trimesh,):
            raise RuntimeError("Unsupported tank mesh type: " f"{type(loaded).__name__}")
        vertices = (np.asarray(loaded.vertices, dtype=float,) * self.scale_vector())
        faces = np.asarray(loaded.faces, dtype=int,)
        if len(vertices) == 0 or len(faces) == 0:
            raise RuntimeError("Tank mesh is empty")
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
        dimensions = (vertices.max(axis=0) - vertices.min(axis=0))
        self.get_logger().info(
            "Loaded tank mesh: "
            f"{len(vertices)} vertices, "
            f"{len(faces)} triangles, "
            f"dimensions="
            f"({dimensions[0]:.4f}, "
            f"{dimensions[1]:.4f}, "
            f"{dimensions[2]:.4f}) m"
        )
        return mesh

    def make_collision_object(self):
        collision = CollisionObject()
        collision.header.stamp = (self.get_clock().now().to_msg())
        collision.header.frame_id = (self.get_parameter("frame_id").value)
        collision.id = self.get_parameter("object_id").value
        collision.meshes = [self.tank_mesh]
        local_pose = Pose()
        local_pose.orientation.w = 1.0
        collision.mesh_poses = [local_pose]
        collision.pose = self.tank_pose()
        collision.operation = CollisionObject.ADD
        return collision

    def validate_reference_frame(self):
        planning_frame = str(self.get_parameter("planning_frame").value)
        reference_frame = str(self.get_parameter("frame_id").value)
        timeout_s = float(self.get_parameter("transform_timeout_s").value)
        available = self.tf_buffer.can_transform(
            planning_frame,
            reference_frame,
            Time(),
            timeout=Duration(seconds=timeout_s),
        )
        if not available:
            raise RuntimeError("Cannot apply workstation scene: " f"transform {planning_frame} -> " f"{reference_frame} is unavailable")

    def make_remove_collision_object(self):
        collision = CollisionObject()
        collision.header.stamp = (self.get_clock().now().to_msg())
        collision.header.frame_id = str(self.get_parameter("planning_frame").value)
        collision.id = str(self.get_parameter("object_id").value)
        collision.operation = CollisionObject.REMOVE
        return collision

    async def remove_tank_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(self.make_remove_collision_object())
        request = ApplyPlanningScene.Request()
        request.scene = scene
        result = await self.apply_client.call_async(request)
        if result is None:
            raise RuntimeError("No response while removing " "workstation scene")
        if not result.success:
            raise RuntimeError("MoveIt rejected workstation-scene " "removal")
        self.registered = False
        self.get_logger().info("Removed tank collision object: " f"{self.get_parameter('object_id').value}")

    async def apply_tank_scene(self):
        self.validate_reference_frame()
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(self.make_collision_object())
        request = ApplyPlanningScene.Request()
        request.scene = scene
        result = await self.apply_client.call_async(request)
        if result is None:
            raise RuntimeError("No response while applying " "workstation scene")
        if not result.success:
            raise RuntimeError("MoveIt rejected workstation-scene " "application")
        self.registered = True
        self.get_logger().info(
            "Applied tank collision object: "
            f"{self.get_parameter('object_id').value} "
            f"in frame "
            f"{self.get_parameter('frame_id').value}"
        )

    async def register_scene(self):
        if self.registered:
            if self.registration_timer is not None:
                self.registration_timer.cancel()
            return
        try:
            await self.apply_tank_scene()
            if self.registration_timer is not None:
                self.registration_timer.cancel()
        except Exception as error:
            self.get_logger().warning("Automatic workstation-scene " f"registration waiting: {error}")

    async def apply_scene_callback(self, request, response,):
        del request
        try:
            await self.apply_tank_scene()
            response.success = True
            response.message = ("Workstation tank collision scene " "applied from " f"{self.get_parameter('frame_id').value}")
        except Exception as error:
            response.success = False
            response.message = str(error)
            self.get_logger().error(f"Apply workstation scene failed: " f"{error}")
        return response

    async def remove_scene_callback(self, request, response,):
        del request
        try:
            await self.remove_tank_scene()
            if self.marker_timer is not None:
                marker = Marker()
                marker.header.stamp = (self.get_clock().now().to_msg())
                marker.header.frame_id = str(self.get_parameter("planning_frame").value)
                marker.ns = "workstation"
                marker.id = 1
                marker.action = Marker.DELETE
                self.marker_publisher.publish(marker)
            response.success = True
            response.message = ("Workstation tank collision scene ""removed")
        except Exception as error:
            response.success = False
            response.message = str(error)
            self.get_logger().error(f"Remove workstation scene failed: " f"{error}")
        return response

    def publish_marker(self):
        marker = Marker()
        marker.header.stamp = (self.get_clock().now().to_msg())
        marker.header.frame_id = (self.get_parameter("frame_id").value)
        marker.ns = "workstation"
        marker.id = 1
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = (self.get_parameter("mesh_resource").value)
        marker.mesh_use_embedded_materials = False
        marker.pose = self.tank_pose()
        marker.scale.x = float(self.get_parameter("scale_x").value)
        marker.scale.y = float(self.get_parameter("scale_y").value)
        marker.scale.z = float(self.get_parameter("scale_z").value)
        marker.color.r = float(self.get_parameter("color_r").value)
        marker.color.g = float(self.get_parameter("color_g").value)
        marker.color.b = float(self.get_parameter("color_b").value)
        marker.color.a = float(self.get_parameter("color_a").value)
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = WorkstationScene()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()