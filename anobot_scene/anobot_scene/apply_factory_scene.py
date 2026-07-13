#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle


try:
    import trimesh
    HAS_TRIMESH = True
except ImportError:
    HAS_TRIMESH = False
    print("CRITICAL ERROR: 'trimesh' library is not installed in the current Python environment.", file=sys.stderr)
    print("Please run: pip install trimesh", file=sys.stderr)
    sys.exit(1)


class ApplyFactoryScene(Node):
    def __init__(self):
        super().__init__("apply_factory_scene")
        self.client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )
        self.get_logger().info("Waiting for /apply_planning_scene service...")
        self.client.wait_for_service()
        self.get_logger().info("Service available.")
        mesh_path = os.path.expanduser("~/anobot_ws/src/anobot/anobot_scene/meshes/tank_7.STL")
        if not os.path.exists(mesh_path):
            self.get_logger().error(f"Mesh file not found at: {mesh_path}")
            self.destroy_node()
            rclpy.shutdown()
            return
        self.apply_scene(mesh_path)

    # def make_box(self, object_id, frame_id, size, xyz, rpy=None):
    #     obj = CollisionObject()
    #     obj.header.frame_id = frame_id
    #     obj.id = object_id
    #     primitive = SolidPrimitive()
    #     primitive.type = SolidPrimitive.BOX
    #     primitive.dimensions = list(size)
    #     pose = Pose()
    #     pose.position.x = xyz[0]
    #     pose.position.y = xyz[1]
    #     pose.position.z = xyz[2]
    #     pose.orientation.w = 1.0
    #     obj.primitives.append(primitive)
    #     obj.primitive_poses.append(pose)
    #     obj.operation = CollisionObject.ADD
    #     return obj

    def load_mesh_from_file(self, file_path, scale=1.0):
        """
        Loads an STL file and returns a shape_msgs.msg.Mesh object.
        """
        mesh_msg = Mesh()
        loaded_mesh = trimesh.load(file_path)
        if isinstance(loaded_mesh, trimesh.Scene):
            loaded_mesh = loaded_mesh.dump(concatenate=True)
        vertices = loaded_mesh.vertices
        faces = loaded_mesh.faces
        for v in vertices:
            p = Point()
            p.x = float(v[0]) * scale
            p.y = float(v[1]) * scale
            p.z = float(v[2]) * scale
            mesh_msg.vertices.append(p)
        for face in faces:
            triangle = MeshTriangle()
            triangle.vertex_indices = [
                int(face[0]),
                int(face[1]),
                int(face[2]),
            ]
            mesh_msg.triangles.append(triangle)
        return mesh_msg

    def make_mesh(self, object_id, frame_id, mesh_path, xyz, rpy=None, scale=1.0):
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = object_id
        mesh = self.load_mesh_from_file(mesh_path, scale=scale)
        pose = Pose()
        pose.position.x = xyz[0]
        pose.position.y = xyz[1]
        pose.position.z = xyz[2]
        if rpy:
            from math import cos, sin
            roll, pitch, yaw = rpy
            cy = cos(yaw * 0.5)
            sy = sin(yaw * 0.5)
            cp = cos(pitch * 0.5)
            sp = sin(pitch * 0.5)
            cr = cos(roll * 0.5)
            sr = sin(roll * 0.5)
            pose.orientation.x = sr * cp * cy - cr * sp * sy
            pose.orientation.y = cr * sp * cy + sr * cp * sy
            pose.orientation.z = cr * cp * sy - sr * sp * cy
            pose.orientation.w = cr * cp * cy + sr * sp * sy
        else:
            pose.orientation.w = 1.0
        obj.meshes.append(mesh)
        obj.mesh_poses.append(pose)
        obj.operation = CollisionObject.ADD
        return obj


    def make_color(self, object_id, r, g, b, a=1.0):
        """
        Create a MoveIt ObjectColor for a collision object.

        Parameters
        ----------
        object_id : str
            Must match the CollisionObject.id.

        r, g, b : float
            Red, green, blue values from 0.0 to 1.0.

        a : float
            Alpha/transparency from 0.0 to 1.0.
            1.0 is fully opaque.
        """
        color = ObjectColor()
        color.id = object_id
        color.color.r = float(r)
        color.color.g = float(g)
        color.color.b = float(b)
        color.color.a = float(a)
        return color


    def apply_scene(self, mesh_path):
        """
        Build a PlanningScene update and send it to MoveIt.
        """
        # workstation_1 = self.make_box(
        #     object_id="workstation_1",
        #     frame_id="world",
        #     size=(1.322, 1.322, 1.200),
        #     xyz=(0.0, 1.5, 0.6),
        # )
        tank_7 = self.make_mesh(
            object_id="tank_7",
            frame_id="world",
            mesh_path=mesh_path,
            xyz=(0.0, 1.5, 1.150), # Position of the tank
            rpy=(1.5708, 0.0, 0.0), # Optional rotation
            scale=0.001,
        )
        # tank_2 = self.make_mesh(
        #     object_id="fake_tank_2",
        #     frame_id="world",
        #     mesh_path=mesh_path,
        #     xyz=(1.5, 1.5, 0.6), # Position of the tank
        #     rpy=(1.5708, 0.0, 0.0), # Optional rotation
        #     scale=0.001,
        # )
        # tank_3 = self.make_mesh(
        #     object_id="fake_tank_3",
        #     frame_id="world",
        #     mesh_path=mesh_path,
        #     xyz=(-1.5, 1.5, 0.6), # Position of the tank
        #     rpy=(1.5708, 0.0, 0.0), # Optional rotation
        #     scale=0.001,
        # )
        scene = PlanningScene()
        scene.is_diff = True
        # scene.world.collision_objects.append(workstation_1)
        scene.world.collision_objects.append(tank_7)
        # scene.world.collision_objects.append(tank_2)
        # scene.world.collision_objects.append(tank_3)
        scene.object_colors.append(self.make_color(
            "tank_7", 0.8, 0.8, 0.8, 1.0))
        # scene.object_colors.append(self.make_color(
        #     "fake_tank_2", 0.0, 1.0, 0.0, 1.0))
        # scene.object_colors.append(self.make_color(
        #     "fake_tank_3", 0.0, 0.0, 1.0, 1.0))
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info("Factory scene applied successfully.")
        else:
            self.get_logger().error("Failed to apply factory scene.")


def main(args=None):
    rclpy.init(args=args)
    node = ApplyFactoryScene()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
