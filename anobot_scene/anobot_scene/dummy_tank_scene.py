#!/usr/bin/env python3

import math
import rclpy
import numpy as np
import trimesh
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle

class DummyTankScene(Node):
  def __init__(self): 
    super().__init__("dummy_tank_scene") 
    self.declare_parameter("frame_id", "workstation_marker_26_calibrated",)
    self.declare_parameter(
      "mesh_resource",
      "package://anobot_scene/meshes/dummy_tank.STL",
    )
    self.declare_parameter("object_id", "dummy_tank")
    self.declare_parameter(
      "mesh_file",
      "meshes/dummy_tank.STL",
    )
    self.declare_parameter("collision_publish_delay_s", 3.0,)
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
    collision_qos = QoSProfile(depth=1)
    collision_qos.reliability = ReliabilityPolicy.RELIABLE
    collision_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    self.collision_publisher = self.create_publisher(
      CollisionObject,
      "/collision_object",
      collision_qos,
    )
    self.collision_mesh = self.load_collision_mesh()
    self.collision_published = False
    self.collision_timer = self.create_timer(
      float(self.get_parameter("collision_publish_delay_s").value),
      self.publish_collision,
    )
    self.add_on_set_parameters_callback(self.parameter_callback)
    # Republish periodically because the parent marker frame is dynamic.
    self.timer = self.create_timer(0.2, self.publish_marker,)
    self.get_logger().info(
      "Publishing dummy tank relative to workstation_marker_26_calibrated"
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
    marker.header.frame_id = self.get_parameter("frame_id").value
    marker.ns = "workstation"
    marker.id = 1
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.mesh_resource = self.get_parameter("mesh_resource").value
    marker.mesh_use_embedded_materials = False
    marker.pose = self.make_pose()
    marker.scale.x = float(self.get_parameter("scale_x").value)
    marker.scale.y = float(self.get_parameter("scale_y").value)
    marker.scale.z = float(self.get_parameter("scale_z").value)
    marker.color.r = float(self.get_parameter("color_r").value)
    marker.color.g = float(self.get_parameter("color_g").value)
    marker.color.b = float(self.get_parameter("color_b").value)
    marker.color.a = float(self.get_parameter("color_a").value)
    # Zero means that the marker remains until replaced or deleted.
    marker.lifetime.sec = 0
    marker.lifetime.nanosec = 0
    self.publisher.publish(marker)

  def load_collision_mesh(self):
    mesh_path = (
      Path(get_package_share_directory("anobot_scene"))
      / self.get_parameter("mesh_file").value
    )
    if not mesh_path.is_file():
      raise FileNotFoundError(f"Mesh not found: {mesh_path}")
    loaded = trimesh.load_mesh(
      str(mesh_path),
      process=False,
    )
    if isinstance(loaded, trimesh.Scene):
      loaded = trimesh.util.concatenate(
        tuple(loaded.geometry.values())
      )
    scale = np.array([
      float(self.get_parameter("scale_x").value),
      float(self.get_parameter("scale_y").value),
      float(self.get_parameter("scale_z").value),
    ])
    vertices = np.asarray(loaded.vertices) * scale
    faces = np.asarray(loaded.faces)
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
    self.get_logger().info(
      f"Loaded collision mesh: "
      f"{len(vertices)} vertices, "
      f"{len(faces)} triangles"
    )
    return mesh

  def make_pose(self):
    pose = Pose()
    pose.position.x = float(self.get_parameter("x").value)
    pose.position.y = float(self.get_parameter("y").value)
    pose.position.z = float(self.get_parameter("z").value)
    roll = math.radians(float(self.get_parameter("roll_deg").value))
    pitch = math.radians(float(self.get_parameter("pitch_deg").value))
    yaw = math.radians(float(self.get_parameter("yaw_deg").value))
    qx, qy, qz, qw = self.quaternion_from_rpy(roll, pitch, yaw,)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose

  def publish_collision(self):
    if self.collision_published:
      return
    frame_id = self.get_parameter("frame_id").value
    object_id = self.get_parameter("object_id").value
    remove = CollisionObject()
    remove.header.stamp = self.get_clock().now().to_msg()
    remove.header.frame_id = frame_id
    remove.id = object_id
    remove.operation = CollisionObject.REMOVE
    self.collision_publisher.publish(remove)
    collision = CollisionObject()
    collision.header.stamp = self.get_clock().now().to_msg()
    collision.header.frame_id = frame_id
    collision.id = object_id
    collision.meshes = [self.collision_mesh]
    collision.mesh_poses = [self.make_pose()]
    collision.operation = CollisionObject.ADD
    self.collision_publisher.publish(collision)
    self.collision_published = True
    self.collision_timer.cancel()
    self.get_logger().info(
      f"Registered collision object '{object_id}' "
      f"relative to frame '{frame_id}'"
    )

def main(args=None):
  rclpy.init(args=args)
  node = DummyTankScene()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()