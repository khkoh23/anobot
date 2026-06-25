#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


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

        self.apply_scene()

    def make_box(self, object_id, frame_id, size, xyz, rpy=None):
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = object_id

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(size)

        pose = Pose()
        pose.position.x = xyz[0]
        pose.position.y = xyz[1]
        pose.position.z = xyz[2]
        pose.orientation.w = 1.0

        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        return obj

    def apply_scene(self):
        workstation_1 = self.make_box(
            object_id="workstation_1",
            frame_id="world",
            size=(1.322, 1.322, 1.200),
            xyz=(0.0, 1.5, 0.6),
        )

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(workstation_1)

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
