#!/usr/bin/env python3

import copy
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    ObjectColor,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


ROD_ID = "anodizing_rod"
WORLD_FRAME = "world"
DEFAULT_SUPPORT_FRAME = "base_footprint"
DEFAULT_SUPPORT_LINK = "base_footprint"
ATTACH_LINK = "anobot_grasp_frame"

ROD_LENGTH = 1.008
ROD_RADIUS = 0.014


# ---------------------------------------------------------------------------
# Pose mathematics
# ---------------------------------------------------------------------------

def quaternion_normalize(q):
    norm = math.sqrt(
        q[0] * q[0]
        + q[1] * q[1]
        + q[2] * q[2]
        + q[3] * q[3]
    )

    if norm < 1.0e-12:
        return 0.0, 0.0, 0.0, 1.0

    return (
        q[0] / norm,
        q[1] / norm,
        q[2] / norm,
        q[3] / norm,
    )


def quaternion_conjugate(q):
    return -q[0], -q[1], -q[2], q[3]


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return quaternion_normalize(
        (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )
    )


def rotate_vector(q, vector):
    q = quaternion_normalize(q)

    vector_quaternion = (
        vector[0],
        vector[1],
        vector[2],
        0.0,
    )

    rotated = quaternion_multiply_raw(
        quaternion_multiply_raw(q, vector_quaternion),
        quaternion_conjugate(q),
    )

    return rotated[0], rotated[1], rotated[2]


def quaternion_multiply_raw(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def pose_to_transform_tuple(pose):
    translation = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
    )

    rotation = quaternion_normalize(
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
    )

    return translation, rotation


def transform_msg_to_tuple(transform):
    translation = (
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    )

    rotation = quaternion_normalize(
        (
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
    )

    return translation, rotation


def transform_compose(transform_a_b, transform_b_c):
    """
    Compose:
        T_a_c = T_a_b * T_b_c
    """

    translation_a_b, rotation_a_b = transform_a_b
    translation_b_c, rotation_b_c = transform_b_c

    rotated_translation = rotate_vector(
        rotation_a_b,
        translation_b_c,
    )

    translation_a_c = (
        translation_a_b[0] + rotated_translation[0],
        translation_a_b[1] + rotated_translation[1],
        translation_a_b[2] + rotated_translation[2],
    )

    rotation_a_c = quaternion_multiply(
        rotation_a_b,
        rotation_b_c,
    )

    return translation_a_c, rotation_a_c


def transform_inverse(transform_a_b):
    """
    Return:
        T_b_a = inverse(T_a_b)
    """

    translation_a_b, rotation_a_b = transform_a_b
    rotation_b_a = quaternion_conjugate(rotation_a_b)

    negative_translation = (
        -translation_a_b[0],
        -translation_a_b[1],
        -translation_a_b[2],
    )

    translation_b_a = rotate_vector(
        rotation_b_a,
        negative_translation,
    )

    return translation_b_a, rotation_b_a


def transform_tuple_to_pose(transform):
    translation, rotation = transform

    pose = Pose()

    pose.position.x = translation[0]
    pose.position.y = translation[1]
    pose.position.z = translation[2]

    pose.orientation.x = rotation[0]
    pose.orientation.y = rotation[1]
    pose.orientation.z = rotation[2]
    pose.orientation.w = rotation[3]

    return pose


def quaternion_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return quaternion_normalize(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


class RodSceneManager(Node):

    def __init__(self):
        super().__init__("rod_scene_manager")

        self.declare_parameter("operation", "status")

        # The initial rod pose is expressed in support_frame.
        self.declare_parameter(
            "support_frame",
            DEFAULT_SUPPORT_FRAME,
        )

        self.declare_parameter(
            "support_link",
            DEFAULT_SUPPORT_LINK,
        )

        self.declare_parameter("rod_x", 0.0)
        self.declare_parameter("rod_y", -0.50)
        self.declare_parameter("rod_z", 0.90)

        # A MoveIt cylinder is aligned along its local Z-axis.
        # Pitch = pi / 2 makes it horizontal along the local X-axis.
        self.declare_parameter("rod_roll", 0.0)
        self.declare_parameter(
            "rod_pitch",
            math.pi / 2.0,
        )
        self.declare_parameter("rod_yaw", 0.0)

        self.declare_parameter(
            "attach_link",
            ATTACH_LINK,
        )

        self.apply_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )

        self.get_scene_client = self.create_client(
            GetPlanningScene,
            "/get_planning_scene",
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=True,
        )

        self.get_logger().info(
            "Waiting for MoveIt planning-scene services..."
        )

        if not self.apply_client.wait_for_service(
            timeout_sec=10.0
        ):
            raise RuntimeError(
                "/apply_planning_scene is unavailable"
            )

        if not self.get_scene_client.wait_for_service(
            timeout_sec=10.0
        ):
            raise RuntimeError(
                "/get_planning_scene is unavailable"
            )

        # Give the TF listener a brief opportunity to populate.
        time.sleep(0.25)

        operation = (
            self.get_parameter("operation")
            .get_parameter_value()
            .string_value
        )

        if operation == "add":
            self.add_rod()
        elif operation == "attach":
            self.attach_rod_preserving_pose()
        elif operation == "place":
            self.place_rod_preserving_pose()
        elif operation == "stow":
            self.stow_rod_preserving_pose()
        elif operation == "remove":
            self.remove_rod()
        elif operation == "status":
            self.print_status()
        else:
            raise ValueError(
                f"Unsupported operation '{operation}'. "
                "Use add, attach, place, stow, remove, or status."
            )

    # -----------------------------------------------------------------------
    # Parameter helpers
    # -----------------------------------------------------------------------

    def get_attach_link(self):
        return (
            self.get_parameter("attach_link")
            .get_parameter_value()
            .string_value
        )

    def get_support_frame(self):
        return (
            self.get_parameter("support_frame")
            .get_parameter_value()
            .string_value
        )
    
    def get_support_link(self):
        return (
        self.get_parameter("support_link")
        .get_parameter_value()
        .string_value
    )

    def get_initial_rod_pose(self):
        pose = Pose()

        pose.position.x = (
            self.get_parameter("rod_x")
            .get_parameter_value()
            .double_value
        )
        pose.position.y = (
            self.get_parameter("rod_y")
            .get_parameter_value()
            .double_value
        )
        pose.position.z = (
            self.get_parameter("rod_z")
            .get_parameter_value()
            .double_value
        )

        roll = (
            self.get_parameter("rod_roll")
            .get_parameter_value()
            .double_value
        )
        pitch = (
            self.get_parameter("rod_pitch")
            .get_parameter_value()
            .double_value
        )
        yaw = (
            self.get_parameter("rod_yaw")
            .get_parameter_value()
            .double_value
        )

        quaternion = quaternion_from_rpy(
            roll,
            pitch,
            yaw,
        )

        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        return pose

    # -----------------------------------------------------------------------
    # Rod geometry
    # -----------------------------------------------------------------------

    @staticmethod
    def make_rod_primitive():
        rod = SolidPrimitive()
        rod.type = SolidPrimitive.CYLINDER

        # Cylinder dimensions are [height, radius].
        rod.dimensions = [
            ROD_LENGTH,
            ROD_RADIUS,
        ]

        return rod

    @staticmethod
    def identity_pose():
        pose = Pose()
        pose.orientation.w = 1.0
        return pose

    @staticmethod
    def make_rod_color():
        color = ObjectColor()
        color.id = ROD_ID
        color.color.r = 0.68
        color.color.g = 0.68
        color.color.b = 0.72
        color.color.a = 1.0
        return color

    # -----------------------------------------------------------------------
    # Planning-scene queries
    # -----------------------------------------------------------------------

    def query_scene(self):
        request = GetPlanningScene.Request()

        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.ROBOT_STATE
        )

        future = self.get_scene_client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        if future.result() is None:
            raise RuntimeError(
                "Timed out while querying the planning scene"
            )

        return future.result().scene

    def find_world_rod(self, scene):
        for collision_object in (
            scene.world.collision_objects
        ):
            if collision_object.id == ROD_ID:
                return copy.deepcopy(collision_object)

        return None

    def find_attached_rod(self, scene):
        for attached_object in (
            scene.robot_state.attached_collision_objects
        ):
            if attached_object.object.id == ROD_ID:
                return copy.deepcopy(attached_object)

        return None

    # -----------------------------------------------------------------------
    # TF and object-pose conversion
    # -----------------------------------------------------------------------

    def lookup_transform_tuple(
        self,
        target_frame,
        source_frame,
    ):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=2.0),
            )
        except TransformException as exc:
            raise RuntimeError(
                f"Could not transform from '{source_frame}' "
                f"to '{target_frame}': {exc}"
            ) from exc

        return transform_msg_to_tuple(
            transform_stamped.transform
        )

    def get_object_pose_in_frame(
        self,
        collision_object,
        target_frame,
    ):
        """
        Return T_target_object for a CollisionObject.

        In the current MoveIt message format, CollisionObject.pose
        represents the object's reference pose, while primitive_poses
        and mesh_poses are local geometry poses.
        """

        source_frame = collision_object.header.frame_id

        if not source_frame:
            source_frame = WORLD_FRAME

        source_to_object = pose_to_transform_tuple(
            collision_object.pose
        )

        if source_frame == target_frame:
            return source_to_object

        target_to_source = self.lookup_transform_tuple(
            target_frame,
            source_frame,
        )

        return transform_compose(
            target_to_source,
            source_to_object,
        )

    # -----------------------------------------------------------------------
    # Apply and verify
    # -----------------------------------------------------------------------

    def apply_scene(self, scene, description):
        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self.apply_client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        # Some previous tests showed that the update could be applied even
        # when the immediate client result appeared unsuccessful. Therefore,
        # methods below verify the resulting planning-scene state explicitly.
        if future.result() is None:
            self.get_logger().warning(
                f"No immediate service response for '{description}'. "
                "The resulting planning scene will be verified."
            )
        elif not future.result().success:
            self.get_logger().warning(
                f"Service reported unsuccessful for '{description}'. "
                "The resulting planning scene will be verified."
            )

        time.sleep(0.15)

    # -----------------------------------------------------------------------
    # Operations
    # -----------------------------------------------------------------------

    def add_rod(self):
        """
        Add the rod as an object attached to the mobile manipulator.

        The rod pose is expressed relative to support_link. Since it is
        attached to a link in the robot model, it travels with the AGV.

        This represents a rod resting in a loading fixture before pickup
        by the manipulator.
        """

        current_scene = self.query_scene()

        existing_world_rod = self.find_world_rod(
            current_scene
        )
        existing_attached_rod = self.find_attached_rod(
            current_scene
        )

        if existing_world_rod is not None:
            raise RuntimeError(
                "Cannot add loading rod: a world rod already exists. "
                "Run operation:=remove first."
            )

        if existing_attached_rod is not None:
            raise RuntimeError(
                "Cannot add loading rod: a rod is already attached "
                f"to '{existing_attached_rod.link_name}'. "
                "Run operation:=remove first."
            )

        support_link = self.get_support_link()

        rod_object = CollisionObject()
        rod_object.header.frame_id = support_link
        rod_object.id = ROD_ID
        rod_object.operation = CollisionObject.ADD

        # The rod pose is directly relative to the mobile support link.
        rod_object.pose = self.get_initial_rod_pose()

        # Cylinder geometry is centered at the rod object frame.
        rod_object.primitives.append(
            self.make_rod_primitive()
        )
        rod_object.primitive_poses.append(
            self.identity_pose()
        )

        attached_rod = AttachedCollisionObject()
        attached_rod.link_name = support_link
        attached_rod.object = rod_object

        # Only the supporting link is considered an intentional contact.
        # base_footprint itself has no collision geometry in your URDF,
        # but keeping it here clearly documents the support relationship.
        attached_rod.touch_links = [
            support_link,
        ]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        scene.robot_state.attached_collision_objects.append(
            attached_rod
        )

        scene.object_colors.append(
            self.make_rod_color()
        )

        self.apply_scene(
            scene,
            "add rod to mobile loading support",
        )

        verification_scene = self.query_scene()

        verified_attached = self.find_attached_rod(
            verification_scene
        )
        verified_world = self.find_world_rod(
            verification_scene
        )

        if verified_attached is None:
            raise RuntimeError(
                "Rod was not found under attached objects after add"
            )

        if verified_world is not None:
            raise RuntimeError(
                "Rod exists as both a world and attached object "
                "after add"
            )

        if verified_attached.link_name != support_link:
            raise RuntimeError(
                "Rod was added, but attached to unexpected link "
                f"'{verified_attached.link_name}' instead of "
                f"'{support_link}'"
            )

        pose = verified_attached.object.pose

        self.get_logger().info(
            "Rod added to mobile loading support. "
            f"Attached to '{support_link}', "
            "relative pose: "
            f"xyz=({pose.position.x:.4f}, "
            f"{pose.position.y:.4f}, "
            f"{pose.position.z:.4f}), "
            f"quaternion=({pose.orientation.x:.4f}, "
            f"{pose.orientation.y:.4f}, "
            f"{pose.orientation.z:.4f}, "
            f"{pose.orientation.w:.4f})."
        )

    def attach_rod_preserving_pose(self):
        """
        Transfer the rod from its current state to the robot grasp frame
        without changing its world pose.

        Supported starting states:

        1. Rod attached to the mobile support link.
        2. Rod stored as a world collision object.

        The resulting rod is attached to attach_link while preserving
        any position or orientation mismatch between the gripper and rod.
        """

        current_scene = self.query_scene()

        attach_link = self.get_attach_link()

        existing_attached = self.find_attached_rod(
            current_scene
        )
        existing_world = self.find_world_rod(
            current_scene
        )

        # ---------------------------------------------------------------
        # Case 1: rod is already attached to the manipulator.
        # ---------------------------------------------------------------

        if (
            existing_attached is not None
            and existing_attached.link_name == attach_link
        ):
            self.get_logger().info(
                f"Rod is already attached to '{attach_link}'."
            )
            return

        # ---------------------------------------------------------------
        # Determine current rod pose in world.
        # ---------------------------------------------------------------

        if existing_attached is not None:
            source_link = existing_attached.link_name
            source_object = existing_attached.object

            # Pose of rod relative to its current support link.
            source_to_rod = pose_to_transform_tuple(
                source_object.pose
            )

            # Pose of current support link in world.
            world_to_source = self.lookup_transform_tuple(
                WORLD_FRAME,
                source_link,
            )

            # Current pose of rod in world.
            world_to_rod = transform_compose(
                world_to_source,
                source_to_rod,
            )

            self.get_logger().info(
                "Transferring rod attachment from "
                f"'{source_link}' to '{attach_link}'."
            )

        elif existing_world is not None:
            source_link = None
            source_object = existing_world

            # Current rod pose in world.
            world_to_rod = self.get_object_pose_in_frame(
                existing_world,
                WORLD_FRAME,
            )

            self.get_logger().info(
                "Attaching rod from world to "
                f"'{attach_link}'."
            )

        else:
            raise RuntimeError(
                "Cannot attach: rod is neither on the mobile "
                "support nor present in the world"
            )

        # ---------------------------------------------------------------
        # Calculate rod pose relative to the robot grasp link.
        #
        # T_attach_rod =
        #     inverse(T_world_attach) * T_world_rod
        # ---------------------------------------------------------------

        world_to_attach = self.lookup_transform_tuple(
            WORLD_FRAME,
            attach_link,
        )

        attach_to_world = transform_inverse(
            world_to_attach
        )

        attach_to_rod = transform_compose(
            attach_to_world,
            world_to_rod,
        )

        # ---------------------------------------------------------------
        # Create the new attached rod.
        # ---------------------------------------------------------------

        rod_object = copy.deepcopy(source_object)

        rod_object.header.frame_id = attach_link
        rod_object.pose = transform_tuple_to_pose(
            attach_to_rod
        )
        rod_object.operation = CollisionObject.ADD

        new_attached = AttachedCollisionObject()
        new_attached.link_name = attach_link
        new_attached.object = rod_object

        new_attached.touch_links = [
            attach_link,
            "anobot_tool_link",
            "ur10e_tool0",
        ]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        # ---------------------------------------------------------------
        # Remove the old representation.
        # ---------------------------------------------------------------

        if existing_attached is not None:
            remove_old_object = CollisionObject()
            remove_old_object.id = ROD_ID
            remove_old_object.operation = (
                CollisionObject.REMOVE
            )

            remove_old_attached = AttachedCollisionObject()
            remove_old_attached.link_name = (
                existing_attached.link_name
            )
            remove_old_attached.object = (
                remove_old_object
            )

            scene.robot_state.attached_collision_objects.append(
                remove_old_attached
            )

        if existing_world is not None:
            remove_world = CollisionObject()
            remove_world.header.frame_id = WORLD_FRAME
            remove_world.id = ROD_ID
            remove_world.operation = CollisionObject.REMOVE

            scene.world.collision_objects.append(
                remove_world
            )

        # Append removal before addition so the final state has one rod.
        scene.robot_state.attached_collision_objects.append(
            new_attached
        )

        self.apply_scene(
            scene,
            "transfer rod to grasp frame while preserving pose",
        )

        # ---------------------------------------------------------------
        # Verify resulting state.
        # ---------------------------------------------------------------

        verification_scene = self.query_scene()

        verified_attached = self.find_attached_rod(
            verification_scene
        )
        verified_world = self.find_world_rod(
            verification_scene
        )

        if verified_attached is None:
            raise RuntimeError(
                "Rod was not found under attached objects "
                "after attachment transfer"
            )

        if verified_attached.link_name != attach_link:
            raise RuntimeError(
                "Rod remains attached to unexpected link "
                f"'{verified_attached.link_name}' instead of "
                f"'{attach_link}'"
            )

        if verified_world is not None:
            raise RuntimeError(
                "Rod exists as both world and attached object"
            )

        pose = verified_attached.object.pose

        self.get_logger().info(
            "Rod transferred to grasp frame without snapping. "
            "Relative rod pose in grasp frame: "
            f"xyz=({pose.position.x:.4f}, "
            f"{pose.position.y:.4f}, "
            f"{pose.position.z:.4f}), "
            f"quaternion=({pose.orientation.x:.4f}, "
            f"{pose.orientation.y:.4f}, "
            f"{pose.orientation.z:.4f}, "
            f"{pose.orientation.w:.4f})."
        )

    def place_rod_preserving_pose(self):
        current_scene = self.query_scene()

        attached_rod = self.find_attached_rod(
            current_scene
        )

        if attached_rod is None:
            raise RuntimeError(
                "Cannot place: rod is not attached"
            )

        attached_object = attached_rod.object
        attached_frame = attached_rod.link_name

        if not attached_frame:
            attached_frame = (
                attached_object.header.frame_id
            )

        # T_attached-frame_rod
        attached_to_rod = pose_to_transform_tuple(
            attached_object.pose
        )

        # T_world_attached-frame
        world_to_attached = (
            self.lookup_transform_tuple(
                WORLD_FRAME,
                attached_frame,
            )
        )

        # T_world_rod
        world_to_rod = transform_compose(
            world_to_attached,
            attached_to_rod,
        )

        world_rod = copy.deepcopy(attached_object)
        world_rod.header.frame_id = WORLD_FRAME
        world_rod.pose = transform_tuple_to_pose(
            world_to_rod
        )
        world_rod.operation = CollisionObject.ADD

        remove_object = CollisionObject()
        remove_object.id = ROD_ID
        remove_object.operation = CollisionObject.REMOVE

        remove_attached = AttachedCollisionObject()
        remove_attached.link_name = attached_frame
        remove_attached.object = remove_object

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        scene.robot_state.attached_collision_objects.append(
            remove_attached
        )
        scene.world.collision_objects.append(
            world_rod
        )
        scene.object_colors.append(
            self.make_rod_color()
        )

        self.apply_scene(
            scene,
            "place rod while preserving pose",
        )

        verification_scene = self.query_scene()

        verified_world = self.find_world_rod(
            verification_scene
        )
        verified_attached = self.find_attached_rod(
            verification_scene
        )

        if verified_world is None:
            raise RuntimeError(
                "Rod was not found in world after place"
            )

        if verified_attached is not None:
            raise RuntimeError(
                "Rod remained attached after place"
            )

        pose = verified_world.pose

        self.get_logger().info(
            "Rod placed without snapping. "
            "Final world pose: "
            f"xyz=({pose.position.x:.4f}, "
            f"{pose.position.y:.4f}, "
            f"{pose.position.z:.4f}), "
            f"quaternion=({pose.orientation.x:.4f}, "
            f"{pose.orientation.y:.4f}, "
            f"{pose.orientation.z:.4f}, "
            f"{pose.orientation.w:.4f})."
        )

    def stow_rod_preserving_pose(self):
        """
        Transfer the rod to the mobile-manipulator loading support without
        changing its current world pose.

        Normal starting state:
            Rod attached to anobot_grasp_frame.

        Also supported:
            Rod currently present as a world collision object.

        Final state:
            Rod attached to support_link and moving together with the AGV.

        No snapping is performed. Any position or orientation mismatch
        relative to the loading support is retained.
        """

        current_scene = self.query_scene()

        support_link = self.get_support_link()

        existing_attached = self.find_attached_rod(
            current_scene
        )

        existing_world = self.find_world_rod(
            current_scene
        )

        # ---------------------------------------------------------------
        # Rod is already on the mobile support.
        # ---------------------------------------------------------------

        if (
            existing_attached is not None
            and existing_attached.link_name == support_link
        ):
            self.get_logger().info(
                f"Rod is already stowed on '{support_link}'."
            )
            return

        # ---------------------------------------------------------------
        # Determine the rod's current pose in the world frame.
        # ---------------------------------------------------------------

        if existing_attached is not None:
            source_link = existing_attached.link_name
            source_object = existing_attached.object

            # T_source_rod
            source_to_rod = pose_to_transform_tuple(
                source_object.pose
            )

            # T_world_source
            world_to_source = self.lookup_transform_tuple(
                WORLD_FRAME,
                source_link,
            )

            # T_world_rod =
            #     T_world_source * T_source_rod
            world_to_rod = transform_compose(
                world_to_source,
                source_to_rod,
            )

            self.get_logger().info(
                "Transferring rod attachment from "
                f"'{source_link}' to mobile support "
                f"'{support_link}'."
            )

        elif existing_world is not None:
            source_link = None
            source_object = existing_world

            # Obtain T_world_rod, accounting for the object's
            # current header frame.
            world_to_rod = self.get_object_pose_in_frame(
                existing_world,
                WORLD_FRAME,
            )

            self.get_logger().info(
                "Transferring rod from the world to mobile support "
                f"'{support_link}'."
            )

        else:
            raise RuntimeError(
                "Cannot stow: rod is neither attached to the "
                "gripper nor present in the world"
            )

        # ---------------------------------------------------------------
        # Calculate the rod pose relative to the mobile support link.
        #
        # T_support_rod =
        #     inverse(T_world_support) * T_world_rod
        # ---------------------------------------------------------------

        world_to_support = self.lookup_transform_tuple(
            WORLD_FRAME,
            support_link,
        )

        support_to_world = transform_inverse(
            world_to_support
        )

        support_to_rod = transform_compose(
            support_to_world,
            world_to_rod,
        )

        # ---------------------------------------------------------------
        # Construct the new support-attached rod.
        # ---------------------------------------------------------------

        stowed_object = copy.deepcopy(
            source_object
        )

        stowed_object.header.frame_id = support_link
        stowed_object.pose = transform_tuple_to_pose(
            support_to_rod
        )
        stowed_object.operation = CollisionObject.ADD

        stowed_attached = AttachedCollisionObject()
        stowed_attached.link_name = support_link
        stowed_attached.object = stowed_object

        # Only support_link is intentionally allowed to touch the rod.
        #
        # With the present model base_footprint has no collision
        # geometry, but this documents the ownership relationship.
        stowed_attached.touch_links = [
            support_link,
        ]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        # ---------------------------------------------------------------
        # Remove the rod's previous representation.
        # ---------------------------------------------------------------

        if existing_attached is not None:
            remove_old_object = CollisionObject()
            remove_old_object.id = ROD_ID
            remove_old_object.operation = (
                CollisionObject.REMOVE
            )

            remove_old_attached = AttachedCollisionObject()
            remove_old_attached.link_name = (
                existing_attached.link_name
            )
            remove_old_attached.object = (
                remove_old_object
            )

            scene.robot_state.attached_collision_objects.append(
                remove_old_attached
            )

        if existing_world is not None:
            remove_world = CollisionObject()
            remove_world.header.frame_id = (
                existing_world.header.frame_id
                or WORLD_FRAME
            )
            remove_world.id = ROD_ID
            remove_world.operation = CollisionObject.REMOVE

            scene.world.collision_objects.append(
                remove_world
            )

        # Removal entries are added before the replacement attachment.
        scene.robot_state.attached_collision_objects.append(
            stowed_attached
        )

        scene.object_colors.append(
            self.make_rod_color()
        )

        self.apply_scene(
            scene,
            "stow rod on mobile support while preserving pose",
        )

        # ---------------------------------------------------------------
        # Verify the final planning-scene state.
        # ---------------------------------------------------------------

        verification_scene = self.query_scene()

        verified_attached = self.find_attached_rod(
            verification_scene
        )

        verified_world = self.find_world_rod(
            verification_scene
        )

        if verified_attached is None:
            raise RuntimeError(
                "Rod was not found under attached objects "
                "after stowing"
            )

        if verified_attached.link_name != support_link:
            raise RuntimeError(
                "Rod was stowed on unexpected link "
                f"'{verified_attached.link_name}' instead of "
                f"'{support_link}'"
            )

        if verified_world is not None:
            raise RuntimeError(
                "Rod exists as both a world object and an "
                "attached object after stowing"
            )

        pose = verified_attached.object.pose

        self.get_logger().info(
            "Rod stowed on mobile support without snapping. "
            f"Support link: '{support_link}'. "
            "Relative rod pose: "
            f"xyz=({pose.position.x:.4f}, "
            f"{pose.position.y:.4f}, "
            f"{pose.position.z:.4f}), "
            f"quaternion=({pose.orientation.x:.4f}, "
            f"{pose.orientation.y:.4f}, "
            f"{pose.orientation.z:.4f}, "
            f"{pose.orientation.w:.4f})."
        )

    def remove_rod(self):
        current_scene = self.query_scene()

        existing_world = self.find_world_rod(
            current_scene
        )
        existing_attached = self.find_attached_rod(
            current_scene
        )

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        if existing_world is not None:
            remove_world = CollisionObject()
            remove_world.header.frame_id = (
                existing_world.header.frame_id
                or WORLD_FRAME
            )
            remove_world.id = ROD_ID
            remove_world.operation = CollisionObject.REMOVE

            scene.world.collision_objects.append(
                remove_world
            )

        if existing_attached is not None:
            remove_attached_object = CollisionObject()
            remove_attached_object.id = ROD_ID
            remove_attached_object.operation = (
                CollisionObject.REMOVE
            )

            remove_attached = AttachedCollisionObject()
            remove_attached.link_name = (
                existing_attached.link_name
            )
            remove_attached.object = (
                remove_attached_object
            )

            scene.robot_state.attached_collision_objects.append(
                remove_attached
            )

        if (
            existing_world is None
            and existing_attached is None
        ):
            self.get_logger().info(
                "Rod is already absent from the planning scene."
            )
            return

        self.apply_scene(
            scene,
            "remove rod",
        )

        verification_scene = self.query_scene()

        if self.find_world_rod(verification_scene):
            raise RuntimeError(
                "World rod still exists after remove"
            )

        if self.find_attached_rod(verification_scene):
            raise RuntimeError(
                "Attached rod still exists after remove"
            )

        self.get_logger().info(
            "Rod removed from planning scene."
        )

    def print_status(self):
        scene = self.query_scene()

        world_rod = self.find_world_rod(scene)
        attached_rod = self.find_attached_rod(scene)

        if attached_rod is not None:
            pose = attached_rod.object.pose
            support_link = self.get_support_link()
            attach_link = self.get_attach_link()

            if attached_rod.link_name == support_link:
                state_name = "ON_MOBILE_SUPPORT"
            elif attached_rod.link_name == attach_link:
                state_name = "ATTACHED_TO_GRIPPER"
            else:
                state_name = "ATTACHED_TO_UNKNOWN_LINK"

            self.get_logger().info(
                f"Rod state: {state_name}; "
                f"attached to '{attached_rod.link_name}', "
                f"relative xyz=({pose.position.x:.4f}, "
                f"{pose.position.y:.4f}, "
                f"{pose.position.z:.4f}), "
                f"quaternion=({pose.orientation.x:.4f}, "
                f"{pose.orientation.y:.4f}, "
                f"{pose.orientation.z:.4f}, "
                f"{pose.orientation.w:.4f})."
            )

        elif world_rod is not None:
            pose = world_rod.pose

            self.get_logger().info(
                "Rod state: PLACED_IN_WORLD; "
                f"frame='{world_rod.header.frame_id}', "
                f"xyz=({pose.position.x:.4f}, "
                f"{pose.position.y:.4f}, "
                f"{pose.position.z:.4f}), "
                f"quaternion=({pose.orientation.x:.4f}, "
                f"{pose.orientation.y:.4f}, "
                f"{pose.orientation.z:.4f}, "
                f"{pose.orientation.w:.4f})."
            )

        else:
            self.get_logger().info(
                "Rod state: NOT_PRESENT."
            )

    def close(self):
        """
        Explicitly stop and release the TF listener before destroying
        the ROS node and shutting down rclpy.
        """
        if getattr(self, "tf_listener", None) is None:
            return
        listener = self.tf_listener
        try:
            listener.unregister()
        except Exception as exc:
            self.get_logger().warning(f"TF listener unregister warning: {exc}")
        # With spin_thread=True, TransformListener owns an internal executor.
        # Shut it down before destroying the parent node.
        executor = getattr(listener, "executor", None)
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except Exception as exc:
                self.get_logger().warning(f"TF listener executor shutdown warning: {exc}")
        self.tf_listener = None


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = RodSceneManager()
    except Exception as exc:
        error_node = None

        try:
            if rclpy.ok():
                error_node = rclpy.create_node(
                    "rod_scene_manager_error"
                )
                error_node.get_logger().error(str(exc))
        finally:
            if error_node is not None:
                error_node.destroy_node()
    finally:
        if node is not None:
            node.close()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()