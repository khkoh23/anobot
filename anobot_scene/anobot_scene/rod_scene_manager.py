#!/usr/bin/env python3

import copy
import math
import os
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose

from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    ObjectColor,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene

from shape_msgs.msg import (
    Mesh,
    MeshTriangle,
    SolidPrimitive,
)

from tf2_ros import (
    Buffer,
    TransformException,
    TransformListener,
)


try:
    import trimesh
except ImportError as exc:
    raise ImportError(
        "The Python package 'trimesh' is required for mesh geometry. "
        "Install it in the ROS Python environment."
    ) from exc


# ============================================================================
# Object and frame configuration
# ============================================================================

ROD_ID = "anodizing_rod"

WORLD_FRAME = "world"

# Temporary mobile fixture/support.
DEFAULT_SUPPORT_LINK = "base_footprint"

# Robot grasp reference.
DEFAULT_ATTACH_LINK = "anobot_grasp_frame"


# ============================================================================
# Cylinder fallback geometry
# ============================================================================

ROD_LENGTH = 1.008
ROD_RADIUS = 0.014


# ============================================================================
# Default mesh configuration
# ============================================================================

DEFAULT_MESH_PACKAGE = "anobot_scene"

DEFAULT_MESH_RELATIVE_PATH = os.path.join(
    "meshes",
    "anodizing_load.STL",
)

# Use 0.001 for an STL exported in millimetres.
# Use 1.0 for a mesh exported in metres.
DEFAULT_MESH_SCALE = 0.001


# ============================================================================
# Quaternion and transform utilities
#
# A transform tuple is represented as:
#
#     (
#         (translation_x, translation_y, translation_z),
#         (quaternion_x, quaternion_y, quaternion_z, quaternion_w),
#     )
# ============================================================================

def quaternion_normalize(quaternion):
    x, y, z, w = quaternion

    norm = math.sqrt(
        x * x
        + y * y
        + z * z
        + w * w
    )

    if norm < 1.0e-12:
        return 0.0, 0.0, 0.0, 1.0

    return (
        x / norm,
        y / norm,
        z / norm,
        w / norm,
    )


def quaternion_conjugate(quaternion):
    x, y, z, w = quaternion

    return -x, -y, -z, w


def quaternion_multiply_raw(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quaternion_multiply(q1, q2):
    return quaternion_normalize(
        quaternion_multiply_raw(q1, q2)
    )


def rotate_vector(quaternion, vector):
    quaternion = quaternion_normalize(
        quaternion
    )

    vector_quaternion = (
        vector[0],
        vector[1],
        vector[2],
        0.0,
    )

    rotated = quaternion_multiply_raw(
        quaternion_multiply_raw(
            quaternion,
            vector_quaternion,
        ),
        quaternion_conjugate(quaternion),
    )

    return (
        rotated[0],
        rotated[1],
        rotated[2],
    )


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


def transform_compose(transform_a_b, transform_b_c):
    """
    Compose two transforms:

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
    Invert a transform:

        T_b_a = inverse(T_a_b)
    """

    translation_a_b, rotation_a_b = transform_a_b

    rotation_b_a = quaternion_conjugate(
        rotation_a_b
    )

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


# ============================================================================
# Rod scene manager
# ============================================================================

class RodSceneManager(Node):

    def __init__(self):
        super().__init__("rod_scene_manager")

        self.loaded_mesh = None

        self.declare_manager_parameters()

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

        # Give TransformListener time to receive the current TF graph.
        time.sleep(0.5)

        operation = self.get_string_parameter(
            "operation"
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

    # ========================================================================
    # Parameters
    # ========================================================================

    def declare_manager_parameters(self):
        self.declare_parameter(
            "operation",
            "status",
        )

        self.declare_parameter(
            "support_link",
            DEFAULT_SUPPORT_LINK,
        )

        self.declare_parameter(
            "attach_link",
            DEFAULT_ATTACH_LINK,
        )

        # Initial pose relative to support_link.
        self.declare_parameter("rod_x", 0.0)
        self.declare_parameter("rod_y", -0.50)
        self.declare_parameter("rod_z", 1.00)

        self.declare_parameter("rod_roll", 0.0)
        self.declare_parameter(
            "rod_pitch",
            math.pi / 2.0,
        )
        self.declare_parameter("rod_yaw", 0.0)

        # Select "mesh" or "cylinder".
        self.declare_parameter(
            "geometry_type",
            "mesh",
        )

        self.declare_parameter(
            "mesh_package",
            DEFAULT_MESH_PACKAGE,
        )

        self.declare_parameter(
            "mesh_relative_path",
            DEFAULT_MESH_RELATIVE_PATH,
        )

        self.declare_parameter(
            "mesh_scale",
            DEFAULT_MESH_SCALE,
        )

    def get_string_parameter(self, name):
        return (
            self.get_parameter(name)
            .get_parameter_value()
            .string_value
        )

    def get_double_parameter(self, name):
        return (
            self.get_parameter(name)
            .get_parameter_value()
            .double_value
        )

    def get_support_link(self):
        return self.get_string_parameter(
            "support_link"
        )

    def get_attach_link(self):
        return self.get_string_parameter(
            "attach_link"
        )

    # ========================================================================
    # Cleanup
    # ========================================================================

    def close(self):
        """
        Explicitly release the TF listener before node destruction.

        This avoids the TransformListener destructor attempting to shut down
        its executor after rclpy has already started destroying ROS handles.
        """

        listener = getattr(
            self,
            "tf_listener",
            None,
        )

        if listener is None:
            return

        try:
            listener.unregister()
        except Exception as exc:
            self.get_logger().debug(
                f"TF listener unregister returned: {exc}"
            )

        executor = getattr(
            listener,
            "executor",
            None,
        )

        if executor is None:
            executor = getattr(
                listener,
                "_executor",
                None,
            )

        if executor is not None:
            try:
                executor.shutdown(
                    timeout_sec=1.0
                )
            except TypeError:
                try:
                    executor.shutdown()
                except Exception:
                    pass
            except Exception:
                pass

        self.tf_listener = None

    # ========================================================================
    # Common pose helpers
    # ========================================================================

    @staticmethod
    def identity_pose():
        pose = Pose()
        pose.orientation.w = 1.0
        return pose

    def get_initial_rod_pose(self):
        pose = Pose()

        pose.position.x = self.get_double_parameter(
            "rod_x"
        )

        pose.position.y = self.get_double_parameter(
            "rod_y"
        )

        pose.position.z = self.get_double_parameter(
            "rod_z"
        )

        quaternion = quaternion_from_rpy(
            self.get_double_parameter("rod_roll"),
            self.get_double_parameter("rod_pitch"),
            self.get_double_parameter("rod_yaw"),
        )

        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        return pose

    # ========================================================================
    # Geometry
    # ========================================================================

    @staticmethod
    def make_cylinder_geometry():
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER

        # MoveIt cylinder dimensions:
        # [height, radius]
        cylinder.dimensions = [
            ROD_LENGTH,
            ROD_RADIUS,
        ]

        return cylinder

    def get_mesh_path(self):
        mesh_package = self.get_string_parameter(
            "mesh_package"
        )

        relative_path = self.get_string_parameter(
            "mesh_relative_path"
        )

        package_share = get_package_share_directory(
            mesh_package
        )

        mesh_path = os.path.join(
            package_share,
            relative_path,
        )

        if not os.path.isfile(mesh_path):
            raise RuntimeError(
                "Anodizing-load mesh does not exist: "
                f"'{mesh_path}'"
            )

        return mesh_path

    def get_mesh_scale(self):
        scale = self.get_double_parameter(
            "mesh_scale"
        )

        if scale <= 0.0:
            raise RuntimeError(
                "mesh_scale must be positive; "
                f"received {scale}"
            )

        return scale

    def load_mesh_from_file(
        self,
        file_path,
        scale,
    ):
        """
        Load a mesh and convert it into shape_msgs/Mesh.

        process=False helps preserve the original CAD origin and geometry.
        """

        loaded = trimesh.load(
            file_path,
            force="mesh",
            process=False,
        )

        if isinstance(loaded, trimesh.Scene):
            geometries = [
                geometry
                for geometry in loaded.geometry.values()
                if geometry is not None
            ]

            if not geometries:
                raise RuntimeError(
                    "Mesh scene contains no geometry: "
                    f"'{file_path}'"
                )

            loaded = trimesh.util.concatenate(
                geometries
            )

        if not isinstance(
            loaded,
            trimesh.Trimesh,
        ):
            raise RuntimeError(
                "Unsupported mesh result for "
                f"'{file_path}': "
                f"{type(loaded).__name__}"
            )

        if len(loaded.vertices) == 0:
            raise RuntimeError(
                f"Mesh has no vertices: '{file_path}'"
            )

        if len(loaded.faces) == 0:
            raise RuntimeError(
                f"Mesh has no faces: '{file_path}'"
            )

        if loaded.faces.shape[1] != 3:
            raise RuntimeError(
                "MoveIt collision meshes require "
                "triangulated faces"
            )

        mesh_message = Mesh()

        for vertex in loaded.vertices:
            point = Point()

            point.x = float(vertex[0]) * scale
            point.y = float(vertex[1]) * scale
            point.z = float(vertex[2]) * scale

            mesh_message.vertices.append(point)

        for face in loaded.faces:
            triangle = MeshTriangle()

            triangle.vertex_indices = [
                int(face[0]),
                int(face[1]),
                int(face[2]),
            ]

            mesh_message.triangles.append(
                triangle
            )

        scaled_bounds = (
            loaded.bounds * scale
        )

        dimensions = (
            scaled_bounds[1]
            - scaled_bounds[0]
        )

        self.get_logger().info(
            "Loaded anodizing-load mesh: "
            f"vertices={len(mesh_message.vertices)}, "
            f"triangles={len(mesh_message.triangles)}, "
            "dimensions="
            f"({dimensions[0]:.4f}, "
            f"{dimensions[1]:.4f}, "
            f"{dimensions[2]:.4f}) m, "
            f"scale={scale}, "
            f"path='{file_path}'."
        )

        return mesh_message

    def get_loaded_mesh(self):
        if self.loaded_mesh is None:
            self.loaded_mesh = self.load_mesh_from_file(
                self.get_mesh_path(),
                self.get_mesh_scale(),
            )

        return copy.deepcopy(
            self.loaded_mesh
        )

    def populate_rod_geometry(
        self,
        collision_object,
    ):
        geometry_type = self.get_string_parameter(
            "geometry_type"
        ).lower()

        if geometry_type == "mesh":
            collision_object.meshes.append(
                self.get_loaded_mesh()
            )

            collision_object.mesh_poses.append(
                self.identity_pose()
            )

        elif geometry_type == "cylinder":
            collision_object.primitives.append(
                self.make_cylinder_geometry()
            )

            collision_object.primitive_poses.append(
                self.identity_pose()
            )

        else:
            raise RuntimeError(
                "Unsupported geometry_type "
                f"'{geometry_type}'. "
                "Use 'mesh' or 'cylinder'."
            )

    @staticmethod
    def make_rod_color():
        color = ObjectColor()
        color.id = ROD_ID

        color.color.r = 0.68
        color.color.g = 0.68
        color.color.b = 0.72
        color.color.a = 1.0

        return color

    # ========================================================================
    # Planning-scene queries
    # ========================================================================

    def query_scene(self):
        request = GetPlanningScene.Request()

        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.ROBOT_STATE
        )

        future = self.get_scene_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        if future.result() is None:
            raise RuntimeError(
                "Timed out while querying planning scene"
            )

        return future.result().scene

    @staticmethod
    def find_world_rod(scene):
        for collision_object in (
            scene.world.collision_objects
        ):
            if collision_object.id == ROD_ID:
                return copy.deepcopy(
                    collision_object
                )

        return None

    @staticmethod
    def find_attached_rod(scene):
        for attached_object in (
            scene.robot_state.attached_collision_objects
        ):
            if attached_object.object.id == ROD_ID:
                return copy.deepcopy(
                    attached_object
                )

        return None

    # ========================================================================
    # TF helpers
    # ========================================================================

    def lookup_transform_tuple(
        self,
        target_frame,
        source_frame,
    ):
        try:
            transform_stamped = (
                self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=3.0),
                )
            )

        except TransformException as exc:
            raise RuntimeError(
                f"Could not transform from "
                f"'{source_frame}' to "
                f"'{target_frame}': {exc}"
            ) from exc

        return transform_msg_to_tuple(
            transform_stamped.transform
        )

    def get_object_pose_in_frame(
        self,
        collision_object,
        target_frame,
    ):
        source_frame = (
            collision_object.header.frame_id
            or WORLD_FRAME
        )

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

    # ========================================================================
    # Apply planning scene
    # ========================================================================

    def apply_scene(
        self,
        scene,
        description,
    ):
        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self.apply_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        if future.result() is None:
            self.get_logger().warning(
                "No immediate service response for "
                f"'{description}'. "
                "The resulting planning scene will "
                "be verified."
            )

        elif not future.result().success:
            self.get_logger().warning(
                "The immediate service response for "
                f"'{description}' was unsuccessful. "
                "The resulting planning scene will "
                "be verified."
            )

        time.sleep(0.20)

    # ========================================================================
    # Add rod to mobile support
    # ========================================================================

    def add_rod(self):
        current_scene = self.query_scene()

        if self.find_world_rod(current_scene):
            raise RuntimeError(
                "Cannot add rod: a world rod already exists. "
                "Run operation:=remove first."
            )

        existing_attached = self.find_attached_rod(
            current_scene
        )

        if existing_attached is not None:
            raise RuntimeError(
                "Cannot add rod: rod is already attached "
                f"to '{existing_attached.link_name}'. "
                "Run operation:=remove first."
            )

        support_link = self.get_support_link()

        rod_object = CollisionObject()
        rod_object.header.frame_id = support_link
        rod_object.id = ROD_ID
        rod_object.operation = CollisionObject.ADD

        # Object-level pose relative to mobile support.
        rod_object.pose = self.get_initial_rod_pose()

        # Add either mesh or cylinder geometry.
        self.populate_rod_geometry(
            rod_object
        )

        attached = AttachedCollisionObject()
        attached.link_name = support_link
        attached.object = rod_object
        attached.touch_links = [
            support_link,
        ]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        scene.robot_state.attached_collision_objects.append(
            attached
        )

        scene.object_colors.append(
            self.make_rod_color()
        )

        self.apply_scene(
            scene,
            "add rod to mobile loading support",
        )

        verification = self.query_scene()

        verified_attached = self.find_attached_rod(
            verification
        )

        verified_world = self.find_world_rod(
            verification
        )

        if verified_attached is None:
            raise RuntimeError(
                "Rod was not found under attached "
                "objects after add"
            )

        if verified_world is not None:
            raise RuntimeError(
                "Rod exists as both world and attached "
                "object after add"
            )

        if verified_attached.link_name != support_link:
            raise RuntimeError(
                "Rod was attached to unexpected link "
                f"'{verified_attached.link_name}'"
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

    # ========================================================================
    # Transfer rod to robot gripper
    # ========================================================================

    def attach_rod_preserving_pose(self):
        current_scene = self.query_scene()

        attach_link = self.get_attach_link()

        existing_attached = self.find_attached_rod(
            current_scene
        )

        existing_world = self.find_world_rod(
            current_scene
        )

        if (
            existing_attached is not None
            and existing_attached.link_name == attach_link
        ):
            self.get_logger().info(
                f"Rod is already attached to "
                f"'{attach_link}'."
            )
            return

        if existing_attached is not None:
            source_link = existing_attached.link_name
            source_object = existing_attached.object

            source_to_rod = pose_to_transform_tuple(
                source_object.pose
            )

            world_to_source = (
                self.lookup_transform_tuple(
                    WORLD_FRAME,
                    source_link,
                )
            )

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

            world_to_rod = (
                self.get_object_pose_in_frame(
                    existing_world,
                    WORLD_FRAME,
                )
            )

            self.get_logger().info(
                "Attaching rod from world to "
                f"'{attach_link}'."
            )

        else:
            raise RuntimeError(
                "Cannot attach: rod is neither on the "
                "mobile support nor present in the world"
            )

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

        rod_object = copy.deepcopy(
            source_object
        )

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

        if existing_attached is not None:
            remove_object = CollisionObject()
            remove_object.id = ROD_ID
            remove_object.operation = (
                CollisionObject.REMOVE
            )

            remove_attached = AttachedCollisionObject()
            remove_attached.link_name = (
                existing_attached.link_name
            )
            remove_attached.object = remove_object

            scene.robot_state.attached_collision_objects.append(
                remove_attached
            )

        if existing_world is not None:
            remove_world = CollisionObject()
            remove_world.header.frame_id = (
                existing_world.header.frame_id
                or WORLD_FRAME
            )
            remove_world.id = ROD_ID
            remove_world.operation = (
                CollisionObject.REMOVE
            )

            scene.world.collision_objects.append(
                remove_world
            )

        # Removal entries are sent before replacement attachment.
        scene.robot_state.attached_collision_objects.append(
            new_attached
        )

        self.apply_scene(
            scene,
            "transfer rod to grasp frame "
            "while preserving pose",
        )

        verification = self.query_scene()

        verified_attached = self.find_attached_rod(
            verification
        )

        verified_world = self.find_world_rod(
            verification
        )

        if verified_attached is None:
            raise RuntimeError(
                "Rod was not found under attached "
                "objects after transfer"
            )

        if verified_attached.link_name != attach_link:
            raise RuntimeError(
                "Rod remains attached to unexpected link "
                f"'{verified_attached.link_name}'"
            )

        if verified_world is not None:
            raise RuntimeError(
                "Rod exists as both world and attached "
                "object after transfer"
            )

        pose = verified_attached.object.pose

        self.get_logger().info(
            "Rod transferred to grasp frame without "
            "snapping. Relative pose: "
            f"xyz=({pose.position.x:.4f}, "
            f"{pose.position.y:.4f}, "
            f"{pose.position.z:.4f}), "
            f"quaternion=({pose.orientation.x:.4f}, "
            f"{pose.orientation.y:.4f}, "
            f"{pose.orientation.z:.4f}, "
            f"{pose.orientation.w:.4f})."
        )

    # ========================================================================
    # Place rod into world
    # ========================================================================

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
        attached_frame = (
            attached_rod.link_name
            or attached_object.header.frame_id
        )

        attached_to_rod = pose_to_transform_tuple(
            attached_object.pose
        )

        world_to_attached = (
            self.lookup_transform_tuple(
                WORLD_FRAME,
                attached_frame,
            )
        )

        world_to_rod = transform_compose(
            world_to_attached,
            attached_to_rod,
        )

        world_rod = copy.deepcopy(
            attached_object
        )

        world_rod.header.frame_id = WORLD_FRAME
        world_rod.pose = transform_tuple_to_pose(
            world_to_rod
        )
        world_rod.operation = CollisionObject.ADD

        remove_object = CollisionObject()
        remove_object.id = ROD_ID
        remove_object.operation = (
            CollisionObject.REMOVE
        )

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

        verification = self.query_scene()

        verified_world = self.find_world_rod(
            verification
        )

        verified_attached = self.find_attached_rod(
            verification
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

    # ========================================================================
    # Return rod to mobile support
    # ========================================================================

    def stow_rod_preserving_pose(self):
        current_scene = self.query_scene()

        support_link = self.get_support_link()

        existing_attached = self.find_attached_rod(
            current_scene
        )

        existing_world = self.find_world_rod(
            current_scene
        )

        if (
            existing_attached is not None
            and existing_attached.link_name == support_link
        ):
            self.get_logger().info(
                f"Rod is already stowed on "
                f"'{support_link}'."
            )
            return

        if existing_attached is not None:
            source_link = existing_attached.link_name
            source_object = existing_attached.object

            source_to_rod = pose_to_transform_tuple(
                source_object.pose
            )

            world_to_source = (
                self.lookup_transform_tuple(
                    WORLD_FRAME,
                    source_link,
                )
            )

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

            world_to_rod = (
                self.get_object_pose_in_frame(
                    existing_world,
                    WORLD_FRAME,
                )
            )

            self.get_logger().info(
                "Transferring rod from world to "
                f"mobile support '{support_link}'."
            )

        else:
            raise RuntimeError(
                "Cannot stow: rod is neither attached "
                "nor present in world"
            )

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
        stowed_attached.touch_links = [
            support_link,
        ]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        if existing_attached is not None:
            remove_object = CollisionObject()
            remove_object.id = ROD_ID
            remove_object.operation = (
                CollisionObject.REMOVE
            )

            remove_attached = AttachedCollisionObject()
            remove_attached.link_name = (
                existing_attached.link_name
            )
            remove_attached.object = remove_object

            scene.robot_state.attached_collision_objects.append(
                remove_attached
            )

        if existing_world is not None:
            remove_world = CollisionObject()
            remove_world.header.frame_id = (
                existing_world.header.frame_id
                or WORLD_FRAME
            )
            remove_world.id = ROD_ID
            remove_world.operation = (
                CollisionObject.REMOVE
            )

            scene.world.collision_objects.append(
                remove_world
            )

        scene.robot_state.attached_collision_objects.append(
            stowed_attached
        )

        scene.object_colors.append(
            self.make_rod_color()
        )

        self.apply_scene(
            scene,
            "stow rod on mobile support while "
            "preserving pose",
        )

        verification = self.query_scene()

        verified_attached = self.find_attached_rod(
            verification
        )

        verified_world = self.find_world_rod(
            verification
        )

        if verified_attached is None:
            raise RuntimeError(
                "Rod was not found under attached "
                "objects after stowing"
            )

        if verified_attached.link_name != support_link:
            raise RuntimeError(
                "Rod was stowed on unexpected link "
                f"'{verified_attached.link_name}'"
            )

        if verified_world is not None:
            raise RuntimeError(
                "Rod exists as both world and attached "
                "object after stowing"
            )

        pose = verified_attached.object.pose

        self.get_logger().info(
            "Rod stowed on mobile support without "
            "snapping. "
            f"Support link: '{support_link}'. "
            "Relative pose: "
            f"xyz=({pose.position.x:.4f}, "
            f"{pose.position.y:.4f}, "
            f"{pose.position.z:.4f}), "
            f"quaternion=({pose.orientation.x:.4f}, "
            f"{pose.orientation.y:.4f}, "
            f"{pose.orientation.z:.4f}, "
            f"{pose.orientation.w:.4f})."
        )

    # ========================================================================
    # Remove rod from all possible states
    # ========================================================================

    def remove_rod(self):
        current_scene = self.query_scene()

        existing_world = self.find_world_rod(
            current_scene
        )

        existing_attached = self.find_attached_rod(
            current_scene
        )

        if (
            existing_world is None
            and existing_attached is None
        ):
            self.get_logger().info(
                "Rod is already absent from the "
                "planning scene."
            )
            return

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
            remove_world.operation = (
                CollisionObject.REMOVE
            )

            scene.world.collision_objects.append(
                remove_world
            )

        if existing_attached is not None:
            remove_object = CollisionObject()
            remove_object.id = ROD_ID
            remove_object.operation = (
                CollisionObject.REMOVE
            )

            remove_attached = AttachedCollisionObject()
            remove_attached.link_name = (
                existing_attached.link_name
            )
            remove_attached.object = remove_object

            scene.robot_state.attached_collision_objects.append(
                remove_attached
            )

        self.apply_scene(
            scene,
            "remove rod",
        )

        verification = self.query_scene()

        if self.find_world_rod(verification):
            raise RuntimeError(
                "World rod still exists after remove"
            )

        if self.find_attached_rod(verification):
            raise RuntimeError(
                "Attached rod still exists after remove"
            )

        self.get_logger().info(
            "Rod removed from planning scene."
        )

    # ========================================================================
    # Status
    # ========================================================================

    def print_status(self):
        scene = self.query_scene()

        world_rod = self.find_world_rod(
            scene
        )

        attached_rod = self.find_attached_rod(
            scene
        )

        support_link = self.get_support_link()
        attach_link = self.get_attach_link()

        if attached_rod is not None:
            pose = attached_rod.object.pose

            if attached_rod.link_name == support_link:
                state_name = "ON_MOBILE_SUPPORT"

            elif attached_rod.link_name == attach_link:
                state_name = "ATTACHED_TO_GRIPPER"

            else:
                state_name = (
                    "ATTACHED_TO_UNKNOWN_LINK"
                )

            geometry_description = (
                self.describe_geometry(
                    attached_rod.object
                )
            )

            self.get_logger().info(
                f"Rod state: {state_name}; "
                f"attached to "
                f"'{attached_rod.link_name}', "
                f"geometry={geometry_description}, "
                f"relative xyz="
                f"({pose.position.x:.4f}, "
                f"{pose.position.y:.4f}, "
                f"{pose.position.z:.4f}), "
                f"quaternion="
                f"({pose.orientation.x:.4f}, "
                f"{pose.orientation.y:.4f}, "
                f"{pose.orientation.z:.4f}, "
                f"{pose.orientation.w:.4f})."
            )

        elif world_rod is not None:
            pose = world_rod.pose

            geometry_description = (
                self.describe_geometry(
                    world_rod
                )
            )

            self.get_logger().info(
                "Rod state: PLACED_IN_WORLD; "
                f"frame='{world_rod.header.frame_id}', "
                f"geometry={geometry_description}, "
                f"xyz=({pose.position.x:.4f}, "
                f"{pose.position.y:.4f}, "
                f"{pose.position.z:.4f}), "
                f"quaternion="
                f"({pose.orientation.x:.4f}, "
                f"{pose.orientation.y:.4f}, "
                f"{pose.orientation.z:.4f}, "
                f"{pose.orientation.w:.4f})."
            )

        else:
            self.get_logger().info(
                "Rod state: NOT_PRESENT."
            )

    @staticmethod
    def describe_geometry(collision_object):
        mesh_count = len(
            collision_object.meshes
        )

        primitive_count = len(
            collision_object.primitives
        )

        if mesh_count > 0:
            total_vertices = sum(
                len(mesh.vertices)
                for mesh in collision_object.meshes
            )

            total_triangles = sum(
                len(mesh.triangles)
                for mesh in collision_object.meshes
            )

            return (
                f"mesh[{mesh_count}], "
                f"vertices={total_vertices}, "
                f"triangles={total_triangles}"
            )

        if primitive_count > 0:
            return (
                f"primitive[{primitive_count}]"
            )

        return "none"


# ============================================================================
# Main
# ============================================================================

def main(args=None):
    rclpy.init(args=args)

    node = None
    error_node = None

    try:
        node = RodSceneManager()

    except Exception as exc:
        try:
            if rclpy.ok():
                error_node = rclpy.create_node(
                    "rod_scene_manager_error"
                )

                error_node.get_logger().error(
                    str(exc)
                )

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