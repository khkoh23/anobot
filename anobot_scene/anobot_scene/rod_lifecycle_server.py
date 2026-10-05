#!/usr/bin/env python3

import copy
import math
import threading
from pathlib import Path

import numpy as np
import rclpy
import trimesh

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import Mesh, MeshTriangle
from std_srvs.srv import Trigger
from tf2_ros import (
    Buffer,
    TransformException,
    TransformListener,
)


STATE_NOT_PRESENT = "NOT_PRESENT"
STATE_WORLD_OBJECT = "WORLD_OBJECT"
STATE_ATTACHED = "ATTACHED_TO_GRIPPER"


class RodLifecycleServer(Node):

    def __init__(self):
        super().__init__("rod_lifecycle_server")

        # --------------------------------------------------------------
        # Parameters
        # --------------------------------------------------------------

        self.declare_parameter(
            "rod_object_id",
            "anodizing_rod",
        )
        self.declare_parameter(
            "attach_link",
            "anobot_grasp_frame",
        )
        self.declare_parameter(
            "planning_frame",
            "ur_base",
        )
        self.declare_parameter(
            "rod_workstation_frame",
            "workstation_marker_26_calibrated",
        )
        self.declare_parameter(
            "rod_mesh_file",
            "meshes/anodizing_rod.STL",
        )

        self.declare_parameter("rod_x", 0.060)
        self.declare_parameter("rod_y", 0.125)
        self.declare_parameter("rod_z", -0.005)

        self.declare_parameter("rod_roll_deg", -90.0)
        self.declare_parameter("rod_pitch_deg", -90.0)
        self.declare_parameter("rod_yaw_deg", 0.0)

        self.declare_parameter("rod_scale_x", 1.0)
        self.declare_parameter("rod_scale_y", 1.0)
        self.declare_parameter("rod_scale_z", 1.0)

        self.declare_parameter("tf_timeout_s", 3.0)

        self.declare_parameter(
            "touch_links",
            [
                "anobot_grasp_frame",
                "anobot_tool_link",
                "end_effector_mount",
                "tool_mount",
                "ur_tool0",
            ],
        )

        # --------------------------------------------------------------
        # Internal lifecycle state
        #
        # This node is the sole owner of the rod state.
        # Call reset_rod_on_workstation once after every server startup.
        # --------------------------------------------------------------

        self.operation_lock = threading.Lock()

        self.rod_state_name = STATE_NOT_PRESENT
        self.world_rod = None
        self.attached_rod = None

        self.callback_group = ReentrantCallbackGroup()

        # --------------------------------------------------------------
        # ApplyPlanningScene client
        # --------------------------------------------------------------

        self.apply_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            "Waiting for /apply_planning_scene"
        )

        if not self.apply_client.wait_for_service(
            timeout_sec=15.0
        ):
            raise RuntimeError(
                "/apply_planning_scene is unavailable"
            )

        # --------------------------------------------------------------
        # Dedicated TF listener
        # --------------------------------------------------------------

        self.tf_node = Node(
            "rod_lifecycle_tf_listener",
            use_global_arguments=False,
        )

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.tf_node,
            spin_thread=True,
        )

        # --------------------------------------------------------------
        # Load rod geometry once
        # --------------------------------------------------------------

        self.rod_mesh = self.load_rod_mesh()

        # --------------------------------------------------------------
        # Services
        # --------------------------------------------------------------

        self.status_service = self.create_service(
            Trigger,
            "/scene/rod_status",
            self.status_callback,
            callback_group=self.callback_group,
        )

        self.reset_service = self.create_service(
            Trigger,
            "/scene/reset_rod_on_workstation",
            self.reset_callback,
            callback_group=self.callback_group,
        )

        self.attach_service = self.create_service(
            Trigger,
            "/scene/attach_rod",
            self.attach_callback,
            callback_group=self.callback_group,
        )

        self.detach_service = self.create_service(
            Trigger,
            "/scene/detach_rod",
            self.detach_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            "Rod lifecycle server is ready"
        )
        self.get_logger().warning(
            "Rod state starts as NOT_PRESENT. Call "
            "/scene/reset_rod_on_workstation once."
        )
        self.get_logger().info(
            "Services:\n"
            "  /scene/rod_status\n"
            "  /scene/reset_rod_on_workstation\n"
            "  /scene/attach_rod\n"
            "  /scene/detach_rod"
        )

    # ------------------------------------------------------------------
    # Quaternion and matrix helpers
    # ------------------------------------------------------------------

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

    @staticmethod
    def quaternion_to_matrix(quaternion):
        x = float(quaternion.x)
        y = float(quaternion.y)
        z = float(quaternion.z)
        w = float(quaternion.w)

        norm = math.sqrt(
            x * x + y * y + z * z + w * w
        )

        if norm < 1.0e-12:
            raise RuntimeError(
                "Cannot convert a zero quaternion"
            )

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        return np.array([
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ])

    @staticmethod
    def matrix_to_quaternion(rotation):
        trace = float(np.trace(rotation))

        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0

            qx = (
                rotation[2, 1] - rotation[1, 2]
            ) / s
            qy = (
                rotation[0, 2] - rotation[2, 0]
            ) / s
            qz = (
                rotation[1, 0] - rotation[0, 1]
            ) / s
            qw = 0.25 * s

        elif (
            rotation[0, 0] > rotation[1, 1]
            and rotation[0, 0] > rotation[2, 2]
        ):
            s = math.sqrt(
                1.0
                + rotation[0, 0]
                - rotation[1, 1]
                - rotation[2, 2]
            ) * 2.0

            qx = 0.25 * s
            qy = (
                rotation[0, 1] + rotation[1, 0]
            ) / s
            qz = (
                rotation[0, 2] + rotation[2, 0]
            ) / s
            qw = (
                rotation[2, 1] - rotation[1, 2]
            ) / s

        elif rotation[1, 1] > rotation[2, 2]:
            s = math.sqrt(
                1.0
                + rotation[1, 1]
                - rotation[0, 0]
                - rotation[2, 2]
            ) * 2.0

            qx = (
                rotation[0, 1] + rotation[1, 0]
            ) / s
            qy = 0.25 * s
            qz = (
                rotation[1, 2] + rotation[2, 1]
            ) / s
            qw = (
                rotation[0, 2] - rotation[2, 0]
            ) / s

        else:
            s = math.sqrt(
                1.0
                + rotation[2, 2]
                - rotation[0, 0]
                - rotation[1, 1]
            ) * 2.0

            qx = (
                rotation[0, 2] + rotation[2, 0]
            ) / s
            qy = (
                rotation[1, 2] + rotation[2, 1]
            ) / s
            qz = 0.25 * s
            qw = (
                rotation[1, 0] - rotation[0, 1]
            ) / s

        quaternion = np.array(
            [qx, qy, qz, qw],
            dtype=float,
        )

        quaternion /= np.linalg.norm(quaternion)

        return quaternion

    @classmethod
    def pose_to_matrix(cls, pose):
        matrix = np.eye(4)

        matrix[:3, :3] = (
            cls.quaternion_to_matrix(
                pose.orientation
            )
        )

        matrix[:3, 3] = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ]

        return matrix

    @classmethod
    def transform_to_matrix(cls, transform):
        matrix = np.eye(4)

        matrix[:3, :3] = (
            cls.quaternion_to_matrix(
                transform.rotation
            )
        )

        matrix[:3, 3] = [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ]

        return matrix

    @classmethod
    def matrix_to_pose(cls, matrix):
        pose = Pose()

        pose.position.x = float(matrix[0, 3])
        pose.position.y = float(matrix[1, 3])
        pose.position.z = float(matrix[2, 3])

        quaternion = cls.matrix_to_quaternion(
            matrix[:3, :3]
        )

        pose.orientation.x = float(quaternion[0])
        pose.orientation.y = float(quaternion[1])
        pose.orientation.z = float(quaternion[2])
        pose.orientation.w = float(quaternion[3])

        return pose

    def transform_pose(
        self,
        pose,
        target_frame,
        source_frame,
    ):
        if target_frame == source_frame:
            return copy.deepcopy(pose)

        timeout = float(
            self.get_parameter(
                "tf_timeout_s"
            ).value
        )

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=timeout),
            )

        except TransformException as error:
            raise RuntimeError(
                f"Cannot transform {source_frame} "
                f"to {target_frame}: {error}"
            ) from error

        target_to_source = (
            self.transform_to_matrix(
                transform.transform
            )
        )

        source_to_object = self.pose_to_matrix(
            pose
        )

        return self.matrix_to_pose(
            target_to_source @ source_to_object
        )

    # ------------------------------------------------------------------
    # Rod geometry
    # ------------------------------------------------------------------

    @staticmethod
    def identity_pose():
        pose = Pose()
        pose.orientation.w = 1.0
        return pose

    def workstation_rod_pose(self):
        pose = Pose()

        pose.position.x = float(
            self.get_parameter("rod_x").value
        )
        pose.position.y = float(
            self.get_parameter("rod_y").value
        )
        pose.position.z = float(
            self.get_parameter("rod_z").value
        )

        roll = math.radians(
            float(
                self.get_parameter(
                    "rod_roll_deg"
                ).value
            )
        )
        pitch = math.radians(
            float(
                self.get_parameter(
                    "rod_pitch_deg"
                ).value
            )
        )
        yaw = math.radians(
            float(
                self.get_parameter(
                    "rod_yaw_deg"
                ).value
            )
        )

        qx, qy, qz, qw = self.quaternion_from_rpy(
            roll,
            pitch,
            yaw,
        )

        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        return pose

    def load_rod_mesh(self):
        mesh_path = (
            Path(
                get_package_share_directory(
                    "anobot_scene"
                )
            )
            / self.get_parameter(
                "rod_mesh_file"
            ).value
        )

        if not mesh_path.is_file():
            raise FileNotFoundError(
                f"Rod mesh not found: {mesh_path}"
            )

        loaded = trimesh.load(
            str(mesh_path),
            force="mesh",
            process=False,
        )

        if isinstance(loaded, trimesh.Scene):
            geometries = tuple(
                loaded.geometry.values()
            )

            if not geometries:
                raise RuntimeError(
                    "Rod mesh contains no geometry"
                )

            loaded = trimesh.util.concatenate(
                geometries
            )

        if not isinstance(
            loaded,
            trimesh.Trimesh,
        ):
            raise RuntimeError(
                "Unsupported rod mesh type: "
                f"{type(loaded).__name__}"
            )

        scale = np.array([
            float(
                self.get_parameter(
                    "rod_scale_x"
                ).value
            ),
            float(
                self.get_parameter(
                    "rod_scale_y"
                ).value
            ),
            float(
                self.get_parameter(
                    "rod_scale_z"
                ).value
            ),
        ])

        vertices = (
            np.asarray(
                loaded.vertices,
                dtype=float,
            )
            * scale
        )

        faces = np.asarray(
            loaded.faces,
            dtype=int,
        )

        if len(vertices) == 0 or len(faces) == 0:
            raise RuntimeError(
                "Rod mesh is empty"
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

        dimensions = (
            vertices.max(axis=0)
            - vertices.min(axis=0)
        )

        self.get_logger().info(
            "Loaded lifecycle-owned rod mesh: "
            f"{len(vertices)} vertices, "
            f"{len(faces)} triangles, "
            f"dimensions="
            f"({dimensions[0]:.4f}, "
            f"{dimensions[1]:.4f}, "
            f"{dimensions[2]:.4f}) m"
        )

        return mesh

    # ------------------------------------------------------------------
    # Planning-scene update
    # ------------------------------------------------------------------

    async def apply_scene(self, scene, description):
        request = ApplyPlanningScene.Request()
        request.scene = scene

        result = await self.apply_client.call_async(
            request
        )

        if result is None:
            raise RuntimeError(
                f"No response while applying: "
                f"{description}"
            )

        if not result.success:
            self.get_logger().warning(
                "MoveIt returned unsuccessful for "
                f"'{description}', but the lifecycle "
                "state will be updated because previous "
                "tests showed the scene diff is still applied"
            )

    # ------------------------------------------------------------------
    # Status service
    # ------------------------------------------------------------------

    async def status_callback(
        self,
        request,
        response,
    ):
        del request

        response.success = True

        if self.rod_state_name == STATE_WORLD_OBJECT:
            if self.world_rod is None:
                response.success = False
                response.message = (
                    "Internal error: WORLD_OBJECT "
                    "without stored world rod"
                )
            else:
                response.message = (
                    "Rod state: WORLD_OBJECT; "
                    f"frame={self.world_rod.header.frame_id}"
                )

        elif self.rod_state_name == STATE_ATTACHED:
            if self.attached_rod is None:
                response.success = False
                response.message = (
                    "Internal error: ATTACHED_TO_GRIPPER "
                    "without stored attached rod"
                )
            else:
                response.message = (
                    "Rod state: ATTACHED_TO_GRIPPER; "
                    f"link={self.attached_rod.link_name}; "
                    f"frame="
                    f"{self.attached_rod.object.header.frame_id}"
                )

        else:
            response.message = (
                "Rod state: NOT_PRESENT"
            )

        return response

    # ------------------------------------------------------------------
    # Reset service
    # ------------------------------------------------------------------

    async def reset_callback(
        self,
        request,
        response,
    ):
        del request

        if not self.operation_lock.acquire(
            blocking=False
        ):
            response.success = False
            response.message = (
                "Another rod operation is active"
            )
            return response

        try:
            rod_id = self.get_parameter(
                "rod_object_id"
            ).value

            workstation_frame = self.get_parameter(
                "rod_workstation_frame"
            ).value

            # ----------------------------------------------------------
            # Stage 1: remove current lifecycle-owned copy.
            # ----------------------------------------------------------

            remove_scene = PlanningScene()
            remove_scene.is_diff = True
            remove_scene.robot_state.is_diff = True

            if self.world_rod is not None:
                remove_world = CollisionObject()
                remove_world.header.frame_id = (
                    self.world_rod.header.frame_id
                    or workstation_frame
                )
                remove_world.id = rod_id
                remove_world.operation = (
                    CollisionObject.REMOVE
                )

                remove_scene.world.collision_objects.append(
                    remove_world
                )

            if self.attached_rod is not None:
                remove_object = CollisionObject()
                remove_object.id = rod_id
                remove_object.operation = (
                    CollisionObject.REMOVE
                )

                remove_attached = (
                    AttachedCollisionObject()
                )
                remove_attached.link_name = (
                    self.attached_rod.link_name
                )
                remove_attached.object = (
                    remove_object
                )

                remove_scene.robot_state\
                    .attached_collision_objects\
                    .append(remove_attached)

            if (
                self.world_rod is not None
                or self.attached_rod is not None
            ):
                await self.apply_scene(
                    remove_scene,
                    "remove previous rod",
                )

            self.world_rod = None
            self.attached_rod = None
            self.rod_state_name = STATE_NOT_PRESENT

            # ----------------------------------------------------------
            # Stage 2: add one canonical workstation rod.
            # ----------------------------------------------------------

            rod = CollisionObject()

            rod.header.stamp = (
                self.get_clock().now().to_msg()
            )
            rod.header.frame_id = (
                workstation_frame
            )

            rod.id = rod_id
            rod.pose = self.workstation_rod_pose()

            rod.meshes = [
                copy.deepcopy(self.rod_mesh)
            ]
            rod.mesh_poses = [
                self.identity_pose()
            ]

            rod.operation = CollisionObject.ADD

            add_scene = PlanningScene()
            add_scene.is_diff = True
            add_scene.world.collision_objects.append(
                rod
            )

            await self.apply_scene(
                add_scene,
                "reset rod on workstation",
            )

            self.world_rod = copy.deepcopy(rod)
            self.attached_rod = None
            self.rod_state_name = STATE_WORLD_OBJECT

            response.success = True
            response.message = (
                "Rod reset on workstation as the "
                "sole lifecycle-owned world object"
            )

            self.get_logger().info(
                response.message
            )

        except Exception as error:
            response.success = False
            response.message = str(error)

            self.get_logger().error(
                f"Reset rod failed: {error}"
            )

        finally:
            self.operation_lock.release()

        return response

    # ------------------------------------------------------------------
    # Attach service
    # ------------------------------------------------------------------

    async def attach_callback(
        self,
        request,
        response,
    ):
        del request

        if not self.operation_lock.acquire(
            blocking=False
        ):
            response.success = False
            response.message = (
                "Another rod operation is active"
            )
            return response

        try:
            if self.rod_state_name == STATE_ATTACHED:
                response.success = True
                response.message = (
                    "Rod is already attached"
                )
                return response

            if (
                self.rod_state_name
                != STATE_WORLD_OBJECT
                or self.world_rod is None
            ):
                response.success = False
                response.message = (
                    "Cannot attach rod from state "
                    f"{self.rod_state_name}. "
                    "Call reset_rod_on_workstation first."
                )
                return response

            attach_link = self.get_parameter(
                "attach_link"
            ).value

            source_frame = (
                self.world_rod.header.frame_id
            )

            if not source_frame:
                raise RuntimeError(
                    "World rod has an empty frame"
                )

            attached_object = copy.deepcopy(
                self.world_rod
            )

            attached_object.header.stamp = (
                self.get_clock().now().to_msg()
            )
            attached_object.header.frame_id = (
                attach_link
            )

            attached_object.pose = (
                self.transform_pose(
                    self.world_rod.pose,
                    attach_link,
                    source_frame,
                )
            )

            attached_object.operation = (
                CollisionObject.ADD
            )

            attached = AttachedCollisionObject()
            attached.link_name = attach_link
            attached.object = attached_object
            attached.touch_links = list(
                self.get_parameter(
                    "touch_links"
                ).value
            )

            remove_world = CollisionObject()
            remove_world.header.frame_id = (
                source_frame
            )
            remove_world.id = (
                self.world_rod.id
            )
            remove_world.operation = (
                CollisionObject.REMOVE
            )

            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True

            scene.world.collision_objects.append(
                remove_world
            )
            scene.robot_state\
                .attached_collision_objects\
                .append(attached)

            await self.apply_scene(
                scene,
                "attach rod to gripper",
            )

            self.world_rod = None
            self.attached_rod = copy.deepcopy(
                attached
            )
            self.rod_state_name = STATE_ATTACHED

            response.success = True
            response.message = (
                "Rod attached to "
                f"{attach_link} while preserving its pose"
            )

            self.get_logger().info(
                response.message
            )

        except Exception as error:
            response.success = False
            response.message = str(error)

            self.get_logger().error(
                f"Attach rod failed: {error}"
            )

        finally:
            self.operation_lock.release()

        return response

    # ------------------------------------------------------------------
    # Detach service
    # ------------------------------------------------------------------

    async def detach_callback(
        self,
        request,
        response,
    ):
        del request

        if not self.operation_lock.acquire(
            blocking=False
        ):
            response.success = False
            response.message = (
                "Another rod operation is active"
            )
            return response

        try:
            if self.rod_state_name == STATE_WORLD_OBJECT:
                response.success = True
                response.message = (
                    "Rod is already a world object"
                )
                return response

            if (
                self.rod_state_name != STATE_ATTACHED
                or self.attached_rod is None
            ):
                response.success = False
                response.message = (
                    "Cannot detach rod from state "
                    f"{self.rod_state_name}"
                )
                return response

            planning_frame = self.get_parameter(
                "planning_frame"
            ).value

            attach_link = (
                self.attached_rod.link_name
                or self.attached_rod
                .object.header.frame_id
            )

            if not attach_link:
                raise RuntimeError(
                    "Attached rod has no frame"
                )

            world_object = copy.deepcopy(
                self.attached_rod.object
            )

            world_object.header.stamp = (
                self.get_clock().now().to_msg()
            )
            world_object.header.frame_id = (
                planning_frame
            )

            world_object.pose = (
                self.transform_pose(
                    self.attached_rod.object.pose,
                    planning_frame,
                    attach_link,
                )
            )

            world_object.operation = (
                CollisionObject.ADD
            )

            remove_object = CollisionObject()
            remove_object.id = (
                self.attached_rod.object.id
            )
            remove_object.operation = (
                CollisionObject.REMOVE
            )

            remove_attached = (
                AttachedCollisionObject()
            )
            remove_attached.link_name = attach_link
            remove_attached.object = remove_object

            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True

            scene.robot_state\
                .attached_collision_objects\
                .append(remove_attached)

            scene.world.collision_objects.append(
                world_object
            )

            await self.apply_scene(
                scene,
                "detach rod into world",
            )

            self.attached_rod = None
            self.world_rod = copy.deepcopy(
                world_object
            )
            self.rod_state_name = STATE_WORLD_OBJECT

            response.success = True
            response.message = (
                "Rod detached into frame "
                f"{planning_frame} while preserving "
                "its pose"
            )

            self.get_logger().info(
                response.message
            )

        except Exception as error:
            response.success = False
            response.message = str(error)

            self.get_logger().error(
                f"Detach rod failed: {error}"
            )

        finally:
            self.operation_lock.release()

        return response

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def close(self):
        try:
            self.tf_listener.unregister()
        except Exception:
            pass

        try:
            self.tf_node.destroy_node()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)

    node = None
    executor = None

    try:
        node = RodLifecycleServer()

        executor = MultiThreadedExecutor(
            num_threads=4
        )

        executor.add_node(node)
        executor.spin()

    except KeyboardInterrupt:
        pass

    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"Rod lifecycle server failed: "
                f"{error}"
            )
        else:
            print(
                f"Rod lifecycle server failed: "
                f"{error}"
            )

    finally:
        if executor is not None:
            try:
                executor.shutdown(
                    timeout_sec=1.0
                )
            except Exception:
                pass

        if node is not None:
            node.close()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()