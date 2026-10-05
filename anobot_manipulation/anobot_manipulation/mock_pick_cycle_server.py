#!/usr/bin/env python3

import threading

import rclpy

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class MockPickCycleServer(Node):

    def __init__(self):
        super().__init__("mock_pick_cycle_server")

        self.declare_parameter(
            "mock_hardware",
            True,
        )

        self.declare_parameter(
            "service_timeout_s",
            30.0,
        )

        self.cycle_lock = threading.Lock()
        self.callback_group = ReentrantCallbackGroup()

        self.service_names = {
            "reset_rod": (
                "/scene/reset_rod_on_workstation"
            ),
            "rod_status": (
                "/scene/rod_status"
            ),
            "plan_pregrasp": (
                "/manipulation/plan_pregrasp"
            ),
            "execute_pregrasp": (
                "/manipulation/execute_pregrasp"
            ),
            "plan_grasp": (
                "/manipulation/plan_grasp_linear"
            ),
            "execute_grasp": (
                "/manipulation/execute_grasp_linear"
            ),
            "attach_rod": (
                "/scene/attach_rod"
            ),
            "plan_retreat": (
                "/manipulation/plan_retreat_linear"
            ),
            "execute_retreat": (
                "/manipulation/execute_retreat_linear"
            ),
        }

        self.service_clients = {}

        for key, service_name in (
            self.service_names.items()
        ):
            self.service_clients[key] = self.create_client(
                Trigger,
                service_name,
                callback_group=self.callback_group,
            )

        self.run_service = self.create_service(
            Trigger,
            "/manipulation/run_mock_pick_cycle",
            self.run_cycle_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            "Mock pickup-cycle server is ready"
        )
        self.get_logger().info(
            "Service: "
            "/manipulation/run_mock_pick_cycle"
        )
        self.get_logger().warning(
            "MOCK-HARDWARE ORCHESTRATION ONLY"
        )

    async def call_trigger(
        self,
        key,
        display_name,
    ):
        client = self.service_clients[key]
        service_name = self.service_names[key]

        timeout = float(
            self.get_parameter(
                "service_timeout_s"
            ).value
        )

        if not client.service_is_ready():
            ready = client.wait_for_service(
                timeout_sec=timeout
            )

            if not ready:
                raise RuntimeError(
                    f"{display_name} unavailable: "
                    f"{service_name}"
                )

        self.get_logger().info(
            f"Starting step: {display_name}"
        )

        request = Trigger.Request()

        result = await client.call_async(
            request
        )

        if result is None:
            raise RuntimeError(
                f"{display_name} returned no response"
            )

        if not result.success:
            raise RuntimeError(
                f"{display_name} failed: "
                f"{result.message}"
            )

        self.get_logger().info(
            f"Completed step: {display_name}: "
            f"{result.message}"
        )

        return result

    async def run_cycle_callback(
        self,
        request,
        response,
    ):
        del request

        if not bool(
            self.get_parameter(
                "mock_hardware"
            ).value
        ):
            response.success = False
            response.message = (
                "Pick-cycle execution rejected: "
                "mock_hardware is false."
            )
            return response

        if not self.cycle_lock.acquire(
            blocking=False
        ):
            response.success = False
            response.message = (
                "A pickup cycle is already running."
            )
            return response

        current_step = "initialization"

        try:
            current_step = "ResetRod"
            await self.call_trigger(
                "reset_rod",
                current_step,
            )

            current_step = "VerifyRodAtWorkstation"
            rod_status = await self.call_trigger(
                "rod_status",
                current_step,
            )

            if "WORLD_OBJECT" not in (
                rod_status.message
            ):
                raise RuntimeError(
                    "Rod is not a world object after "
                    f"reset: {rod_status.message}"
                )

            current_step = "PlanPregrasp"
            await self.call_trigger(
                "plan_pregrasp",
                current_step,
            )

            current_step = "ExecutePregrasp"
            await self.call_trigger(
                "execute_pregrasp",
                current_step,
            )

            current_step = "PlanLinearGrasp"
            await self.call_trigger(
                "plan_grasp",
                current_step,
            )

            current_step = "ExecuteLinearGrasp"
            await self.call_trigger(
                "execute_grasp",
                current_step,
            )

            current_step = "AttachRod"
            await self.call_trigger(
                "attach_rod",
                current_step,
            )

            current_step = "VerifyRodAttached"
            attached_status = await self.call_trigger(
                "rod_status",
                current_step,
            )

            if "ATTACHED_TO_GRIPPER" not in (
                attached_status.message
            ):
                raise RuntimeError(
                    "Rod attachment verification failed: "
                    f"{attached_status.message}"
                )

            current_step = "PlanLinearRetreat"
            await self.call_trigger(
                "plan_retreat",
                current_step,
            )

            current_step = "ExecuteLinearRetreat"
            await self.call_trigger(
                "execute_retreat",
                current_step,
            )

            current_step = "VerifyRodAfterRetreat"
            final_status = await self.call_trigger(
                "rod_status",
                current_step,
            )

            if "ATTACHED_TO_GRIPPER" not in (
                final_status.message
            ):
                raise RuntimeError(
                    "Rod was not attached after retreat: "
                    f"{final_status.message}"
                )

            response.success = True
            response.message = (
                "Mock pickup cycle completed. "
                "The rod remains attached to the gripper "
                "at the pre-grasp position."
            )

            self.get_logger().info(
                response.message
            )

        except Exception as error:
            response.success = False
            response.message = (
                f"Mock pickup cycle stopped at "
                f"{current_step}: {error}"
            )

            self.get_logger().error(
                response.message
            )

        finally:
            self.cycle_lock.release()

        return response


def main(args=None):
    rclpy.init(args=args)

    node = None
    executor = None

    try:
        node = MockPickCycleServer()

        executor = MultiThreadedExecutor(
            num_threads=4
        )

        executor.add_node(node)
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        if executor is not None:
            try:
                executor.shutdown(
                    timeout_sec=1.0
                )
            except Exception:
                pass

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()