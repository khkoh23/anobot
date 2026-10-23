#!/usr/bin/env python3

import rclpy
import re
import threading
import time
from rclpy.node import Node
from std_srvs.srv import Trigger
from typing import Any, Optional
try:
    import serial
except ImportError:
    serial = None


class EndEffectorServer(Node):

    def __init__(self):
        super().__init__("end_effector_server")
        self.declare_parameter("device", "/tmp/ttyUR",)
        self.declare_parameter("baud_rate", 9600,)
        self.declare_parameter("read_timeout_s", 0.20,)
        self.declare_parameter("write_timeout_s", 0.20,)
        self.declare_parameter("response_delay_s", 0.05,)
        self.declare_parameter("dry_run", True,)
        self.declare_parameter("open_command_hex", "",)
        self.declare_parameter("tighten_command_hex", "",)
        self.declare_parameter("loosen_command_hex", "",)
        self.declare_parameter("status_command_hex", "",)
        self.declare_parameter("open_expected_text", "Open",)
        self.declare_parameter("tighten_expected_text", "Tight",)
        self.declare_parameter("loosen_expected_text", "Loose",)
        # self.declare_parameter("status_expected_hex", "",)
        self.declare_parameter("read_size", 64,)
        self.declare_parameter("flush_input_before_command", True,)
        self.declare_parameter("append_crc16_modbus", False,)
        self.declare_parameter("inter_byte_quiet_s", 0.05,)
        self.device = str(self.get_parameter("device").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.read_timeout_s = float(self.get_parameter("read_timeout_s").value)
        self.write_timeout_s = float(self.get_parameter("write_timeout_s").value)
        self.response_delay_s = float(self.get_parameter("response_delay_s").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.read_size = int(self.get_parameter("read_size").value)
        self.flush_input_before_command = bool(self.get_parameter("flush_input_before_command").value)
        self.append_crc16_modbus = bool(self.get_parameter("append_crc16_modbus").value)
        self.inter_byte_quiet_s = float(self.get_parameter("inter_byte_quiet_s").value)
        if self.inter_byte_quiet_s <= 0.0:
            raise ValueError("inter_byte_quiet_s must be positive")
        # ``serial`` is optional at runtime, so it may be ``None`` and cannot
        # be used as a type-expression namespace by static type checkers.
        self.serial_port: Optional[Any] = None
        self.io_lock = threading.Lock()
        self.last_operation = "NONE"
        self.last_response = b""
        self.last_success = False
        self.open_service = self.create_service(Trigger, "/tool/open_gripper", self.open_callback,)
        self.tighten_service = self.create_service(Trigger, "/tool/tighten_gripper", self.tighten_callback,)
        self.loosen_service = self.create_service(Trigger, "/tool/loosen_gripper", self.loosen_callback,)
        self.status_service = self.create_service(Trigger, "/tool/gripper_status", self.status_callback,)
        self.connect_service = self.create_service(Trigger, "/tool/connect", self.connect_callback,)
        self.disconnect_service = (self.create_service(Trigger, "/tool/disconnect", self.disconnect_callback,))
        if self.dry_run:
            self.get_logger().warning("END EFFECTOR SERVER: DRY-RUN MODE. " "No RS485 bytes will be transmitted.")
        else:
            self.get_logger().warning("END EFFECTOR SERVER: RS485 ENABLED")
        self.get_logger().info(f"Tool device: {self.device}")
        self.get_logger().info(f"Configured baud rate: " f"{self.baud_rate}")

    @staticmethod
    def parse_hex(value: str) -> bytes: 
        cleaned = re.sub(r"[^0-9a-fA-F]", "", value,)
        if not cleaned:
            return b""
        if len(cleaned) % 2 != 0:
            raise ValueError("Hex command must contain an even " "number of hexadecimal digits")
        return bytes.fromhex(cleaned)

    @staticmethod
    def format_hex(value: bytes) -> str:
        if not value:
            return "<empty>"
        return " ".join(f"{byte:02X}" for byte in value)

    @staticmethod
    def crc16_modbus(data: bytes) -> bytes:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return bytes([crc & 0xFF, (crc >> 8) & 0xFF,])

    @staticmethod
    def decode_ascii_response(response: bytes) -> str:
        return response.decode("ascii", errors="replace",).strip("\x00\r\n ")

    @staticmethod
    def parse_status_response(text: str):
        match = re.search(
            r"FSR\s*1\s*:\s*(-?\d+)"
            r"\s*/\s*"
            r"FSR\s*2\s*:\s*(-?\d+)",
            text,
            flags=re.IGNORECASE,
        )
        if match is None:
            raise ValueError("Unrecognized FSR status response: " f"{text!r}")
        return (int(match.group(1)), int(match.group(2)),)

    @staticmethod
    def response_matches_text(response: bytes, expected_text: str,) -> bool:
        actual = EndEffectorServer.decode_ascii_response(response)
        return actual == expected_text

    def read_ascii_response(self, maximum_size: int,) -> bytes:
        response = bytearray()
        overall_deadline = (time.monotonic() + self.read_timeout_s)
        last_byte_time = None
        while (len(response) < maximum_size and time.monotonic() < overall_deadline):
            waiting = int(self.serial_port.in_waiting)
            if waiting > 0:
                chunk = self.serial_port.read(min(waiting, maximum_size - len(response),))
                if chunk:
                    response.extend(chunk)
                    last_byte_time = (time.monotonic())
                    # Accept an explicit line terminator when one is supplied.
                    if (b"\n" in response or b"\r" in response):
                        break
            elif last_byte_time is not None:
                quiet_time_s = (time.monotonic() - last_byte_time)
                if (quiet_time_s >= self.inter_byte_quiet_s):
                    break
            time.sleep(0.005)
        return bytes(response)

    def command_from_parameter(self, parameter_name: str, ) -> bytes:
        value = str(self.get_parameter(parameter_name).value)
        command = self.parse_hex(value)
        if (command and self.append_crc16_modbus):
            command += self.crc16_modbus(command)
        return command

    def connect_serial(self):
        if self.dry_run:
            return
        if serial is None:
            raise RuntimeError("Python pyserial is not installed")
        if (self.serial_port is not None and self.serial_port.is_open):
            return
        self.serial_port = serial.Serial(
            port=self.device,
            baudrate=self.baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.read_timeout_s,
            write_timeout=self.write_timeout_s,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.get_logger().info(f"Connected to {self.device}")

    def disconnect_serial(self):
        if self.serial_port is None:
            return
        try:
            if self.serial_port.is_open:
                self.serial_port.close()
        finally:
            self.serial_port = None

    def execute_command(self, operation: str, command_parameter: str, expected_text: str = "", parse_fsr_status: bool = False,):
        command = self.command_from_parameter(command_parameter)
        if not command:
            raise RuntimeError(f"{operation} command is not configured")
        if self.dry_run:
            self.last_operation = operation
            self.last_response = b""
            self.last_success = True
            return (True, f"DRY RUN: {operation}; " f"command={self.format_hex(command)}",)
        with self.io_lock:
            self.connect_serial()
            if self.flush_input_before_command:
                self.serial_port.reset_input_buffer()
            self.get_logger().info(f"RAW TX bytes: {command!r}")
            self.get_logger().info(f"RAW TX hex: " f"{self.format_hex(command)}")
            bytes_written = self.serial_port.write(command)
            self.serial_port.flush()
            if bytes_written != len(command):
                raise RuntimeError(f"Only {bytes_written}/" f"{len(command)} command bytes " "were written")
            time.sleep(self.response_delay_s)
            response = self.read_ascii_response(self.read_size)
            self.get_logger().info(f"RAW RX bytes: {response!r}")
            self.get_logger().info(f"RAW RX hex: " f"{self.format_hex(response)}")
            self.get_logger().info(f"RAW RX length: " f"{len(response)} byte(s)")
        self.last_operation = operation
        self.last_response = response
        response_text = self.decode_ascii_response(response)
        if not response:
            self.last_success = False
            return (False, f"{operation}; " f"tx={self.format_hex(command)}; " "rx=<empty>; no response received",)
        if parse_fsr_status:
            fsr_1, fsr_2 = (self.parse_status_response(response_text))
            self.last_success = True
            return (
                True,
                f"{operation}; "
                f"tx={self.format_hex(command)}; "
                f"rx={response_text!r}; "
                f"FSR 1={fsr_1}; "
                f"FSR 2={fsr_2}",
            )
        if expected_text:
            success = (response_text.strip().casefold() == expected_text.strip().casefold())
        else:
            success = True
        self.last_success = success
        message = (f"{operation}; " f"tx={self.format_hex(command)}; " f"rx={response_text!r}")
        if expected_text:
            message += (f"; expected={expected_text!r}")
        return success, message

    def perform_operation(self, operation: str, command_parameter: str, response, expected_text: str = "", parse_fsr_status: bool = False,):
        try:
            (response.success, response.message,) = self.execute_command(
                operation=operation,
                command_parameter=command_parameter,
                expected_text=expected_text,
                parse_fsr_status=parse_fsr_status,
            )
            if response.success:
                self.get_logger().info(response.message)
            else:
                self.get_logger().error(response.message)
        except Exception as error:
            self.last_operation = operation
            self.last_success = False
            response.success = False
            response.message = (f"{operation} failed: {error}")
            self.get_logger().error(response.message)
        return response
    
    def open_callback(self, request, response,):
        del request
        expected_text = str(self.get_parameter("open_expected_text").value)
        return self.perform_operation(
            operation="OPEN", 
            command_parameter="open_command_hex", 
            expected_text=expected_text, 
            response=response,
        )

    def tighten_callback(self, request, response,):
        del request
        expected_text = str(self.get_parameter("tighten_expected_text").value)
        return self.perform_operation(
            operation="TIGHTEN",
            command_parameter="tighten_command_hex",
            expected_text=expected_text,
            response=response,
        )

    def loosen_callback(self, request, response,):
        del request
        expected_text = str(self.get_parameter("loosen_expected_text").value)
        return self.perform_operation(
            operation="LOOSEN",
            command_parameter="loosen_command_hex",
            expected_text=expected_text,
            response=response,
        )

    def status_callback(self, request, response,):
        del request
        return self.perform_operation(
            operation="STATUS",
            command_parameter="status_command_hex",
            parse_fsr_status=True,
            response=response,
        )

    def connect_callback(self, request, response,):
        del request
        try:
            self.connect_serial()
            response.success = True
            response.message = ("Dry-run connection ready" if self.dry_run else f"Connected to {self.device}")
        except Exception as error:
            response.success = False
            response.message = (f"Connection failed: {error}")
        return response

    def disconnect_callback(self, request, response,):
        del request
        self.disconnect_serial()
        response.success = True
        response.message = ("End-effector serial connection closed")
        return response

    def destroy_node(self):
        self.disconnect_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorServer()
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