#!/usr/bin/env python3
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from lifecycle_msgs.msg import Transition
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from conveyor_driver.altivar_driver import AltivarOpcDriver
from conveyor_driver.base_driver import MockConveyorDriver


class ConveyorLifecycleNode(LifecycleNode):
    def __init__(self, driver_type: str = "mock") -> None:
        super().__init__("conveyor_driver")
        self.driver_type = driver_type
        self.declare_parameter("driver_type", driver_type)
        self.declare_parameter("ip", "10.118.5.83")
        self.declare_parameter("port", 4840)
        self.declare_parameter("username", "etfrobot")
        self.declare_parameter("password", "Etfrobot1!")
        self.declare_parameter("speed_min_mps", 0.05)
        self.declare_parameter("speed_max_mps", 0.2)
        self.declare_parameter("encoder_scale", 1.0)
        self.declare_parameter("default_speed_mps", 0.1)
        self.declare_parameter("watchdog_timeout", 3600.0)
        self.declare_parameter("control_loop_hz", 20.0)

        self._driver = None
        self._timer = None
        self._status_pub = None
        self._diag_pub = None
        self._cmd_sub = None
        self._start_srv = None
        self._stop_srv = None
        self._reset_fault_srv = None
        self._command_speed_mps = 0.0
        self._last_cmd_ts = 0.0
        self._running = False
        self._fault = False
        self._fault_message = ""
        self._distance_m = 0.0
        self._speed_mps = 0.0

    def _publish_diagnostics(self) -> None:
        if self._diag_pub is None:
            return

        status = DiagnosticStatus()
        if self._fault:
            status.level = DiagnosticStatus.ERROR
            status.message = self._fault_message or "Conveyor fault"
        elif self._running:
            status.level = DiagnosticStatus.OK
            status.message = f"Conveyor running at {self._speed_mps:.3f} m/s"
        else:
            status.level = DiagnosticStatus.WARN
            status.message = "Conveyor idle"
        status.name = "conveyor_driver.health"
        status.hardware_id = "conveyor_driver"

        msg = DiagnosticArray()
        msg.status = [status]
        self._diag_pub.publish(msg)

    def _build_driver(self):
        driver_type = self.get_parameter("driver_type").value
        if driver_type == "opc_ua":
            return AltivarOpcDriver(
                ip=self.get_parameter("ip").value,
                port=self.get_parameter("port").value,
                username=self.get_parameter("username").value,
                password=self.get_parameter("password").value,
                speed_min_mps=self.get_parameter("speed_min_mps").value,
                speed_max_mps=self.get_parameter("speed_max_mps").value,
                encoder_scale=self.get_parameter("encoder_scale").value,
            )
        return MockConveyorDriver(
            speed_min_mps=self.get_parameter("speed_min_mps").value,
            speed_max_mps=self.get_parameter("speed_max_mps").value,
            encoder_scale=self.get_parameter("encoder_scale").value,
        )

    def _publish_state(self) -> None:
        if self._status_pub is None:
            return
        state = self._driver.read_state() if self._driver is not None else {
            "speed_mps": 0.0,
            "distance_m": 0.0,
            "running": False,
            "fault": False,
            "fault_message": "",
        }
        msg = Float64()
        msg.data = float(state["speed_mps"])
        self._status_pub.publish(msg)

    def _on_cmd_velocity(self, msg: Float64) -> None:
        self._last_cmd_ts = time.monotonic()
        self._command_speed_mps = float(msg.data)
        if self._driver is not None:
            self._driver.set_target_speed(self._command_speed_mps)

    def _watchdog(self) -> None:
        timeout = float(self.get_parameter("watchdog_timeout").value)
        if self._running and (time.monotonic() - self._last_cmd_ts) > timeout:
            self._fault = True
            self._fault_message = f"Watchdog timeout after {timeout:.1f} s"
            if self._driver is not None:
                self._driver.fault = True
                self._driver.fault_message = self._fault_message

    def _loop(self) -> None:
        if self._driver is None:
            return
        now = time.monotonic()
        dt = max(0.0, now - self._last_cmd_ts)
        state = self._driver.update(dt)
        self._speed_mps = float(state["speed_mps"])
        self._distance_m = float(state["distance_m"])
        self._running = bool(state["running"])
        self._fault = bool(state["fault"])
        self._fault_message = str(state.get("fault_message", ""))
        self._watchdog()
        self._publish_state()
        self._publish_diagnostics()

    def _start_service_callback(self, request, response):
        if self._driver is None:
            response.success = False
            response.message = "Driver not configured"
            return response
        if self._fault:
            response.success = False
            response.message = self._fault_message
            return response
        desired_speed = self._command_speed_mps or float(self.get_parameter("default_speed_mps").value)
        self._driver.set_target_speed(desired_speed)
        self._driver.start()
        self._running = True
        self._fault = False
        self._fault_message = ""
        response.success = True
        response.message = f"Conveyor started at {desired_speed:.3f} m/s"
        return response

    def _stop_service_callback(self, request, response):
        if self._driver is not None:
            self._driver.stop()
        self._running = False
        self._fault = False
        self._fault_message = ""
        response.success = True
        response.message = "Conveyor stopped"
        return response

    def _reset_fault_service_callback(self, request, response):
        if self._driver is not None:
            self._driver.reset_fault()
        self._fault = False
        self._fault_message = ""
        response.success = True
        response.message = "Fault reset"
        return response

    def on_configure(self, state: Transition) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring conveyor driver")
        self._driver = self._build_driver()
        self._driver.configure()
        self._last_cmd_ts = time.monotonic()
        self._command_speed_mps = float(self.get_parameter("default_speed_mps").value)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._cmd_sub = self.create_subscription(Float64, "cmd_velocity", self._on_cmd_velocity, qos)
        self._status_pub = self.create_publisher(Float64, "state", qos)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._start_srv = self.create_service(Trigger, "start", self._start_service_callback)
        self._stop_srv = self.create_service(Trigger, "stop", self._stop_service_callback)
        self._reset_fault_srv = self.create_service(Trigger, "reset_fault", self._reset_fault_service_callback)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: Transition) -> TransitionCallbackReturn:
        self.get_logger().info("Activating conveyor driver")
        hz = float(self.get_parameter("control_loop_hz").value)
        self._timer = self.create_timer(1.0 / hz, self._loop)
        self._publish_diagnostics()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: Transition) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating conveyor driver")
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        if self._driver is not None:
            self._driver.stop()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: Transition) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up conveyor driver")
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        if self._driver is not None:
            self._driver.stop()
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorLifecycleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
