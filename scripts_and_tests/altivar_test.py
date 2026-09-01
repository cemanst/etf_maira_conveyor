import asyncio
import threading
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from asyncua import Client, ua

OPC_URL = "opc.tcp://10.118.5.83:4840"
NODE_ENABLE = "ns=2;s=Application.AltivarControl.bEnableLoop"
NODE_FREQ = "ns=2;s=Application.AltivarControl.ATV_ReferenceFreq"
NODE_CMD = "ns=2;s=Application.AltivarControl.ATV_Command"

# The PLC uses a 0.1 Hz resolution, and the effective drive frequency is 10x smaller than
# the value we write in the control layer. The real mapping is:
#   speed_mps = 0.005 * real_frequency_hz
# so:
#   real_frequency_hz = speed_mps / 0.005
# and because the PLC stores 0.1 Hz units, the value written to the device is scaled by 10.
HERTZ_TO_SPEED_MPS = 0.005
PLC_FREQUENCY_SCALE = 10.0


def speed_to_plc_frequency(speed_mps: float) -> int:
    real_frequency_hz = speed_mps / HERTZ_TO_SPEED_MPS
    plc_frequency = real_frequency_hz * PLC_FREQUENCY_SCALE
    return int(round(plc_frequency))


class ConveyorAsyncNode(Node):
    def __init__(self):
        super().__init__('conveyor_driver')
        self.opc_client = Client(url=OPC_URL)
        self._is_connected = False
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self._opc_loop, daemon=True)

        self.srv_start = self.create_service(Trigger, '~/start', self.start_callback)
        self.srv_stop = self.create_service(Trigger, '~/stop', self.stop_callback)

        self.declare_parameter('conveyor_speed_mps', 0.5)
        self.declare_parameter('conveyor_speed_min_mps', 0.0)
        self.declare_parameter('conveyor_speed_max_mps', 2.0)

        self.loop_thread.start()

    def _opc_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.setup_opc())
        self.loop.run_forever()

    async def setup_opc(self):
        try:
            self.opc_client.set_user("etfrobot")
            self.opc_client.set_password("Etfrobot1!")
            await self.opc_client.connect()
            self._is_connected = True
            self.get_logger().info("OPC UA client connected successfully (security none).")
        except Exception as exc:
            self._is_connected = False
            self.get_logger().error(f"Error connecting to PLC: {exc}")

    async def ensure_connected(self):
        try:
            if self._is_connected:
                return

            if self.opc_client is None:
                self.opc_client = Client(url=OPC_URL)

            await self.opc_client.connect()
            self._is_connected = True
            self.get_logger().info("OPC UA reconnect OK")
        except Exception as exc:
            self._is_connected = False
            self.get_logger().error(f"Reconnect failed: {exc}")
            raise

    async def read_opc_data_type(self, node_id):
        try:
            node = self.opc_client.get_node(node_id)
            try:
                data_type = await node.read_data_type_as_variant_type()
                self.get_logger().info(f"DEBUG TYPE: {node_id} = {data_type}")
                return data_type
            except Exception:
                raw_type = await node.read_data_type()
                self.get_logger().info(f"DEBUG TYPE RAW: {node_id} = {raw_type}")
                return raw_type
        except Exception as exc:
            self.get_logger().error(f"Error reading data type for {node_id}: {exc}")
            raise

    @staticmethod
    def coerce_value_for_variant(value, variant_type):
        if variant_type is None:
            return value

        if variant_type in (ua.VariantType.Float, ua.VariantType.Double):
            return float(value)
        if variant_type in (
            ua.VariantType.Byte,
            ua.VariantType.SByte,
            ua.VariantType.Int16,
            ua.VariantType.UInt16,
            ua.VariantType.Int32,
            ua.VariantType.UInt32,
            ua.VariantType.Int64,
            ua.VariantType.UInt64,
        ):
            return int(value)
        if variant_type == ua.VariantType.Boolean:
            return bool(value)
        return value

    async def write_opc_node(self, node_id, value, variant_type=None):
        try:
            await self.ensure_connected()
            if variant_type is None:
                variant_type = await self.read_opc_data_type(node_id)

            coerced_value = self.coerce_value_for_variant(value, variant_type)
            node = self.opc_client.get_node(node_id)
            dv = ua.DataValue(ua.Variant(coerced_value, variant_type))
            await node.write_value(dv)
            self.get_logger().info(f"Wrote {coerced_value} ({variant_type}) to {node_id}")
        except Exception as exc:
            self.get_logger().error(
                f"Error writing to {node_id} with type {variant_type} and value {value}: {exc}"
            )
            raise

    def write_opc_node_sync(self, node_id, value, variant_type=None):
        if not self.loop.is_running():
            raise RuntimeError("OPC UA asyncio loop is not running.")

        future = asyncio.run_coroutine_threadsafe(
            self.write_opc_node(node_id, value, variant_type),
            self.loop,
        )
        return future.result(timeout=10)

    async def read_opc_node(self, node_id):
        try:
            node = self.opc_client.get_node(node_id)
            value = await node.read_value()
            self.get_logger().info(f"DEBUG: {node_id} = {value}")
            return value
        except Exception as exc:
            self.get_logger().error(f"Error reading {node_id}: {exc}")
            raise

    def read_opc_node_sync(self, node_id):
        if not self.loop.is_running():
            raise RuntimeError("OPC UA asyncio loop is not running.")

        future = asyncio.run_coroutine_threadsafe(
            self.read_opc_node(node_id),
            self.loop,
        )
        return future.result(timeout=10)

    def verify_write(self, node_id, expected_value):
        actual_value = self.read_opc_node_sync(node_id)
        self.get_logger().info(
            f"DEBUG VERIFY: {node_id} expected={expected_value}, actual={actual_value}"
        )
        return actual_value

    def start_callback(self, request, response):
        speed_mps = self.get_parameter('conveyor_speed_mps').value
        plc_frequency = speed_to_plc_frequency(speed_mps)

        self.get_logger().info(f"Starting conveyor at {speed_mps} m/s (PLC value: {plc_frequency})")
        try:
            self.write_opc_node_sync(NODE_FREQ, plc_frequency)
            self.verify_write(NODE_FREQ, plc_frequency)

            self.write_opc_node_sync(NODE_CMD, 7, ua.VariantType.UInt16)
            self.verify_write(NODE_CMD, 7)

            time.sleep(0.2)

            self.write_opc_node_sync(NODE_CMD, 15, ua.VariantType.UInt16)
            self.verify_write(NODE_CMD, 15)

            response.success = True
            response.message = f"Conveyor started at {speed_mps} m/s"
        except Exception as exc:
            self.get_logger().error(f"Start callback failed: {exc}")
            response.success = False
            response.message = str(exc)
        return response

    def stop_callback(self, request, response):
        self.get_logger().info("Stopping conveyor...")
        try:
            self.write_opc_node_sync(NODE_CMD, 6, ua.VariantType.UInt16)
            self.verify_write(NODE_CMD, 6)

            self.write_opc_node_sync(NODE_FREQ, 0)
            self.verify_write(NODE_FREQ, 0)

            response.success = True
            response.message = "Conveyor stopped"
        except Exception as exc:
            self.get_logger().error(f"Stop callback failed: {exc}")
            response.success = False
            response.message = str(exc)
        return response

    def shutdown(self):
        try:
            if self._is_connected:
                future = asyncio.run_coroutine_threadsafe(self.opc_client.disconnect(), self.loop)
                future.result(timeout=10)
                self._is_connected = False
        except Exception as exc:
            self.get_logger().warning(f"Failed to cleanly disconnect OPC UA client: {exc}")
        finally:
            if self.loop.is_running():
                self.loop.call_soon_threadsafe(self.loop.stop)
            if self.loop_thread.is_alive():
                self.loop_thread.join(timeout=5)


def main(args=None):
    rclpy.init(args=args)
    conveyor_node = ConveyorAsyncNode()

    try:
        rclpy.spin(conveyor_node)
    except KeyboardInterrupt:
        pass
    finally:
        conveyor_node.shutdown()
        conveyor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()