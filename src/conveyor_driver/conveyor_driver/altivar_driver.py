import asyncio
import threading
import time
from typing import Dict

from asyncua import Client, ua

from conveyor_driver.base_driver import ConveyorDriverBase

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


class AltivarOpcDriver(ConveyorDriverBase):
    """Adapter for Schneider ATV drive using OPC UA."""

    def __init__(
        self,
        ip: str = "10.118.5.83",
        port: int = 4840,
        username: str = "etfrobot",
        password: str = "Etfrobot1!",
        speed_min_mps: float = 0.05,
        speed_max_mps: float = 0.2,
        encoder_scale: float = 1.0,
    ):
        super().__init__(speed_min_mps=speed_min_mps, speed_max_mps=speed_max_mps, encoder_scale=encoder_scale)
        self.ip = ip
        self.port = port
        self.username = username
        self.password = password
        self.url = f"opc.tcp://{ip}:{port}"
        
        self.client = Client(url=self.url)
        self.connected = False

        self.node_enable = "ns=2;s=Application.AltivarControl.bEnableLoop"
        self.node_frequency = "ns=2;s=Application.AltivarControl.ATV_ReferenceFreq"
        self.node_command = "ns=2;s=Application.AltivarControl.ATV_Command"

        # Initialize background thread for asyncio event loop to handle OPC UA networking
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self._opc_loop_worker, daemon=True)
        self.loop_thread.start()

    def _opc_loop_worker(self) -> None:
        """Maintains the OPC UA connection in an isolated background thread."""
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._connect_async())
        self.loop.run_forever()

    async def _connect_async(self) -> None:
        """Asynchronously establishes the OPC UA connection with the PLC."""
        if self.connected:
            return
        self.client.set_user(self.username)
        self.client.set_password(self.password)
        try:
            await self.client.connect()
            self.connected = True
        except Exception:
            pass

    @staticmethod
    def _coerce_variant(value, variant_type):
        """Coerces Python primitives into the corresponding OPC UA variant types."""
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

    async def _write_value_async(self, node_id: str, value, variant_type=None) -> None:
        """Asynchronously writes a value to the specified OPC UA node."""
        if not self.connected:
            await self._connect_async()
            
        node = self.client.get_node(node_id)
        if variant_type is None:
            try:
                variant_type = await node.read_data_type_as_variant_type()
            except Exception:
                variant_type = await node.read_data_type()
                
        coerced_value = self._coerce_variant(value, variant_type)
        dv = ua.DataValue(ua.Variant(coerced_value, variant_type))
        await node.write_value(dv)

    def _write_value(self, node_id: str, value, variant_type=None) -> None:
        """Synchronous wrapper for writing OPC UA values safely from ROS 2 callbacks."""
        if not self.loop.is_running():
            return
            
        future = asyncio.run_coroutine_threadsafe(
            self._write_value_async(node_id, value, variant_type),
            self.loop
        )
        try:
            # Block the calling ROS thread until the background write completes (timeout 5.0s)
            future.result(timeout=5.0)
        except Exception:
            pass

    def configure(self) -> None:
        """Configures the driver. Connection logic is offloaded to the background thread."""
        super().configure()

    def start(self) -> None:
        """Starts the conveyor belt by setting the reference frequency and drivecom commands."""
        super().start()
        self._write_value(self.node_frequency, speed_to_plc_frequency(self.command_speed_mps))
        self._write_value(self.node_command, 7, ua.VariantType.UInt16)
        time.sleep(0.1)
        self._write_value(self.node_command, 15, ua.VariantType.UInt16)

    def stop(self) -> None:
        """Stops the conveyor belt and clears references."""
        self.running = False
        self.speed_mps = 0.0
        self.command_speed_mps = 0.0
        self._write_value(self.node_command, 6, ua.VariantType.UInt16)
        self._write_value(self.node_frequency, 0)

    def set_target_speed(self, speed_mps: float) -> float:
        """Updates the command speed and sends the new frequency to the PLC."""
        speed = super().set_target_speed(speed_mps)
        self._write_value(self.node_frequency, speed_to_plc_frequency(speed))
        return speed

    def reset_fault(self) -> None:
        """Resets active faults on the ATV drive."""
        super().reset_fault()
        self._write_value(self.node_enable, True, ua.VariantType.Boolean)

    def update(self, dt: float) -> Dict[str, float | bool | str]:
        """
        Updates internal state variables. 
        Network writes are removed from this 20Hz loop to prevent OPC UA operation overflow.
        """
        if self.running:
            self.speed_mps = self.command_speed_mps
            self.distance_m += self.speed_mps * dt * self.encoder_scale
        else:
            self.speed_mps = 0.0
        return self.read_state()