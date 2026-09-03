import time
from typing import Dict, Optional


class ConveyorDriverBase:
    """Base interface for conveyor hardware and mock implementations."""

    def __init__(self, speed_min_mps: float = 0.05, speed_max_mps: float = 0.2, encoder_scale: float = 1.0):
        self.speed_min_mps = float(speed_min_mps)
        self.speed_max_mps = float(speed_max_mps)
        self.encoder_scale = float(encoder_scale)
        self.command_speed_mps = 0.0
        self.running = False
        self.fault = False
        self.fault_message = ""
        self.speed_mps = 0.0
        self.distance_m = 0.0
        self._last_update_ts = time.monotonic()

    def configure(self) -> None:
        """Hook for driver configuration."""
        self._last_update_ts = time.monotonic()

    def start(self) -> None:
        self.running = True
        self.fault = False
        self.fault_message = ""

    def stop(self) -> None:
        self.running = False
        self.speed_mps = 0.0
        self.command_speed_mps = 0.0

    def set_target_speed(self, speed_mps: float) -> float:
        speed_mps = float(speed_mps)
        if speed_mps < self.speed_min_mps:
            speed_mps = self.speed_min_mps
        if speed_mps > self.speed_max_mps:
            speed_mps = self.speed_max_mps
        self.command_speed_mps = speed_mps
        return self.command_speed_mps

    def reset_fault(self) -> None:
        self.fault = False
        self.fault_message = ""

    def update(self, dt: float) -> Dict[str, float | bool | str]:
        now = time.monotonic()
        if dt <= 0.0:
            dt = max(0.0, now - self._last_update_ts)
        if self.running:
            self.speed_mps = self.command_speed_mps
            self.distance_m += self.speed_mps * dt * self.encoder_scale
        else:
            self.speed_mps = 0.0
        self._last_update_ts = now
        return self.read_state()

    def read_state(self) -> Dict[str, float | bool | str]:
        return {
            "speed_mps": float(self.speed_mps),
            "distance_m": float(self.distance_m),
            "running": bool(self.running),
            "fault": bool(self.fault),
            "fault_message": self.fault_message,
        }


class MockConveyorDriver(ConveyorDriverBase):
    """Simple mock driver used for development and tests."""

    def __init__(self, speed_min_mps: float = 0.05, speed_max_mps: float = 0.2, encoder_scale: float = 1.0):
        super().__init__(speed_min_mps=speed_min_mps, speed_max_mps=speed_max_mps, encoder_scale=encoder_scale)

    def configure(self) -> None:
        super().configure()
        self.fault = False
        self.fault_message = ""

    def start(self) -> None:
        super().start()
        self.fault_message = ""

    def reset_fault(self) -> None:
        super().reset_fault()
        self.running = False

    def update(self, dt: float) -> Dict[str, float | bool | str]:
        if self.running:
            self.speed_mps = self.command_speed_mps
            self.distance_m += self.speed_mps * dt * self.encoder_scale
        else:
            self.speed_mps = 0.0
        return self.read_state()
