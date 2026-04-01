from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class Goal:
    x: float
    y: float
    z: float


@dataclass
class ControlCommand:
    roll: float
    pitch: float
    thrust: int


@dataclass
class PIDGains:
    kpx: float = 1.5291
    kpy: float = 1.5291
    kpz: float = 2.4
    kdx: float = 0.8155
    kdy: float = 0.8155
    kdz: float = 0.4104


class PIDPositionController:
    """Position controller using the same basic logic as the original script."""

    def __init__(self, window_size: int = 5, gains: PIDGains | None = None):
        self.window_size = window_size
        self.gains = gains or PIDGains()
        self.reset()

    def reset(self) -> None:
        self._x_history = deque(maxlen=self.window_size)
        self._y_history = deque(maxlen=self.window_size)
        self._z_history = deque(maxlen=self.window_size)
        self._time_history = deque(maxlen=self.window_size)

    def add_sample(self, x: float, y: float, z: float, timestamp: float) -> None:
        self._x_history.append(x)
        self._y_history.append(y)
        self._z_history.append(z)
        self._time_history.append(timestamp)

    def compute_command(
        self,
        current_yaw: float,
        goal_x: float,
        goal_y: float,
        goal_z: float,
    ) -> ControlCommand:
        x_pos = np.asarray(self._x_history, dtype=float)
        y_pos = np.asarray(self._y_history, dtype=float)
        z_pos = np.asarray(self._z_history, dtype=float)
        times = np.asarray(self._time_history, dtype=float)

        if len(x_pos) == 0:
            return ControlCommand(roll=0.0, pitch=0.0, thrust=0)

        x_error = goal_x - x_pos
        y_error = goal_y - y_pos
        z_error = goal_z - z_pos

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        rotation = np.array([
            [cos_yaw, sin_yaw, 0.0],
            [-sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ])

        for i in range(len(x_error)):
            global_errors = np.array([[x_error[i]], [y_error[i]], [z_error[i]]])
            local_errors = rotation @ global_errors
            x_error[i] = local_errors[0, 0]
            y_error[i] = local_errors[1, 0]
            z_error[i] = local_errors[2, 0]

        dx = self._safe_derivative(x_error, times)
        dy = self._safe_derivative(y_error, times)
        dz = self._safe_derivative(z_error, times)

        theta = self.gains.kpx * x_error[-1] + self.gains.kdx * dx
        phi = self.gains.kpy * y_error[-1] + self.gains.kdy * dy
        thrust = self.gains.kpz * z_error[-1] + self.gains.kdz * dz

        pitch_deg = np.rad2deg(theta)
        roll_deg = -np.rad2deg(phi)

        pitch_deg = float(np.clip(pitch_deg, -15.0, 15.0))
        roll_deg = float(np.clip(roll_deg, -15.0, 15.0))

        thrust_cmd = thrust * (50000 / 2.3346) + 10001
        thrust_cmd = int(np.clip(thrust_cmd, 0, 0xFFFF))

        return ControlCommand(roll=roll_deg, pitch=pitch_deg, thrust=thrust_cmd)

    @staticmethod
    def _safe_derivative(values: np.ndarray, times: np.ndarray) -> float:
        if len(values) < 2 or len(times) < 2:
            return 0.0

        dt = np.gradient(times)
        if np.any(np.isclose(dt, 0.0)):
            return 0.0

        return float(np.mean(np.gradient(values) / dt))
