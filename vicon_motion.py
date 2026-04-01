from dataclasses import dataclass
import math
import time

import motioncapture
import numpy as np


@dataclass
class Pose:
    x: float
    y: float
    z: float
    yaw: float
    timestamp: float


class ViconMotionClient:
    """Reads rigid body poses from the motion-capture system."""

    def __init__(self, host_name: str, mocap_system_type: str = 'vicon'):
        self._mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})

    def wait_for_frame(self) -> dict[str, Pose]:
        self._mc.waitForNextFrame()
        timestamp = time.time()
        poses: dict[str, Pose] = {}

        for name, obj in self._mc.rigidBodies.items():
            pos = obj.position
            if not self._is_valid_position(pos):
                continue

            yaw = self._quaternion_to_yaw(
                obj.rotation.w,
                obj.rotation.x,
                obj.rotation.y,
                obj.rotation.z,
            )
            poses[name] = Pose(
                x=pos[0],
                y=pos[1],
                z=pos[2],
                yaw=yaw,
                timestamp=timestamp,
            )

        return poses

    def get_pose(self, rigid_body_name: str) -> Pose:
        while True:
            poses = self.wait_for_frame()
            pose = poses.get(rigid_body_name)
            if pose is not None:
                return pose

    @staticmethod
    def _is_valid_position(pos) -> bool:
        return (
            np.abs(pos[0]) > 1.0e-9 and np.abs(pos[0]) < 1.0e4 and
            np.abs(pos[1]) > 1.0e-9 and np.abs(pos[1]) < 1.0e4 and
            np.abs(pos[2]) > 1.0e-9 and np.abs(pos[2]) < 1.0e4
        )

    @staticmethod
    def _quaternion_to_yaw(q_w: float, q_x: float, q_y: float, q_z: float) -> float:
        return math.atan2(
            2 * (q_w * q_z + q_x * q_y),
            1 - 2 * (q_y * q_y + q_z * q_z),
        )
