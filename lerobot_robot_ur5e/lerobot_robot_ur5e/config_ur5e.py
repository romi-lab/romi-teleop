"""Configuration dataclass for the UR5e robot plugin."""

from dataclasses import dataclass, field

import numpy as np
from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig

# # right arm start joints: [-90, -90, -90, -90, 90, 0]
# DEFAULT_UR5E_START_JOINTS = (
#     -np.pi / 2,
#     -np.pi / 2,
#     -np.pi / 2,
#     -np.pi / 2,
#     np.pi / 2,
#     0.0,
# )


# left arm start joints: [90, -90, 90, -90, -90, 0]
DEFAULT_UR5E_START_JOINTS = (
    np.pi / 2,
    -np.pi / 2,
    np.pi / 2,
    -np.pi / 2,
    -np.pi / 2,
    0.0,
)

@RobotConfig.register_subclass("ur5e")
@dataclass
class UR5EConfig(RobotConfig):
    ip: str
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    start_joints: tuple[float, ...] = DEFAULT_UR5E_START_JOINTS
    reset_acceleration: float = 0.2
    reset_speed: float = 0.5

    rtde_frequency: float = -1.0
    rtde_ur_cap_port: int = 30004
    servo_acceleration: float = 0.2
    servo_speed: float = 0.2
    servoj_t: float = 1.0 / 500
    servoj_lookahead: float = 0.2
    servoj_gain: int = 100
    max_joint_delta_per_step: float | None = 0.1

    with_gripper: bool = True
    gripper_port: int = 41414
    gripper_socket_timeout_s: float = 2.0
    gripper_rg_id: int = 0
    gripper_open_width_mm: float = 100.0
    gripper_closed_width_mm: float = 0.0
    gripper_force_n: float = 40.0
    gripper_min_command_delta: float = 0.01
    gripper_min_command_period_s: float = 0.1
    gripper_verify_on_connect: bool = True

    def start_joints_array(self) -> np.ndarray:
        joints = np.asarray(self.start_joints, dtype=float)
        if joints.shape != (6,):
            raise ValueError(f"Expected 6 start joints for UR5e, got shape {joints.shape}.")
        return joints
