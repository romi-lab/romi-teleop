"""Configuration dataclass for the bimanual UR5e robot plugin.

Left/right constants live here so robot IPs, robot start joints, and GELLO
calibration positions stay aligned.
"""

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from lerobot.robots import RobotConfig
from lerobot_robot_ur5e import UR5EConfig


# ---------------------------------------------------------------------------
# Left arm
# ---------------------------------------------------------------------------
# Left UR5e controller IP. Override this in scripts/configs if your lab network
# assigns a different address.
LEFT_UR5E_IP = "192.168.0.8"

# Left UR5e start joints in radians.
# Degrees: [90, -90, 90, -90, -90, 0] --- IGNORE ---
# [-90, -90, -90, -90, 90, 0] 
LEFT_UR5E_START_JOINTS = (
    -np.pi / 2,
    -np.pi / 2,
    -np.pi / 2,
    -np.pi / 2,
    np.pi / 2,
    0.0,
)

# GELLO reference pose for the left leader. This must match
# LEFT_UR5E_START_JOINTS so a calibrated, unmoved GELLO commands the left UR5e
# to its start pose.
LEFT_UR5E_GELLO_CALIBRATION_POSITION = [
    -1.5708,
    -1.5708,
    -1.5708,
    -1.5708,
    1.5708,
    0.0,
]


# ---------------------------------------------------------------------------
# Right arm
# ---------------------------------------------------------------------------
# Right UR5e controller IP. Keep it distinct from LEFT_UR5E_IP.
RIGHT_UR5E_IP = "192.168.0.10"

# Right UR5e start joints in radians.
# Degrees: [-90, -90, -90, -90, 90, 0]  --- IGNORE ---
# [90, -90, 90, -90, -90, 0] 
RIGHT_UR5E_START_JOINTS = (
    np.pi / 2,
    -np.pi / 2,
    np.pi / 2,
    -np.pi / 2,
    -np.pi / 2,
    0.0,
)

# GELLO reference pose for the right leader. This must match
# RIGHT_UR5E_START_JOINTS so a calibrated, unmoved GELLO commands the right
# UR5e to its start pose.
RIGHT_UR5E_GELLO_CALIBRATION_POSITION = [
    1.5708,
    -1.5708,
    1.5708,
    -1.5708,
    -1.5708,
    0.0,
]


def default_left_ur5e_config() -> UR5EConfig:
    return UR5EConfig(
        id="left_ur5e",
        ip=LEFT_UR5E_IP,
        calibration_dir=Path("calibration") / "robots" / "bi_ur5e" / "left",
        cameras={},
        start_joints=LEFT_UR5E_START_JOINTS,
    )


def default_right_ur5e_config() -> UR5EConfig:
    return UR5EConfig(
        id="right_ur5e",
        ip=RIGHT_UR5E_IP,
        calibration_dir=Path("calibration") / "robots" / "bi_ur5e" / "right",
        cameras={},
        start_joints=RIGHT_UR5E_START_JOINTS,
    )


@RobotConfig.register_subclass("bi_ur5e")
@dataclass(kw_only=True)
class BiUR5EConfig(RobotConfig):
    """Configuration for two UR5e arms controlled as one bimanual robot."""

    left_arm_config: UR5EConfig = field(default_factory=default_left_ur5e_config)
    right_arm_config: UR5EConfig = field(default_factory=default_right_ur5e_config)
