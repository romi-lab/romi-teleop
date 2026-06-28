"""Configuration dataclass for the bimanual UR3 robot plugin."""

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from lerobot.robots import RobotConfig
from lerobot_robot_ur3 import UR3Config


LEFT_UR3_IP = "158.132.172.193"
RIGHT_UR3_IP = "158.132.172.214"

LEFT_UR3_START_JOINTS = (
    -np.pi / 2,
    -np.pi / 2,
    -np.pi / 2,
    -np.pi / 2,
    np.pi / 2,
    np.pi / 2,
)

RIGHT_UR3_START_JOINTS = (
    np.pi / 2,
    -np.pi / 2,
    np.pi / 2,
    -np.pi / 2,
    -np.pi / 2,
    np.pi / 2,
)


def default_left_ur3_config() -> UR3Config:
    return UR3Config(
        id="left_ur3",
        ip=LEFT_UR3_IP,
        calibration_dir=Path("calibration") / "robots" / "bi_ur3" / "left",
        cameras={},
        start_joints=LEFT_UR3_START_JOINTS,
    )


def default_right_ur3_config() -> UR3Config:
    return UR3Config(
        id="right_ur3",
        ip=RIGHT_UR3_IP,
        calibration_dir=Path("calibration") / "robots" / "bi_ur3" / "right",
        cameras={},
        start_joints=RIGHT_UR3_START_JOINTS,
    )


@RobotConfig.register_subclass("bi_ur3")
@dataclass(kw_only=True)
class BiUR3Config(RobotConfig):
    """Configuration for two UR3 arms controlled as one bimanual robot."""

    left_arm_config: UR3Config = field(default_factory=default_left_ur3_config)
    right_arm_config: UR3Config = field(default_factory=default_right_ur3_config)
