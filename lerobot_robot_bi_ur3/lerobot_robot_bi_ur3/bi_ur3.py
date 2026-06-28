"""Bimanual UR3 robot wrapper built from two existing UR3 robot instances."""

from __future__ import annotations

import logging
from dataclasses import replace
from functools import cached_property
from typing import Any

from lerobot.robots import Robot
from lerobot_robot_ur3 import UR3

from .config_bi_ur3 import BiUR3Config

logger = logging.getLogger(__name__)


class BiUR3(Robot):
    """Two UR3 arms exposed as one LeRobot robot with left_/right_ feature prefixes."""

    config_class = BiUR3Config
    name = "bi_ur3"

    def __init__(self, config: BiUR3Config):
        super().__init__(config)
        self.config = config

        left_arm_config = config.left_arm_config
        right_arm_config = config.right_arm_config
        if config.id or config.calibration_dir is not None:
            left_calibration_dir = left_arm_config.calibration_dir
            right_calibration_dir = right_arm_config.calibration_dir
            if config.calibration_dir is not None:
                left_calibration_dir = config.calibration_dir / "left"
                right_calibration_dir = config.calibration_dir / "right"

            left_arm_config = replace(
                left_arm_config,
                id=f"{config.id}_left" if config.id else left_arm_config.id,
                calibration_dir=left_calibration_dir,
            )
            right_arm_config = replace(
                right_arm_config,
                id=f"{config.id}_right" if config.id else right_arm_config.id,
                calibration_dir=right_calibration_dir,
            )

        self.left_arm = UR3(left_arm_config)
        self.right_arm = UR3(right_arm_config)

        # Compatibility for code that expects a robot.cameras mapping. The child
        # UR3 instances still own connection and reads; these names avoid collisions.
        self.cameras = {
            **{f"left_{name}": camera for name, camera in self.left_arm.cameras.items()},
            **{f"right_{name}": camera for name, camera in self.right_arm.cameras.items()},
        }

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            **{f"left_{key}": value for key, value in self.left_arm._motors_ft.items()},
            **{f"right_{key}": value for key, value in self.right_arm._motors_ft.items()},
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            **{f"left_{key}": value for key, value in self.left_arm._cameras_ft.items()},
            **{f"right_{key}": value for key, value in self.right_arm._cameras_ft.items()},
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            return

        self.left_arm.connect(calibrate)
        try:
            self.right_arm.connect(calibrate)
        except Exception:
            self.left_arm.disconnect()
            raise
        logger.info("%s connected.", self)

    def configure(self) -> None:
        self.left_arm.configure()
        self.right_arm.configure()

    def disconnect(self) -> None:
        if self.left_arm.is_connected:
            self.left_arm.disconnect()
        if self.right_arm.is_connected:
            self.right_arm.disconnect()

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        self.left_arm.calibrate()
        self.right_arm.calibrate()

    def get_observation(self) -> dict[str, Any]:
        left_obs = self.left_arm.get_observation()
        right_obs = self.right_arm.get_observation()

        return {
            **{f"left_{key}": value for key, value in left_obs.items()},
            **{f"right_{key}": value for key, value in right_obs.items()},
        }

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        left_action = {
            key.removeprefix("left_"): value for key, value in action.items() if key.startswith("left_")
        }
        right_action = {
            key.removeprefix("right_"): value for key, value in action.items() if key.startswith("right_")
        }

        sent_left_action = self.left_arm.send_action(left_action)
        sent_right_action = self.right_arm.send_action(right_action)

        return {
            **{f"left_{key}": value for key, value in sent_left_action.items()},
            **{f"right_{key}": value for key, value in sent_right_action.items()},
        }

    def move_to_start_joints(self, wait: bool = True) -> None:
        self.left_arm.move_to_start_joints(wait=wait)
        self.right_arm.move_to_start_joints(wait=wait)
