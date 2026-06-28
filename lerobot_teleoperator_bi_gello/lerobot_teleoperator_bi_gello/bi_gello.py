"""Bimanual GELLO teleoperator wrapper built from two existing GELLO instances."""

from __future__ import annotations

import logging
from dataclasses import replace
from functools import cached_property

from lerobot.teleoperators import Teleoperator
from lerobot_teleoperator_gello import Gello

from .config_bi_gello import BiGelloConfig

logger = logging.getLogger(__name__)


class BiGello(Teleoperator):
    """Two GELLO leaders exposed as one LeRobot teleoperator with left_/right_ prefixes."""

    config_class = BiGelloConfig
    name = "bi_gello"

    def __init__(self, config: BiGelloConfig):
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

        self.left_arm = Gello(left_arm_config)
        self.right_arm = Gello(right_arm_config)

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            **{f"left_{key}": value for key, value in self.left_arm.action_features.items()},
            **{f"right_{key}": value for key, value in self.right_arm.action_features.items()},
        }

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

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

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        self.left_arm.calibrate()
        self.right_arm.calibrate()

    def configure(self) -> None:
        self.left_arm.configure()
        self.right_arm.configure()

    def setup_motors(self) -> None:
        self.left_arm.setup_motors()
        self.right_arm.setup_motors()

    def get_action(self) -> dict[str, float]:
        left_action = self.left_arm.get_action()
        right_action = self.right_arm.get_action()

        return {
            **{f"left_{key}": value for key, value in left_action.items()},
            **{f"right_{key}": value for key, value in right_action.items()},
        }

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError

    def disconnect(self) -> None:
        if self.left_arm.is_connected:
            self.left_arm.disconnect()
        if self.right_arm.is_connected:
            self.right_arm.disconnect()
