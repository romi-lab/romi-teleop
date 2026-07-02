"""UR3 robot interface using RTDE protocol and a Robotiq 2F-85 gripper."""

from __future__ import annotations

import logging
from functools import cached_property
from typing import Any

import numpy as np
import rtde_control
import rtde_receive
from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot
from lerobot.utils.errors import DeviceNotConnectedError

from .config_ur3 import UR3Config
from .robotiq_gripper import RobotiqGripper

logger = logging.getLogger(__name__)


class UR3(Robot):
    config_class = UR3Config
    name = "ur3"

    def __init__(self, config: UR3Config):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)

        self.robot_ip = config.ip
        self.rtde_ctrl: rtde_control.RTDEControlInterface | None = None
        self.rtde_rec: rtde_receive.RTDEReceiveInterface | None = None

        self.gripper: RobotiqGripper | None = RobotiqGripper() if config.with_gripper else None
        self._last_gripper_action = 0.0

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            "joint_0": float,
            "joint_1": float,
            "joint_2": float,
            "joint_3": float,
            "joint_4": float,
            "joint_5": float,
            "gripper": float,
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        features: dict[str, tuple] = {}
        for camera_name, camera in self.cameras.items():
            if getattr(camera, "use_rgb", True):
                features[camera_name] = (camera.height, camera.width, 3)
            if getattr(camera, "use_depth", False):
                features[f"{camera_name}_depth"] = (camera.height, camera.width, 1)
        return features

    @cached_property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        rtde_connected = (
            self.rtde_ctrl is not None
            and self.rtde_rec is not None
            and self.rtde_ctrl.isConnected()
            and self.rtde_rec.isConnected()
        )
        cameras_connected = all(camera.is_connected for camera in self.cameras.values())
        return rtde_connected and cameras_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            return

        self.rtde_ctrl = rtde_control.RTDEControlInterface(self.robot_ip)
        self.rtde_rec = rtde_receive.RTDEReceiveInterface(self.robot_ip)

        if self.gripper is not None:
            self.gripper.connect(self.robot_ip, self.config.gripper_port)
            self.gripper.activate(auto_calibrate=self.config.gripper_auto_calibrate)
            logger.info(
                "Robotiq gripper connected on %s:%s, auto_calibrate=%s",
                self.robot_ip,
                self.config.gripper_port,
                self.config.gripper_auto_calibrate,
            )

        for camera in self.cameras.values():
            camera.connect()

        self.configure()
        logger.info("%s connected.", self)

    def configure(self) -> None:
        pass

    def disconnect(self) -> None:
        if self.rtde_ctrl is not None:
            try:
                self.rtde_ctrl.servoStop()
            except Exception:
                pass
            self.rtde_ctrl.disconnect()
            self.rtde_ctrl = None

        if self.rtde_rec is not None:
            self.rtde_rec.disconnect()
            self.rtde_rec = None

        if self.gripper is not None and self.gripper.socket is not None:
            self.gripper.disconnect()

        for camera in self.cameras.values():
            camera.disconnect()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def _require_connection(self) -> tuple[rtde_control.RTDEControlInterface, rtde_receive.RTDEReceiveInterface]:
        if not self.is_connected or self.rtde_ctrl is None or self.rtde_rec is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        return self.rtde_ctrl, self.rtde_rec

    def get_observation(self) -> dict[str, Any]:
        _, rtde_rec = self._require_connection()

        joint_positions = rtde_rec.getActualQ()
        obs_dict = {f"joint_{i}": float(value) for i, value in enumerate(joint_positions)}
        obs_dict["gripper"] = self._last_gripper_action

        for camera_name, camera in self.cameras.items():
            if getattr(camera, "use_rgb", True):
                obs_dict[camera_name] = camera.async_read()
            if getattr(camera, "use_depth", False):
                obs_dict[f"{camera_name}_depth"] = camera.async_read_depth()

        return obs_dict

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        rtde_ctrl, rtde_rec = self._require_connection()
        if set(action) != set(self.action_features):
            raise ValueError(f"Invalid action: {action}, features: {self.action_features}")

        goal_joint_positions = np.asarray([float(action[f"joint_{i}"]) for i in range(6)], dtype=float)
        max_delta = self.config.max_joint_delta_per_step
        if max_delta is not None and max_delta > 0:
            current_joint_positions = np.asarray(rtde_rec.getActualQ(), dtype=float)
            joint_delta = np.clip(
                goal_joint_positions - current_joint_positions,
                -max_delta,
                max_delta,
            )
            goal_joint_positions = current_joint_positions + joint_delta

        rtde_ctrl.servoJ(
            goal_joint_positions.tolist(),
            self.config.servo_acceleration,
            self.config.servo_speed,
            self.config.servoj_t,
            self.config.servoj_lookahead,
            self.config.servoj_gain,
        )

        gripper_position = int(np.clip(float(action["gripper"]) * 255.0, 0, 255))
        if self.gripper is not None:
            self.gripper.move(gripper_position, self.config.gripper_speed, self.config.gripper_force)

        sent_action = {f"joint_{i}": float(goal_joint_positions[i]) for i in range(6)}
        sent_action["gripper"] = gripper_position / 255.0
        self._last_gripper_action = sent_action["gripper"]
        return sent_action

    def move_to_start_joints(self, wait: bool = True) -> None:
        self.move_to_joint_positions(
            self.config.start_joints_array(),
            speed=self.config.reset_speed,
            acceleration=self.config.reset_acceleration,
            wait=wait,
        )

    def move_to_joint_positions(
        self,
        joint_positions: np.ndarray | list[float] | tuple[float, ...],
        speed: float | None = None,
        acceleration: float | None = None,
        wait: bool = True,
    ) -> None:
        rtde_ctrl, _ = self._require_connection()
        joints = np.asarray(joint_positions, dtype=float)
        if joints.shape != (6,):
            raise ValueError(f"Expected 6 joint positions, got shape {joints.shape}.")

        rtde_ctrl.moveJ(
            joints.tolist(),
            speed if speed is not None else self.config.reset_speed,
            acceleration if acceleration is not None else self.config.reset_acceleration,
            not wait,
        )
